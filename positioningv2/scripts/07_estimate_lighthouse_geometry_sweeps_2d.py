#!/usr/bin/env python3
"""Dynamic 2D Lighthouse calibration from paired LH2 sweep measurements.

Input is the CSV produced by 06_convert_lfsr_to_sweeps.py. Unlike the angular
camera test solver, this script compares ordered sweep0/sweep1 measurements.
It still uses the simple v7 linear LFSR->sweep-degree calibration, so the next
physics improvement is to replace predict_sweeps() with the exact LH2 plane
model.
"""

from __future__ import annotations

import argparse
import math
from collections import defaultdict

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from dynamic_lh2_common import (
    angle_residual,
    load_json,
    load_sensors_layout,
    pose_look_at,
    predict_angles,
    residual_quality,
    save_json,
    sensor_world_position,
)


TAN_30 = math.tan(math.radians(30.0))
BASE_COLUMNS = ["timestamp", "sensor_id", "lighthouse_id"]


def load_sweep_observations(path, sweep_columns):
    df = pd.read_csv(path)
    required = BASE_COLUMNS + list(sweep_columns)
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"Missing sweep CSV columns: {missing}")
    sweep0_col, sweep1_col = sweep_columns
    df = df.dropna(subset=[sweep0_col, sweep1_col]).copy()
    df = df[df[sweep0_col].astype(str) != ""]
    df = df[df[sweep1_col].astype(str) != ""]
    df["timestamp"] = df["timestamp"].astype(float)
    df["sensor_id"] = df["sensor_id"].astype(int)
    df["lighthouse_id"] = df["lighthouse_id"].astype(int)
    df["measure_sweep0_deg"] = df[sweep0_col].astype(float)
    df["measure_sweep1_deg"] = df[sweep1_col].astype(float)
    return df.sort_values("timestamp").reset_index(drop=True)


def group_sweeps_by_time(df, frame_window_s, min_observations, expected_lighthouses=None):
    expected = None if expected_lighthouses is None else {int(x) for x in expected_lighthouses}
    t0 = float(df["timestamp"].min())
    buckets = defaultdict(list)

    for row in df.itertuples(index=False):
        lighthouse_id = int(row.lighthouse_id)
        if expected is not None and lighthouse_id not in expected:
            continue
        frame_index = int((float(row.timestamp) - t0) / frame_window_s)
        buckets[frame_index].append({
            "timestamp": float(row.timestamp),
            "sensor_id": int(row.sensor_id),
            "lighthouse_id": lighthouse_id,
            "sweeps_rad": np.radians([float(row.measure_sweep0_deg), float(row.measure_sweep1_deg)]),
        })

    frames = []
    rejected = 0
    for frame_index in sorted(buckets):
        rows = buckets[frame_index]
        if len(rows) < min_observations:
            rejected += 1
            continue
        frames.append({
            "frame_index": int(frame_index),
            "timestamp": float(np.median([row["timestamp"] for row in rows])),
            "observations": rows,
        })
    return frames, rejected


def predict_sweeps(sensor_position_world, lighthouse_translation, lighthouse_rotvec):
    """Predict ordered LH2 sweep angles from the current simplified model.

    Approximation:
      az, el = angular camera coordinates in Lighthouse frame
      sweep0 = az + tan(30 deg) * el
      sweep1 = az - tan(30 deg) * el
    """
    azimuth, elevation = predict_angles(
        sensor_position_world,
        lighthouse_translation,
        lighthouse_rotvec,
    )
    return np.array([
        azimuth + TAN_30 * elevation,
        azimuth - TAN_30 * elevation,
    ], dtype=float)


def pack_initial_parameters(lighthouse_ids, frames, settings):
    z_guess = float(settings.get("lighthouse_z_guess", 1.5))
    default_guesses = {
        lighthouse_ids[0]: np.array([1.5, 2.0, z_guess], dtype=float),
    }
    if len(lighthouse_ids) > 1:
        default_guesses[lighthouse_ids[1]] = np.array([-1.5, 2.0, z_guess], dtype=float)

    values = []
    lower = []
    upper = []

    for lh_id in lighthouse_ids:
        translation = default_guesses.get(lh_id, np.array([0.0, 2.0, z_guess], dtype=float))
        rotvec = pose_look_at(translation, target=(0.0, 0.0, float(settings.get("drone_z", 0.0))))
        values.extend(translation.tolist())
        values.extend(rotvec.tolist())
        lower.extend([-10.0, -10.0, 0.2, -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi])
        upper.extend([+10.0, +10.0, 4.0, +2.0 * math.pi, +2.0 * math.pi, +2.0 * math.pi])

    # Gauge: first drone frame is fixed. Remaining frames are optimized.
    for idx in range(1, len(frames)):
        phase = 2.0 * math.pi * idx / max(len(frames), 1)
        values.extend([0.4 * math.cos(phase), 0.4 * math.sin(phase), 0.0])
        lower.extend([-5.0, -5.0, -math.pi])
        upper.extend([+5.0, +5.0, +math.pi])

    return np.asarray(values), (np.asarray(lower), np.asarray(upper))


def unpack_parameters(params, lighthouse_ids, frame_count):
    offset = 0
    lighthouses = {}
    for lh_id in lighthouse_ids:
        lighthouses[lh_id] = {
            "translation": params[offset:offset + 3],
            "rotation_vector": params[offset + 3:offset + 6],
        }
        offset += 6

    drone_poses = [np.array([0.0, 0.0, 0.0], dtype=float)]
    for _ in range(1, frame_count):
        drone_poses.append(params[offset:offset + 3])
        offset += 3
    return lighthouses, drone_poses


def residuals_from_params(params, lighthouse_ids, frames, sensors, settings):
    lighthouses, drone_poses = unpack_parameters(params, lighthouse_ids, len(frames))
    drone_z = float(settings.get("drone_z", 0.0))
    residuals = []

    for frame_idx, frame in enumerate(frames):
        pose = drone_poses[frame_idx]
        for obs in frame["observations"]:
            sensor_pos = sensors.get(obs["sensor_id"])
            lighthouse = lighthouses.get(obs["lighthouse_id"])
            if sensor_pos is None or lighthouse is None:
                continue
            p_world = sensor_world_position(pose, sensor_pos, drone_z)
            predicted = predict_sweeps(
                p_world,
                lighthouse["translation"],
                lighthouse["rotation_vector"],
            )
            residuals.extend(angle_residual(obs["sweeps_rad"], predicted).tolist())
    return np.asarray(residuals, dtype=float)


def observation_error_deg(obs, pose, sensors, lighthouse, drone_z):
    sensor_pos = sensors.get(obs["sensor_id"])
    if sensor_pos is None or lighthouse is None:
        return float("inf")
    p_world = sensor_world_position(pose, sensor_pos, drone_z)
    predicted = predict_sweeps(
        p_world,
        lighthouse["translation"],
        lighthouse["rotation_vector"],
    )
    residual = angle_residual(obs["sweeps_rad"], predicted)
    return float(np.degrees(np.max(np.abs(residual))))


def filter_outlier_observations(frames, lighthouses, drone_poses, sensors, settings, reject_deg, min_observations):
    drone_z = float(settings.get("drone_z", 0.0))
    filtered = []
    removed = 0

    for frame, pose in zip(frames, drone_poses):
        kept = []
        for obs in frame["observations"]:
            lighthouse = lighthouses.get(obs["lighthouse_id"])
            err_deg = observation_error_deg(obs, pose, sensors, lighthouse, drone_z)
            if err_deg <= reject_deg:
                kept.append(obs)
            else:
                removed += 1
        if len(kept) >= min_observations:
            filtered.append({
                "frame_index": frame["frame_index"],
                "timestamp": frame["timestamp"],
                "observations": kept,
            })
        else:
            removed += len(kept)

    return filtered, removed


def solve(frames, sensors, settings, max_nfev):
    lighthouse_ids = [int(x) for x in settings.get("expected_lighthouses", [4, 10])]
    x0, bounds = pack_initial_parameters(lighthouse_ids, frames, settings)
    f_scale = math.radians(float(settings.get("max_angle_error_deg", 5.0)))

    result = least_squares(
        residuals_from_params,
        x0,
        bounds=bounds,
        args=(lighthouse_ids, frames, sensors, settings),
        loss=settings.get("robust_loss", "soft_l1"),
        f_scale=f_scale,
        max_nfev=max_nfev,
        verbose=0,
    )
    residuals = residuals_from_params(result.x, lighthouse_ids, frames, sensors, settings)
    lighthouses, drone_poses = unpack_parameters(result.x, lighthouse_ids, len(frames))
    return result, residuals, lighthouses, drone_poses


def solve_with_optional_outlier_rejection(frames, sensors, settings, max_nfev, reject_deg, reject_rounds, min_observations):
    current_frames = frames
    total_removed = 0
    result = residuals = lighthouses = drone_poses = None

    rounds = max(1, reject_rounds + 1 if reject_deg is not None else 1)
    for round_index in range(rounds):
        result, residuals, lighthouses, drone_poses = solve(current_frames, sensors, settings, max_nfev)
        if reject_deg is None or round_index >= reject_rounds:
            break

        next_frames, removed = filter_outlier_observations(
            current_frames,
            lighthouses,
            drone_poses,
            sensors,
            settings,
            reject_deg,
            min_observations,
        )
        total_removed += removed
        print(
            f"Outlier round {round_index + 1}: removed={removed} "
            f"frames {len(current_frames)} -> {len(next_frames)}"
        )
        if len(next_frames) < 2 or removed == 0:
            break
        current_frames = next_frames

    return result, residuals, lighthouses, drone_poses, current_frames, total_removed


def make_output(settings, frames, loaded_count, residuals, lighthouses, drone_poses, frame_window_s):
    quality = residual_quality(residuals)
    quality.update({
        "num_frames": len(frames),
        "num_observations": int(sum(len(frame["observations"]) for frame in frames)),
        "num_loaded_observations": int(loaded_count),
        "frame_window_s": float(frame_window_s),
    })
    return {
        "version": 1,
        "model": "lh2_ordered_sweep_approximation_2d_dynamic_calibration",
        "units": "meters",
        "angle_units": "radians",
        "drone_z_assumed": float(settings.get("drone_z", 0.0)),
        "gauge": {
            "first_frame_pose_fixed": [0.0, 0.0, 0.0],
            "note": "The map is relative: first drone frame defines world origin and yaw.",
        },
        "lighthouses": [
            {
                "id": int(lh_id),
                "translation": lighthouses[lh_id]["translation"].tolist(),
                "rotation_vector": lighthouses[lh_id]["rotation_vector"].tolist(),
            }
            for lh_id in sorted(lighthouses)
        ],
        "calibration_quality": quality,
        "estimated_trajectory": [
            {
                "timestamp": float(frame["timestamp"]),
                "x": float(pose[0]),
                "y": float(pose[1]),
                "yaw_rad": float(pose[2]),
                "num_observations": int(len(frame["observations"])),
            }
            for frame, pose in zip(frames, drone_poses)
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--settings", default="config/calibration_settings.json")
    parser.add_argument("--input", default="data/captures/calibration_dynamic_001_sweeps.csv")
    parser.add_argument("--output", default="config/lighthouse_geometry_sweeps.json")
    parser.add_argument("--frame-window-ms", type=float, default=100.0)
    parser.add_argument(
        "--sweep-source",
        choices=["ordered", "model"],
        default="model",
        help="ordered uses polynomial sweep0/1; model uses current auto-swapped approximation columns.",
    )
    parser.add_argument("--min-observations", type=int, default=None)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--max-nfev", type=int, default=800)
    parser.add_argument("--reject-outliers-deg", type=float, default=None)
    parser.add_argument("--reject-rounds", type=int, default=2)
    args = parser.parse_args()

    _, sensors = load_sensors_layout(args.layout)
    settings = load_json(args.settings)
    min_observations = args.min_observations
    if min_observations is None:
        min_observations = int(settings.get("min_observations_per_pose", 4))

    sweep_columns = ("sweep0_deg", "sweep1_deg")
    if args.sweep_source == "model":
        sweep_columns = ("model_sweep0_deg", "model_sweep1_deg")
    df = load_sweep_observations(args.input, sweep_columns)
    frame_window_s = args.frame_window_ms / 1000.0
    frames, rejected = group_sweeps_by_time(
        df,
        frame_window_s=frame_window_s,
        min_observations=min_observations,
        expected_lighthouses=settings.get("expected_lighthouses"),
    )
    if args.frame_stride > 1:
        frames = frames[::args.frame_stride]
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if len(frames) < 2:
        raise ValueError("Need at least 2 kept frames. Increase --frame-window-ms or lower --min-observations.")

    print("=" * 70)
    print("Dynamic LH2 sweep geometry calibration")
    print(f"Input: {args.input}")
    print(f"Loaded paired observations: {len(df)}")
    print(f"Sweep source: {args.sweep_source} columns={sweep_columns[0]},{sweep_columns[1]}")
    print(f"Frame window: {args.frame_window_ms:.1f} ms")
    print(f"Kept frames: {len(frames)}")
    print(f"Rejected sparse frames: {rejected}")
    print("=" * 70)

    result, residuals, lighthouses, drone_poses, solved_frames, removed_outliers = solve_with_optional_outlier_rejection(
        frames,
        sensors,
        settings,
        args.max_nfev,
        args.reject_outliers_deg,
        args.reject_rounds,
        min_observations,
    )
    frames = solved_frames
    output = make_output(settings, frames, len(df), residuals, lighthouses, drone_poses, frame_window_s)
    output["calibration_quality"]["removed_outliers"] = int(removed_outliers)
    save_json(args.output, output)

    quality = output["calibration_quality"]
    print(f"Success: {result.success} | cost={result.cost:.6g} | nfev={result.nfev}")
    for item in output["lighthouses"]:
        t = item["translation"]
        r = item["rotation_vector"]
        print(
            f"BS{item['id']}: "
            f"translation=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m "
            f"rotvec=({r[0]:+.3f},{r[1]:+.3f},{r[2]:+.3f})"
        )
    print(
        f"RMSE={quality['rmse_deg']:.3f} deg | "
        f"median={quality['median_error_deg']:.3f} deg | "
        f"max={quality['max_error_deg']:.3f} deg"
    )
    max_allowed = float(settings.get("max_angle_error_deg", 5.0))
    if quality["rmse_deg"] > max_allowed:
        print(f"WARNING: RMSE is above max_angle_error_deg={max_allowed:.3f}")
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
