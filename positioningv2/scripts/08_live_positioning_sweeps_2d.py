#!/usr/bin/env python3
"""Estimate 2D drone pose from paired LH2 sweep observations and fixed geometry."""

from __future__ import annotations

import argparse
import math
from collections import defaultdict

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from dynamic_lh2_common import (
    angle_residual,
    lighthouse_pose_from_geometry,
    load_json,
    load_sensors_layout,
    predict_angles,
    sensor_world_position,
)


TAN_30 = math.tan(math.radians(30.0))


def load_sweep_observations(path, sweep_columns):
    df = pd.read_csv(path)
    required = ["timestamp", "sensor_id", "lighthouse_id"] + list(sweep_columns)
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


def group_frames(df, frame_window_s, min_observations):
    t0 = float(df["timestamp"].min())
    buckets = defaultdict(list)
    for row in df.itertuples(index=False):
        frame_index = int((float(row.timestamp) - t0) / frame_window_s)
        buckets[frame_index].append({
            "timestamp": float(row.timestamp),
            "sensor_id": int(row.sensor_id),
            "lighthouse_id": int(row.lighthouse_id),
            "sweeps_rad": np.radians([float(row.measure_sweep0_deg), float(row.measure_sweep1_deg)]),
        })

    frames = []
    for frame_index in sorted(buckets):
        rows = buckets[frame_index]
        if len(rows) < min_observations:
            continue
        frames.append({
            "frame_index": frame_index,
            "timestamp": float(np.median([row["timestamp"] for row in rows])),
            "observations": rows,
        })
    return frames


def predict_sweeps(sensor_position_world, lighthouse_translation, lighthouse_rotvec):
    azimuth, elevation = predict_angles(sensor_position_world, lighthouse_translation, lighthouse_rotvec)
    return np.array([
        azimuth + TAN_30 * elevation,
        azimuth - TAN_30 * elevation,
    ], dtype=float)


def pose_residuals(pose, frame, sensors, lighthouses, drone_z):
    residuals = []
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


def solve_live_pose(frame, sensors, lighthouses, drone_z, initial_pose, max_nfev):
    result = least_squares(
        pose_residuals,
        initial_pose,
        args=(frame, sensors, lighthouses, drone_z),
        bounds=([-5.0, -5.0, -math.pi], [5.0, 5.0, math.pi]),
        loss="soft_l1",
        f_scale=math.radians(5.0),
        max_nfev=max_nfev,
    )
    residuals = pose_residuals(result.x, frame, sensors, lighthouses, drone_z)
    rmse = float(np.degrees(np.sqrt(np.mean(residuals ** 2)))) if residuals.size else float("nan")
    if rmse < 1.0:
        quality = "good"
    elif rmse < 3.0:
        quality = "medium"
    else:
        quality = "bad"
    return result.x, rmse, quality


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--geometry", default="config/lighthouse_geometry_sweeps.json")
    parser.add_argument("--input", default="data/captures/calibration_dynamic_001_sweeps.csv")
    parser.add_argument("--frame-window-ms", type=float, default=100.0)
    parser.add_argument(
        "--sweep-source",
        choices=["ordered", "model"],
        default="model",
        help="ordered uses polynomial sweep0/1; model uses current auto-swapped approximation columns.",
    )
    parser.add_argument("--min-observations", type=int, default=4)
    parser.add_argument("--max-nfev", type=int, default=80)
    parser.add_argument("--limit", type=int, default=80)
    args = parser.parse_args()

    _, sensors = load_sensors_layout(args.layout)
    geometry = load_json(args.geometry)
    lighthouses = lighthouse_pose_from_geometry(geometry)
    drone_z = float(geometry.get("drone_z_assumed", 0.0))
    sweep_columns = ("sweep0_deg", "sweep1_deg")
    if args.sweep_source == "model":
        sweep_columns = ("model_sweep0_deg", "model_sweep1_deg")
    df = load_sweep_observations(args.input, sweep_columns)
    frames = group_frames(df, args.frame_window_ms / 1000.0, args.min_observations)
    if args.limit:
        frames = frames[:args.limit]

    print("=" * 70)
    print("Replay/live 2D pose from LH2 sweeps")
    print(f"Geometry: {args.geometry}")
    print(f"Input: {args.input}")
    print(f"Sweep source: {args.sweep_source} columns={sweep_columns[0]},{sweep_columns[1]}")
    print(f"Frames: {len(frames)}")
    print("=" * 70)

    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    for frame in frames:
        pose, rmse_deg, quality = solve_live_pose(
            frame,
            sensors,
            lighthouses,
            drone_z,
            pose,
            args.max_nfev,
        )
        print(
            f"POSE,{frame['timestamp']:.6f},"
            f"{pose[0]:+.3f},{pose[1]:+.3f},{math.degrees(pose[2]):+.1f},"
            f"{quality},{len(frame['observations'])},{rmse_deg:.3f}"
        )


if __name__ == "__main__":
    main()
