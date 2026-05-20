import argparse
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from dynamic_lh2_common import (
    angle_residual,
    group_observations_by_frame,
    load_json,
    load_observations_csv,
    load_sensors_layout,
    pose_look_at,
    predict_angles,
    residual_quality,
    save_json,
    sensor_world_position,
)


def pack_initial_parameters(lighthouse_ids, frames, settings):
    z_guess = float(settings.get("lighthouse_z_guess", 1.5))
    guesses = {
        lighthouse_ids[0]: np.array([1.5, 2.0, z_guess], dtype=float),
        lighthouse_ids[1] if len(lighthouse_ids) > 1 else lighthouse_ids[0]: np.array([-1.5, 2.0, z_guess], dtype=float),
    }

    values = []
    for lh_id in lighthouse_ids:
        translation = guesses.get(lh_id, np.array([0.0, 2.0, z_guess], dtype=float))
        rotvec = pose_look_at(translation, target=(0.0, 0.0, float(settings.get("drone_z", 0.0))))
        values.extend(translation.tolist())
        values.extend(rotvec.tolist())

    # Gauge choice: first frame is fixed at x=0, y=0, yaw=0.
    # The remaining frames are initialized with a small circular walk to avoid
    # all poses starting exactly identical.
    for idx in range(1, len(frames)):
        phase = 2.0 * math.pi * idx / max(len(frames), 1)
        values.extend([0.25 * math.cos(phase), 0.25 * math.sin(phase), 0.0])
    return np.asarray(values, dtype=float)


def unpack_parameters(params, lighthouse_ids, frame_count):
    offset = 0
    lighthouses = {}
    for lh_id in lighthouse_ids:
        translation = params[offset:offset + 3]
        rotvec = params[offset + 3:offset + 6]
        offset += 6
        lighthouses[lh_id] = {
            "translation": translation,
            "rotation_vector": rotvec,
        }

    drone_poses = [np.array([0.0, 0.0, 0.0], dtype=float)]
    for _ in range(1, frame_count):
        drone_poses.append(params[offset:offset + 3])
        offset += 3
    return lighthouses, drone_poses


def build_residuals_dynamic_calibration(params, lighthouse_ids, frames, sensors, settings):
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
            predicted = predict_angles(
                p_world,
                lighthouse["translation"],
                lighthouse["rotation_vector"],
            )
            residuals.extend(angle_residual(obs["angles_rad"], predicted).tolist())

    return np.asarray(residuals, dtype=float)


def solve_dynamic_calibration(frames, sensors, settings, max_nfev=2000):
    lighthouse_ids = [int(x) for x in settings.get("expected_lighthouses", [4, 10])]
    x0 = pack_initial_parameters(lighthouse_ids, frames, settings)
    f_scale = math.radians(float(settings.get("max_angle_error_deg", 5.0)))
    loss = settings.get("robust_loss", "soft_l1")

    result = least_squares(
        build_residuals_dynamic_calibration,
        x0,
        args=(lighthouse_ids, frames, sensors, settings),
        loss=loss,
        f_scale=f_scale,
        max_nfev=max_nfev,
        verbose=0,
    )
    residuals = build_residuals_dynamic_calibration(result.x, lighthouse_ids, frames, sensors, settings)
    lighthouses, drone_poses = unpack_parameters(result.x, lighthouse_ids, len(frames))
    return result, residuals, lighthouses, drone_poses


def make_output(settings, frames, df_count, residuals, lighthouses, drone_poses):
    quality = residual_quality(residuals)
    quality.update({
        "num_frames": len(frames),
        "num_observations": int(sum(len(frame["observations"]) for frame in frames)),
        "num_loaded_observations": int(df_count),
    })

    return {
        "version": 1,
        "model": "angular_camera_approximation_2d_dynamic_calibration",
        "units": "meters",
        "angle_units": "radians",
        "drone_z_assumed": float(settings.get("drone_z", 0.0)),
        "gauge": {
            "first_frame_pose_fixed": [0.0, 0.0, 0.0],
            "note": "Global translation/yaw gauge is fixed by the first drone frame.",
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
            }
            for frame, pose in zip(frames, drone_poses)
        ],
    }


def main():
    parser = argparse.ArgumentParser(description="Dynamic 2D calibration of Lighthouse geometry from moving drone observations.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--settings", default="config/calibration_settings.json")
    parser.add_argument("--input", default="data/captures/calibration_001.csv")
    parser.add_argument("--output", default="config/lighthouse_geometry.json")
    parser.add_argument("--max-nfev", type=int, default=2000)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-stride", type=int, default=1)
    args = parser.parse_args()

    _, sensors = load_sensors_layout(args.layout)
    settings = load_json(args.settings)
    df = load_observations_csv(args.input)

    frames, rejected = group_observations_by_frame(
        df,
        min_observations=int(settings.get("min_observations_per_pose", 4)),
        expected_lighthouses=settings.get("expected_lighthouses"),
    )
    if args.frame_stride > 1:
        frames = frames[::args.frame_stride]
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if len(frames) < 2:
        raise ValueError("Need at least 2 kept frames for dynamic calibration.")

    print("=" * 70)
    print("Dynamic Lighthouse geometry calibration")
    print(f"Input: {args.input}")
    print(f"Loaded observations: {len(df)}")
    print(f"Kept frames: {len(frames)}")
    print(f"Rejected frames: {rejected}")
    print("=" * 70)

    result, residuals, lighthouses, drone_poses = solve_dynamic_calibration(
        frames,
        sensors,
        settings,
        max_nfev=args.max_nfev,
    )
    output = make_output(settings, frames, len(df), residuals, lighthouses, drone_poses)
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
