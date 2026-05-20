import argparse
import math

import numpy as np
from scipy.optimize import least_squares

from dynamic_lh2_common import (
    angle_residual,
    group_observations_by_frame,
    lighthouse_pose_from_geometry,
    load_json,
    load_observations_csv,
    load_sensors_layout,
    predict_angles,
    residual_quality,
    sensor_world_position,
)


def pose_residuals(pose, frame, sensors, lighthouses, drone_z):
    residuals = []
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


def solve_live_pose(frame, sensors, lighthouses, drone_z, initial_pose, robust_loss="soft_l1"):
    result = least_squares(
        pose_residuals,
        np.asarray(initial_pose, dtype=float),
        args=(frame, sensors, lighthouses, drone_z),
        loss=robust_loss,
        f_scale=math.radians(3.0),
        max_nfev=100,
    )
    residuals = pose_residuals(result.x, frame, sensors, lighthouses, drone_z)
    return result.x, residuals, result


def quality_label(rmse_deg):
    if rmse_deg < 1.0:
        return "good"
    if rmse_deg < 3.0:
        return "medium"
    return "bad"


def main():
    parser = argparse.ArgumentParser(description="Live/test 2D positioning using solved Lighthouse geometry.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--geometry", default="config/lighthouse_geometry.json")
    parser.add_argument("--input", required=True, help="CSV observation file for testing.")
    parser.add_argument("--settings", default="config/calibration_settings.json")
    parser.add_argument("--max-frames", type=int, default=None)
    args = parser.parse_args()

    _, sensors = load_sensors_layout(args.layout)
    geometry = load_json(args.geometry)
    settings = load_json(args.settings)
    lighthouses = lighthouse_pose_from_geometry(geometry)
    drone_z = float(geometry.get("drone_z_assumed", settings.get("drone_z", 0.0)))

    df = load_observations_csv(args.input)
    frames, rejected = group_observations_by_frame(
        df,
        min_observations=int(settings.get("min_observations_per_pose", 4)),
        expected_lighthouses=[int(item["id"]) for item in geometry["lighthouses"]],
    )
    if args.max_frames is not None:
        frames = frames[:args.max_frames]

    print("=" * 70)
    print("2D positioning from fixed Lighthouse geometry")
    print(f"Input: {args.input}")
    print(f"Frames: {len(frames)} | rejected={rejected}")
    print("=" * 70)

    previous_pose = np.array([0.0, 0.0, 0.0], dtype=float)
    for frame in frames:
        pose, residuals, result = solve_live_pose(
            frame,
            sensors,
            lighthouses,
            drone_z,
            previous_pose,
            robust_loss=settings.get("robust_loss", "soft_l1"),
        )
        previous_pose = pose
        stats = residual_quality(residuals)
        label = quality_label(stats["rmse_deg"])
        print(
            f"{frame['timestamp']:.6f},"
            f"x={pose[0]:+.3f},y={pose[1]:+.3f},yaw_deg={math.degrees(pose[2]):+.1f},"
            f"quality={label},obs={len(frame['observations'])},rmse_deg={stats['rmse_deg']:.3f}"
        )


if __name__ == "__main__":
    main()
