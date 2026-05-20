#!/usr/bin/env python3

import argparse
import importlib.util
import json
import math
from pathlib import Path


def load_live_position_module():
    module_path = Path(__file__).with_name("05_live_position.py")
    spec = importlib.util.spec_from_file_location("live_position", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose_observations(pose):
    observations = []
    for m in pose["measurements"]:
        item = {
            "sensor": int(m["sensor"]),
            "basestation": int(m["basestation"]),
            "sweep": int(m["sweep"]),
            "lfsr_location": float(m["median_lfsr_location"]),
        }
        if "raw_angle_rad" in m:
            item["raw_angle_rad"] = float(m["raw_angle_rad"])
        observations.append(item)

    return observations


def main():
    parser = argparse.ArgumentParser(description="Validate a Lighthouse geometry file against known calibration poses.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--geometry", default="config/lighthouse_geometry_lh2_guided_ultrafast.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--position-only", action="store_true")
    parser.add_argument("--planar-2d", action="store_true", help="Validate by solving x,y at a fixed z.")
    parser.add_argument("--fixed-z", type=float, default=0.0)
    parser.add_argument("--solve-yaw", action="store_true", help="With --planar-2d, solve x,y,yaw.")
    parser.add_argument("--xy-bound", type=float, default=5.0)
    parser.add_argument("--z-min", type=float, default=-0.20)
    parser.add_argument("--z-max", type=float, default=3.00)
    args = parser.parse_args()

    live = load_live_position_module()
    layout = live.load_layout(args.layout)
    geometry = live.load_geometry(args.geometry)

    with open(args.poses, "r") as f:
        calibration = json.load(f)

    solve_attitude = not args.position_only
    planar_2d = bool(args.planar_2d)
    previous = None
    errors = []
    errors_xy = []

    print("=" * 70)
    print("Validate Lighthouse geometry")
    print(f"Geometry: {args.geometry}")
    if planar_2d:
        mode = "planar 2D + yaw" if args.solve_yaw else "planar 2D"
    else:
        mode = "6D pose" if solve_attitude else "position only"
    print(f"Mode:     {mode}")
    print("=" * 70)

    for pose in calibration["poses"]:
        observations = pose_observations(pose)
        if planar_2d:
            previous, rmse_deg, used, ok = live.solve_planar_pose(
                observations,
                layout,
                geometry,
                previous,
                args.fixed_z,
                args.solve_yaw,
                args.xy_bound,
            )
            est_x = float(previous[0])
            est_y = float(previous[1])
            est_z = float(args.fixed_z)
        else:
            previous, rmse_deg, used, ok = live.solve_pose(
                observations,
                layout,
                geometry,
                previous,
                solve_attitude,
                args.xy_bound,
                (args.z_min, args.z_max),
            )
            est_x = float(previous[0])
            est_y = float(previous[1])
            est_z = float(previous[2])

        expected_x = float(pose["x_m"])
        expected_y = float(pose["y_m"])
        expected_z = float(pose.get("z_m", calibration.get("drone_z_m", 0.0)))
        dx = est_x - expected_x
        dy = est_y - expected_y
        dz = est_z - expected_z
        err_xy = math.hypot(dx, dy)
        err_xyz = math.sqrt(dx * dx + dy * dy + dz * dz)
        errors.append(err_xyz)
        errors_xy.append(err_xy)

        print(
            f"{pose['name']}: "
            f"est=({est_x:+.3f}, {est_y:+.3f}, {est_z:+.3f}) m | "
            f"expected=({expected_x:+.3f}, {expected_y:+.3f}, {expected_z:+.3f}) m | "
            f"err_xy={err_xy:.3f} m | err_xyz={err_xyz:.3f} m | rmse={rmse_deg:.2f} deg | obs={used}"
        )

    mean_error = sum(errors) / len(errors) if errors else float("nan")
    mean_xy_error = sum(errors_xy) / len(errors_xy) if errors_xy else float("nan")
    print("=" * 70)
    print(f"Mean XY error: {mean_xy_error:.3f} m")
    if not planar_2d:
        print(f"Mean 3D error: {mean_error:.3f} m")
    print("=" * 70)


if __name__ == "__main__":
    main()
