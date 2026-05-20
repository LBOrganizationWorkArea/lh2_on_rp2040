#!/usr/bin/env python3
"""
Estimate one drone pose from observations and saved Lighthouse geometry.
This can be used with one frame exported as CSV.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd

try:
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as R
except ImportError:
    raise SystemExit("Missing dependency: py -m pip install scipy numpy pandas")


def load_layout(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {str(k): np.array([float(v["x"]), float(v["y"]), float(v.get("z", 0.0))]) for k, v in data["sensors"].items()}


def project_to_angles(lh_pos, lh_rotvec, point_world):
    rot = R.from_rotvec(lh_rotvec)
    p = rot.inv().apply(point_world - lh_pos)
    x, y, z = p
    z = max(z, 1e-6)
    theta = np.arctan2(-x, z)
    phi = np.arctan2(-y, np.sqrt(x*x + z*z))
    return np.array([theta, phi])


def pose_residuals(x, df, sensors, geometry, mode_2d=True):
    if mode_2d:
        pos = np.array([x[0], x[1], 0.0])
        rotvec = R.from_euler("z", x[2]).as_rotvec()
    else:
        pos = x[0:3]
        rotvec = x[3:6]
    rr = []
    for row in df.itertuples(index=False):
        sid = str(row.sensor_id)
        lh = str(row.lighthouse_id)
        if sid not in sensors or lh not in geometry["lighthouses"]:
            continue
        sensor_world = pos + R.from_rotvec(rotvec).apply(sensors[sid])
        lhdata = geometry["lighthouses"][lh]
        pred = project_to_angles(np.array(lhdata["position"]), np.array(lhdata["rotation_rotvec"]), sensor_world)
        meas = np.array([float(row.theta), float(row.phi)])
        rr.extend((pred - meas).tolist())
    return np.array(rr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--geometry", required=True)
    ap.add_argument("--observations", required=True)
    ap.add_argument("--mode", choices=["2d", "3d"], default="2d")
    args = ap.parse_args()

    sensors = load_layout(args.layout)
    geometry = json.loads(Path(args.geometry).read_text(encoding="utf-8"))
    df = pd.read_csv(args.observations)
    df["sensor_id"] = df["sensor_id"].astype(str)
    df["lighthouse_id"] = df["lighthouse_id"].astype(str)

    mode_2d = args.mode == "2d"
    x0 = np.array([0.0, 0.0, 0.0]) if mode_2d else np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    result = least_squares(pose_residuals, x0, args=(df, sensors, geometry, mode_2d), loss="huber", f_scale=0.01)

    if mode_2d:
        cx, cy, yaw = result.x
        print(f"drone_center_x = {cx:.6f} m")
        print(f"drone_center_y = {cy:.6f} m")
        print(f"drone_yaw      = {yaw:.6f} rad")
    else:
        print("drone_position_xyz =", result.x[0:3])
        print("drone_rotation_rotvec =", result.x[3:6])

    err = pose_residuals(result.x, df, sensors, geometry, mode_2d)
    print(f"rms_angle_error_rad = {np.sqrt(np.mean(err**2)) if len(err) else None}")


if __name__ == "__main__":
    main()
