#!/usr/bin/env python3
"""
Bundle-adjustment style dynamic Lighthouse calibration.

Input observations CSV columns:
frame_id,timestamp,sensor_id,lighthouse_id,theta,phi,valid

This solver fixes the reference Lighthouse at origin with identity rotation.
It estimates the second Lighthouse pose and one 6D drone pose per frame.
The result is relative to the reference Lighthouse, with metric scale coming from sensors_layout.json.
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
    sensors = {}
    for sid, p in data["sensors"].items():
        sensors[str(sid)] = np.array([float(p["x"]), float(p["y"]), float(p.get("z", 0.0))], dtype=float)
    return sensors


def project_to_angles(lh_pos, lh_rotvec, point_world):
    # Transform world point into Lighthouse frame.
    rot = R.from_rotvec(lh_rotvec)
    p_lh = rot.inv().apply(point_world - lh_pos)
    x, y, z = p_lh
    if z <= 1e-6:
        # point behind Lighthouse: add a large but finite penalty through strange angles
        z = 1e-6
    theta = np.arctan2(-x, z)
    phi = np.arctan2(-y, np.sqrt(x*x + z*z))
    return np.array([theta, phi])


def unpack_params(x, n_frames):
    # x = [lh1_pos3, lh1_rotvec3, frame0_pos3, frame0_rotvec3, ...]
    lh1_pos = x[0:3]
    lh1_rot = x[3:6]
    poses = []
    offset = 6
    for _ in range(n_frames):
        pos = x[offset:offset+3]
        rot = x[offset+3:offset+6]
        poses.append((pos, rot))
        offset += 6
    return lh1_pos, lh1_rot, poses


def residuals(x, df, frame_ids, sensors, lighthouses, ref_lh):
    lh1_id = [lh for lh in lighthouses if lh != ref_lh][0]
    lh1_pos, lh1_rot, poses = unpack_params(x, len(frame_ids))
    frame_to_i = {fid: i for i, fid in enumerate(frame_ids)}
    res = []

    for row in df.itertuples(index=False):
        sid = str(row.sensor_id)
        lh = str(row.lighthouse_id)
        if sid not in sensors:
            continue
        i = frame_to_i[row.frame_id]
        drone_pos, drone_rotvec = poses[i]
        sensor_world = drone_pos + R.from_rotvec(drone_rotvec).apply(sensors[sid])

        if lh == ref_lh:
            lh_pos = np.zeros(3)
            lh_rot = np.zeros(3)
        elif lh == lh1_id:
            lh_pos = lh1_pos
            lh_rot = lh1_rot
        else:
            continue

        pred = project_to_angles(lh_pos, lh_rot, sensor_world)
        meas = np.array([float(row.theta), float(row.phi)])
        res.extend((pred - meas).tolist())

    return np.array(res)


def initial_guess(df, frame_ids):
    # Very rough initialization.
    # Reference LH at origin looking +Z. Second LH to the right/front. Frames in front.
    n = len(frame_ids)
    x0 = np.zeros(6 + 6*n)
    x0[0:3] = np.array([1.5, 0.0, 0.0])      # second LH position relative to reference
    x0[3:6] = np.array([0.0, 0.2, 0.0])      # slight rotation
    for k, fid in enumerate(frame_ids):
        off = 6 + 6*k
        x0[off:off+3] = np.array([0.0, 0.0, 2.0]) + np.array([0.01*k, 0.0, 0.0])
        x0[off+3:off+6] = np.array([0.0, 0.0, 0.0])
    return x0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--observations", required=True)
    ap.add_argument("--config", default="config/calibration_config.json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max_frames", type=int, default=250, help="Limit frames for first tests; increase after it works")
    args = ap.parse_args()

    sensors = load_layout(args.layout)
    cfg = json.loads(Path(args.config).read_text(encoding="utf-8")) if Path(args.config).exists() else {}
    ref_lh = str(cfg.get("reference_lighthouse_id", "4"))

    df = pd.read_csv(args.observations)
    df["sensor_id"] = df["sensor_id"].astype(str)
    df["lighthouse_id"] = df["lighthouse_id"].astype(str)
    if "frame_id" not in df.columns:
        df["frame_id"] = (df["timestamp"] / 0.02).round().astype(int)

    lighthouses = sorted(df["lighthouse_id"].unique().tolist())
    if ref_lh not in lighthouses:
        ref_lh = lighthouses[0]
    if len(lighthouses) != 2:
        raise SystemExit(f"This first solver expects exactly 2 lighthouses, found: {lighthouses}")

    # Keep frames with observations from both lighthouses and at least 3 sensors.
    good = []
    for fid, g in df.groupby("frame_id"):
        if g["lighthouse_id"].nunique() == 2 and g["sensor_id"].nunique() >= 3:
            good.append(fid)
    if len(good) > args.max_frames:
        step = max(1, len(good) // args.max_frames)
        good = good[::step][:args.max_frames]
    df = df[df["frame_id"].isin(good)].copy()
    frame_ids = sorted(df["frame_id"].unique().tolist())

    print(f"Lighthouses: {lighthouses}, reference={ref_lh}")
    print(f"Sensors in layout: {sorted(sensors.keys())}")
    print(f"Frames used: {len(frame_ids)}")
    print(f"Observations used: {len(df)}")
    if len(frame_ids) < 20:
        print("WARNING: use more frames for a robust calibration.")

    x0 = initial_guess(df, frame_ids)
    print("Optimizing... this can take time.")
    result = least_squares(
        residuals,
        x0,
        args=(df, frame_ids, sensors, lighthouses, ref_lh),
        loss="huber",
        f_scale=0.01,
        max_nfev=300,
        verbose=1
    )

    lh1_id = [lh for lh in lighthouses if lh != ref_lh][0]
    lh1_pos, lh1_rot, poses = unpack_params(result.x, len(frame_ids))
    err = residuals(result.x, df, frame_ids, sensors, lighthouses, ref_lh)
    rms_rad = float(np.sqrt(np.mean(err**2))) if len(err) else None

    out_data = {
        "unit": "m",
        "angle_unit": "rad",
        "calibration_type": "dynamic_rigid_body_bundle_adjustment_v1",
        "reference_frame": f"Lighthouse {ref_lh}",
        "rms_angle_error_rad": rms_rad,
        "lighthouses": {
            ref_lh: {"position": [0.0, 0.0, 0.0], "rotation_rotvec": [0.0, 0.0, 0.0]},
            lh1_id: {"position": lh1_pos.tolist(), "rotation_rotvec": lh1_rot.tolist()}
        },
        "notes": "Relative geometry. Do not move lighthouses after calibration. If the result is mirrored/rotated, align it later to your room/floor convention."
    }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(out_data, indent=2), encoding="utf-8")
    print(f"RMS angle error: {rms_rad}")
    print(f"Saved geometry: {out}")


if __name__ == "__main__":
    main()
