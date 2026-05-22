#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(values):
    values = np.asarray(values, dtype=float)
    return math.atan2(float(np.mean(np.sin(values))), float(np.mean(np.cos(values))))


def unwrap_near(values):
    center = circular_mean(values)
    return np.array([center + angle_diff(float(v), center) for v in values], dtype=float)


def load_layout(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {
        int(item["sensor"]): np.array(
            [float(item["x_m"]), float(item["y_m"]), float(item.get("z_m", 0.0))],
            dtype=float,
        )
        for item in data["sensors"]
    }


def sensor_world(pose, local):
    roll = math.radians(float(pose.get("roll_deg", 0.0)))
    pitch = math.radians(float(pose.get("pitch_deg", 0.0)))
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    pos = np.array(
        [float(pose["x_m"]), float(pose["y_m"]), float(pose.get("z_m", 0.0))],
        dtype=float,
    )
    return pos + rot @ local


def load_observations(poses_path, layout, angle_key):
    with open(poses_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    obs = []
    for pose in data["poses"]:
        for m in pose.get("measurements", []):
            sensor = int(m["sensor"])
            if sensor not in layout or angle_key not in m:
                continue
            obs.append(
                {
                    "pose": pose["name"],
                    "sensor": sensor,
                    "basestation": int(m["basestation"]),
                    "sweep": int(m["sweep"]),
                    "p": sensor_world(pose, layout[sensor]),
                    "angle": float(m[angle_key]),
                }
            )
    return obs


def fit_affine(group):
    points = np.array([o["p"] for o in group], dtype=float)
    values = unwrap_near([o["angle"] for o in group])
    A = np.column_stack([points, np.ones(len(points))])
    coeff, *_ = np.linalg.lstsq(A, values, rcond=None)
    pred = A @ coeff
    residual = np.array([angle_diff(float(p), float(v)) for p, v in zip(pred, values)], dtype=float)
    rmse = math.degrees(math.sqrt(float(np.mean(residual ** 2)))) if len(residual) else float("nan")
    med = math.degrees(float(np.median(np.abs(residual)))) if len(residual) else float("nan")
    mx = math.degrees(float(np.max(np.abs(residual)))) if len(residual) else float("nan")
    return coeff, rmse, med, mx


def paired_sweep_diagnostics(obs):
    by_key = {}
    for o in obs:
        key = (o["pose"], o["sensor"], o["basestation"])
        by_key.setdefault(key, {})[int(o["sweep"])] = o

    pairs = []
    for (pose, sensor, bs), sweeps in by_key.items():
        if 0 not in sweeps or 1 not in sweeps:
            continue
        a0 = float(sweeps[0]["angle"])
        a1 = float(sweeps[1]["angle"])
        mid = a1 + angle_diff(a0, a1) * 0.5
        sep = angle_diff(a0, a1)
        pairs.append({
            "pose": pose,
            "sensor": sensor,
            "basestation": bs,
            "p": sweeps[0]["p"],
            "mid": mid,
            "sep": sep,
        })
    return pairs


def main():
    parser = argparse.ArgumentParser(description="Diagnose whether known wand pose data is internally coherent.")
    parser.add_argument("--layout", default="config/sensors_layout_vertical_head_down.json")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d.json")
    parser.add_argument("--angle-key", default="raw_angle_rad", choices=["raw_angle_rad", "calibrated_angle_rad"])
    args = parser.parse_args()

    layout = load_layout(args.layout)
    obs = load_observations(args.poses, layout, args.angle_key)

    print("=" * 70)
    print("Wand pose data diagnostic")
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print(f"Angle:  {args.angle_key}")
    print(f"Observations: {len(obs)}")
    print("=" * 70)
    print("This fits a flexible local affine angle model per BS/sweep.")
    print("Low RMSE means poses/layout/angles are broadly coherent; high RMSE means data or layout is inconsistent.")
    print()

    for bs in sorted({o["basestation"] for o in obs}):
        for sweep in sorted({o["sweep"] for o in obs if o["basestation"] == bs}):
            group = [o for o in obs if o["basestation"] == bs and o["sweep"] == sweep]
            if len(group) < 4:
                continue
            coeff, rmse, med, mx = fit_affine(group)
            print(
                f"BS{bs} sweep{sweep}: n={len(group):3d} "
                f"affine_rmse={rmse:7.3f} deg | median={med:7.3f} deg | max={mx:7.3f} deg "
                f"| coeff=[{coeff[0]:+.3f}, {coeff[1]:+.3f}, {coeff[2]:+.3f}, {coeff[3]:+.3f}]"
            )

    print()
    print("Paired sweep diagnostics:")
    pairs = paired_sweep_diagnostics(obs)
    for bs in sorted({p["basestation"] for p in pairs}):
        group = [p for p in pairs if p["basestation"] == bs]
        if len(group) < 4:
            continue
        sep_deg = [math.degrees(angle_diff(p["sep"], 0.0)) for p in group]
        print(
            f"BS{bs}: pairs={len(group)} | "
            f"sweep0-sweep1 median={float(np.median(sep_deg)):+.3f} deg | "
            f"min={float(np.min(sep_deg)):+.3f} | max={float(np.max(sep_deg)):+.3f}"
        )
        mid_group = [
            {
                "p": p["p"],
                "angle": p["mid"],
            }
            for p in group
        ]
        coeff, rmse, med, mx = fit_affine(mid_group)
        print(
            f"      midpoint affine_rmse={rmse:7.3f} deg | median={med:7.3f} deg | max={mx:7.3f} deg "
            f"| coeff=[{coeff[0]:+.3f}, {coeff[1]:+.3f}, {coeff[2]:+.3f}, {coeff[3]:+.3f}]"
        )


if __name__ == "__main__":
    main()
