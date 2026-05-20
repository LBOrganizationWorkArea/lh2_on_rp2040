#!/usr/bin/env python3

import json
import math
import argparse
from pathlib import Path
from statistics import median

import numpy as np


TICKS_PER_REV = 833333
STABLE_BASESTATIONS = [4, 10]


def lfsr_to_rad(lfsr_location):
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


def angle_diff(a, b):
    d = a - b
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def angle_from_measurement(m):
    if "angle_rad" in m:
        return float(m["angle_rad"])
    if "angle_deg" in m:
        return math.radians(float(m["angle_deg"]))
    if "median_lfsr_location" in m:
        return lfsr_to_rad(float(m["median_lfsr_location"]))
    if "lfsr_location" in m:
        return lfsr_to_rad(float(m["lfsr_location"]))
    raise ValueError(m)


def make_feature_keys():
    keys = []
    for sensor in range(4):
        for bs in STABLE_BASESTATIONS:
            for sweep in range(2):
                keys.append((sensor, bs, sweep))
    return keys


def features_from_measurements(measurements, feature_keys):
    grouped = {}

    for m in measurements:
        sensor = int(m["sensor"])
        bs = int(m["basestation"])
        sweep = int(m["sweep"])

        if bs not in STABLE_BASESTATIONS:
            continue

        key = (sensor, bs, sweep)
        grouped.setdefault(key, []).append(angle_from_measurement(m))

    out = []

    for key in feature_keys:
        if key not in grouped:
            return None
        out.append(float(median(grouped[key])))

    return np.array(out, dtype=float)


def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)

    keys = make_feature_keys()
    points = {}

    for p in data["points"]:
        fvec = features_from_measurements(p["measurements"], keys)
        if fvec is None:
            print("ignored missing:", p["name"])
            continue

        points[p["name"]] = {
            "x": float(p["x_m"]),
            "y": float(p["y_m"]),
            "features": fvec,
        }

    return points, keys


def build_jacobian(points):
    p0 = points["P0_center"]
    pr = points["P1_right_30cm"]
    pl = points["P2_left_30cm"]
    pu = points["P3_up_30cm"]
    pd = points["P4_down_30cm"]

    xr = pr["x"]
    xl = pl["x"]
    yu = pu["y"]
    yd = pd["y"]

    f0 = p0["features"]

    df_dx = np.array([
        angle_diff(pr["features"][i], pl["features"][i]) / (xr - xl)
        for i in range(len(f0))
    ])

    df_dy = np.array([
        angle_diff(pu["features"][i], pd["features"][i]) / (yu - yd)
        for i in range(len(f0))
    ])

    A = np.vstack([df_dx, df_dy]).T

    strengths = np.linalg.norm(A, axis=1)
    threshold = np.percentile(strengths, 35)
    keep = strengths > threshold

    if np.sum(keep) < 6:
        keep = np.ones_like(strengths, dtype=bool)

    return f0, A, keep


def estimate(A, keep, f0, f, damping):
    delta = np.array([angle_diff(f[i], f0[i]) for i in range(len(f0))])
    A2 = A[keep]
    b = delta[keep]

    xy = np.linalg.solve(A2.T @ A2 + damping * np.eye(2), A2.T @ b)

    pred = A2 @ xy
    rmse = math.degrees(float(np.sqrt(np.mean((b - pred) ** 2))))

    return float(xy[0]), float(xy[1]), rmse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", default="config/init_poses_2d.json")
    parser.add_argument("--damping", type=float, default=0.05)
    args = parser.parse_args()

    points, keys = load_calibration(Path(args.calibration))
    f0, A, keep = build_jacobian(points)

    print("Jacobian calibration diagnostic")
    print("=" * 60)
    print(f"Calibration: {args.calibration}")
    print(f"Damping: {args.damping}")
    print(f"Features kept: {int(np.sum(keep))}/{len(keys)}")
    print("=" * 60)

    for name, p in points.items():
        x, y, rmse = estimate(A, keep, f0, p["features"], args.damping)
        print(
            f"{name:16s} | "
            f"target=({p['x']:+.3f},{p['y']:+.3f}) m | "
            f"pred=({x:+.3f},{y:+.3f}) m | "
            f"rmse={rmse:.3f} deg"
        )


if __name__ == "__main__":
    main()
