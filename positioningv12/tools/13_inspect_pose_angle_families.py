#!/usr/bin/env python3

import argparse
import json
import math
from collections import defaultdict

import numpy as np


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def circ_mean(values):
    return math.atan2(
        float(np.mean([math.sin(v) for v in values])),
        float(np.mean([math.cos(v) for v in values])),
    )


def circ_spread_deg(values):
    if not values:
        return float("nan")
    center = circ_mean(values)
    return math.degrees(max(abs(angle_diff(v, center)) for v in values))


def midpoint(a0, a1):
    return a1 + 0.5 * angle_diff(a0, a1)


def load_pose_pairs(path, angle_key):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    rows = []
    for pose in data["poses"]:
        grouped = defaultdict(dict)
        for m in pose.get("measurements", []):
            if angle_key not in m:
                continue
            key = (int(m["sensor"]), int(m["basestation"]))
            grouped[key][int(m["sweep"])] = float(m[angle_key])

        for (sensor, bs), sweeps in grouped.items():
            if 0 not in sweeps or 1 not in sweeps:
                continue
            a0 = sweeps[0]
            a1 = sweeps[1]
            rows.append({
                "pose": pose["name"],
                "x": float(pose["x_m"]),
                "y": float(pose["y_m"]),
                "z": float(pose.get("z_m", 0.0)),
                "sensor": sensor,
                "bs": bs,
                "mid": midpoint(a0, a1),
                "sep": angle_diff(a0, a1),
            })
    return rows


def main():
    parser = argparse.ArgumentParser(description="Inspect LH2 angle families per known pose.")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d.json")
    parser.add_argument("--angle-key", default="raw_angle_rad", choices=["raw_angle_rad", "calibrated_angle_rad"])
    args = parser.parse_args()

    rows = load_pose_pairs(args.poses, args.angle_key)
    print("=" * 90)
    print("Pose angle family inspection")
    print(f"Poses: {args.poses}")
    print(f"Angle: {args.angle_key}")
    print("=" * 90)
    print("For each pose/BS: mid is the circular midpoint of sweep0/sweep1.")
    print("Large sensor_spread means sensors disagree at the same physical pose.")
    print()

    by_pose_bs = defaultdict(list)
    for row in rows:
        by_pose_bs[(row["pose"], row["bs"])].append(row)

    print(f"{'pose':28s} {'bs':>4s} {'xyz':>24s} {'mid_med_deg':>12s} {'sensor_spread':>14s} {'sep_med_deg':>12s} {'sep_spread':>11s}")
    for key in sorted(by_pose_bs):
        pose, bs = key
        group = by_pose_bs[key]
        mids = [r["mid"] for r in group]
        seps = [r["sep"] for r in group]
        xyz = f"{group[0]['x']:+.2f},{group[0]['y']:+.2f},{group[0]['z']:+.2f}"
        mid_med = circ_mean(mids)
        sep_med = circ_mean(seps)
        print(
            f"{pose:28s} {bs:4d} {xyz:>24s} "
            f"{math.degrees(mid_med):12.3f} {circ_spread_deg(mids):14.3f} "
            f"{math.degrees(sep_med):12.3f} {circ_spread_deg(seps):11.3f}"
        )


if __name__ == "__main__":
    main()
