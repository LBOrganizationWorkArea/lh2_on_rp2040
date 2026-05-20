#!/usr/bin/env python3

import argparse
import json
from collections import defaultdict


def main():
    parser = argparse.ArgumentParser(description="Diagnose LH2 LFSR pose data spans and jumps.")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    args = parser.parse_args()

    with open(args.poses, "r", encoding="utf-8") as f:
        data = json.load(f)

    grouped = defaultdict(list)
    for pose in data["poses"]:
        for m in pose["measurements"]:
            key = (int(m["basestation"]), int(m["sweep"]), int(m["sensor"]))
            grouped[key].append({
                "pose": pose["name"],
                "x": float(pose["x_m"]),
                "y": float(pose["y_m"]),
                "lfsr": float(m["median_lfsr_location"]),
            })

    print("=" * 70)
    print("LH2 pose data diagnostic")
    print(f"Poses: {args.poses}")
    print("=" * 70)

    all_rows = []
    for (bs, sweep, sensor), rows in sorted(grouped.items()):
        values = [r["lfsr"] for r in rows]
        span = max(values) - min(values)
        all_rows.append((span, bs, sweep, sensor, min(values), max(values)))

    print("Largest LFSR spans by BS/sweep/sensor:")
    for span, bs, sweep, sensor, min_v, max_v in sorted(all_rows, reverse=True)[:16]:
        print(f"  BS{bs} sweep{sweep} sensor{sensor}: span={span:.1f} ticks min={min_v:.1f} max={max_v:.1f}")

    print()
    print("Largest jumps from P0_center:")
    p0_by_key = {}
    for pose in data["poses"]:
        if pose["name"] != "P0_center":
            continue
        for m in pose["measurements"]:
            p0_by_key[(int(m["basestation"]), int(m["sweep"]), int(m["sensor"]))] = float(m["median_lfsr_location"])

    jumps = []
    for pose in data["poses"]:
        if pose["name"] == "P0_center":
            continue
        for m in pose["measurements"]:
            key = (int(m["basestation"]), int(m["sweep"]), int(m["sensor"]))
            if key not in p0_by_key:
                continue
            delta = float(m["median_lfsr_location"]) - p0_by_key[key]
            jumps.append((abs(delta), delta, pose["name"], key, float(m["median_lfsr_location"]), p0_by_key[key]))

    for abs_delta, delta, pose_name, key, value, p0 in sorted(jumps, reverse=True)[:24]:
        bs, sweep, sensor = key
        print(
            f"  {pose_name:16s} BS{bs} sweep{sweep} sensor{sensor}: "
            f"delta={delta:+.1f} ticks value={value:.1f} P0={p0:.1f}"
        )


if __name__ == "__main__":
    main()
