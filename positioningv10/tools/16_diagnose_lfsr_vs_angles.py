#!/usr/bin/env python3

import argparse
import json
import math
from collections import defaultdict


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def period_distance(a, b, period):
    diff = abs((float(a) % period) - (float(b) % period))
    return min(diff, period - diff)


def quantile(values, q):
    if not values:
        return float("nan")
    values = sorted(values)
    return values[int((len(values) - 1) * q)]


def paired_measurements(data):
    rows = []
    for pose in data.get("poses", []):
        grouped = defaultdict(dict)
        for measurement in pose.get("measurements", []):
            key = (int(measurement["sensor"]), int(measurement["basestation"]))
            grouped[key][int(measurement["sweep"])] = measurement

        for (sensor, basestation), sweeps in grouped.items():
            if 0 not in sweeps or 1 not in sweeps:
                continue
            first = sweeps[0]
            second = sweeps[1]
            rows.append({
                "pose": pose.get("name", ""),
                "sensor": sensor,
                "basestation": basestation,
                "lfsr0": first.get("lfsr_location"),
                "lfsr1": second.get("lfsr_location"),
                "offset0": first.get("offset_ticks"),
                "offset1": second.get("offset_ticks"),
                "angle0": first.get("raw_angle_rad"),
                "angle1": second.get("raw_angle_rad"),
            })
    return rows


def summarize_distances(rows, first_key, second_key, period):
    distances = [
        period_distance(row[first_key], row[second_key], period)
        for row in rows
        if row.get(first_key) is not None and row.get(second_key) is not None
    ]
    if not distances:
        return None
    return {
        "count": len(distances),
        "min": min(distances),
        "q25": quantile(distances, 0.25),
        "median": quantile(distances, 0.50),
        "q75": quantile(distances, 0.75),
        "max": max(distances),
    }


def print_summary(label, summary, period):
    if summary is None:
        print(f"{label}: not present in this JSON")
        return
    print(
        f"{label}: n={summary['count']} | "
        f"min={summary['min']:.1f} q25={summary['q25']:.1f} "
        f"median={summary['median']:.1f} q75={summary['q75']:.1f} max={summary['max']:.1f} ticks"
    )
    print(
        f"  equivalent at period {period:g}: "
        f"median={summary['median'] * 360.0 / period:.3f} deg | "
        f"max={summary['max'] * 360.0 / period:.3f} deg"
    )


def main():
    parser = argparse.ArgumentParser(description="Diagnose whether LH2 LFSR/offset values can explain the stored raw angles.")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d_lfsr.json")
    parser.add_argument("--period", type=float, default=120000.0)
    parser.add_argument("--examples", type=int, default=12)
    args = parser.parse_args()

    with open(args.poses, "r", encoding="utf-8") as f:
        data = json.load(f)

    rows = paired_measurements(data)

    print("=" * 88)
    print("LH2 LFSR / angle diagnostic")
    print(f"Poses:  {args.poses}")
    print(f"Pairs:  {len(rows)}")
    print(f"Period: {args.period:g}")
    print("=" * 88)

    print_summary("sweep0/sweep1 lfsr distance", summarize_distances(rows, "lfsr0", "lfsr1", args.period), args.period)
    print_summary("sweep0/sweep1 offset distance", summarize_distances(rows, "offset0", "offset1", args.period), args.period)

    print()
    print("Examples:")
    for row in rows[: args.examples]:
        lfsr_dist = None
        if row.get("lfsr0") is not None and row.get("lfsr1") is not None:
            lfsr_dist = period_distance(row["lfsr0"], row["lfsr1"], args.period)
        offset_dist = None
        if row.get("offset0") is not None and row.get("offset1") is not None:
            offset_dist = period_distance(row["offset0"], row["offset1"], args.period)
        angle_sep = None
        if row.get("angle0") is not None and row.get("angle1") is not None:
            angle_sep = math.degrees(angle_diff(float(row["angle0"]), float(row["angle1"])))
        print(
            f"{row['pose']} BS{row['basestation']} S{row['sensor']}: "
            f"lfsr=({row.get('lfsr0')},{row.get('lfsr1')}) d={lfsr_dist if lfsr_dist is not None else 'na'} | "
            f"offset=({row.get('offset0')},{row.get('offset1')}) d={offset_dist if offset_dist is not None else 'na'} | "
            f"raw_sep={angle_sep:.2f} deg" if angle_sep is not None else ""
        )

    print()
    print("Interpretation:")
    print("- If lfsr distance is near zero but raw_sep is near 120 deg, the 120 deg separation is mostly produced by the +/-60 deg formula.")
    print("- In that case, LFSR location alone is probably not the physical beam angle needed for geometry fitting.")
    print("- If offset distance is missing, the capture JSON was made before offset_ticks was preserved in summaries.")


if __name__ == "__main__":
    main()
