#!/usr/bin/env python3
"""Convert anchored raw LFSR point captures into paired sweep observations."""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path

from lh2_lfsr_common import (
    lfsr_pair_to_measurement,
    lfsr_pair_to_ordered_sweeps,
    load_lfsr_coefficients,
)


FIELDS = [
    "point_id",
    "point_x",
    "point_y",
    "point_z",
    "point_yaw_deg",
    "sensor_id",
    "lighthouse_id",
    "lfsr0",
    "lfsr1",
    "n0",
    "n1",
    "sweep0_deg",
    "sweep1_deg",
    "model_sweep0_deg",
    "model_sweep1_deg",
    "azimuth_deg",
    "elevation_deg",
    "mode",
]


def load_rows(path):
    with open(path, "r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["point_x"] = float(row["point_x"])
        row["point_y"] = float(row["point_y"])
        row["point_z"] = float(row["point_z"])
        row["point_yaw_deg"] = float(row["point_yaw_deg"])
        row["sensor_id"] = int(row["sensor_id"])
        row["lighthouse_id"] = int(row["lighthouse_id"])
        row["sweep"] = int(row["sweep"])
        row["lfsr"] = int(row["lfsr"])
    return rows


def median(values):
    return float(statistics.median(values))


def parse_mode_by_lighthouse(text):
    modes = {}
    if not text:
        return modes
    for chunk in text.split(","):
        if not chunk.strip():
            continue
        lighthouse_text, mode = chunk.split("=", 1)
        mode = mode.strip().lower()
        if mode not in ("auto", "normal", "swapped"):
            raise ValueError(f"Bad conversion mode '{mode}', expected auto, normal, or swapped")
        modes[int(lighthouse_text.strip())] = mode
    return modes


def convert(rows, coeffs, min_samples, mode_by_lighthouse):
    buckets = {}
    meta = {}
    for row in rows:
        key = (row["point_id"], row["sensor_id"], row["lighthouse_id"])
        buckets.setdefault(key, {0: [], 1: []})
        meta[key] = row
        if row["sweep"] in (0, 1):
            buckets[key][row["sweep"]].append(row["lfsr"])

    output = []
    for key in sorted(buckets):
        point_id, sensor_id, lighthouse_id = key
        by_sweep = buckets[key]
        if len(by_sweep[0]) < min_samples or len(by_sweep[1]) < min_samples:
            continue
        row = meta[key]
        lfsr0 = median(by_sweep[0])
        lfsr1 = median(by_sweep[1])
        item = {
            "point_id": point_id,
            "point_x": row["point_x"],
            "point_y": row["point_y"],
            "point_z": row["point_z"],
            "point_yaw_deg": row["point_yaw_deg"],
            "sensor_id": sensor_id,
            "lighthouse_id": lighthouse_id,
            "lfsr0": f"{lfsr0:.3f}",
            "lfsr1": f"{lfsr1:.3f}",
            "n0": len(by_sweep[0]),
            "n1": len(by_sweep[1]),
            "sweep0_deg": "",
            "sweep1_deg": "",
            "model_sweep0_deg": "",
            "model_sweep1_deg": "",
            "azimuth_deg": "",
            "elevation_deg": "",
            "mode": "",
        }
        if lighthouse_id in coeffs:
            ordered = lfsr_pair_to_ordered_sweeps(lfsr0, lfsr1, coeffs[lighthouse_id])
            mode = mode_by_lighthouse.get(lighthouse_id, "auto")
            debug_angles = lfsr_pair_to_measurement(lfsr0, lfsr1, coeffs[lighthouse_id], mode=mode)
            item.update({
                "sweep0_deg": f"{ordered['sweep0_deg']:.6f}",
                "sweep1_deg": f"{ordered['sweep1_deg']:.6f}",
                "model_sweep0_deg": f"{debug_angles['sweep0_deg']:.6f}",
                "model_sweep1_deg": f"{debug_angles['sweep1_deg']:.6f}",
                "azimuth_deg": f"{debug_angles['azimuth_deg']:.6f}",
                "elevation_deg": f"{debug_angles['elevation_deg']:.6f}",
                "mode": debug_angles["mode"],
            })
        output.append(item)
    return output


def print_coverage(rows, converted):
    sensors = sorted({row["sensor_id"] for row in rows})
    lighthouses = sorted({row["lighthouse_id"] for row in rows})
    points = sorted({row["point_id"] for row in rows})
    print(f"Raw coverage: points={len(points)} sensors={sensors} BS={lighthouses}")

    converted_keys = {
        (row["point_id"], int(row["sensor_id"]), int(row["lighthouse_id"]))
        for row in converted
    }
    expected_keys = {
        (point, sensor, lighthouse)
        for point in points
        for sensor in sensors
        for lighthouse in lighthouses
    }
    missing = sorted(expected_keys - converted_keys)
    print(f"Paired coverage: {len(converted_keys)}/{len(expected_keys)} point/sensor/BS combinations")
    if missing:
        preview = ", ".join(f"{p}:S{s}:BS{b}" for p, s, b in missing[:12])
        suffix = " ..." if len(missing) > 12 else ""
        print(f"Missing paired combinations: {preview}{suffix}")

    modes = {}
    for row in converted:
        mode = row.get("mode", "")
        modes[mode] = modes.get(mode, 0) + 1
    if modes:
        print("Model sweep modes: " + ", ".join(f"{mode or 'blank'}={count}" for mode, count in sorted(modes.items())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="data/captures/anchored_floor9_lfsr_raw.csv")
    parser.add_argument("--output", default="data/captures/anchored_floor9_sweeps.csv")
    parser.add_argument("--coefficients", default="config/history_calibration.txt")
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument(
        "--mode-by-lighthouse",
        default="",
        help="Force conversion mode per lighthouse, e.g. '10=normal' or '4=auto,10=swapped'.",
    )
    args = parser.parse_args()

    rows = load_rows(args.input)
    coeffs = load_lfsr_coefficients(args.coefficients)
    mode_by_lighthouse = parse_mode_by_lighthouse(args.mode_by_lighthouse)
    converted = convert(rows, coeffs, args.min_samples, mode_by_lighthouse)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(converted)

    print("=" * 70)
    print("Convert anchored raw LH2 LFSR to paired sweeps")
    print(f"Input rows: {len(rows)}")
    print(f"Output paired observations: {len(converted)}")
    if mode_by_lighthouse:
        print(
            "Forced conversion modes: "
            + ", ".join(f"BS{lh_id}={mode}" for lh_id, mode in sorted(mode_by_lighthouse.items()))
        )
    print_coverage(rows, converted)
    print(f"Output: {output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
