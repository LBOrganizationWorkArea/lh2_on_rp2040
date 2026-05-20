#!/usr/bin/env python3
"""Convert raw LH2 LFSR CSV into paired sweep observations.

This is the bridge between the working Pico firmware and the PC-side geometry.
It keeps the raw LFSR values, and optionally adds the current provisional
linear sweep-angle conversion from history_calibration.txt.
"""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path

from lh2_lfsr_common import (
    group_raw_rows_by_window,
    lfsr_pair_to_ordered_sweeps,
    lfsr_pair_to_measurement,
    load_lfsr_coefficients,
    read_raw_lfsr_csv,
)


FIELDS = [
    "timestamp",
    "frame_index",
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


def median(values: list[int]) -> float:
    return float(statistics.median(values))


def convert_rows(rows: list[dict], coeffs: dict[int, dict[str, float]], window_s: float, min_samples: int) -> list[dict]:
    output = []
    grouped = group_raw_rows_by_window(rows, window_s)

    for frame_index in sorted(grouped):
        frame_rows = grouped[frame_index]
        timestamp = min(row["pc_time"] for row in frame_rows)
        buckets: dict[tuple[int, int], dict[int, list[int]]] = {}

        for row in frame_rows:
            key = (row["sensor_id"], row["lighthouse_id"])
            buckets.setdefault(key, {0: [], 1: []})
            if row["sweep"] in (0, 1):
                buckets[key][row["sweep"]].append(row["lfsr"])

        for (sensor_id, lighthouse_id), by_sweep in sorted(buckets.items()):
            if len(by_sweep[0]) < min_samples or len(by_sweep[1]) < min_samples:
                continue

            lfsr0 = median(by_sweep[0])
            lfsr1 = median(by_sweep[1])
            item = {
                "timestamp": f"{timestamp:.6f}",
                "frame_index": frame_index,
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
                debug_angles = lfsr_pair_to_measurement(lfsr0, lfsr1, coeffs[lighthouse_id])
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="data/captures/calibration_001_lfsr_raw.csv")
    parser.add_argument("--output", default="data/captures/calibration_001_sweeps.csv")
    parser.add_argument("--coefficients", default="config/history_calibration.txt")
    parser.add_argument("--window-ms", type=float, default=50.0)
    parser.add_argument("--min-samples", type=int, default=1)
    args = parser.parse_args()

    rows = read_raw_lfsr_csv(args.input)
    coeff_path = Path(args.coefficients)
    coeffs = load_lfsr_coefficients(coeff_path) if coeff_path.exists() else {}
    converted = convert_rows(rows, coeffs, args.window_ms / 1000.0, args.min_samples)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(converted)

    with_angles = sum(1 for row in converted if row["azimuth_deg"] != "")
    print("=" * 70)
    print("Convert raw LH2 LFSR to paired sweeps")
    print(f"Input rows: {len(rows)}")
    print(f"Output paired observations: {len(converted)}")
    print(f"With provisional az/el: {with_angles}")
    print(f"Output: {output}")
    if not coeffs:
        print("Warning: no coefficients loaded, angle columns are empty.")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
