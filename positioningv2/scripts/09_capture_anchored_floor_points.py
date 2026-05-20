#!/usr/bin/env python3
"""Capture raw LH2 LFSR observations at known floor anchor points."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from dynamic_lh2_common import load_json
from lh2_lfsr_common import parse_lh2_serial_line


FIELDS = [
    "point_id",
    "point_x",
    "point_y",
    "point_z",
    "point_yaw_deg",
    "pc_time",
    "firmware_time_us",
    "sensor_id",
    "lighthouse_id",
    "polynomial",
    "sweep",
    "lfsr",
    "raw_line",
]


def parse_ids(text):
    return {int(part.strip()) for part in text.split(",") if part.strip()}


def capture_point(ser, point, seconds, expected_lighthouses):
    rows = []
    ignored = 0
    ser.reset_input_buffer()
    deadline = time.time() + seconds
    while time.time() < deadline:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue
        observations = parse_lh2_serial_line(raw, pc_time=time.time())
        if not observations:
            ignored += 1
            continue
        for obs in observations:
            if expected_lighthouses and obs["lighthouse_id"] not in expected_lighthouses:
                ignored += 1
                continue
            item = {
                "point_id": point["id"],
                "point_x": point["x"],
                "point_y": point["y"],
                "point_z": point.get("z", 0.0),
                "point_yaw_deg": point.get("yaw_deg", 0.0),
            }
            item.update(obs)
            rows.append(item)
    return rows, ignored


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--points", default="config/anchored_floor9_points.json")
    parser.add_argument("--output", default="data/captures/anchored_floor9_lfsr_raw.csv")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--lighthouses", default="4,10")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is required: py -m pip install pyserial", file=sys.stderr)
        return 2

    data = load_json(args.points)
    points = data["points"]
    expected_lighthouses = parse_ids(args.lighthouses) if args.lighthouses else set()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Capture anchored floor points")
    print(f"Points: {args.points}")
    print(f"Output: {output}")
    print("Keep the drone orientation fixed relative to the world.")
    print("=" * 70)

    all_rows = []
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        for point in points:
            print()
            print("=" * 70)
            print(
                f"Place drone at {point['id']} | "
                f"x={point['x']:+.3f} y={point['y']:+.3f} z={point.get('z', 0.0):+.3f} "
                f"yaw={point.get('yaw_deg', 0.0):+.1f} deg"
            )
            input("Press ENTER when still...")
            rows, ignored = capture_point(ser, point, args.seconds, expected_lighthouses)
            all_rows.extend(rows)
            sensors = sorted({row["sensor_id"] for row in rows})
            lighthouses = sorted({row["lighthouse_id"] for row in rows})
            polynomials = sorted({row["polynomial"] for row in rows})
            print(
                f"Captured raw observations: {len(rows)} | ignored={ignored} | "
                f"sensors={sensors} | BS={lighthouses} | polynomials={polynomials}"
            )
            answer = input("ENTER=keep | r=retry this point | q=stop and save: ").strip().lower()
            while answer == "r":
                all_rows = [row for row in all_rows if row["point_id"] != point["id"]]
                rows, ignored = capture_point(ser, point, args.seconds, expected_lighthouses)
                all_rows.extend(rows)
                sensors = sorted({row["sensor_id"] for row in rows})
                lighthouses = sorted({row["lighthouse_id"] for row in rows})
                polynomials = sorted({row["polynomial"] for row in rows})
                print(
                    f"Captured raw observations: {len(rows)} | ignored={ignored} | "
                    f"sensors={sensors} | BS={lighthouses} | polynomials={polynomials}"
                )
                answer = input("ENTER=keep | r=retry this point | q=stop and save: ").strip().lower()
            if answer == "q":
                break

    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        for row in all_rows:
            writer.writerow({field: row.get(field, "") for field in FIELDS})

    print("=" * 70)
    print(f"Saved: {output}")
    print(f"raw observations: {len(all_rows)}")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
