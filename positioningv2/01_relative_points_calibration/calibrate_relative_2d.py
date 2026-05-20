#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from statistics import median

import serial


CALIBRATION_POINTS = [
    {"name": "P0_center", "x_m": 0.00, "y_m": 0.00},
    {"name": "P1_right_30cm", "x_m": 0.30, "y_m": 0.00},
    {"name": "P2_left_30cm", "x_m": -0.30, "y_m": 0.00},
    {"name": "P3_front_30cm", "x_m": 0.00, "y_m": 0.30},
    {"name": "P4_back_30cm", "x_m": 0.00, "y_m": -0.30},
]


def parse_lh2_line(line):
    line = line.strip()

    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    if len(parts) != 7:
        return None

    try:
        return {
            "time_us": int(parts[1]),
            "sensor": int(parts[2]),
            "sweep": int(parts[3]),
            "basestation": int(parts[4]),
            "polynomial": int(parts[5]),
            "lfsr_location": int(parts[6]),
        }
    except ValueError:
        return None


def capture_point(ser, duration_s):
    measurements = {}

    start = time.time()

    while time.time() - start < duration_s:
        raw = ser.readline().decode(errors="ignore").strip()
        data = parse_lh2_line(raw)

        if data is None:
            continue

        key = (
            data["sensor"],
            data["basestation"],
            data["sweep"],
        )

        measurements.setdefault(key, []).append(data["lfsr_location"])

    result = []

    for (sensor, basestation, sweep), values in sorted(measurements.items()):
        result.append({
            "sensor": sensor,
            "basestation": basestation,
            "sweep": sweep,
            "median_lfsr_location": float(median(values)),
            "sample_count": len(values),
        })

    return result


def main():
    parser = argparse.ArgumentParser(description="Calibrate relative 2D model with 30 cm grid.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--output", default="config/calibration_relative_2d.json")
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("Relative 2D calibration")
    print("=" * 60)
    print("Grid spacing: 30 cm")
    print("Keep the drone orientation fixed for all points.")
    print("x = forward from initial drone direction")
    print("y = left from initial drone direction")
    print("=" * 60)

    calibration = {
        "description": "Relative 2D calibration using a 30 cm grid.",
        "created_unix_time_s": time.time(),
        "duration_s_per_point": args.duration,
        "frame": {
            "origin": "drone center at point A",
            "x": "forward from drone orientation at point A",
            "y": "left from drone orientation at point A",
            "unit": "meter"
        },
        "points": []
    }

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        for point in CALIBRATION_POINTS:
            print()
            print("=" * 60)
            print(f"Place drone at point: {point['name']}")
            print(f"x = {point['x_m']} m, y = {point['y_m']} m")
            print("Keep drone still.")
            input("Press ENTER when ready...")

            print(f"Capturing {args.duration} seconds...")
            measurements = capture_point(ser, args.duration)

            print(f"Captured channels: {len(measurements)}")

            calibration["points"].append({
                "name": point["name"],
                "x_m": point["x_m"],
                "y_m": point["y_m"],
                "measurements": measurements,
            })

    with open(output_path, "w") as f:
        json.dump(calibration, f, indent=2)

    print()
    print("=" * 60)
    print(f"Saved calibration to: {output_path}")
    print("Done.")


if __name__ == "__main__":
    main()
