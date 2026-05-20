#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from statistics import median

import serial


def parse_lh2_line(line):
    """
    Expected format:
    LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location

    Example:
    LH2,12345678,0,0,4,8,51232
    """
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


def main():
    parser = argparse.ArgumentParser(description="Capture t=0 Lighthouse reference for positioningv2.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0, help="Capture duration in seconds")
    parser.add_argument("--output", default="config/t0_reference.json")
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("Capture t=0 reference")
    print("=" * 60)
    print("Place the drone on the ground.")
    print("Do not move the drone during capture.")
    print(f"Port: {args.port}")
    print(f"Duration: {args.duration} s")
    print(f"Output: {output_path}")
    print("=" * 60)

    measurements = {}

    start = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while time.time() - start < args.duration:
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

            print(
                f"sensor={data['sensor']} | "
                f"bs={data['basestation']} | "
                f"sweep={data['sweep']} | "
                f"lfsr={data['lfsr_location']}"
            )

    reference = {
        "description": "t=0 reference. Drone center is defined as [0, 0].",
        "created_unix_time_s": time.time(),
        "duration_s": args.duration,
        "format": {
            "sensor": "TS4231 sensor id",
            "basestation": "Lighthouse base station id",
            "sweep": "sweep id",
            "median_lfsr_location": "median lfsr_location during t=0 capture",
            "sample_count": "number of samples used"
        },
        "reference": []
    }

    for (sensor, basestation, sweep), values in sorted(measurements.items()):
        reference["reference"].append({
            "sensor": sensor,
            "basestation": basestation,
            "sweep": sweep,
            "median_lfsr_location": float(median(values)),
            "sample_count": len(values)
        })

    with open(output_path, "w") as f:
        json.dump(reference, f, indent=2)

    print()
    print("=" * 60)
    print(f"Saved t=0 reference to: {output_path}")
    print(f"Number of channels captured: {len(reference['reference'])}")

    sensors = sorted({item["sensor"] for item in reference["reference"]})
    basestations = sorted({item["basestation"] for item in reference["reference"]})

    print(f"Sensors seen: {sensors}")
    print(f"Basestations seen: {basestations}")


if __name__ == "__main__":
    main()
