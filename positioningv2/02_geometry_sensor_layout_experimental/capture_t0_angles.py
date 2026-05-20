#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from statistics import median

import serial


TICKS_PER_REV = 833333
STABLE_BASESTATIONS = [4, 10]


def lfsr_to_deg(lfsr_location):
    return (((lfsr_location % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0


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


def main():
    parser = argparse.ArgumentParser(description="Capture t=0 Lighthouse angles.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--output", default="config/t0_angles.json")
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("Capture t=0 angles")
    print("=" * 60)
    print("Place the drone at the start position.")
    print("This position becomes world origin: x=0, y=0, z=0.")
    print("Do not move the drone during capture.")
    print(f"Port: {args.port}")
    print(f"Duration: {args.duration} s")
    print(f"Output: {output_path}")
    print("=" * 60)

    input("Press ENTER when ready...")

    buffer = {}

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        start = time.time()

        while time.time() - start < args.duration:
            raw = ser.readline().decode(errors="ignore").strip()
            data = parse_lh2_line(raw)

            if data is None:
                continue

            if data["basestation"] not in STABLE_BASESTATIONS:
                continue

            key = (
                data["sensor"],
                data["basestation"],
                data["sweep"],
            )

            buffer.setdefault(key, []).append(data["lfsr_location"])

            print(
                f"sensor={data['sensor']} | "
                f"bs={data['basestation']} | "
                f"sweep={data['sweep']} | "
                f"lfsr={data['lfsr_location']}"
            )

    measurements = []

    for (sensor, basestation, sweep), values in sorted(buffer.items()):
        med_lfsr = float(median(values))
        angle_deg = float(lfsr_to_deg(med_lfsr))

        measurements.append({
            "sensor": sensor,
            "basestation": basestation,
            "sweep": sweep,
            "median_lfsr_location": med_lfsr,
            "angle_deg": angle_deg,
            "angle_rad": angle_deg * 3.141592653589793 / 180.0,
            "sample_count": len(values)
        })

    result = {
        "description": "t=0 Lighthouse angle capture. Drone pose at this moment is world origin.",
        "created_unix_time_s": time.time(),
        "ticks_per_rev": TICKS_PER_REV,
        "stable_basestations": STABLE_BASESTATIONS,
        "duration_s": args.duration,
        "measurements": measurements
    }

    with open(output_path, "w") as f:
        json.dump(result, f, indent=2)

    sensors_seen = sorted(set(m["sensor"] for m in measurements))
    basestations_seen = sorted(set(m["basestation"] for m in measurements))

    print()
    print("=" * 60)
    print(f"Saved: {output_path}")
    print(f"Measurements: {len(measurements)}")
    print(f"Sensors seen: {sensors_seen}")
    print(f"Basestations seen: {basestations_seen}")
    print("=" * 60)


if __name__ == "__main__":
    main()
