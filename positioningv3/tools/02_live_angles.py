#!/usr/bin/env python3

import argparse
import math
import time
from statistics import median

import serial


TICKS_PER_REV = 120000
DEFAULT_BASESTATIONS = [4, 10]


def lfsr_to_deg(lfsr_location, sweep):
    angle = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 360.0) - 180.0
    if int(sweep) == 0:
        return angle + 60.0
    return angle - 60.0


def parse_lh2_line(line):
    line = line.strip()

    if line.startswith("LH2A,"):
        parts = line.split(",")
        try:
            if len(parts) == 7:
                return {
                    "time_us": None,
                    "sensor": int(parts[1]),
                    "sweep": int(parts[2]),
                    "basestation": int(parts[3]),
                    "polynomial": int(parts[4]),
                    "lfsr_location": int(parts[5]),
                    "raw_angle_deg": math.degrees(int(parts[6]) / 1000000.0),
                }
            if len(parts) == 8:
                return {
                    "time_us": int(parts[1]),
                    "sensor": int(parts[2]),
                    "sweep": int(parts[3]),
                    "basestation": int(parts[4]),
                    "polynomial": int(parts[5]),
                    "lfsr_location": int(parts[6]),
                    "raw_angle_deg": math.degrees(int(parts[7]) / 1000000.0),
                }
        except ValueError:
            return None

        return None

    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    try:
        if len(parts) == 6:
            return {
                "time_us": None,
                "sensor": int(parts[1]),
                "sweep": int(parts[2]),
                "basestation": int(parts[3]),
                "polynomial": int(parts[4]),
                "lfsr_location": int(parts[5]),
            }
        if len(parts) == 7:
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

    return None


def main():
    parser = argparse.ArgumentParser(description="Live Lighthouse angles.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--basestations", default="4,10")
    args = parser.parse_args()

    basestations = [int(x) for x in args.basestations.split(",")]

    print("=" * 70)
    print("Live LH2 angles")
    print(f"Port: {args.port}")
    print(f"Basestations: {basestations}")
    print("Need sensors 0,1,2,3 with sweep 0 and 1 for both basestations.")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    buffer = {}
    last_print = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            data = parse_lh2_line(raw)

            if data is None:
                continue

            if data["basestation"] not in basestations:
                continue

            key = (
                data["sensor"],
                data["basestation"],
                data["sweep"],
            )

            buffer.setdefault(key, []).append(data)

            now = time.time()

            if now - last_print < args.window:
                continue

            print()
            print(f"--- angles {now:.2f} ---")

            for sensor in range(4):
                for bs in basestations:
                    for sweep in range(2):
                        key = (sensor, bs, sweep)
                        values = buffer.get(key, [])

                        if not values:
                            print(f"sensor={sensor} | bs={bs} | sweep={sweep} | MISSING")
                            continue

                        lfsr_values = [item["lfsr_location"] for item in values]
                        angle_values = [
                            item["raw_angle_deg"]
                            for item in values
                            if "raw_angle_deg" in item
                        ]

                        med_lfsr = float(median(lfsr_values))
                        angle_deg = float(median(angle_values)) if angle_values else lfsr_to_deg(med_lfsr, sweep)

                        print(
                            f"sensor={sensor} | "
                            f"bs={bs} | "
                            f"sweep={sweep} | "
                            f"lfsr={med_lfsr:.0f} | "
                            f"angle={angle_deg:+.3f} deg | "
                            f"n={len(values)}"
                        )

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
