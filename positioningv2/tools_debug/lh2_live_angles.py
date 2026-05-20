#!/usr/bin/env python3

import argparse
import time
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
    parser = argparse.ArgumentParser(description="Live Lighthouse angles from LFSR locations.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--window", type=float, default=0.5)
    args = parser.parse_args()

    print("Live LH2 angles")
    print("=" * 60)
    print(f"Port: {args.port}")
    print("Angles are approximate from lfsr_location.")
    print("Press Ctrl+C to stop.")
    print("=" * 60)

    buffer = {}
    last_print = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
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

            now = time.time()

            if now - last_print < args.window:
                continue

            print()
            print(f"--- angles {now:.2f} ---")

            for sensor in range(4):
                for basestation in STABLE_BASESTATIONS:
                    for sweep in range(2):
                        key = (sensor, basestation, sweep)
                        values = buffer.get(key, [])

                        if not values:
                            continue

                        lfsr = median(values)
                        angle = lfsr_to_deg(lfsr)

                        print(
                            f"sensor={sensor} | "
                            f"bs={basestation} | "
                            f"sweep={sweep} | "
                            f"lfsr={lfsr:.0f} | "
                            f"angle={angle:+.3f} deg"
                        )

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
