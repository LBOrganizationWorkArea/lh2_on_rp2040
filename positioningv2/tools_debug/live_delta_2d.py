#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from statistics import median

import serial


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


def load_t0_reference(path):
    with open(path, "r") as f:
        data = json.load(f)

    ref = {}

    for item in data["reference"]:
        key = (
            item["sensor"],
            item["basestation"],
            item["sweep"],
        )
        ref[key] = item["median_lfsr_location"]

    return ref


def main():
    parser = argparse.ArgumentParser(description="Live delta compared to t=0 reference.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--reference", default="config/t0_reference.json")
    parser.add_argument("--window", type=float, default=0.5, help="Median window duration in seconds")
    args = parser.parse_args()

    ref_path = Path(args.reference)

    if not ref_path.exists():
        raise FileNotFoundError(f"Missing reference file: {ref_path}")

    t0_ref = load_t0_reference(ref_path)

    print("Live delta compared to t=0")
    print("=" * 60)
    print(f"Reference: {ref_path}")
    print(f"Channels in reference: {len(t0_ref)}")
    print(f"Port: {args.port}")
    print("Move the drone slowly. Press Ctrl+C to stop.")
    print("=" * 60)

    buffer = {}
    last_print = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            data = parse_lh2_line(raw)

            if data is None:
                continue

            key = (
                data["sensor"],
                data["basestation"],
                data["sweep"],
            )

            if key not in t0_ref:
                continue

            buffer.setdefault(key, []).append(data["lfsr_location"])

            now = time.time()

            if now - last_print < args.window:
                continue

            print()
            print(f"--- live delta {now:.2f} ---")

            for key in sorted(t0_ref.keys()):
                values = buffer.get(key, [])

                if not values:
                    continue

                current = median(values)
                reference = t0_ref[key]
                delta = current - reference

                sensor, basestation, sweep = key

                print(
                    f"sensor={sensor} | "
                    f"bs={basestation} | "
                    f"sweep={sweep} | "
                    f"delta_lfsr={delta:+.1f}"
                )

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
