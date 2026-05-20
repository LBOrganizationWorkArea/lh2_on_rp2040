#!/usr/bin/env python3

import argparse
import time
from collections import Counter

import serial


def parse_lh2r(line):
    parts = line.strip().split(",")
    if len(parts) != 12 or parts[0] != "LH2R":
        return None
    try:
        return {
            "sensor": int(parts[3]),
            "sweep": int(parts[4]),
            "basestation": int(parts[5]),
            "polynomial": int(parts[6]),
            "lfsr_location": int(parts[9]),
        }
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(description="Summarize LH2R frames by sensor/base station/polynomial.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--window", type=float, default=1.0)
    args = parser.parse_args()

    print("=" * 90)
    print("LH2 frame diagnostics")
    print(f"Port: {args.port}")
    print("Requires firmware with LH2_OUTPUT_EXTENDED_FRAMES=1.")
    print("Press Ctrl+C to stop.")
    print("=" * 90)

    counts = Counter()
    lh2p_count = 0
    hb_count = 0
    start = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            if raw.startswith("LH2P;"):
                lh2p_count += 1
                continue
            if raw.startswith("HB;"):
                hb_count += 1
                continue

            item = parse_lh2r(raw)
            if item is not None:
                counts[(item["sensor"], item["basestation"], item["polynomial"])] += 1

            now = time.time()
            if now - start < args.window:
                continue

            print()
            print(f"--- {now:.2f} | LH2P={lh2p_count} | HB={hb_count} ---")
            for sensor in range(4):
                cells = []
                for basestation in (4, 10):
                    poly0 = basestation * 2
                    poly1 = basestation * 2 + 1
                    c0 = counts[(sensor, basestation, poly0)]
                    c1 = counts[(sensor, basestation, poly1)]
                    cells.append(f"BS{basestation}:p{poly0}={c0:3d} p{poly1}={c1:3d}")
                print(f"sensor {sensor}: " + " | ".join(cells))

            counts.clear()
            lh2p_count = 0
            hb_count = 0
            start = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
