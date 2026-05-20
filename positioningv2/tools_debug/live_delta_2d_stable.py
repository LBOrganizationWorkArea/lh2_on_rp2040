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

    # ref[(sensor, basestation)] = [sweep0_ref, sweep1_ref]
    ref = {}

    for item in data["reference"]:
        sensor = item["sensor"]
        basestation = item["basestation"]
        sweep = item["sweep"]
        value = item["median_lfsr_location"]

        key = (sensor, basestation)
        ref.setdefault(key, {})[sweep] = value

    return ref


def best_delta(current, ref_sweeps):
    """
    Choose the closest reference sweep automatically.
    This avoids false huge jumps when sweep 0 / sweep 1 are swapped.
    """
    candidates = []

    for sweep, ref_value in ref_sweeps.items():
        delta = current - ref_value
        candidates.append((abs(delta), delta, sweep))

    if not candidates:
        return None

    candidates.sort(key=lambda x: x[0])
    _, delta, ref_sweep = candidates[0]
    return delta, ref_sweep


def main():
    parser = argparse.ArgumentParser(description="Stable live delta compared to t=0 reference.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--reference", default="config/t0_reference.json")
    parser.add_argument("--window", type=float, default=0.5)
    args = parser.parse_args()

    ref_path = Path(args.reference)

    if not ref_path.exists():
        raise FileNotFoundError(f"Missing reference file: {ref_path}")

    t0_ref = load_t0_reference(ref_path)

    print("Stable live delta compared to t=0")
    print("=" * 60)
    print(f"Reference: {ref_path}")
    print(f"Sensor/Lighthouse pairs in reference: {len(t0_ref)}")
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

            pair_key = (data["sensor"], data["basestation"])

            if pair_key not in t0_ref:
                continue

            # Group by sensor + basestation + live sweep
            live_key = (
                data["sensor"],
                data["basestation"],
                data["sweep"],
            )

            buffer.setdefault(live_key, []).append(data["lfsr_location"])

            now = time.time()

            if now - last_print < args.window:
                continue

            print()
            print(f"--- stable live delta {now:.2f} ---")

            summary = {}

            for live_key, values in sorted(buffer.items()):
                sensor, basestation, live_sweep = live_key
                current = median(values)

                result = best_delta(current, t0_ref[(sensor, basestation)])

                if result is None:
                    continue

                delta, ref_sweep = result

                summary.setdefault(sensor, []).append(delta)

                print(
                    f"sensor={sensor} | "
                    f"bs={basestation} | "
                    f"live_sweep={live_sweep} | "
                    f"matched_ref_sweep={ref_sweep} | "
                    f"delta_lfsr={delta:+.1f}"
                )

            print("--- average per sensor ---")
            for sensor in sorted(summary.keys()):
                avg = sum(summary[sensor]) / len(summary[sensor])
                print(f"sensor={sensor} | avg_delta_lfsr={avg:+.1f}")

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
