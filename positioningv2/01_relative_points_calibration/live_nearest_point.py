#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path
from statistics import median

import serial


STABLE_BASESTATIONS = [4, 10]


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


def measurements_to_pairs(measurements):
    pairs = {}

    for m in measurements:
        sensor = int(m["sensor"])
        basestation = int(m["basestation"])
        sweep = int(m["sweep"])

        if basestation not in STABLE_BASESTATIONS:
            continue

        value = float(m["median_lfsr_location"])
        pairs.setdefault((sensor, basestation), {})[sweep] = value

    return pairs


def buffer_to_pairs(buffer):
    pairs = {}

    for (sensor, basestation, sweep), samples in buffer.items():
        if basestation not in STABLE_BASESTATIONS:
            continue

        if not samples:
            continue

        value = float(median(samples))
        pairs.setdefault((sensor, basestation), {})[sweep] = value

    return pairs


def pair_distance(live_sweeps, ref_sweeps):
    if 0 not in live_sweeps or 1 not in live_sweeps:
        return None

    if 0 not in ref_sweeps or 1 not in ref_sweeps:
        return None

    l0 = live_sweeps[0]
    l1 = live_sweeps[1]
    r0 = ref_sweeps[0]
    r1 = ref_sweeps[1]

    normal = (l0 - r0) ** 2 + (l1 - r1) ** 2
    swapped = (l0 - r1) ** 2 + (l1 - r0) ** 2

    return min(normal, swapped)


def distance_to_point(live_pairs, ref_pairs):
    total = 0.0
    count = 0

    for sensor in range(4):
        for basestation in STABLE_BASESTATIONS:
            key = (sensor, basestation)

            if key not in live_pairs:
                continue

            if key not in ref_pairs:
                continue

            d = pair_distance(live_pairs[key], ref_pairs[key])

            if d is None:
                continue

            total += d
            count += 1

    if count == 0:
        return None

    return math.sqrt(total / count)


def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)

    points = []

    for p in data["points"]:
        points.append({
            "name": p["name"],
            "x_m": float(p["x_m"]),
            "y_m": float(p["y_m"]),
            "pairs": measurements_to_pairs(p["measurements"]),
        })

    return points


def main():
    parser = argparse.ArgumentParser(description="Show nearest calibrated point.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--calibration", default="config/calibration_relative_2d.json")
    parser.add_argument("--window", type=float, default=1.0)
    args = parser.parse_args()

    calibration_path = Path(args.calibration)

    if not calibration_path.exists():
        raise FileNotFoundError(f"Missing calibration file: {calibration_path}")

    points = load_calibration(calibration_path)

    print("Nearest calibrated point")
    print("=" * 60)
    print(f"Calibration: {calibration_path}")
    print("Put the drone exactly on one calibration point and keep it still.")
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

            live_pairs = buffer_to_pairs(buffer)

            distances = []
            for p in points:
                d = distance_to_point(live_pairs, p["pairs"])
                if d is not None:
                    distances.append((d, p))

            if not distances:
                print("Waiting for enough channels...")
            else:
                distances.sort(key=lambda item: item[0])

                best_d, best_p = distances[0]

                second_d = distances[1][0] if len(distances) > 1 else None
                confidence = ""
                if second_d is not None:
                    confidence = f" | gap={second_d - best_d:.1f}"

                print(
                    f"NEAREST = {best_p['name']} "
                    f"| x={best_p['x_m']:+.2f} m "
                    f"| y={best_p['y_m']:+.2f} m "
                    f"| d={best_d:.1f}"
                    f"{confidence}"
                )

                print(
                    "ranking: "
                    + " | ".join(
                        f"{p['name']}:{d:.1f}"
                        for d, p in distances[:5]
                    )
                )

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
