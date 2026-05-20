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
    """
    Return:
    pairs[(sensor, basestation)] = {0: lfsr_sweep0, 1: lfsr_sweep1}
    """
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
    """
    Compare live sweeps to reference sweeps.
    Try normal and swapped assignment, keep the best one.
    """
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


def distance_to_calibration_point(live_pairs, calib_pairs):
    total = 0.0
    count = 0

    for sensor in range(4):
        for basestation in STABLE_BASESTATIONS:
            key = (sensor, basestation)

            if key not in live_pairs or key not in calib_pairs:
                continue

            d = pair_distance(live_pairs[key], calib_pairs[key])

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


def estimate_position(live_pairs, calibration_points):
    distances = []

    for p in calibration_points:
        d = distance_to_calibration_point(live_pairs, p["pairs"])

        if d is None:
            continue

        distances.append((d, p))

    if not distances:
        return None

    distances.sort(key=lambda item: item[0])

    # Use inverse-distance weighted average of the 3 closest points.
    nearest = distances[:3]

    # If extremely close to a calibration point, return it directly.
    if nearest[0][0] < 5.0:
        p = nearest[0][1]
        return p["x_m"], p["y_m"], nearest

    weights = []
    for d, p in nearest:
        weights.append(1.0 / max(d, 1e-6))

    total_w = sum(weights)

    x = sum(w * p["x_m"] for w, (d, p) in zip(weights, nearest)) / total_w
    y = sum(w * p["y_m"] for w, (d, p) in zip(weights, nearest)) / total_w

    return x, y, nearest


def main():
    parser = argparse.ArgumentParser(description="Live 2D position using KNN from calibration points.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--calibration", default="config/calibration_relative_2d.json")
    parser.add_argument("--window", type=float, default=0.5)
    args = parser.parse_args()

    calibration_path = Path(args.calibration)

    if not calibration_path.exists():
        raise FileNotFoundError(f"Missing calibration file: {calibration_path}")

    calibration_points = load_calibration(calibration_path)

    print("Live 2D position KNN")
    print("=" * 60)
    print(f"Calibration: {calibration_path}")
    print(f"Points: {[p['name'] for p in calibration_points]}")
    print(f"Port: {args.port}")
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
            result = estimate_position(live_pairs, calibration_points)

            if result is None:
                print("Waiting for enough channels...")
            else:
                x, y, nearest = result

                nearest_txt = " | ".join(
                    f"{p['name']}:d={d:.1f}"
                    for d, p in nearest
                )

                print(f"x = {x:+.3f} m | y = {y:+.3f} m | nearest: {nearest_txt}")

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
