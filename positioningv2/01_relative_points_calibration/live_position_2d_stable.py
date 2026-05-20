#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from statistics import median

import numpy as np
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


def load_model(path):
    with open(path, "r") as f:
        return json.load(f)


def build_ref_by_pair(model):
    ref_by_pair = {}

    for feature, ref_value in zip(model["feature_names"], model["reference_features"]):
        sensor = int(feature["sensor"])
        basestation = int(feature["basestation"])
        sweep = int(feature["sweep"])

        ref_by_pair.setdefault((sensor, basestation), {})[sweep] = float(ref_value)

    return ref_by_pair


def stable_features_from_buffer(buffer, model):
    """
    Build the 16-feature vector, but correct live sweep/ref sweep swapping.

    For each sensor+basestation, we receive two live sweep values.
    We try both assignments:
      live0 -> ref0 and live1 -> ref1
      live0 -> ref1 and live1 -> ref0
    and choose the assignment with the smaller absolute delta.
    """
    live_by_pair = {}

    for (sensor, basestation, live_sweep), samples in buffer.items():
        if samples:
            live_by_pair.setdefault((sensor, basestation), {})[live_sweep] = float(median(samples))

    ref_by_pair = build_ref_by_pair(model)

    stable_values = {}

    for pair, ref_sweeps in ref_by_pair.items():
        if pair not in live_by_pair:
            return None

        live_sweeps = live_by_pair[pair]

        if 0 not in live_sweeps or 1 not in live_sweeps:
            return None

        if 0 not in ref_sweeps or 1 not in ref_sweeps:
            return None

        live0 = live_sweeps[0]
        live1 = live_sweeps[1]
        ref0 = ref_sweeps[0]
        ref1 = ref_sweeps[1]

        cost_normal = abs(live0 - ref0) + abs(live1 - ref1)
        cost_swapped = abs(live0 - ref1) + abs(live1 - ref0)

        sensor, basestation = pair

        if cost_normal <= cost_swapped:
            stable_values[(sensor, basestation, 0)] = live0
            stable_values[(sensor, basestation, 1)] = live1
        else:
            stable_values[(sensor, basestation, 0)] = live1
            stable_values[(sensor, basestation, 1)] = live0

    features = []

    for f in model["feature_names"]:
        key = (
            int(f["sensor"]),
            int(f["basestation"]),
            int(f["sweep"]),
        )

        if key not in stable_values:
            return None

        features.append(stable_values[key])

    return np.array(features, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Stable live 2D position from Lighthouse relative model.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--model", default="config/model_relative_2d.json")
    parser.add_argument("--window", type=float, default=0.5)
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Missing model file: {model_path}")

    model = load_model(model_path)
    reference_features = np.array(model["reference_features"], dtype=float)
    weights = np.array(model["weights"], dtype=float)

    valid_pairs = {
        (int(f["sensor"]), int(f["basestation"]))
        for f in model["feature_names"]
    }

    print("Stable live 2D position")
    print("=" * 60)
    print(f"Model: {model_path}")
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

            pair = (data["sensor"], data["basestation"])
            if pair not in valid_pairs:
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

            features = stable_features_from_buffer(buffer, model)

            if features is None:
                print("Waiting for all channels...")
                buffer.clear()
                last_print = now
                continue

            delta = features - reference_features
            design = np.concatenate([[1.0], delta])
            xy = design @ weights

            x_m = float(xy[0])
            y_m = float(xy[1])

            print(f"x = {x_m:+.3f} m | y = {y_m:+.3f} m")

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
