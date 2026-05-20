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


def build_feature_vector(buffer, model):
    values = {}

    for key, samples in buffer.items():
        if samples:
            values[key] = float(median(samples))

    features = []

    for f in model["feature_names"]:
        key = (
            int(f["sensor"]),
            int(f["basestation"]),
            int(f["sweep"]),
        )

        if key not in values:
            return None

        features.append(values[key])

    return np.array(features, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Live 2D position from Lighthouse relative model.")
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

    print("Live 2D position")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Port: {args.port}")
    print("Press Ctrl+C to stop.")
    print("=" * 60)

    buffer = {}
    last_print = time.time()

    valid_keys = {
        (
            int(f["sensor"]),
            int(f["basestation"]),
            int(f["sweep"]),
        )
        for f in model["feature_names"]
    }

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

            if key not in valid_keys:
                continue

            buffer.setdefault(key, []).append(data["lfsr_location"])

            now = time.time()

            if now - last_print < args.window:
                continue

            features = build_feature_vector(buffer, model)

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
