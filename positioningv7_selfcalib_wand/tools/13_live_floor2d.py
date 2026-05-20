import argparse
import statistics
import time

import numpy as np
import serial

from wand_common import DEFAULT_BS_POLYS, load_json, parse_lh2_line


def predict_xy(calibration, raw_features):
    raw = np.asarray(raw_features, dtype=float)
    mean = np.asarray(calibration["mean"], dtype=float)
    scale = np.asarray(calibration["scale"], dtype=float)
    x = (raw - mean) / scale

    if calibration["model"] == "affine_lfsr_floor2d":
        coeff = np.asarray(calibration["coefficients"], dtype=float)
        features = np.concatenate([[1.0], x])
        return features @ coeff

    centers = np.asarray(calibration["centers"], dtype=float)
    weights = np.asarray(calibration["weights"], dtype=float)
    epsilon = float(calibration["epsilon"])
    diff = centers - x[None, :]
    dist2 = np.sum(diff * diff, axis=1)
    kernel = np.exp(-dist2 / max(epsilon * epsilon, 1e-9))
    return kernel @ weights


def main():
    parser = argparse.ArgumentParser(description="Live direct floor-only 2D LH2 position.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--calibration", default="config/floor2d_calibration.json")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--window", type=float, default=0.35)
    parser.add_argument("--min-samples", type=int, default=3)
    args = parser.parse_args()

    calibration = load_json(args.calibration)
    sensor = int(calibration["sensor"])
    basestations = [int(x) for x in calibration["basestations"]]
    state = {bs: {0: [], 1: [], "last": {}} for bs in basestations}
    last_print = 0.0
    period = 1.0 / max(args.rate, 0.1)

    print("=" * 70)
    print("Live floor 2D position")
    print(f"Calibration: {args.calibration}")
    print(f"sensor={sensor} basestations={basestations}")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is None:
                continue

            if data["sensor"] != sensor:
                continue
            bs = data["basestation"]
            poly = data["polynomial"]
            if bs not in state:
                continue
            if bs in DEFAULT_BS_POLYS and poly not in DEFAULT_BS_POLYS[bs]:
                continue

            sweep = poly & 1
            lfsr = int(data["lfsr"])
            if state[bs]["last"].get(sweep) != lfsr:
                state[bs][sweep].append((time.time(), lfsr))
                state[bs]["last"][sweep] = lfsr

            now = time.time()
            cutoff = now - args.window
            for bs_id in basestations:
                for sweep_id in (0, 1):
                    state[bs_id][sweep_id] = [
                        item for item in state[bs_id][sweep_id]
                        if item[0] >= cutoff
                    ]

            if now - last_print < period:
                continue
            if not all(
                len(state[bs][0]) >= args.min_samples and len(state[bs][1]) >= args.min_samples
                for bs in basestations
            ):
                continue

            features = []
            for bs_id in basestations:
                features.extend([
                    int(round(statistics.median(item[1] for item in state[bs_id][0]))),
                    int(round(statistics.median(item[1] for item in state[bs_id][1]))),
                ])
            xy = predict_xy(calibration, features)
            print(f"XY,{xy[0]:+.3f},{xy[1]:+.3f}")
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
