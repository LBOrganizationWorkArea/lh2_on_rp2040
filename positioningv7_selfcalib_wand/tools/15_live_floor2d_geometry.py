import argparse
import math
import statistics
import time

import numpy as np
import serial

from wand_common import DEFAULT_BS_POLYS, load_json, parse_lh2_line


def bearing_from_feature(norm_feature, coeff):
    return coeff[0] + coeff[1] * norm_feature[0] + coeff[2] * norm_feature[1]


def intersect_rays(stations):
    rows = []
    rhs = []
    for x, y, theta in stations:
        normal = np.array([-math.sin(theta), math.cos(theta)], dtype=float)
        rows.append(normal)
        rhs.append(float(normal @ np.array([x, y], dtype=float)))
    A = np.vstack(rows)
    b = np.asarray(rhs)
    point, *_ = np.linalg.lstsq(A, b, rcond=None)
    return point


def predict_xy(calibration, features_by_bs):
    stations = []
    for bs_item in calibration["basestations"]:
        bs = int(bs_item["id"])
        feat = features_by_bs.get(bs)
        if feat is None:
            return None
        norm = (np.asarray(feat, dtype=float) - np.asarray(bs_item["feature_mean"])) / np.asarray(bs_item["feature_scale"])
        theta = bs_item["yaw_rad"] + bearing_from_feature(norm, np.asarray(bs_item["bearing_coefficients"]))
        stations.append([bs_item["x_m"], bs_item["y_m"], theta])
    return intersect_rays(stations)


def main():
    parser = argparse.ArgumentParser(description="Live effective 2D Lighthouse geometry position.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--calibration", default="config/floor2d_geometry.json")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--window", type=float, default=0.5)
    parser.add_argument("--min-samples", type=int, default=3)
    args = parser.parse_args()

    calibration = load_json(args.calibration)
    sensor = int(calibration["sensor"])
    basestations = [int(item["id"]) for item in calibration["basestations"]]
    state = {bs: {0: [], 1: [], "last": {}} for bs in basestations}
    period = 1.0 / max(args.rate, 0.1)
    last_print = 0.0

    print("=" * 70)
    print("Live floor 2D geometry position")
    print(f"Calibration: {args.calibration}")
    print(f"sensor={sensor} basestations={basestations}")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is None or data["sensor"] != sensor:
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
                    state[bs_id][sweep_id] = [item for item in state[bs_id][sweep_id] if item[0] >= cutoff]

            if now - last_print < period:
                continue
            if not all(
                len(state[bs][0]) >= args.min_samples and len(state[bs][1]) >= args.min_samples
                for bs in basestations
            ):
                continue

            features_by_bs = {}
            for bs_id in basestations:
                features_by_bs[bs_id] = [
                    int(round(statistics.median(item[1] for item in state[bs_id][0]))),
                    int(round(statistics.median(item[1] for item in state[bs_id][1]))),
                ]
            xy = predict_xy(calibration, features_by_bs)
            if xy is not None:
                print(f"XY,{xy[0]:+.3f},{xy[1]:+.3f}")
                last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
