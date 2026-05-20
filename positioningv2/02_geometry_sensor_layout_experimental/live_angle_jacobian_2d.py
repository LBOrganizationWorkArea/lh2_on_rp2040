#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path
from statistics import median

import numpy as np
import serial


TICKS_PER_REV = 833333
STABLE_BASESTATIONS = [4, 10]


def lfsr_to_rad(lfsr_location):
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


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


def angle_from_measurement(m):
    if "angle_rad" in m:
        return float(m["angle_rad"])

    if "angle_deg" in m:
        return math.radians(float(m["angle_deg"]))

    if "median_lfsr_location" in m:
        return lfsr_to_rad(float(m["median_lfsr_location"]))

    if "lfsr_location" in m:
        return lfsr_to_rad(float(m["lfsr_location"]))

    raise ValueError(f"Cannot find angle/lfsr in measurement: {m}")


def build_feature_vector_from_measurements(measurements, feature_keys):
    grouped = {}

    for m in measurements:
        sensor = int(m["sensor"])
        bs = int(m["basestation"])
        sweep = int(m["sweep"])

        if bs not in STABLE_BASESTATIONS:
            continue

        key = (sensor, bs, sweep)
        grouped.setdefault(key, []).append(angle_from_measurement(m))

    features = []

    for key in feature_keys:
        values = grouped.get(key)

        if not values:
            return None

        features.append(float(median(values)))

    return np.array(features, dtype=float)


def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)

    points = data["points"]

    feature_keys = []

    for sensor in range(4):
        for bs in STABLE_BASESTATIONS:
            for sweep in range(2):
                feature_keys.append((sensor, bs, sweep))

    calib = {}

    for p in points:
        name = p["name"]
        x = float(p["x_m"])
        y = float(p["y_m"])

        vec = build_feature_vector_from_measurements(p["measurements"], feature_keys)

        if vec is None:
            print(f"Warning: missing features for {name}, ignored.")
            continue

        calib[name] = {
            "name": name,
            "x": x,
            "y": y,
            "features": vec,
        }

    required = [
        "P0_center",
        "P1_right_30cm",
        "P2_left_30cm",
        "P3_up_30cm",
        "P4_down_30cm",
    ]

    for name in required:
        if name not in calib:
            raise RuntimeError(f"Missing calibration point: {name}")

    return calib, feature_keys


def angle_diff(live, ref):
    """
    Difference between angles, wrapped to [-pi, pi].
    """
    d = live - ref
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def build_jacobian_model(calib):
    p0 = calib["P0_center"]
    pr = calib["P1_right_30cm"]
    pl = calib["P2_left_30cm"]
    pu = calib["P3_up_30cm"]
    pd = calib["P4_down_30cm"]

    f0 = p0["features"]

    # Coordinates from the calibration file.
    xr = pr["x"]
    xl = pl["x"]
    yu = pu["y"]
    yd = pd["y"]

    if abs(xr - xl) < 1e-9:
        raise RuntimeError("Right/left calibration x coordinates are identical.")

    if abs(yu - yd) < 1e-9:
        raise RuntimeError("Up/down calibration y coordinates are identical.")

    # Central finite differences in angle-space.
    df_dx = np.array([
        angle_diff(pr["features"][i], pl["features"][i]) / (xr - xl)
        for i in range(len(f0))
    ])

    df_dy = np.array([
        angle_diff(pu["features"][i], pd["features"][i]) / (yu - yd)
        for i in range(len(f0))
    ])

    # Matrix A maps [x, y] -> delta_angles
    A = np.vstack([df_dx, df_dy]).T

    # Remove almost useless features to reduce noise.
    feature_strength = np.linalg.norm(A, axis=1)
    keep = feature_strength > np.percentile(feature_strength, 30)

    A_kept = A[keep]

    if A_kept.shape[0] < 4:
        raise RuntimeError("Not enough useful angle features.")

    return {
        "f0": f0,
        "A": A,
        "keep": keep,
        "A_kept": A_kept,
    }


def buffer_to_features(buffer, feature_keys):
    features = []

    for key in feature_keys:
        values = buffer.get(key)

        if not values:
            return None

        angle_values = [lfsr_to_rad(v) for v in values]
        features.append(float(median(angle_values)))

    return np.array(features, dtype=float)


def estimate_position(model, live_features):
    f0 = model["f0"]
    A = model["A"]
    keep = model["keep"]
    A_kept = model["A_kept"]

    delta = np.array([
        angle_diff(live_features[i], f0[i])
        for i in range(len(f0))
    ])

    delta_kept = delta[keep]

    # Solve A * [x,y] = delta using least squares.
    xy, residuals, rank, s = np.linalg.lstsq(A_kept, delta_kept, rcond=None)

    x = float(xy[0])
    y = float(xy[1])

    pred = A_kept @ xy
    err = delta_kept - pred
    rmse_rad = float(np.sqrt(np.mean(err ** 2)))
    rmse_deg = math.degrees(rmse_rad)

    return x, y, rmse_deg, int(A_kept.shape[0])


def main():
    parser = argparse.ArgumentParser(description="Live 2D position using angle Jacobian model.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--calibration", default="config/init_poses_2d.json")
    parser.add_argument("--window", type=float, default=0.5)
    args = parser.parse_args()

    calibration_path = Path(args.calibration)

    calib, feature_keys = load_calibration(calibration_path)
    model = build_jacobian_model(calib)

    print("Live 2D position from Lighthouse angles")
    print("=" * 60)
    print(f"Calibration: {calibration_path}")
    print(f"Useful angle features: {int(model['A_kept'].shape[0])}/{len(feature_keys)}")
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

            live_features = buffer_to_features(buffer, feature_keys)

            if live_features is None:
                print("Waiting for all angle channels...")
            else:
                x, y, rmse_deg, n_features = estimate_position(model, live_features)

                print(
                    f"x = {x:+.3f} m | "
                    f"y = {y:+.3f} m | "
                    f"angle_rmse = {rmse_deg:.3f} deg | "
                    f"features = {n_features}"
                )

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
