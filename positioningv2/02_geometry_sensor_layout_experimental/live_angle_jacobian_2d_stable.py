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


def angle_diff(a, b):
    """
    Wrapped angular difference: a - b in [-pi, pi].
    """
    d = a - b
    return (d + math.pi) % (2.0 * math.pi) - math.pi


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


def make_feature_keys():
    keys = []

    for sensor in range(4):
        for basestation in STABLE_BASESTATIONS:
            for sweep in range(2):
                keys.append((sensor, basestation, sweep))

    return keys


def build_feature_vector_from_measurements(measurements, feature_keys):
    grouped = {}

    for m in measurements:
        sensor = int(m["sensor"])
        basestation = int(m["basestation"])
        sweep = int(m["sweep"])

        if basestation not in STABLE_BASESTATIONS:
            continue

        key = (sensor, basestation, sweep)
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

    feature_keys = make_feature_keys()
    calib = {}

    for p in data["points"]:
        name = p["name"]
        x = float(p["x_m"])
        y = float(p["y_m"])

        vec = build_feature_vector_from_measurements(
            p["measurements"],
            feature_keys
        )

        if vec is None:
            print(f"Warning: missing angle channels for {name}, ignored.")
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


def build_jacobian(calib):
    p0 = calib["P0_center"]
    pr = calib["P1_right_30cm"]
    pl = calib["P2_left_30cm"]
    pu = calib["P3_up_30cm"]
    pd = calib["P4_down_30cm"]

    f0 = p0["features"]

    xr = pr["x"]
    xl = pl["x"]
    yu = pu["y"]
    yd = pd["y"]

    if abs(xr - xl) < 1e-9:
        raise RuntimeError("Right and left points have same x.")

    if abs(yu - yd) < 1e-9:
        raise RuntimeError("Up and down points have same y.")

    df_dx = np.array([
        angle_diff(pr["features"][i], pl["features"][i]) / (xr - xl)
        for i in range(len(f0))
    ])

    df_dy = np.array([
        angle_diff(pu["features"][i], pd["features"][i]) / (yu - yd)
        for i in range(len(f0))
    ])

    # A maps [x, y] -> delta_angles
    A = np.vstack([df_dx, df_dy]).T

    strengths = np.linalg.norm(A, axis=1)

    # Keep strongest channels only.
    # This avoids noisy channels dominating the inverse.
    threshold = np.percentile(strengths, 35)
    keep = strengths > threshold

    if np.sum(keep) < 6:
        # fallback: keep all if too few
        keep = np.ones_like(strengths, dtype=bool)

    A_kept = A[keep]

    return {
        "calibration_center": f0,
        "A": A,
        "A_kept": A_kept,
        "keep": keep,
        "strengths": strengths,
    }


def buffer_to_features(buffer, feature_keys):
    features = []

    for key in feature_keys:
        values = buffer.get(key)

        if not values:
            return None

        angles = [lfsr_to_rad(v) for v in values]
        features.append(float(median(angles)))

    return np.array(features, dtype=float)


def collect_feature_window(ser, feature_keys, duration_s, label):
    print()
    print("=" * 60)
    print(label)
    print(f"Capturing for {duration_s:.1f} seconds...")
    print("Do not move the drone.")
    print("=" * 60)

    buffer = {}
    start = time.time()

    while time.time() - start < duration_s:
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

    features = buffer_to_features(buffer, feature_keys)

    if features is None:
        missing = []
        for key in feature_keys:
            if key not in buffer:
                missing.append(key)

        raise RuntimeError(f"Missing channels during zero capture: {missing}")

    print("Zero capture OK.")
    return features


def estimate_xy_regularized(A_kept, delta_kept, damping):
    """
    Solve:
      A x = delta

    with damped least squares:
      x = inv(A.T A + lambda I) A.T delta
    """
    AtA = A_kept.T @ A_kept
    Atb = A_kept.T @ delta_kept

    I = np.eye(2)
    xy = np.linalg.solve(AtA + damping * I, Atb)

    pred = A_kept @ xy
    err = delta_kept - pred
    rmse_rad = float(np.sqrt(np.mean(err ** 2)))
    rmse_deg = math.degrees(rmse_rad)

    return float(xy[0]), float(xy[1]), rmse_deg


def estimate_position(model, live_features, live_zero_features, damping):
    keep = model["keep"]
    A_kept = model["A_kept"]

    delta = np.array([
        angle_diff(live_features[i], live_zero_features[i])
        for i in range(len(live_features))
    ])

    delta_kept = delta[keep]

    x, y, rmse_deg = estimate_xy_regularized(
        A_kept,
        delta_kept,
        damping=damping
    )

    return x, y, rmse_deg, int(A_kept.shape[0])


def main():
    parser = argparse.ArgumentParser(
        description="Stable live 2D position using Lighthouse angles and Jacobian."
    )
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--calibration", default="config/init_poses_2d.json")
    parser.add_argument("--zero-duration", type=float, default=2.0)
    parser.add_argument("--window", type=float, default=0.6)
    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--alpha", type=float, default=0.25, help="Low-pass filter strength, 0..1")
    args = parser.parse_args()

    calibration_path = Path(args.calibration)

    calib, feature_keys = load_calibration(calibration_path)
    model = build_jacobian(calib)

    print("Stable live 2D position from Lighthouse angles")
    print("=" * 60)
    print(f"Calibration: {calibration_path}")
    print(f"Total angle channels: {len(feature_keys)}")
    print(f"Useful angle channels: {int(np.sum(model['keep']))}/{len(feature_keys)}")
    print(f"Damping: {args.damping}")
    print(f"Filter alpha: {args.alpha}")
    print("=" * 60)
    print("IMPORTANT:")
    print("Place the drone exactly at the center/origin now.")
    print("Keep the drone still during zero capture.")
    print("=" * 60)

    input("Press ENTER when the drone is at the center...")

    filtered_x = None
    filtered_y = None

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        live_zero_features = collect_feature_window(
            ser,
            feature_keys,
            args.zero_duration,
            "Live zero capture at center"
        )

        print()
        print("=" * 60)
        print("Live tracking started.")
        print("Move the drone slowly.")
        print("Press Ctrl+C to stop.")
        print("=" * 60)

        buffer = {}
        last_print = time.time()

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
                buffer.clear()
                last_print = now
                continue

            x, y, rmse_deg, n_features = estimate_position(
                model,
                live_features,
                live_zero_features,
                damping=args.damping
            )

            if filtered_x is None:
                filtered_x = x
                filtered_y = y
            else:
                filtered_x = (1.0 - args.alpha) * filtered_x + args.alpha * x
                filtered_y = (1.0 - args.alpha) * filtered_y + args.alpha * y

            print(
                f"x = {filtered_x:+.3f} m | "
                f"y = {filtered_y:+.3f} m | "
                f"raw=({x:+.3f},{y:+.3f}) | "
                f"angle_rmse={rmse_deg:.3f} deg | "
                f"features={n_features}"
            )

            buffer.clear()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
