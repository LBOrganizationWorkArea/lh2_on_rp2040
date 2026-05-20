# -*- coding: utf-8 -*-

import argparse
import json
import math
import re
import time
from statistics import median

import numpy as np
import serial
from scipy.optimize import least_squares


TICKS_PER_REV = 833333

LINE_RE = re.compile(
    r"^LH2,"
    r"(?P<time_us>\d+),"
    r"(?P<sensor>\d+),"
    r"(?P<sweep>\d+),"
    r"(?P<basestation>\d+),"
    r"(?P<polynomial>-?\d+),"
    r"(?P<lfsr_location>-?\d+)"
    r"$"
)


def parse_line(line):
    m = LINE_RE.match(line.strip())
    if not m:
        return None

    return {
        "time_us": int(m.group("time_us")),
        "sensor": int(m.group("sensor")),
        "sweep": int(m.group("sweep")),
        "basestation": int(m.group("basestation")),
        "polynomial": int(m.group("polynomial")),
        "lfsr_location": int(m.group("lfsr_location")),
    }


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def lfsr_to_alpha_rad(lfsr_location):
    deg = (((lfsr_location % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def alphas_to_theta_phi(alpha0, alpha1):
    theta = (alpha0 + alpha1) / 2.0
    numerator = math.sin(((alpha1 - alpha0) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha0 + alpha1) / 2.0)
    phi = math.atan2(numerator, denominator)
    return theta, phi


def angle_wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def collect_window(ser, duration_s):
    samples = {}
    t0 = time.time()

    while time.time() - t0 < duration_s:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        data = parse_line(line)

        if data is None:
            continue

        bs = data["basestation"]
        sensor = data["sensor"]
        sweep = data["sweep"]

        if sweep not in (0, 1):
            continue

        key = (bs, sensor, sweep)
        samples.setdefault(key, []).append(data["lfsr_location"])

    return samples


def build_measurements(samples, geometry):
    measurements = []

    for bs_key, bs_info in geometry["basestations"].items():
        bs = int(bs_key)
        sweep_swap = bool(bs_info.get("sweep_swap", False))

        for sensor in bs_info["used_sensors"]:
            k0 = (bs, sensor, 0)
            k1 = (bs, sensor, 1)

            if k0 not in samples or k1 not in samples:
                continue

            if len(samples[k0]) < 2 or len(samples[k1]) < 2:
                continue

            l0 = int(median(samples[k0]))
            l1 = int(median(samples[k1]))

            a0 = lfsr_to_alpha_rad(l0)
            a1 = lfsr_to_alpha_rad(l1)

            if sweep_swap:
                a0, a1 = a1, a0

            theta, phi = alphas_to_theta_phi(a0, a1)

            measurements.append({
                "bs": bs,
                "sensor": int(sensor),
                "theta": theta,
                "phi": phi
            })

    return measurements


def sensor_world_position(x, y, yaw, sensor_body_xy):
    c = math.cos(yaw)
    s = math.sin(yaw)

    sx = sensor_body_xy[0]
    sy = sensor_body_xy[1]

    wx = x + c * sx - s * sy
    wy = y + s * sx + c * sy
    wz = 0.0

    return np.array([wx, wy, wz], dtype=np.float64)


def predict_theta_phi(point_world, bs_info):
    R = np.array(bs_info["R_lighthouse_from_origin"], dtype=np.float64)
    t = np.array(bs_info["tvec"], dtype=np.float64).reshape(3)

    # origin/world point -> lighthouse/camera frame
    p_lh = R @ point_world + t

    X = p_lh[0]
    Y = p_lh[1]
    Z = p_lh[2]

    if Z <= 1e-6:
        Z = 1e-6

    theta = math.atan2(X, Z)
    phi = math.atan2(Y, math.sqrt(X * X + Z * Z))

    return theta, phi


def residuals_pose(params, measurements, layout, geometry):
    x, y, yaw = params

    res = []

    for m in measurements:
        sid = str(m["sensor"])
        bs_key = str(m["bs"])

        if sid not in layout["sensors"]:
            continue

        if bs_key not in geometry["basestations"]:
            continue

        sinfo = layout["sensors"][sid]
        sensor_body = np.array([float(sinfo["x"]), float(sinfo["y"])], dtype=np.float64)

        p_world = sensor_world_position(x, y, yaw, sensor_body)
        pred_theta, pred_phi = predict_theta_phi(p_world, geometry["basestations"][bs_key])

        res.append(angle_wrap(pred_theta - m["theta"]))
        res.append(angle_wrap(pred_phi - m["phi"]))

    return np.array(res, dtype=np.float64)


def solve_pose(measurements, layout, geometry, previous_pose):
    if len(measurements) < 4:
        return None

    x0 = previous_pose if previous_pose is not None else np.array([0.0, 0.0, 0.0], dtype=np.float64)

    sol = least_squares(
        residuals_pose,
        x0,
        args=(measurements, layout, geometry),
        method="trf",
        loss="soft_l1",
        f_scale=0.05,
        max_nfev=100
    )

    pose = sol.x
    rmse_rad = float(np.sqrt(np.mean(sol.fun * sol.fun))) if len(sol.fun) else 999.0
    rmse_deg = math.degrees(rmse_rad)

    return {
        "pose": pose,
        "rmse_deg": rmse_deg,
        "cost": float(sol.cost),
        "n_measurements": len(measurements)
    }


def main():
    parser = argparse.ArgumentParser(description="Live 2D triangulation using two Lighthouse basestations.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--window", type=float, default=0.8)
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--deadband", type=float, default=0.01)
    args = parser.parse_args()

    layout = load_json(args.layout)
    geometry = load_json(args.geometry)

    print("Live triangulation 2D")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Layout: {args.layout}")
    print(f"Geometry: {args.geometry}")
    print(f"Window: {args.window} s")
    print(f"Filter alpha: {args.alpha}")
    print("=" * 60)
    print("Estimate: x, y, yaw")
    print("Press Ctrl+C to stop.")
    print()

    pose_filter = None
    previous_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)

        try:
            while True:
                samples = collect_window(ser, args.window)
                measurements = build_measurements(samples, geometry)

                result = solve_pose(measurements, layout, geometry, previous_pose)

                if result is None:
                    print(f"No valid pose | measurements={len(measurements)}")
                    continue

                raw_pose = result["pose"]
                previous_pose = raw_pose.copy()

                if abs(raw_pose[0]) < args.deadband:
                    raw_pose[0] = 0.0

                if abs(raw_pose[1]) < args.deadband:
                    raw_pose[1] = 0.0

                if pose_filter is None:
                    pose_filter = raw_pose.copy()
                else:
                    pose_filter = args.alpha * raw_pose + (1.0 - args.alpha) * pose_filter

                x, y, yaw = pose_filter

                used_bs = sorted(set(m["bs"] for m in measurements))
                used_sensors = sorted(set(m["sensor"] for m in measurements))

                print(
                    f"x={x:+.3f} m | "
                    f"y={y:+.3f} m | "
                    f"yaw={math.degrees(yaw):+.2f} deg | "
                    f"rmse={result['rmse_deg']:.3f} deg | "
                    f"meas={result['n_measurements']} | "
                    f"bs={used_bs} | sensors={used_sensors}"
                )

        except KeyboardInterrupt:
            print()
            print("Stopped.")


if __name__ == "__main__":
    main()
