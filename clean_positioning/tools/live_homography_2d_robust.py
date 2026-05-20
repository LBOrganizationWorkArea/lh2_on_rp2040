import argparse
import json
import math
import re
import time
from pathlib import Path
from statistics import median

import numpy as np
import serial


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


def parse_lh2_line(line):
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


def lfsr_to_alpha_rad(lfsr_location):
    deg = (((lfsr_location % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def alphas_to_theta_phi(alpha1, alpha2):
    theta = (alpha1 + alpha2) / 2.0

    numerator = math.sin(((alpha2 - alpha1) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha1 + alpha2) / 2.0)

    phi = math.atan2(numerator, denominator)
    return theta, phi


def theta_phi_to_image_point(theta, phi):
    ix = -math.tan(theta)
    iy = -math.tan(phi) / math.cos(theta)
    return ix, iy


def apply_homography(H, u, v):
    p = np.array([u, v, 1.0], dtype=float)
    q = H @ p

    if abs(q[2]) < 1e-12:
        raise RuntimeError("Invalid homography result")

    q = q / q[2]
    return float(q[0]), float(q[1])


def capture_sensor_image_points(port, basestation, duration, baud=115200):
    samples = {}

    with serial.Serial(port, baud, timeout=0.5) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)

        t0 = time.time()

        while time.time() - t0 < duration:
            raw = ser.readline().decode(errors="ignore").strip()
            data = parse_lh2_line(raw)

            if data is None:
                continue

            if data["basestation"] != basestation:
                continue

            if data["sensor"] not in (0, 1, 2, 3):
                continue

            if data["sweep"] not in (0, 1):
                continue

            key = (data["sensor"], data["sweep"])
            samples.setdefault(key, []).append(data["lfsr_location"])

    output = {}

    for sensor in (0, 1, 2, 3):
        k0 = (sensor, 0)
        k1 = (sensor, 1)

        if k0 not in samples or k1 not in samples:
            continue

        if len(samples[k0]) < 3 or len(samples[k1]) < 3:
            continue

        lfsr0 = int(median(samples[k0]))
        lfsr1 = int(median(samples[k1]))

        alpha0 = lfsr_to_alpha_rad(lfsr0)
        alpha1 = lfsr_to_alpha_rad(lfsr1)

        theta, phi = alphas_to_theta_phi(alpha0, alpha1)
        ix, iy = theta_phi_to_image_point(theta, phi)

        output[sensor] = {
            "image_x": ix,
            "image_y": iy,
            "samples0": len(samples[k0]),
            "samples1": len(samples[k1]),
        }

    return output


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def robust_fuse(predictions, reject_radius_m=0.20):
    """
    predictions: list of (sensor, x, y)
    Returns robust median position and kept predictions.
    """

    if not predictions:
        raise RuntimeError("No predictions to fuse")

    xs = [p[1] for p in predictions]
    ys = [p[2] for p in predictions]

    mx = float(median(xs))
    my = float(median(ys))

    kept = []
    for sensor, x, y in predictions:
        d = math.sqrt((x - mx) ** 2 + (y - my) ** 2)
        if d <= reject_radius_m:
            kept.append((sensor, x, y, d))

    if not kept:
        kept = [(sensor, x, y, math.sqrt((x - mx) ** 2 + (y - my) ** 2)) for sensor, x, y in predictions]

    fx = float(median([p[1] for p in kept]))
    fy = float(median([p[2] for p in kept]))

    return fx, fy, kept


def main():
    parser = argparse.ArgumentParser(description="Robust live 2D position using one homography per sensor.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--duration", type=float, default=0.8)
    parser.add_argument("--alpha", type=float, default=0.6)
    parser.add_argument("--reject-radius", type=float, default=0.20)
    parser.add_argument("--deadband", type=float, default=0.01, help="Meters. Small movements around zero are set to zero.")
    args = parser.parse_args()

    calib = load_json(args.calibration)
    basestation = int(calib["basestation"])

    sensor_models = {}
    for sensor_str, model in calib["sensor_models"].items():
        sensor_models[int(sensor_str)] = np.array(model["H_image_to_world"], dtype=float)

    print("Robust live 2D position")
    print("=" * 60)
    print(f"Calibration: {args.calibration}")
    print(f"Basestation: {basestation}")
    print(f"Port: {args.port}")
    print(f"Capture window: {args.duration} s")
    print(f"Filter alpha: {args.alpha}")
    print(f"Reject radius: {args.reject_radius} m")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    print()

    xf = None
    yf = None

    while True:
        measurements = capture_sensor_image_points(
            port=args.port,
            basestation=basestation,
            duration=args.duration,
        )

        predictions = []

        for sensor, meas in measurements.items():
            if sensor not in sensor_models:
                continue

            H = sensor_models[sensor]
            x, y = apply_homography(H, meas["image_x"], meas["image_y"])
            predictions.append((sensor, x, y))

        if not predictions:
            print("No valid prediction")
            continue

        x, y, kept = robust_fuse(predictions, reject_radius_m=args.reject_radius)

        if abs(x) < args.deadband:
            x = 0.0
        if abs(y) < args.deadband:
            y = 0.0

        if xf is None:
            xf, yf = x, y
        else:
            a = args.alpha
            xf = a * x + (1.0 - a) * xf
            yf = a * y + (1.0 - a) * yf

        detail = " | ".join(
            f"s{sensor}=({sx:+.2f},{sy:+.2f})"
            for sensor, sx, sy, _ in kept
        )

        print(
            f"x={xf:+.3f} m | y={yf:+.3f} m | "
            f"raw=({x:+.3f},{y:+.3f}) | "
            f"used={len(kept)}/{len(predictions)} | "
            f"{detail}"
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
