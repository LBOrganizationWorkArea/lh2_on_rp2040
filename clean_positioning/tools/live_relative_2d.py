# -*- coding: utf-8 -*-

import argparse
import json
import math
import re
import time
from pathlib import Path
from statistics import median

import cv2
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


def angles_to_image_point(theta, phi):
    u = math.tan(theta)
    v = math.tan(phi) / math.cos(theta)
    return np.array([u, v], dtype=np.float64)


def apply_homography(H, uv):
    pts = np.array([[uv]], dtype=np.float64)
    out = cv2.perspectiveTransform(pts, H).reshape(2)
    return out


def median_lfsr(values):
    if not values:
        return None
    return int(median(values))


def estimate_from_window(samples, maps):
    """
    samples[(bs, sensor, sweep)] = [lfsr...]
    returns list of estimated sensor positions in origin/world 2D frame
    """
    sensor_world_estimations = []

    for bs_key, bs_map in maps["basestations"].items():
        bs = int(bs_key)
        H = np.array(bs_map["H_image_to_world"], dtype=np.float64)
        sweep_swap = bool(bs_map.get("sweep_swap", False))

        for sensor in bs_map["used_sensors"]:
            k0 = (bs, sensor, 0)
            k1 = (bs, sensor, 1)

            if k0 not in samples or k1 not in samples:
                continue

            l0 = median_lfsr(samples[k0])
            l1 = median_lfsr(samples[k1])

            if l0 is None or l1 is None:
                continue

            a0 = lfsr_to_alpha_rad(l0)
            a1 = lfsr_to_alpha_rad(l1)

            if sweep_swap:
                a0, a1 = a1, a0

            theta, phi = alphas_to_theta_phi(a0, a1)
            uv = angles_to_image_point(theta, phi)

            xy = apply_homography(H, uv)

            sensor_world_estimations.append({
                "bs": bs,
                "sensor": int(sensor),
                "x": float(xy[0]),
                "y": float(xy[1]),
            })

    return sensor_world_estimations


def robust_center_from_sensor_positions(sensor_positions, layout, reject_radius):
    """
    Each measured sensor world position gives an estimate of drone center:
    center = measured_sensor_world - sensor_offset_body
    This first version assumes yaw ~= 0.
    """
    centers = []

    for item in sensor_positions:
        sid = str(item["sensor"])

        if sid not in layout["sensors"]:
            continue

        s = layout["sensors"][sid]
        cx = item["x"] - float(s["x"])
        cy = item["y"] - float(s["y"])

        centers.append({
            "bs": item["bs"],
            "sensor": item["sensor"],
            "x": cx,
            "y": cy,
        })

    if not centers:
        return None, []

    xs = np.array([c["x"] for c in centers], dtype=np.float64)
    ys = np.array([c["y"] for c in centers], dtype=np.float64)

    med = np.array([np.median(xs), np.median(ys)], dtype=np.float64)

    kept = []
    for c in centers:
        d = math.hypot(c["x"] - med[0], c["y"] - med[1])
        if d <= reject_radius:
            kept.append(c)

    if not kept:
        kept = centers

    x = float(np.median([c["x"] for c in kept]))
    y = float(np.median([c["y"] for c in kept]))

    return {"x": x, "y": y}, kept


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


def main():
    parser = argparse.ArgumentParser(description="Live 2D relative position from Lighthouse angles.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--maps", required=True)
    parser.add_argument("--window", type=float, default=0.5)
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--reject-radius", type=float, default=0.20)
    parser.add_argument("--deadband", type=float, default=0.01)
    args = parser.parse_args()

    layout = load_json(args.layout)
    maps = load_json(args.maps)

    print("Live relative 2D position")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Layout: {args.layout}")
    print(f"Maps: {args.maps}")
    print(f"Window: {args.window} s")
    print(f"Filter alpha: {args.alpha}")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    print()

    filt = None

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)

        try:
            while True:
                samples = collect_window(ser, args.window)
                sensor_positions = estimate_from_window(samples, maps)
                center, kept = robust_center_from_sensor_positions(
                    sensor_positions,
                    layout,
                    args.reject_radius
                )

                if center is None:
                    print("No valid position")
                    continue

                raw_x = center["x"]
                raw_y = center["y"]

                if abs(raw_x) < args.deadband:
                    raw_x = 0.0
                if abs(raw_y) < args.deadband:
                    raw_y = 0.0

                if filt is None:
                    filt = np.array([raw_x, raw_y], dtype=np.float64)
                else:
                    raw = np.array([raw_x, raw_y], dtype=np.float64)
                    filt = args.alpha * raw + (1.0 - args.alpha) * filt

                used_txt = " ".join(
                    f"bs{c['bs']}:s{c['sensor']}"
                    for c in kept
                )

                print(
                    f"x={filt[0]:+.3f} m | "
                    f"y={filt[1]:+.3f} m | "
                    f"raw=({raw_x:+.3f},{raw_y:+.3f}) | "
                    f"used={len(kept)} | {used_txt}"
                )

        except KeyboardInterrupt:
            print()
            print("Stopped.")


if __name__ == "__main__":
    main()
