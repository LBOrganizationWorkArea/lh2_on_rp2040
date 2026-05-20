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

CALIBRATION_POINTS = [
    ("P0_center", 0.0, 0.0),
    ("P1_right_30cm", -0.3, 0.0),
    ("P2_left_30cm", 0.3, 0.0),
    ("P3_up_30cm", 0.0, 0.3),
    ("P4_down_30cm", 0.0, -0.3),
]


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


def compute_homography(image_points, world_points):
    A = []

    for (u, v), (x, y) in zip(image_points, world_points):
        A.append([-u, -v, -1, 0, 0, 0, x * u, x * v, x])
        A.append([0, 0, 0, -u, -v, -1, y * u, y * v, y])

    A = np.array(A, dtype=float)

    _, _, vh = np.linalg.svd(A)
    h = vh[-1, :]
    H = h.reshape(3, 3)

    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]

    return H


def apply_homography(H, u, v):
    p = np.array([u, v, 1.0], dtype=float)
    q = H @ p

    if abs(q[2]) < 1e-12:
        raise RuntimeError("Invalid homography result")

    q = q / q[2]
    return float(q[0]), float(q[1])


def capture_sensor_image_points(port, basestation, duration, baud=115200):
    """
    Returns:
    {
      sensor_id: {
        "image_x": ...,
        "image_y": ...,
        "samples0": ...,
        "samples1": ...
      }
    }
    """

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


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(description="Robust 2D calibration: one homography per sensor.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--basestation", type=int, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    print("Robust 2D calibration")
    print("=" * 60)
    print(f"Basestation: {args.basestation}")
    print(f"Port: {args.port}")
    print(f"Duration per point: {args.duration} s")
    print("=" * 60)
    print("IMPORTANT:")
    print("- Keep Lighthouse fixed.")
    print("- Keep drone orientation fixed.")
    print("- Place drone, remove hand, wait 2 seconds, then press ENTER.")
    print("=" * 60)

    point_records = []

    sensor_image_points = {0: [], 1: [], 2: [], 3: []}
    sensor_world_points = {0: [], 1: [], 2: [], 3: []}

    for name, wx, wy in CALIBRATION_POINTS:
        print()
        print("=" * 60)
        print(f"Place drone at {name}")
        print(f"Target world position: x={wx:+.3f} m, y={wy:+.3f} m")
        input("Press ENTER when ready...")

        print("Capturing...")
        measurements = capture_sensor_image_points(
            port=args.port,
            basestation=args.basestation,
            duration=args.duration,
        )

        print(f"Visible sensors: {sorted(measurements.keys())}")

        record = {
            "name": name,
            "world_x_m": wx,
            "world_y_m": wy,
            "measurements": measurements,
        }
        point_records.append(record)

        for sensor, meas in measurements.items():
            sensor_image_points[sensor].append((meas["image_x"], meas["image_y"]))
            sensor_world_points[sensor].append((wx, wy))

            print(
                f"sensor={sensor} | "
                f"u={meas['image_x']:+.8f}, v={meas['image_y']:+.8f} | "
                f"samples=({meas['samples0']},{meas['samples1']})"
            )

    sensor_models = {}

    print()
    print("=" * 60)
    print("Fitting one homography per sensor")
    print("=" * 60)

    for sensor in (0, 1, 2, 3):
        if len(sensor_image_points[sensor]) < 4:
            print(f"sensor={sensor}: not enough points, skipped")
            continue

        H = compute_homography(sensor_image_points[sensor], sensor_world_points[sensor])

        errors = []
        for (u, v), (wx, wy) in zip(sensor_image_points[sensor], sensor_world_points[sensor]):
            px, py = apply_homography(H, u, v)
            err = math.sqrt((px - wx) ** 2 + (py - wy) ** 2)
            errors.append(err)

        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))

        sensor_models[str(sensor)] = {
            "H_image_to_world": H.tolist(),
            "fit_rmse_m": rmse,
            "num_points": len(sensor_image_points[sensor]),
        }

        print(f"sensor={sensor} | points={len(errors)} | fit_rmse={rmse*100:.1f} cm")

    output = {
        "method": "robust_single_lighthouse_2d_per_sensor_homography",
        "basestation": args.basestation,
        "points": point_records,
        "sensor_models": sensor_models,
    }

    save_json(args.output, output)

    print()
    print("=" * 60)
    print(f"Saved: {args.output}")
    print("=" * 60)


if __name__ == "__main__":
    main()
