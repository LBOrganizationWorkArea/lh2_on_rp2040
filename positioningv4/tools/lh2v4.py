import json
import math
import re
import time
from pathlib import Path
from statistics import median

import numpy as np


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


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def parse_lh2_line(line):
    match = LINE_RE.match(line.strip())
    if not match:
        return None

    return {
        "time_us": int(match.group("time_us")),
        "sensor": int(match.group("sensor")),
        "sweep": int(match.group("sweep")),
        "basestation": int(match.group("basestation")),
        "polynomial": int(match.group("polynomial")),
        "lfsr_location": int(match.group("lfsr_location")),
    }


def lfsr_to_alpha_rad(lfsr_location):
    deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def alphas_to_theta_phi(alpha0, alpha1):
    theta = (alpha0 + alpha1) / 2.0
    numerator = math.sin(((alpha1 - alpha0) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha0 + alpha1) / 2.0)
    phi = math.atan2(numerator, denominator)
    return theta, phi


def theta_phi_to_image(theta, phi):
    u = math.tan(theta)
    v = math.tan(phi) / max(1e-9, math.cos(theta))
    return float(u), float(v)


def lfsr_pair_to_image(lfsr0, lfsr1, sweep_swap=False):
    alpha0 = lfsr_to_alpha_rad(lfsr0)
    alpha1 = lfsr_to_alpha_rad(lfsr1)
    if sweep_swap:
        alpha0, alpha1 = alpha1, alpha0
    theta, phi = alphas_to_theta_phi(alpha0, alpha1)
    return theta_phi_to_image(theta, phi)


def layout_by_sensor(layout):
    return {
        int(item["sensor"]): np.array([float(item["x_m"]), float(item["y_m"])], dtype=float)
        for item in layout["sensors"]
    }


def sensor_world_xy(drone_pose, sensor_offset):
    yaw = math.radians(float(drone_pose.get("yaw_deg", 0.0)))
    c = math.cos(yaw)
    s = math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    center = np.array([float(drone_pose["x_m"]), float(drone_pose["y_m"])], dtype=float)
    return center + rot @ sensor_offset


def collect_window(ser, duration_s, basestations=None):
    samples = {}
    start = time.time()
    wanted_bs = None if basestations is None else {int(x) for x in basestations}

    while time.time() - start < duration_s:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        data = parse_lh2_line(raw)
        if data is None:
            continue

        if wanted_bs is not None and data["basestation"] not in wanted_bs:
            continue

        if data["sweep"] not in (0, 1):
            continue

        key = (int(data["basestation"]), int(data["sensor"]), int(data["sweep"]))
        samples.setdefault(key, []).append(int(data["lfsr_location"]))

    return samples


def median_observations(samples, min_samples=2):
    observations = []
    keys = sorted({(bs, sensor) for (bs, sensor, _sweep) in samples})

    for bs, sensor in keys:
        k0 = (bs, sensor, 0)
        k1 = (bs, sensor, 1)
        if len(samples.get(k0, [])) < min_samples or len(samples.get(k1, [])) < min_samples:
            continue

        observations.append({
            "basestation": int(bs),
            "sensor": int(sensor),
            "lfsr0": float(median(samples[k0])),
            "lfsr1": float(median(samples[k1])),
            "samples0": int(len(samples[k0])),
            "samples1": int(len(samples[k1])),
        })

    return observations


def fit_homography(image_points, world_points):
    if len(image_points) < 4:
        raise ValueError("Need at least 4 points to fit a homography.")

    rows = []
    for (u, v), (x, y) in zip(image_points, world_points):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])

    _, _, vh = np.linalg.svd(np.array(rows, dtype=float))
    H = vh[-1].reshape(3, 3)
    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]
    return H


def apply_homography(H, u, v):
    q = np.array(H, dtype=float) @ np.array([float(u), float(v), 1.0], dtype=float)
    if abs(q[2]) < 1e-12:
        raise ValueError("Invalid homography projection.")
    q = q / q[2]
    return float(q[0]), float(q[1])


def robust_median_xy(points, reject_radius_m):
    if not points:
        return None, []

    arr = np.array(points, dtype=float)
    med = np.median(arr, axis=0)
    kept = []
    for p in arr:
        if float(np.linalg.norm(p - med)) <= reject_radius_m:
            kept.append(p)

    if not kept:
        kept = list(arr)

    out = np.median(np.array(kept, dtype=float), axis=0)
    return (float(out[0]), float(out[1])), kept
