import math
import re
import time
import json
from pathlib import Path
from statistics import median

import numpy as np
import serial


TICKS_PER_REV = 833333

# Firmware format:
# LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location
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
    # alpha in radians, approx from Lighthouse counter
    deg = (((lfsr_location % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def alphas_to_theta_phi(alpha1, alpha2):
    # Paper equation (1)
    theta = (alpha1 + alpha2) / 2.0

    numerator = math.sin(((alpha2 - alpha1) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha1 + alpha2) / 2.0)

    phi = math.atan2(numerator, denominator)
    return theta, phi


def theta_phi_to_image_point(theta, phi):
    # Paper equation (3)
    ix = -math.tan(theta)
    iy = -math.tan(phi) / math.cos(theta)
    return ix, iy


def capture_image_point(port, basestation, duration=3.0, baud=115200):
    """
    Capture LH2 lines for one basestation.
    For each sensor, get sweep0 and sweep1 median.
    Convert to theta/phi then image point.
    Return the median image point over all visible sensors.
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

    sensor_points = []

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

        sensor_points.append((ix, iy))

    if not sensor_points:
        raise RuntimeError("No valid sensor image points captured. Check Lighthouse visibility.")

    ix = float(median([p[0] for p in sensor_points]))
    iy = float(median([p[1] for p in sensor_points]))

    return {
        "image_x": ix,
        "image_y": iy,
        "visible_sensors": len(sensor_points),
        "sensor_points": sensor_points,
    }


def compute_homography(image_points, world_points):
    """
    DLT homography.
    image point [u,v,1] -> world point [x,y,1]
    """

    A = []

    for (u, v), (x, y) in zip(image_points, world_points):
        A.append([-u, -v, -1, 0, 0, 0, x*u, x*v, x])
        A.append([0, 0, 0, -u, -v, -1, y*u, y*v, y])

    A = np.array(A, dtype=float)

    _, _, vh = np.linalg.svd(A)
    h = vh[-1, :]
    H = h.reshape(3, 3)

    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]

    return H


def apply_homography(H, image_x, image_y):
    p = np.array([image_x, image_y, 1.0], dtype=float)
    q = H @ p

    if abs(q[2]) < 1e-12:
        raise RuntimeError("Invalid homography result.")

    q = q / q[2]
    return float(q[0]), float(q[1])


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)
