import csv
import json
import math
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


TICKS_PER_REV = 833333.0
LH2_LINE_RE = re.compile(
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
    match = LH2_LINE_RE.match(line.strip())
    if match is None:
        return None

    return {
        "time_us": int(match.group("time_us")),
        "sensor": int(match.group("sensor")),
        "sweep": int(match.group("sweep")),
        "basestation": int(match.group("basestation")),
        "polynomial": int(match.group("polynomial")),
        "lfsr_location": int(match.group("lfsr_location")),
    }


def load_sensor_layout(path):
    data = load_json(path)
    return {
        int(item["sensor"]): np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ], dtype=float)
        for item in data["sensors"]
    }


def load_lh2_csv(path):
    rows = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "pc_time_s": float(row["pc_time_s"]),
                "time_us": int(row["time_us"]),
                "sensor": int(row["sensor"]),
                "sweep": int(row["sweep"]),
                "basestation": int(row["basestation"]),
                "polynomial": int(row["polynomial"]),
                "lfsr_location": float(row["lfsr_location"]),
            })
    return rows


def load_mocap_csv(path):
    rows = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            item = {
                "pc_time_s": float(row["pc_time_s"]),
                "position": np.array([
                    float(row["x_m"]),
                    float(row["y_m"]),
                    float(row["z_m"]),
                ], dtype=float),
            }

            if {"qx", "qy", "qz", "qw"}.issubset(row):
                item["rotation"] = Rotation.from_quat([
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                ])
            else:
                item["rotation"] = Rotation.from_euler("xyz", [
                    math.radians(float(row.get("roll_deg", 0.0))),
                    math.radians(float(row.get("pitch_deg", 0.0))),
                    math.radians(float(row.get("yaw_deg", 0.0))),
                ])

            rows.append(item)

    rows.sort(key=lambda item: item["pc_time_s"])
    return rows


class MocapInterpolator:
    def __init__(self, mocap_rows):
        if len(mocap_rows) < 2:
            raise ValueError("Need at least two mocap rows for interpolation.")

        self.times = np.array([row["pc_time_s"] for row in mocap_rows], dtype=float)
        self.positions = np.array([row["position"] for row in mocap_rows], dtype=float)
        self.slerp = Slerp(self.times, Rotation.concatenate([row["rotation"] for row in mocap_rows]))

    def contains(self, t):
        return self.times[0] <= t <= self.times[-1]

    def pose_at(self, t):
        t = float(t)
        pos = np.array([
            np.interp(t, self.times, self.positions[:, 0]),
            np.interp(t, self.times, self.positions[:, 1]),
            np.interp(t, self.times, self.positions[:, 2]),
        ], dtype=float)
        rot = self.slerp([t])[0]
        return pos, rot


def sensor_world_position(mocap_position, mocap_rotation, sensor_local):
    return mocap_position + mocap_rotation.as_matrix() @ sensor_local


def angle_wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def default_lfsr_to_alpha(lfsr_location):
    deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def lh2_sweep_angle_from_point(point_lh, tilt_rad):
    x, y, z = point_lh
    r = math.sqrt(x * x + y * y)
    if r < 1e-9:
        r = 1e-9

    value = (z * math.tan(tilt_rad)) / r
    value = max(-0.999999, min(0.999999, value))
    return math.atan2(y, x) + math.asin(value)
