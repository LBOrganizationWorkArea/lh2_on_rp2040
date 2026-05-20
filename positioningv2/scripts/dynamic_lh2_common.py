import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


REQUIRED_OBSERVATION_COLUMNS = [
    "timestamp",
    "sensor_id",
    "lighthouse_id",
    "angle_1_deg",
    "angle_2_deg",
]


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_sensors_layout(path):
    data = load_json(path)
    sensors = {}
    for item in data["sensors"]:
        sensor_id = int(item.get("id", item.get("sensor")))
        if "position" in item:
            position = item["position"]
        else:
            position = [item["x_m"], item["y_m"], item.get("z_m", 0.0)]
        sensors[sensor_id] = np.asarray(position, dtype=float)
    return data, sensors


def load_observations_csv(path):
    df = pd.read_csv(path)
    missing = [col for col in REQUIRED_OBSERVATION_COLUMNS if col not in df.columns]
    if missing:
        raise ValueError(f"Missing observation CSV columns: {missing}")
    df = df[REQUIRED_OBSERVATION_COLUMNS].copy()
    df["sensor_id"] = df["sensor_id"].astype(int)
    df["lighthouse_id"] = df["lighthouse_id"].astype(int)
    for col in ("timestamp", "angle_1_deg", "angle_2_deg"):
        df[col] = df[col].astype(float)
    return df.sort_values("timestamp").reset_index(drop=True)


def group_observations_by_frame(df, min_observations=4, expected_lighthouses=None):
    expected = None if expected_lighthouses is None else {int(x) for x in expected_lighthouses}
    frames = []
    rejected = 0
    for timestamp, group in df.groupby("timestamp", sort=True):
        rows = []
        for row in group.itertuples(index=False):
            if expected is not None and int(row.lighthouse_id) not in expected:
                continue
            rows.append({
                "timestamp": float(row.timestamp),
                "sensor_id": int(row.sensor_id),
                "lighthouse_id": int(row.lighthouse_id),
                "angles_rad": np.radians([float(row.angle_1_deg), float(row.angle_2_deg)]),
            })
        if len(rows) < min_observations:
            rejected += 1
            continue
        frames.append({"timestamp": float(timestamp), "observations": rows})
    return frames, rejected


def angle_wrap(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def angle_residual(measured_rad, predicted_rad):
    measured = np.asarray(measured_rad, dtype=float)
    predicted = np.asarray(predicted_rad, dtype=float)
    return angle_wrap(measured - predicted)


def rotation_z(yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def sensor_world_position(drone_pose, sensor_position_body, drone_z):
    x, y, yaw = drone_pose
    base = np.array([x, y, drone_z], dtype=float)
    return base + rotation_z(yaw) @ np.asarray(sensor_position_body, dtype=float)


def predict_angles(sensor_position_world, lighthouse_translation, lighthouse_rotvec):
    """Angular-camera approximation.

    The Lighthouse is treated as a calibrated angular camera. This is not the
    exact LH2 sweep-plane model, but it gives a clean modular baseline:
    p_lh = R_lh.T @ (p_world - t_lh), then azimuth/elevation are read in the
    Lighthouse frame.
    """
    rotation = Rotation.from_rotvec(lighthouse_rotvec).as_matrix()
    p_lh = rotation.T @ (np.asarray(sensor_position_world) - np.asarray(lighthouse_translation))
    horizontal = math.hypot(float(p_lh[0]), float(p_lh[1]))
    azimuth = math.atan2(float(p_lh[1]), float(p_lh[0]))
    elevation = math.atan2(float(p_lh[2]), max(horizontal, 1e-12))
    return np.array([azimuth, elevation], dtype=float)


def pose_look_at(origin, target=(0.0, 0.0, 0.0)):
    """Return a rotation vector whose local +X axis points roughly at target."""
    origin = np.asarray(origin, dtype=float)
    target = np.asarray(target, dtype=float)
    forward = target - origin
    norm = np.linalg.norm(forward)
    if norm < 1e-9:
        return np.zeros(3)
    forward /= norm

    up_hint = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, up_hint))) > 0.95:
        up_hint = np.array([0.0, 1.0, 0.0])
    y_axis = np.cross(up_hint, forward)
    y_axis /= max(np.linalg.norm(y_axis), 1e-12)
    z_axis = np.cross(forward, y_axis)
    z_axis /= max(np.linalg.norm(z_axis), 1e-12)
    rotation = np.column_stack([forward, y_axis, z_axis])
    return Rotation.from_matrix(rotation).as_rotvec()


def lighthouse_pose_from_geometry(geometry):
    poses = {}
    for item in geometry["lighthouses"]:
        poses[int(item["id"])] = {
            "translation": np.asarray(item["translation"], dtype=float),
            "rotation_vector": np.asarray(item["rotation_vector"], dtype=float),
        }
    return poses


def residual_quality(residuals_rad):
    residuals = np.asarray(residuals_rad, dtype=float)
    abs_deg = np.degrees(np.abs(residuals))
    rmse_rad = float(np.sqrt(np.mean(residuals ** 2))) if residuals.size else float("nan")
    return {
        "rmse_rad": rmse_rad,
        "rmse_deg": float(np.degrees(rmse_rad)) if residuals.size else float("nan"),
        "median_error_deg": float(np.median(abs_deg)) if residuals.size else float("nan"),
        "max_error_deg": float(np.max(abs_deg)) if residuals.size else float("nan"),
    }
