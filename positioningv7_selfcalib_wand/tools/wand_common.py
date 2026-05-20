import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


TAN_30 = math.tan(math.radians(30.0))
DEFAULT_BS_POLYS = {
    4: (8, 9),
    10: (20, 21),
}


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_angle_modes(path):
    data = load_json(path)
    modes = {}
    for bs_key, sensors in data.get("modes", {}).items():
        bs = int(bs_key)
        modes[bs] = {int(sensor): mode for sensor, mode in sensors.items()}
    return modes


def parse_lh2_line(line):
    parts = line.strip().split(",")
    if not parts or parts[0] != "LH2":
        return None

    try:
        if len(parts) == 7:
            return {
                "sensor": int(parts[2]),
                "sweep": int(parts[3]),
                "basestation": int(parts[4]),
                "polynomial": int(parts[5]),
                "lfsr": int(parts[6]),
            }

        if len(parts) == 6:
            return {
                "sensor": int(parts[1]),
                "sweep": int(parts[2]),
                "basestation": int(parts[3]),
                "polynomial": int(parts[4]),
                "lfsr": int(parts[5]),
            }
    except ValueError:
        return None

    return None


def load_latest_coefficients(path):
    coeffs = {}

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if "DATE_TIME" in line or line.startswith("-") or not line.strip():
                continue

            parts = [p.strip() for p in line.split("|")]
            if len(parts) < 7:
                continue

            try:
                coeffs[int(parts[1])] = {
                    "A0": float(parts[3]),
                    "B0": float(parts[4]),
                    "A1": float(parts[5]),
                    "B1": float(parts[6]),
                }
            except ValueError:
                continue

    return coeffs


def load_object_points(path, sensor_order):
    data = load_json(path)
    by_sensor = {
        int(item["sensor"]): [
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ]
        for item in data["sensors"]
    }
    return np.array([by_sensor[sensor] for sensor in sensor_order], dtype=np.float32).reshape((-1, 1, 3))


def unwrap_near(angle_deg, reference_deg):
    while angle_deg - reference_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg - reference_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def compute_az_el(sweep0_deg, sweep1_deg):
    sweep1_deg = unwrap_near(sweep1_deg, sweep0_deg)
    azimuth = (sweep0_deg + sweep1_deg) / 2.0
    elevation = (sweep0_deg - sweep1_deg) / (2.0 * TAN_30)
    return azimuth, elevation


def score_angles(azimuth, elevation):
    return abs(elevation) + max(0.0, abs(azimuth) - 90.0)


def compute_best_az_el(lfsr0, lfsr1, coeffs):
    normal_az, normal_el = compute_az_el(
        coeffs["A0"] * lfsr0 + coeffs["B0"],
        coeffs["A1"] * lfsr1 + coeffs["B1"],
    )
    swapped_az, swapped_el = compute_az_el(
        coeffs["A0"] * lfsr1 + coeffs["B0"],
        coeffs["A1"] * lfsr0 + coeffs["B1"],
    )

    if score_angles(swapped_az, swapped_el) < score_angles(normal_az, normal_el):
        return swapped_az, swapped_el, "swapped"
    return normal_az, normal_el, "normal"


def compute_mode_az_el(lfsr0, lfsr1, coeffs, mode):
    if mode == "normal":
        az, el = compute_az_el(
            coeffs["A0"] * lfsr0 + coeffs["B0"],
            coeffs["A1"] * lfsr1 + coeffs["B1"],
        )
        return az, el, mode

    if mode == "swapped":
        az, el = compute_az_el(
            coeffs["A0"] * lfsr1 + coeffs["B0"],
            coeffs["A1"] * lfsr0 + coeffs["B1"],
        )
        return az, el, mode

    raise ValueError(f"Unknown LH2 angle mode: {mode}")


def new_state(sensor_order, basestations):
    return {
        sensor: {
            bs: {"lfsr_by_sweep": {}, "az": None, "el": None, "age": None, "mode": None}
            for bs in basestations
        }
        for sensor in sensor_order
    }


def update_state_from_lh2(state, data, coeffs, angle_modes=None):
    sensor = data["sensor"]
    bs = data["basestation"]
    sweep = data["sweep"]
    poly = data["polynomial"]
    lfsr = data["lfsr"]

    if sensor not in state or bs not in coeffs:
        return False
    if sweep not in (0, 1):
        return False
    if bs in DEFAULT_BS_POLYS and poly not in DEFAULT_BS_POLYS[bs]:
        return False

    sweep_state = state[sensor][bs]["lfsr_by_sweep"]
    sweep_state[sweep] = lfsr

    if 0 in sweep_state and 1 in sweep_state:
        forced_mode = None
        if angle_modes is not None:
            forced_mode = angle_modes.get(bs, {}).get(sensor)

        if forced_mode is None:
            az, el, mode = compute_best_az_el(sweep_state[0], sweep_state[1], coeffs[bs])
        else:
            az, el, mode = compute_mode_az_el(sweep_state[0], sweep_state[1], coeffs[bs], forced_mode)

        state[sensor][bs].update({
            "az": az,
            "el": el,
            "age": time.time(),
            "mode": mode,
        })
        sweep_state.clear()
        return True

    return False


def transform_distance(T, previous_T):
    if previous_T is None:
        return 0.0

    dt = float(np.linalg.norm(T[:3, 3] - previous_T[:3, 3]))
    dR = T[:3, :3] @ previous_T[:3, :3].T
    trace = np.trace(dR)
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    return dt + 0.25 * angle


def choose_pnp_solution(rvecs, tvecs, reprojection_errors, object_points, image_points, previous_T):
    candidates = []
    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    for idx, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        rvec = np.asarray(rvec, dtype=float).reshape(3)
        tvec = np.asarray(tvec, dtype=float).reshape(3)

        if tvec[2] <= 0.0:
            continue

        if reprojection_errors is not None and len(reprojection_errors) > idx:
            reproj_rmse = float(np.asarray(reprojection_errors[idx]).reshape(-1)[0])
        else:
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
            residual = image_points.reshape((-1, 2)) - projected.reshape((-1, 2))
            reproj_rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))

        T = rt_to_matrix(rvec, tvec)
        continuity = transform_distance(T, previous_T)
        score = reproj_rmse + 0.05 * continuity
        candidates.append((score, reproj_rmse, rvec, tvec))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    _score, reproj_rmse, rvec, tvec = candidates[0]
    return rvec, tvec, reproj_rmse


def solve_bs_pose(state, bs, sensor_order, object_points, max_age_s=0.5, y_sign=1.0, previous_T=None):
    image = []
    modes = []
    now = time.time()

    for sensor in sensor_order:
        item = state[sensor][bs]
        if item["az"] is None or item["el"] is None:
            return None
        if now - item["age"] > max_age_s:
            return None

        image.append([
            math.tan(math.radians(item["az"])),
            y_sign * math.tan(math.radians(item["el"])),
        ])
        modes.append(item["mode"])

    image_points = np.array(image, dtype=np.float32).reshape((-1, 1, 2))
    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    try:
        success, rvecs, tvecs, reprojection_errors = cv2.solvePnPGeneric(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error:
        success = False

    chosen = None
    if success:
        chosen = choose_pnp_solution(rvecs, tvecs, reprojection_errors, object_points, image_points, previous_T)

    if chosen is None:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None

        rvec = rvec.reshape(3)
        tvec = tvec.reshape(3)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        reproj = image_points.reshape((-1, 2)) - projected.reshape((-1, 2))
        reproj_rmse = float(np.sqrt(np.mean(np.sum(reproj * reproj, axis=1))))
    else:
        rvec, tvec, reproj_rmse = chosen

    return {
        "rvec": rvec,
        "tvec": tvec,
        "reproj_rmse": reproj_rmse,
        "modes": modes,
    }


def rt_to_matrix(rvec, tvec):
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rotation.from_rotvec(np.array(rvec, dtype=float)).as_matrix()
    T[:3, 3] = np.array(tvec, dtype=float)
    return T


def matrix_to_rt(T):
    rotvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    tvec = T[:3, 3]
    return rotvec, tvec


def average_transforms(transforms):
    if not transforms:
        raise ValueError("No transforms to average.")

    translations = np.array([T[:3, 3] for T in transforms], dtype=float)
    median_t = np.median(translations, axis=0)

    rotations = Rotation.from_matrix(np.array([T[:3, :3] for T in transforms], dtype=float))
    mean_R = rotations.mean().as_matrix()

    out = np.eye(4, dtype=float)
    out[:3, :3] = mean_R
    out[:3, 3] = median_t
    return out
