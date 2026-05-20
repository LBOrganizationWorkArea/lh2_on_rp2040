import argparse
import math

import cv2
import numpy as np

from wand_common import (
    load_json,
    load_latest_coefficients,
    save_json,
)


TAN_30 = math.tan(math.radians(30.0))


def load_layout(path):
    data = load_json(path)
    return {
        int(item["sensor"]): np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ], dtype=float)
        for item in data["sensors"]
    }


def pose_rotation(pose):
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
    pitch = math.radians(float(pose.get("pitch_deg", 0.0)))
    roll = math.radians(float(pose.get("roll_deg", 0.0)))

    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    return rz @ ry @ rx


def sensor_world_point(frame, sensor, layout):
    pose = frame["pose"]
    center = np.array([
        float(pose["x_m"]),
        float(pose["y_m"]),
        float(pose["z_m"]),
    ], dtype=float)
    return center + pose_rotation(pose) @ layout[sensor]


def unwrap_near(angle_deg, reference_deg):
    while angle_deg - reference_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg - reference_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def sweeps_to_image(lfsr0, lfsr1, coeffs, mode):
    if mode == "swapped":
        lfsr0, lfsr1 = lfsr1, lfsr0

    alpha0 = coeffs["A0"] * lfsr0 + coeffs["B0"]
    alpha1 = coeffs["A1"] * lfsr1 + coeffs["B1"]
    alpha1 = unwrap_near(alpha1, alpha0)

    # Same small-angle image coordinates used by live PnP tools.
    azimuth = (alpha0 + alpha1) / 2.0
    elevation = (alpha0 - alpha1) / (2.0 * TAN_30)
    return [
        math.tan(math.radians(azimuth)),
        math.tan(math.radians(elevation)),
    ]


def collect_points(record, layout, bs, sensor, coeffs, mode):
    object_points = []
    image_points = []

    for frame in record["frames"]:
        item = frame.get("observations", {}).get(str(bs), {}).get(str(sensor))
        if item is None:
            continue

        object_points.append(sensor_world_point(frame, sensor, layout))
        image_points.append(sweeps_to_image(
            float(item["lfsr0"]),
            float(item["lfsr1"]),
            coeffs,
            mode,
        ))

    return np.array(object_points, dtype=np.float32), np.array(image_points, dtype=np.float32)


def fit_reprojection_rmse(object_points, image_points):
    if len(object_points) < 6:
        return None

    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points.reshape((-1, 1, 3)),
            image_points.reshape((-1, 1, 2)),
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error:
        return None

    if not success:
        return None

    projected, _ = cv2.projectPoints(
        object_points.reshape((-1, 1, 3)),
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    residual = image_points - projected.reshape((-1, 2))
    return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))


def main():
    parser = argparse.ArgumentParser(description="Calibrate LH2 normal/swapped angle modes per basestation and sensor.")
    parser.add_argument("--input", default="data/angle3d_calibration.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--output", default="config/angle_modes.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    args = parser.parse_args()

    record = load_json(args.input)
    layout = load_layout(args.layout)
    coeffs = load_latest_coefficients(args.history)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensors = [int(x) for x in args.sensors.split(",")]

    modes = {}

    print("=" * 70)
    print("Calibrate LH2 angle modes")
    print(f"Input: {args.input}")
    print("=" * 70)

    for bs in basestations:
        modes[str(bs)] = {}
        print(f"BS{bs}")
        for sensor in sensors:
            scores = {}
            for mode in ("normal", "swapped"):
                object_points, image_points = collect_points(record, layout, bs, sensor, coeffs[bs], mode)
                scores[mode] = fit_reprojection_rmse(object_points, image_points)

            valid = {mode: score for mode, score in scores.items() if score is not None}
            if not valid:
                print(f"  sensor={sensor}: not enough data")
                continue

            best_mode = min(valid, key=valid.get)
            modes[str(bs)][str(sensor)] = best_mode
            normal = scores.get("normal")
            swapped = scores.get("swapped")
            normal_text = "nan" if normal is None else f"{normal:.4f}"
            swapped_text = "nan" if swapped is None else f"{swapped:.4f}"
            print(
                f"  sensor={sensor}: mode={best_mode} "
                f"normal_rmse={normal_text} swapped_rmse={swapped_text}"
            )

    save_json(args.output, {
        "description": "Fixed LH2 sweep pairing modes calibrated from known 3D points.",
        "modes": modes,
    })

    print("=" * 70)
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
