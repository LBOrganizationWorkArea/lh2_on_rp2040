import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def angles_to_image_point(theta, phi):
    """
    Convert Lighthouse angular measurements to normalized image coordinates.
    Same idea as the PR PnP prototype.
    """
    u = math.tan(theta)
    v = math.tan(phi) / math.cos(theta)
    return [u, v]


def project_points(object_points, rvec, tvec):
    camera_matrix = np.eye(3, dtype=np.float64)
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)

    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs
    )

    return projected.reshape(-1, 2)


def reprojection_rmse(image_points, projected_points):
    err = image_points - projected_points
    return float(np.sqrt(np.mean(np.sum(err * err, axis=1))))


def estimate_one_basestation(bs_id, layout, origin_angles):
    measurements = origin_angles["measurements"].get(str(bs_id), {})

    object_points = []
    image_points = []
    used_sensors = []

    for sensor_id_str, sensor_info in layout["sensors"].items():
        sensor_id = int(sensor_id_str)

        if sensor_id_str not in measurements:
            continue

        m = measurements[sensor_id_str]

        x = float(sensor_info["x"])
        y = float(sensor_info["y"])
        z = float(sensor_info["z"])

        theta = float(m["theta"])
        phi = float(m["phi"])

        object_points.append([x, y, z])
        image_points.append(angles_to_image_point(theta, phi))
        used_sensors.append(sensor_id)

    if len(object_points) < 4:
        raise RuntimeError(f"Basestation {bs_id}: need 4 sensors, got {len(object_points)}")

    object_points = np.array(object_points, dtype=np.float64)
    image_points = np.array(image_points, dtype=np.float64)

    camera_matrix = np.eye(3, dtype=np.float64)
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)

    # Try IPPE first because the 4 sensors are coplanar.
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE
    )

    if not success:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

    if not success:
        raise RuntimeError(f"Basestation {bs_id}: solvePnP failed")

    projected = project_points(object_points, rvec, tvec)
    rmse_image = reprojection_rmse(image_points, projected)

    R, _ = cv2.Rodrigues(rvec)

    # Camera/Lighthouse position in origin-drone frame:
    # X_lighthouse = R * X_origin + t
    # C_origin = -R.T * t
    position_origin = -R.T @ tvec

    return {
        "basestation": int(bs_id),
        "used_sensors": used_sensors,
        "rvec": rvec.reshape(3).tolist(),
        "tvec": tvec.reshape(3).tolist(),
        "R_lighthouse_from_origin": R.tolist(),
        "position_in_origin_frame_m": position_origin.reshape(3).tolist(),
        "quality": {
            "rmse_normalized_image": rmse_image
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Estimate Lighthouse geometry from origin angles.")
    parser.add_argument("--layout", required=True)
    parser.add_argument("--origin", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--basestations", default="4,10")
    args = parser.parse_args()

    layout = load_json(args.layout)
    origin = load_json(args.origin)

    basestations = [int(x.strip()) for x in args.basestations.split(",") if x.strip()]

    print("Estimate Lighthouse geometry")
    print("=" * 60)
    print(f"Layout: {args.layout}")
    print(f"Origin: {args.origin}")
    print(f"Basestations: {basestations}")
    print("=" * 60)

    result = {
        "type": "lighthouse_geometry",
        "frame": "origin_drone_frame",
        "assumption": "drone origin pose is flat and fixed during capture",
        "layout_file": args.layout,
        "origin_file": args.origin,
        "basestations": {}
    }

    for bs in basestations:
        est = estimate_one_basestation(bs, layout, origin)
        result["basestations"][str(bs)] = est

        p = est["position_in_origin_frame_m"]
        q = est["quality"]

        print()
        print(f"Basestation {bs}")
        print(f"  used sensors: {est['used_sensors']}")
        print(f"  position approx in origin frame:")
        print(f"    x={p[0]:+.3f} m, y={p[1]:+.3f} m, z={p[2]:+.3f} m")
        print(f"  rmse normalized image: {q['rmse_normalized_image']:.6f}")

    save_json(args.output, result)

    print()
    print("=" * 60)
    print(f"Saved: {args.output}")
    print("=" * 60)


if __name__ == "__main__":
    main()
