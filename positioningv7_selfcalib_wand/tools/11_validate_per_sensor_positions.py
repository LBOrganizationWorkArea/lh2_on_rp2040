import argparse
import math

import numpy as np

from wand_common import load_json


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

    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)

    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rz @ ry @ rx


def known_sensor_point(frame, sensor, layout, anchor_sensor=None):
    pose = frame["pose"]
    center = np.array([float(pose["x_m"]), float(pose["y_m"]), float(pose["z_m"])], dtype=float)
    if anchor_sensor is not None and sensor == anchor_sensor:
        return center
    return center + pose_rotation(pose) @ layout[sensor]


def unwrap_near(angle_deg, reference_deg):
    while angle_deg - reference_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg - reference_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def measured_image_abs(item, bs_calib):
    coeffs = bs_calib["coefficients"]
    mode = bs_calib.get("modes_by_sensor", {}).get(str(item["sensor"]), "normal")

    if mode == "swapped":
        lfsr0 = float(item["lfsr1"])
        lfsr1 = float(item["lfsr0"])
    else:
        lfsr0 = float(item["lfsr0"])
        lfsr1 = float(item["lfsr1"])

    sweep0 = coeffs["A0"] * lfsr0 + coeffs["B0"]
    sweep1 = coeffs["A1"] * lfsr1 + coeffs["B1"]
    sweep1 = unwrap_near(sweep1, sweep0)

    alpha0 = math.radians(sweep0)
    alpha1 = math.radians(sweep1)
    azimuth = (alpha0 + alpha1) / 2.0
    half_delta = abs((alpha1 - alpha0) / 2.0)

    cos_azimuth = math.cos(azimuth)
    if abs(cos_azimuth) < 1e-6:
        cos_azimuth = math.copysign(1e-6, cos_azimuth)

    u = -math.tan(azimuth)
    v = (
        -math.sin(half_delta - math.pi / 3.0)
        / math.tan(math.pi / 6.0)
        / cos_azimuth
    )
    return u, v


def measured_direction_sweep(item, bs_calib):
    coeffs = bs_calib["coefficients"]
    mode = bs_calib.get("modes_by_sensor", {}).get(str(item["sensor"]), "normal")

    if mode == "swapped":
        lfsr0 = float(item["lfsr1"])
        lfsr1 = float(item["lfsr0"])
    else:
        lfsr0 = float(item["lfsr0"])
        lfsr1 = float(item["lfsr1"])

    sweep0 = coeffs["A0"] * lfsr0 + coeffs["B0"]
    sweep1 = coeffs["A1"] * lfsr1 + coeffs["B1"]
    sweep1 = unwrap_near(sweep1, sweep0)

    if float(bs_calib.get("elevation_sign", 1.0)) < 0.0:
        sweep0, sweep1 = sweep1, sweep0

    alpha0 = math.radians(sweep0)
    alpha1 = math.radians(sweep1)
    theta = (alpha0 + alpha1) / 2.0
    half_delta = (alpha1 - alpha0) / 2.0
    value = math.sin(half_delta - math.pi / 3.0)

    cos_theta = math.cos(theta)
    if abs(cos_theta) < 1e-6:
        cos_theta = math.copysign(1e-6, cos_theta)

    x_over_z = math.tan(theta)
    y_over_z = value / (math.tan(math.pi / 6.0) * cos_theta * cos_theta)
    direction = np.array([x_over_z, y_over_z, 1.0], dtype=float)
    direction /= np.linalg.norm(direction)
    return direction


def ray_from_observation(item, bs_calib):
    model = bs_calib.get("model", item.get("model", "image-abs"))
    if model == "sweep":
        direction_bs = measured_direction_sweep(item, bs_calib)
    else:
        u, v = measured_image_abs(item, bs_calib)
        x, y = -u, v
        direction_bs = np.array([x, y, 1.0], dtype=float)
        direction_bs /= np.linalg.norm(direction_bs)

    T_world_from_bs = np.array(bs_calib["T_world_from_bs"], dtype=float)
    origin = T_world_from_bs[:3, 3]
    direction = T_world_from_bs[:3, :3] @ direction_bs
    direction /= np.linalg.norm(direction)
    return origin, direction


def triangulate_two_rays(ray_a, ray_b):
    origin_a, direction_a = ray_a
    origin_b, direction_b = ray_b
    A = np.column_stack([direction_a, -direction_b])
    b = origin_b - origin_a
    t, *_ = np.linalg.lstsq(A, b, rcond=None)
    point_a = origin_a + t[0] * direction_a
    point_b = origin_b + t[1] * direction_b
    return (point_a + point_b) / 2.0, float(np.linalg.norm(point_a - point_b))


def main():
    parser = argparse.ArgumentParser(description="Validate per-sensor LH2 XYZ reconstruction on captured calibration points.")
    parser.add_argument("--input", default="data/angle3d_calibration.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--calibration", default="config/angle3d_calibration_per_sensor.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--model", choices=("auto", "image-abs", "sweep"), default="auto")
    parser.add_argument("--anchor-sensor", type=int, default=None)
    parser.add_argument("--only-sensor", type=int, default=None)
    args = parser.parse_args()

    record = load_json(args.input)
    layout = load_layout(args.layout)
    calibration = load_json(args.calibration)
    calibration_model = calibration.get("model", "image-abs") if args.model == "auto" else args.model
    bs_ids = [int(x) for x in args.basestations.split(",")]
    if len(bs_ids) != 2:
        raise ValueError("This validator expects exactly two basestations.")

    print("=" * 70)
    print("Validate per-sensor LH2 XYZ reconstruction")
    print(f"Input: {args.input}")
    print(f"Calibration: {args.calibration}")
    print("=" * 70)

    all_errors = []
    for sensor_key, sensor_calib in calibration.get("sensors", {}).items():
        sensor = int(sensor_key)
        if args.only_sensor is not None and sensor != args.only_sensor:
            continue
        errors = []
        ray_gaps = []
        worst = []

        for frame_idx, frame in enumerate(record["frames"]):
            observations = frame.get("observations", {})
            rays = []
            usable = True
            for bs in bs_ids:
                item = observations.get(str(bs), {}).get(str(sensor))
                bs_calib = sensor_calib.get("basestations", {}).get(str(bs))
                if item is None or bs_calib is None:
                    usable = False
                    break
                item = dict(item)
                item["sensor"] = sensor
                item["model"] = calibration_model
                bs_calib = dict(bs_calib)
                bs_calib["model"] = calibration_model
                rays.append(ray_from_observation(item, bs_calib))

            if not usable:
                continue

            estimated, ray_gap = triangulate_two_rays(rays[0], rays[1])
            expected = known_sensor_point(frame, sensor, layout, args.anchor_sensor)
            error = float(np.linalg.norm(estimated - expected))
            errors.append(error)
            ray_gaps.append(ray_gap)
            worst.append((error, frame_idx, frame["pose"].get("name", str(frame_idx)), estimated, expected, ray_gap))

        if not errors:
            print(f"sensor={sensor}: no usable frames")
            continue

        all_errors.extend(errors)
        errors_np = np.array(errors, dtype=float)
        gaps_np = np.array(ray_gaps, dtype=float)
        print(
            f"sensor={sensor}: n={len(errors)} "
            f"median_err={np.median(errors_np):.3f} m mean_err={np.mean(errors_np):.3f} m max_err={np.max(errors_np):.3f} m "
            f"median_ray_gap={np.median(gaps_np):.3f} m"
        )
        for error, frame_idx, pose_name, estimated, expected, ray_gap in sorted(worst, reverse=True)[:5]:
            print(
                f"  worst frame={frame_idx:02d} {pose_name} err={error:.3f} m ray_gap={ray_gap:.3f} m "
                f"est=({estimated[0]:+.3f},{estimated[1]:+.3f},{estimated[2]:+.3f}) "
                f"ref=({expected[0]:+.3f},{expected[1]:+.3f},{expected[2]:+.3f})"
            )

    if all_errors:
        all_np = np.array(all_errors, dtype=float)
        print("-" * 70)
        print(
            f"all sensors: n={len(all_errors)} median_err={np.median(all_np):.3f} m "
            f"mean_err={np.mean(all_np):.3f} m max_err={np.max(all_np):.3f} m"
        )


if __name__ == "__main__":
    main()
