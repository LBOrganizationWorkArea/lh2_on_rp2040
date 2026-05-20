import argparse
import math
import time

import numpy as np
import serial
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from wand_common import (
    load_angle_modes,
    load_json,
    load_latest_coefficients,
    new_state,
    parse_lh2_line,
    rt_to_matrix,
    solve_bs_pose,
    update_state_from_lh2,
)


def load_layout(path, sensor_order):
    data = load_json(path)
    by_sensor = {
        int(item["sensor"]): np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ], dtype=float)
        for item in data["sensors"]
    }
    return np.array([by_sensor[sensor] for sensor in sensor_order], dtype=float)


def current_measurements(state, basestations, sensor_order, max_age_s):
    now = time.time()
    out = {}

    for bs in basestations:
        out[bs] = []
        for sensor in sensor_order:
            item = state[sensor][bs]
            if item["az"] is None or item["el"] is None or now - item["age"] > max_age_s:
                return None
            out[bs].append([
                math.tan(math.radians(float(item["az"]))),
                math.tan(math.radians(float(item["el"]))),
            ])
        out[bs] = np.array(out[bs], dtype=float)

    return out


def project(points_cam):
    z = points_cam[:, 2]
    z_safe = np.where(np.abs(z) < 1e-6, 1e-6, z)
    return np.column_stack([points_cam[:, 0] / z_safe, points_cam[:, 1] / z_safe])


def residuals_pose(params, object_points, measurements, world_bs, other_bs, T_world_other):
    R_obj = Rotation.from_rotvec(params[0:3]).as_matrix()
    t_obj = params[3:6]

    p_world = (R_obj @ object_points.T).T + t_obj
    pred_world = project(p_world)

    Rwo = T_world_other[:3, :3]
    two = T_world_other[:3, 3]
    p_other = (Rwo.T @ (p_world - two).T).T
    pred_other = project(p_other)

    out = []
    out.extend((pred_world - measurements[world_bs]).reshape(-1))
    out.extend((pred_other - measurements[other_bs]).reshape(-1))

    out.extend(np.maximum(0.02 - p_world[:, 2], 0.0) * 10.0)
    out.extend(np.maximum(0.02 - p_other[:, 2], 0.0) * 10.0)

    return np.array(out, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Live direct pose using direct self-calibration.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--modes", default="config/angle_modes.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--calibration", default="config/direct_selfcalib.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--max-age", type=float, default=0.60)
    parser.add_argument("--print-period", type=float, default=0.10)
    args = parser.parse_args()

    coeffs = load_latest_coefficients(args.history)
    angle_modes = load_angle_modes(args.modes)
    calib = load_json(args.calibration)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensor_order = [int(x) for x in args.sensors.split(",")]
    world_bs = int(calib["world_bs"])
    other_bs = int(calib["other_bs"])
    T_world_other = np.array(calib["transform_world_from_other"], dtype=float)

    object_points = load_layout(args.layout, sensor_order)
    state = new_state(sensor_order, basestations)
    previous = None
    last_print = 0.0

    print("=" * 70)
    print("Live direct pose")
    print(f"World frame: BS{world_bs}")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is not None:
                update_state_from_lh2(state, data, coeffs, angle_modes)

            now = time.time()
            if now - last_print < args.print_period:
                continue

            measurements = current_measurements(state, basestations, sensor_order, args.max_age)
            print("\033[H\033[J", end="")
            print("Direct pose")
            print("-" * 70)

            if measurements is None:
                print("waiting for fresh 4-sensor angles from both Lighthouses...")
                last_print = now
                continue

            if previous is None:
                # Use BS4 PnP only as a first pose guess; direct optimization refines with both BS.
                obj_cv = object_points.astype(np.float32).reshape((-1, 1, 3))
                pose = solve_bs_pose(state, world_bs, sensor_order, obj_cv)
                if pose is None:
                    previous = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0], dtype=float)
                else:
                    previous = np.array([*pose["rvec"], *pose["tvec"]], dtype=float)

            lower = np.array([-math.pi, -math.pi, -math.pi, -5.0, -5.0, 0.05], dtype=float)
            upper = np.array([+math.pi, +math.pi, +math.pi, +5.0, +5.0, 10.0], dtype=float)
            x0 = np.clip(previous, lower + 1e-6, upper - 1e-6)

            result = least_squares(
                residuals_pose,
                x0,
                bounds=(lower, upper),
                args=(object_points, measurements, world_bs, other_bs, T_world_other),
                loss="soft_l1",
                f_scale=0.01,
                max_nfev=50,
            )
            previous = result.x
            rmse = float(np.sqrt(np.mean(residuals_pose(result.x, object_points, measurements, world_bs, other_bs, T_world_other) ** 2)))
            t = result.x[3:6]

            print(f"x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m | rmse_image={rmse:.5f}")
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
