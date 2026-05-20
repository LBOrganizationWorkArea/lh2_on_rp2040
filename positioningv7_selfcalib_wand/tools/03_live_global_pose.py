import argparse
import time

import numpy as np
import serial

from wand_common import (
    average_transforms,
    load_json,
    load_latest_coefficients,
    load_object_points,
    matrix_to_rt,
    new_state,
    parse_lh2_line,
    rt_to_matrix,
    solve_bs_pose,
    update_state_from_lh2,
)


def main():
    parser = argparse.ArgumentParser(description="Live global drone pose using calibrated relative Lighthouse transform.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--relative", default="config/bs_relative.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--print-period", type=float, default=0.1)
    parser.add_argument("--y-sign", type=float, default=1.0)
    args = parser.parse_args()

    coeffs = load_latest_coefficients(args.history)
    relative = load_json(args.relative)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensor_order = [int(x) for x in args.sensors.split(",")]
    object_points = load_object_points(args.layout, sensor_order)
    state = new_state(sensor_order, basestations)
    previous_T = {bs: None for bs in basestations}

    world_bs = int(relative["world_bs"])
    other_bs = int(relative["other_bs"])
    T_world_other = np.array(relative["transform_world_from_other"], dtype=float)

    print("=" * 70)
    print("Live global pose")
    print(f"World frame: BS{world_bs}")
    print(f"Using relative transform for BS{other_bs}")
    print("=" * 70)

    last_print = 0.0

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is not None:
                update_state_from_lh2(state, data, coeffs)

            now = time.time()
            if now - last_print < args.print_period:
                continue

            world_poses = []
            labels = []

            pose_world = solve_bs_pose(state, world_bs, sensor_order, object_points, y_sign=args.y_sign, previous_T=previous_T[world_bs])
            if pose_world is not None:
                T_world_obj = rt_to_matrix(pose_world["rvec"], pose_world["tvec"])
                previous_T[world_bs] = T_world_obj
                world_poses.append(T_world_obj)
                labels.append(f"BS{world_bs}")

            pose_other = solve_bs_pose(state, other_bs, sensor_order, object_points, y_sign=args.y_sign, previous_T=previous_T[other_bs])
            if pose_other is not None:
                T_other_obj = rt_to_matrix(pose_other["rvec"], pose_other["tvec"])
                previous_T[other_bs] = T_other_obj
                world_poses.append(T_world_other @ T_other_obj)
                labels.append(f"BS{other_bs}->BS{world_bs}")

            print("\033[H\033[J", end="")
            print("Global pose")
            print("-" * 70)

            if not world_poses:
                print("waiting for fresh poses...")
            else:
                fused = average_transforms(world_poses)
                _rvec, tvec = matrix_to_rt(fused)
                print(f"fused: x={tvec[0]:+.3f} y={tvec[1]:+.3f} z={tvec[2]:+.3f} m | sources={','.join(labels)}")

                if len(world_poses) == 2:
                    delta = float(np.linalg.norm(world_poses[0][:3, 3] - world_poses[1][:3, 3]))
                    print(f"BS agreement: {delta:.3f} m")

            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
