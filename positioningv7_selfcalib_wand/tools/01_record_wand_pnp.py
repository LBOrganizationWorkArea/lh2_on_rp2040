import argparse
import time

import serial

from wand_common import (
    load_latest_coefficients,
    load_object_points,
    new_state,
    parse_lh2_line,
    rt_to_matrix,
    save_json,
    solve_bs_pose,
    update_state_from_lh2,
)


def pose_to_json(pose):
    return {
        "rvec": [float(x) for x in pose["rvec"]],
        "tvec": [float(x) for x in pose["tvec"]],
        "reproj_rmse": float(pose["reproj_rmse"]),
        "modes": list(pose["modes"]),
    }


def fresh_count(state, bs, sensor_order, max_age_s=0.8):
    now = time.time()
    fresh = []
    missing = []

    for sensor in sensor_order:
        item = state[sensor][bs]
        if item["az"] is None or item["el"] is None:
            missing.append(sensor)
            continue
        if now - item["age"] > max_age_s:
            missing.append(sensor)
            continue
        fresh.append(sensor)

    return fresh, missing


def main():
    parser = argparse.ArgumentParser(description="Record simultaneous BS4/BS10 PnP poses while using the drone as a wand.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--output", default="data/wand_pnp_record.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--period", type=float, default=0.10)
    parser.add_argument("--max-reproj", type=float, default=0.01)
    parser.add_argument("--y-sign", type=float, default=1.0)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    coeffs = load_latest_coefficients(args.history)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensor_order = [int(x) for x in args.sensors.split(",")]
    object_points = load_object_points(args.layout, sensor_order)
    state = new_state(sensor_order, basestations)

    frames = []
    start = time.time()
    last_frame = 0.0
    previous_T = {bs: None for bs in basestations}
    last_debug = 0.0
    updates = 0
    pose_ok_counts = {bs: 0 for bs in basestations}
    pose_none_counts = {bs: 0 for bs in basestations}
    reproj_reject_counts = {bs: 0 for bs in basestations}

    print("=" * 70)
    print("Record wand PnP frames")
    print(f"Duration: {args.duration:.1f} s")
    print("Move the drone in 3D with yaw/pitch/roll while both Lighthouses see it.")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        while time.time() - start < args.duration:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is not None:
                if update_state_from_lh2(state, data, coeffs):
                    updates += 1

            now = time.time()
            if now - last_frame < args.period:
                continue

            poses = {}
            current_T = {}
            ok = True

            for bs in basestations:
                pose = solve_bs_pose(state, bs, sensor_order, object_points, y_sign=args.y_sign, previous_T=previous_T[bs])
                if pose is None:
                    pose_none_counts[bs] += 1
                    ok = False
                    continue
                pose_ok_counts[bs] += 1
                if pose["reproj_rmse"] > args.max_reproj:
                    reproj_reject_counts[bs] += 1
                    ok = False
                    continue
                poses[str(bs)] = pose_to_json(pose)
                current_T[bs] = rt_to_matrix(pose["rvec"], pose["tvec"])

            if ok and len(poses) == len(basestations):
                previous_T.update(current_T)
                frames.append({
                    "pc_time_s": now,
                    "poses": poses,
                })
                print(f"\rframes={len(frames)}", end="", flush=True)

            last_frame = now

            if args.debug and now - last_debug >= 1.0:
                fresh_info = {
                    bs: {
                        "fresh": fresh_count(state, bs, sensor_order)[0],
                        "missing": fresh_count(state, bs, sensor_order)[1],
                    }
                    for bs in basestations
                }
                print()
                print(
                    f"debug updates={updates} frames={len(frames)} "
                    f"pose_ok={pose_ok_counts} pose_none={pose_none_counts} "
                    f"reproj_reject={reproj_reject_counts} fresh={fresh_info}"
                )
                last_debug = now

    output = {
        "description": "Simultaneous PnP poses from each Lighthouse while moving the drone as a calibration wand.",
        "basestations": basestations,
        "sensors": sensor_order,
        "duration_s": args.duration,
        "y_sign": args.y_sign,
        "frames": frames,
    }
    save_json(args.output, output)

    print()
    print("=" * 70)
    print(f"Saved {len(frames)} frames to {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
