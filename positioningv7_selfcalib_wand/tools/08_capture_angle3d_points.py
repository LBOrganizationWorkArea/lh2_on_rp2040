import argparse
import statistics
import time

import serial

from wand_common import DEFAULT_BS_POLYS, load_json, parse_lh2_line, save_json


def select_poses(poses, selector):
    if not selector:
        return poses

    selected = []
    by_name = {pose["name"]: pose for pose in poses}
    for item in selector.split(","):
        key = item.strip()
        if not key:
            continue

        if key.isdigit():
            idx = int(key)
            if idx < 0 or idx >= len(poses):
                raise ValueError(f"Point index out of range: {idx}")
            selected.append(poses[idx])
            continue

        if key not in by_name:
            raise ValueError(f"Unknown point name: {key}")
        selected.append(by_name[key])

    return selected


def merge_frames(output_path, captured_frames):
    try:
        existing = load_json(output_path)
        frames = existing.get("frames", [])
    except FileNotFoundError:
        frames = []

    captured_by_name = {frame["pose"]["name"]: frame for frame in captured_frames}
    merged = []
    used = set()

    for frame in frames:
        name = frame.get("pose", {}).get("name")
        if name in captured_by_name:
            merged.append(captured_by_name[name])
            used.add(name)
        else:
            merged.append(frame)

    for frame in captured_frames:
        name = frame["pose"]["name"]
        if name not in used:
            merged.append(frame)

    return merged


def empty_bucket(basestations, sensors):
    return {
        bs: {
            sensor: {0: [], 1: [], "last": {}}
            for sensor in sensors
        }
        for bs in basestations
    }


def add_sample(bucket, data, basestations, sensors, dedupe=True):
    bs = data["basestation"]
    sensor = data["sensor"]
    poly = data["polynomial"]
    lfsr = int(data["lfsr"])

    if bs not in basestations or sensor not in sensors:
        return False
    if bs in DEFAULT_BS_POLYS and poly not in DEFAULT_BS_POLYS[bs]:
        return False
    sweep = poly & 1

    sensor_bucket = bucket[bs][sensor]
    if dedupe and sensor_bucket["last"].get(sweep) == lfsr:
        return False

    sensor_bucket["last"][sweep] = lfsr
    sensor_bucket[sweep].append(lfsr)
    return True


def summarize_bucket(bucket, basestations, sensors, min_samples):
    out = {}
    usable = 0

    for bs in basestations:
        bs_out = {}
        for sensor in sensors:
            sweeps = bucket[bs][sensor]
            n0 = len(sweeps[0])
            n1 = len(sweeps[1])
            if n0 < min_samples or n1 < min_samples:
                continue

            bs_out[str(sensor)] = {
                "lfsr0": int(round(statistics.median(sweeps[0]))),
                "lfsr1": int(round(statistics.median(sweeps[1]))),
                "n0": n0,
                "n1": n1,
            }
            usable += 1

        out[str(bs)] = bs_out

    return out, usable


def main():
    parser = argparse.ArgumentParser(description="Capture known 3D points for LH2 angle calibration.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--points", default="config/angle3d_points.json")
    parser.add_argument("--output", default="data/angle3d_calibration.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--allow-repeats", action="store_true")
    parser.add_argument(
        "--only-point",
        default=None,
        help="Capture only these point names or zero-based indexes, comma-separated. Example: F03_centre_gauche or 3,4",
    )
    parser.add_argument(
        "--merge-output",
        action="store_true",
        help="Keep existing output frames and replace only captured point names.",
    )
    args = parser.parse_args()

    points = load_json(args.points)
    poses = select_poses(points["poses"], args.only_point)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensors = [int(x) for x in args.sensors.split(",")]
    frames = []

    print("=" * 70)
    print("Capture LH2 3D angle calibration points")
    print(f"Points: {args.points}")
    print(f"Output: {args.output}")
    print("Keep drone orientation exactly as written in the point file.")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()

        stop_capture = False
        for pose in poses:
            if stop_capture:
                break

            while True:
                print()
                print("=" * 70)
                print(f"Place drone at {pose['name']}")
                print(
                    f"x={pose['x_m']:+.3f} y={pose['y_m']:+.3f} z={pose['z_m']:+.3f} m | "
                    f"yaw={pose.get('yaw_deg', 0.0):+.1f} pitch={pose.get('pitch_deg', 0.0):+.1f} roll={pose.get('roll_deg', 0.0):+.1f} deg"
                )
                input("Press ENTER when still...")

                bucket = empty_bucket(basestations, sensors)
                accepted = 0
                repeated = 0
                start = time.time()
                while time.time() - start < args.seconds:
                    raw = ser.readline().decode("utf-8", errors="ignore").strip()
                    data = parse_lh2_line(raw)
                    if data is not None:
                        if add_sample(bucket, data, basestations, sensors, dedupe=not args.allow_repeats):
                            accepted += 1
                        else:
                            repeated += 1

                observations, usable = summarize_bucket(bucket, basestations, sensors, args.min_samples)
                print(f"Captured usable sensor observations: {usable} | accepted={accepted} ignored={repeated}")
                for bs_key, bs_obs in observations.items():
                    for sensor_key, item in bs_obs.items():
                        print(f"  bs={bs_key} sensor={sensor_key} n0={item['n0']} n1={item['n1']}")

                choice = input("ENTER=keep | r=retry this point | s=skip point | q=stop and save: ").strip().lower()
                if choice in ("r", "retry"):
                    continue
                if choice in ("s", "skip"):
                    break
                if choice in ("q", "quit"):
                    frames.append({
                        "pose": pose,
                        "observations": observations,
                    })
                    stop_capture = True
                    break

                frames.append({
                    "pose": pose,
                    "observations": observations,
                })
                break

    output_frames = merge_frames(args.output, frames) if args.merge_output else frames
    save_json(args.output, {
        "description": "Known 3D drone poses with raw LH2 LFSR observations.",
        "points_file": args.points,
        "basestations": basestations,
        "sensors": sensors,
        "frames": output_frames,
    })

    print()
    print("=" * 70)
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
