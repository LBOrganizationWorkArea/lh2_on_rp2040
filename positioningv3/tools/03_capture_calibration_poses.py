#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from statistics import median

import serial


CALIBRATION_POSES = [
    {"name": "P0_center", "x_m": 0.00, "y_m": 0.00, "z_m": 0.00, "yaw_deg": 0.0},

    {"name": "P1_right_40cm", "x_m": 0.40, "y_m": 0.00, "z_m": 0.00, "yaw_deg": 0.0},
    {"name": "P2_left_40cm", "x_m": -0.40, "y_m": 0.00, "z_m": 0.00, "yaw_deg": 0.0},
    {"name": "P3_front_40cm", "x_m": 0.00, "y_m": 0.40, "z_m": 0.00, "yaw_deg": 0.0},
    {"name": "P4_back_40cm", "x_m": 0.00, "y_m": -0.40, "z_m": 0.00, "yaw_deg": 0.0},

    {"name": "P5_front_right", "x_m": 0.40, "y_m": 0.40, "z_m": 0.00, "yaw_deg": 0.0},
    {"name": "P6_front_left", "x_m": -0.40, "y_m": 0.40, "z_m": 0.00, "yaw_deg": 0.0},
    {"name": "P7_back_left", "x_m": -0.40, "y_m": -0.40, "z_m": 0.00, "yaw_deg": 0.0},
    {"name": "P8_back_right", "x_m": 0.40, "y_m": -0.40, "z_m": 0.00, "yaw_deg": 0.0},
]


def parse_lh2_line(line):
    line = line.strip()

    if line.startswith("LH2A,"):
        parts = line.split(",")
        try:
            if len(parts) == 7:
                return {
                    "time_us": None,
                    "sensor": int(parts[1]),
                    "sweep": int(parts[2]),
                    "basestation": int(parts[3]),
                    "polynomial": int(parts[4]),
                    "lfsr_location": int(parts[5]),
                    "raw_angle_rad": int(parts[6]) / 1000000.0,
                }
            if len(parts) == 8:
                return {
                    "time_us": int(parts[1]),
                    "sensor": int(parts[2]),
                    "sweep": int(parts[3]),
                    "basestation": int(parts[4]),
                    "polynomial": int(parts[5]),
                    "lfsr_location": int(parts[6]),
                    "raw_angle_rad": int(parts[7]) / 1000000.0,
                }
        except ValueError:
            return None

        return None

    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    try:
        if len(parts) == 6:
            return {
                "time_us": None,
                "sensor": int(parts[1]),
                "sweep": int(parts[2]),
                "basestation": int(parts[3]),
                "polynomial": int(parts[4]),
                "lfsr_location": int(parts[5]),
            }
        if len(parts) == 7:
            return {
                "time_us": int(parts[1]),
                "sensor": int(parts[2]),
                "sweep": int(parts[3]),
                "basestation": int(parts[4]),
                "polynomial": int(parts[5]),
                "lfsr_location": int(parts[6]),
            }
    except ValueError:
        return None

    return None


def capture_pose(ser, duration_s, basestations):
    buffer = {}

    start = time.time()

    while time.time() - start < duration_s:
        raw = ser.readline().decode(errors="ignore").strip()
        data = parse_lh2_line(raw)

        if data is None:
            continue

        if data["basestation"] not in basestations:
            continue

        key = (
            data["sensor"],
            data["basestation"],
            data["sweep"],
        )

        buffer.setdefault(key, []).append(data)

    measurements = []

    for (sensor, bs, sweep), values in sorted(buffer.items()):
        lfsr_values = [item["lfsr_location"] for item in values]
        raw_angle_values = [
            item["raw_angle_rad"]
            for item in values
            if "raw_angle_rad" in item
        ]

        item = {
            "sensor": int(sensor),
            "basestation": int(bs),
            "sweep": int(sweep),
            "median_lfsr_location": float(median(lfsr_values)),
            "sample_count": int(len(values)),
        }

        if raw_angle_values:
            item["raw_angle_rad"] = float(median(raw_angle_values))

        measurements.append({
            **item
        })

    return measurements


def check_required_channels(measurements, basestations):
    found = {
        (int(m["sensor"]), int(m["basestation"]), int(m["sweep"]))
        for m in measurements
    }

    missing = []

    for sensor in range(4):
        for bs in basestations:
            for sweep in range(2):
                key = (sensor, bs, sweep)
                if key not in found:
                    missing.append(key)

    return missing


def load_pose_list(path):
    if path is None:
        return CALIBRATION_POSES

    with open(path, "r") as f:
        data = json.load(f)

    poses = data["poses"] if isinstance(data, dict) else data
    required = {"name", "x_m", "y_m"}

    for pose in poses:
        missing = sorted(required - set(pose))
        if missing:
            raise ValueError(f"Pose {pose} is missing fields: {missing}")

    return poses


def main():
    parser = argparse.ArgumentParser(description="Capture known calibration poses.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--output", default="config/calibration_poses_2d.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--pose-file", help="Optional JSON file with poses containing x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg.")
    args = parser.parse_args()

    basestations = [int(x) for x in args.basestations.split(",")]
    calibration_poses = load_pose_list(args.pose_file)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Calibration capture")
    print(f"Port: {args.port}")
    print(f"Basestations: {basestations}")
    print(f"Duration per pose: {args.duration:.1f} s")
    print()
    print("IMPORTANT:")
    print("- Place the Lighthouses freely, then do not move them.")
    print("- P0 is the origin.")
    print("- Use many poses if you want full 3D positioning.")
    print("- Include different heights or mocap poses to remove geometry ambiguity.")
    print()
    print("Pattern:")
    print("          P3_front_30cm")
    print("                |")
    print("P2_left_30cm -- P0_center -- P1_right_30cm")
    print("                |")
    print("          P4_back_30cm")
    print("=" * 70)

    calibration = {
        "description": "Known 2D calibration poses for estimating Lighthouse geometry.",
        "created_unix_time_s": time.time(),
        "basestations": basestations,
        "duration_s_per_pose": args.duration,
        "frame": {
            "origin": "P0_center, drone center",
            "x_positive": "right from initial drone orientation",
            "y_positive": "front from initial drone orientation",
            "yaw": "kept fixed at 0 deg during calibration"
        },
        "poses": []
    }

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        for pose in calibration_poses:
            print()
            print("=" * 70)
            print(f"Move drone to: {pose['name']}")
            print(
                f"x={float(pose['x_m']):+.2f} m | "
                f"y={float(pose['y_m']):+.2f} m | "
                f"z={float(pose.get('z_m', 0.0)):+.2f} m | "
                f"roll={float(pose.get('roll_deg', 0.0)):+.1f} deg | "
                f"pitch={float(pose.get('pitch_deg', 0.0)):+.1f} deg | "
                f"yaw={float(pose.get('yaw_deg', 0.0)):+.1f} deg"
            )
            print("Keep the drone still.")
            input("Press ENTER to capture this pose...")

            measurements = capture_pose(ser, args.duration, basestations)
            missing = check_required_channels(measurements, basestations)

            print(f"Captured measurements: {len(measurements)}")

            if missing:
                print("WARNING: Missing channels:")
                for m in missing:
                    print(f"  sensor={m[0]} bs={m[1]} sweep={m[2]}")
            else:
                print("All 16 required channels captured.")

            calibration["poses"].append({
                "name": pose["name"],
                "x_m": float(pose["x_m"]),
                "y_m": float(pose["y_m"]),
                "z_m": float(pose.get("z_m", 0.0)),
                "roll_deg": float(pose.get("roll_deg", 0.0)),
                "pitch_deg": float(pose.get("pitch_deg", 0.0)),
                "yaw_deg": float(pose.get("yaw_deg", 0.0)),
                "measurements": measurements,
                "missing_channels": [
                    {"sensor": m[0], "basestation": m[1], "sweep": m[2]}
                    for m in missing
                ]
            })

    with open(output_path, "w") as f:
        json.dump(calibration, f, indent=2)

    print()
    print("=" * 70)
    print(f"Saved calibration poses to: {output_path}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
