#!/usr/bin/env python3

import argparse
from collections import deque
import json
import threading
import time
from pathlib import Path

import serial

from lh2_factory_model import load_factory_calibration_map
from lh2v10 import frame_axes, frame_to_observations, observation_quality_counts, parse_lh2p_line, select_clean_lh2p_frames, summarize_observation_buffer


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


class SerialLineReader:
    def __init__(self, ser, max_lines=20000):
        self.ser = ser
        self.lines = deque(maxlen=max_lines)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.total_lines = 0
        self.last_line = None
        self.last_error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                self.last_error = str(exc)
                break
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if not line:
                continue
            with self.lock:
                self.lines.append(line)
                self.total_lines += 1
                self.last_line = line

    def clear(self):
        with self.lock:
            self.lines.clear()

    def snapshot(self):
        with self.lock:
            return {
                "alive": self.thread.is_alive(),
                "queued": len(self.lines),
                "total_lines": self.total_lines,
                "last_line": self.last_line,
                "last_error": self.last_error,
            }

    def read_line(self, timeout_s=0.1):
        end = time.time() + timeout_s
        while time.time() < end:
            with self.lock:
                if self.lines:
                    return self.lines.popleft()
            time.sleep(0.005)
        return None

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)


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


def capture_pose(reader, duration_s, basestations, factory_calibs=None, max_sensor_spread_ticks=None, angle_outlier_deg=8.0, min_channel_samples=1):
    buffer = {}
    lh2p_frames = []
    stats = {
        "raw_lh2p": 0,
        "parsed_lh2p": 0,
        "selected_bs": 0,
        "valid_poly": 0,
        "clean_lh2p": 0,
        "heartbeat": 0,
        "last_heartbeat": None,
        "reader_before": reader.snapshot(),
        "reader_after": None,
    }

    start = time.time()

    while time.time() - start < duration_s:
        raw = reader.read_line(timeout_s=0.1)
        if raw is None:
            continue
        if raw.startswith("HB;"):
            stats["heartbeat"] += 1
            stats["last_heartbeat"] = raw
            continue
        if raw.startswith("LH2P;"):
            stats["raw_lh2p"] += 1

        frame = parse_lh2p_line(raw)
        if frame is not None:
            stats["parsed_lh2p"] += 1
            if frame.basestation in basestations:
                stats["selected_bs"] += 1
                lh2p_frames.append(frame)
            continue

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

    stats["reader_after"] = reader.snapshot()

    if lh2p_frames:
        clean_frames = select_clean_lh2p_frames(lh2p_frames, max_sensor_spread_ticks=max_sensor_spread_ticks)
        stats["valid_poly"] = sum(1 for frame in lh2p_frames if frame_axes(frame) is not None)
        stats["clean_lh2p"] = len(clean_frames)
        buffer = {}
        for frame in clean_frames:
            for data in frame_to_observations(frame, factory_calibs):
                key = (data["sensor"], data["basestation"], data["sweep"])
                buffer.setdefault(key, []).append(data)

    measurements = summarize_observation_buffer(
        buffer,
        angle_outlier_deg=angle_outlier_deg,
        min_samples=min_channel_samples,
    )
    return measurements, stats


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


def check_capture_quality(measurements, min_observations, min_sensors, min_basestations):
    counts = observation_quality_counts(measurements)
    warnings = []
    if counts["channels"] < min_observations:
        warnings.append(f"only {counts['channels']} channels, need {min_observations}")
    if counts["sensors"] < min_sensors:
        warnings.append(f"only {counts['sensors']} sensors, need {min_sensors}")
    if counts["basestations"] < min_basestations:
        warnings.append(f"only {counts['basestations']} basestations, need {min_basestations}")
    return counts, warnings


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


def load_existing_calibration(path):
    if not path.is_file():
        return None

    with open(path, "r") as f:
        data = json.load(f)

    if not isinstance(data, dict) or "poses" not in data:
        raise ValueError(f"Existing calibration file has an unexpected format: {path}")

    return data


def upsert_pose(calibration, pose_entry):
    pose_name = pose_entry["name"]
    for index, existing in enumerate(calibration["poses"]):
        if existing.get("name") == pose_name:
            calibration["poses"][index] = pose_entry
            return "replaced"

    calibration["poses"].append(pose_entry)
    return "added"


def save_calibration(path, calibration):
    with open(path, "w") as f:
        json.dump(calibration, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Capture known calibration poses.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--output", default="config/calibration_poses_2d.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument(
        "--max-sensor-spread-ticks",
        type=int,
        default=0,
        help="Optional internal LH2P frame spread filter. 0 disables it.",
    )
    parser.add_argument(
        "--factory-calibs",
        default="auto",
        help="Factory calibration JSON map used for LH2P offsets. Use 'none' to disable or '4=path,10=path'.",
    )
    parser.add_argument("--angle-outlier-deg", type=float, default=8.0, help="Reject per-channel angle samples farther than this from the robust center. 0 disables.")
    parser.add_argument("--min-channel-samples", type=int, default=1, help="Minimum kept samples required for a channel.")
    parser.add_argument("--min-observations", type=int, default=6, help="Warn when a pose has fewer channels than this. You can still keep it.")
    parser.add_argument("--min-sensors", type=int, default=2, help="Warn when a pose sees fewer sensors than this. You can still keep it.")
    parser.add_argument("--min-basestations", type=int, default=1, help="Warn when a pose sees fewer basestations than this. Use 2 for bridge poses.")
    parser.add_argument("--pose-file", help="Optional JSON file with poses containing x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg.")
    parser.add_argument("--resume", action="store_true", help="Load the existing output file and replace captured poses instead of starting from an empty file.")
    parser.add_argument(
        "--only",
        help="Comma-separated pose names to capture, for example P3_front_40cm,P7_back_left.",
    )
    args = parser.parse_args()

    basestations = [int(x) for x in args.basestations.split(",")]
    factory_calibs = load_factory_calibration_map(args.factory_calibs)
    calibration_poses = load_pose_list(args.pose_file)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Calibration capture")
    print(f"Port: {args.port}")
    print(f"Basestations: {basestations}")
    if factory_calibs:
        loaded = ", ".join(f"BS{bs}:{entry['path']}" for bs, entry in sorted(factory_calibs.items()))
        print(f"Factory: {loaded}")
    else:
        print("Factory: disabled/not found")
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

    if args.only:
        wanted_names = {name.strip() for name in args.only.split(",") if name.strip()}
        calibration_poses = [pose for pose in calibration_poses if pose["name"] in wanted_names]
        missing_names = sorted(wanted_names - {pose["name"] for pose in calibration_poses})
        if missing_names:
            raise SystemExit(f"Unknown pose name(s): {', '.join(missing_names)}")

    calibration = load_existing_calibration(output_path) if args.resume else None
    if calibration is None:
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
    else:
        calibration["basestations"] = basestations
        calibration["duration_s_per_pose"] = args.duration

    with serial.Serial(args.port, args.baudrate, timeout=0.1) as ser:
        reader = SerialLineReader(ser)
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

            while True:
                reader.clear()
                measurements, stats = capture_pose(
                    reader,
                    args.duration,
                    basestations,
                    factory_calibs,
                    args.max_sensor_spread_ticks if args.max_sensor_spread_ticks > 0 else None,
                    args.angle_outlier_deg,
                    args.min_channel_samples,
                )
                missing = check_required_channels(measurements, basestations)
                quality_counts, quality_warnings = check_capture_quality(
                    measurements,
                    args.min_observations,
                    args.min_sensors,
                    args.min_basestations,
                )

                print(f"Captured measurements: {len(measurements)}")
                print(
                    "Quality: "
                    f"sensors={quality_counts['sensors']}/4 | "
                    f"basestations={quality_counts['basestations']}/{len(basestations)} | "
                    f"channels={quality_counts['channels']} | "
                    f"samples={quality_counts['samples']} | "
                    f"rejected={quality_counts['rejected']}"
                )
                print(
                    "Capture debug: "
                    f"HB={stats['heartbeat']} | "
                    f"LH2P raw={stats['raw_lh2p']} | "
                    f"parsed={stats['parsed_lh2p']} | "
                    f"selected_bs={stats['selected_bs']} | "
                    f"valid_poly={stats['valid_poly']} | "
                    f"clean={stats['clean_lh2p']}"
                )
                before = stats["reader_before"]
                after = stats["reader_after"]
                print(
                    "Reader debug: "
                    f"alive={after['alive']} | "
                    f"queued_before={before['queued']} | "
                    f"queued_after={after['queued']} | "
                    f"total_before={before['total_lines']} | "
                    f"total_after={after['total_lines']}"
                )
                if after["last_error"]:
                    print(f"Reader error: {after['last_error']}")
                if after["last_line"]:
                    print(f"Reader last line: {after['last_line']}")
                if stats["last_heartbeat"]:
                    print(f"Last heartbeat: {stats['last_heartbeat']}")

                if missing:
                    print("WARNING: Missing channels:")
                    for m in missing:
                        print(f"  sensor={m[0]} bs={m[1]} sweep={m[2]}")
                else:
                    print("All 16 required channels captured.")
                if quality_warnings:
                    print("WARNING: Weak partial capture:")
                    for warning in quality_warnings:
                        print(f"  {warning}")

                choice = input("Press ENTER to keep, r + ENTER to retry this pose, s + ENTER to skip: ").strip().lower()
                if choice == "r":
                    print("Retrying this pose...")
                    continue
                if choice == "s":
                    print(f"Skipped: {pose['name']}")
                    measurements = None
                    break
                if choice == "":
                    break
                print("Unknown choice. Keeping this pose.")
                break

            if measurements is None:
                continue

            pose_entry = {
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
            }
            action = upsert_pose(calibration, pose_entry)
            print(f"Pose {action}: {pose['name']}")
            save_calibration(output_path, calibration)
            print(f"Progress saved to: {output_path}")

        reader.stop()

    save_calibration(output_path, calibration)

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
