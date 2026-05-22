#!/usr/bin/env python3

import argparse
from collections import deque
import importlib.util
import json
import threading
import time
from pathlib import Path

import serial

from lh2_factory_model import load_factory_calibration_map


def load_capture_module():
    path = Path(__file__).resolve().parent / "03_capture_calibration_poses.py"
    spec = importlib.util.spec_from_file_location("capture_calibration_poses_v10", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(description="Record moving wand wave frames with the current v10 LH2P parser.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--window", type=float, default=0.20, help="Time window used to summarize each wave frame.")
    parser.add_argument("--period", type=float, default=0.25, help="Time between saved wave frames.")
    parser.add_argument("--output", default="config/wand_wave_record.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--factory-calibs", default="auto")
    parser.add_argument("--max-sensor-spread-ticks", type=int, default=0)
    parser.add_argument("--max-sensor-spread-deg", type=float, default=0.0)
    parser.add_argument("--angle-source", choices=["offsets", "lfsr", "auto"], default="offsets")
    parser.add_argument("--angle-outlier-deg", type=float, default=8.0)
    parser.add_argument("--min-channel-samples", type=int, default=1)
    parser.add_argument("--min-observations", type=int, default=6)
    parser.add_argument("--min-sensors", type=int, default=2)
    parser.add_argument("--min-basestations", type=int, default=1)
    args = parser.parse_args()

    capture = load_capture_module()
    basestations = [int(x) for x in args.basestations.split(",")]
    factory_calibs = load_factory_calibration_map(args.factory_calibs)

    frames = []
    start = time.time()
    next_frame = start

    print("=" * 70)
    print("Record wand wave")
    print(f"Duration: {args.duration:.1f} s")
    print(f"Output: {args.output}")
    print("Move the drone/wand through the volume. Points still anchor the world frame; this records dense moving constraints.")
    print("=" * 70)

    with serial.Serial(args.port, args.baudrate, timeout=0.05) as ser:
        reader = SerialLineReader(ser)
        try:
            while time.time() - start < args.duration:
                now = time.time()
                if now < next_frame:
                    time.sleep(0.01)
                    continue

                measurements, stats = capture.capture_pose(
                    reader,
                    args.window,
                    basestations,
                    factory_calibs,
                    args.max_sensor_spread_ticks if args.max_sensor_spread_ticks > 0 else None,
                    args.angle_outlier_deg,
                    args.min_channel_samples,
                    args.angle_source,
                    args.max_sensor_spread_deg if args.max_sensor_spread_deg > 0 else None,
                )
                counts, warnings = capture.check_capture_quality(
                    measurements,
                    args.min_observations,
                    args.min_sensors,
                    args.min_basestations,
                )

                if not warnings:
                    frames.append({
                        "pc_time_s": float(now),
                        "measurements": measurements,
                        "quality": counts,
                    })

                print(
                    f"\rframes={len(frames)} "
                    f"channels={counts['channels']} sensors={counts['sensors']} bs={counts['basestations']} "
                    f"raw_lh2p={stats['raw_lh2p']} clean={stats['clean_lh2p']}",
                    end="",
                    flush=True,
                )
                next_frame = now + args.period
        finally:
            reader.stop()

    save_json(args.output, {
        "description": "Moving wand wave record captured with v10 LH2P/factory parser. Use together with known point captures to validate or refine coverage.",
        "created_unix_time_s": time.time(),
        "basestations": basestations,
        "duration_s": args.duration,
        "window_s": args.window,
        "period_s": args.period,
        "frames": frames,
    })

    print()
    print("=" * 70)
    print(f"Saved {len(frames)} wave frames to: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
