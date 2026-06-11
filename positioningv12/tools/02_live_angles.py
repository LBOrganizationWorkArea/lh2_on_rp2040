#!/usr/bin/env python3

import argparse
import math
import time
from statistics import median

import serial

from lh2_factory_model import load_factory_calibration_map
from lh2v10 import POLY_TO_AXIS, frame_axes, frame_to_observations, parse_lh2p_line, select_clean_lh2p_frames, summarize_observation_buffer


TICKS_PER_REV = 120000
DEFAULT_BASESTATIONS = [4, 10]


def lfsr_to_deg(lfsr_location, sweep):
    angle = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 360.0) - 180.0
    if int(sweep) == 0:
        return angle + 60.0
    return angle - 60.0


def parse_lh2_line(line):
    line = line.strip()

    if line.startswith("LH2R,"):
        parts = line.split(",")
        try:
            if len(parts) == 12:
                polynomial = int(parts[6])
                axis = POLY_TO_AXIS.get(polynomial)
                if axis is None:
                    return None
                lfsr_location = int(parts[9])
                return {
                    "time_us": int(parts[1]),
                    "timestamp_24": int(parts[2]),
                    "sensor": int(parts[3]),
                    "raw_sweep": int(parts[4]),
                    "sweep": int(axis),
                    "basestation": int(parts[5]),
                    "polynomial": polynomial,
                    "bit_offset": int(parts[7]),
                    "lfsr_bits": int(parts[8]),
                    "lfsr_location": lfsr_location,
                    "offset_ticks": int(parts[10]),
                    "timestamp0_24": int(parts[11]),
                    "raw_angle_rad": math.radians(lfsr_to_deg(lfsr_location, axis)),
                }
        except ValueError:
            return None

        return None

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
                    "raw_angle_deg": math.degrees(int(parts[6]) / 1000000.0),
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
                    "raw_angle_deg": math.degrees(int(parts[7]) / 1000000.0),
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


def main():
    parser = argparse.ArgumentParser(description="Live Lighthouse angles.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0, help="Keep displaying the last clean value for sparse base stations.")
    parser.add_argument("--debug", action="store_true", help="Print LH2P receive/filter counters for diagnostics.")
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
    parser.add_argument("--min-channel-samples", type=int, default=1, help="Minimum kept samples required to display a channel.")
    parser.add_argument("--prefer-direct-lh2a", action="store_true", help="Display direct LH2A angle frames instead of replacing them with paired LH2P/LH2P2 frames.")
    args = parser.parse_args()

    basestations = [int(x) for x in args.basestations.split(",")]
    factory_calibs = load_factory_calibration_map(args.factory_calibs)

    print("=" * 70)
    print("Live LH2 angles")
    print(f"Port: {args.port}")
    print(f"Basestations: {basestations}")
    if factory_calibs:
        loaded = ", ".join(f"BS{bs}:{entry['path']}" for bs, entry in sorted(factory_calibs.items()))
        print(f"Factory: {loaded}")
    else:
        print("Factory: disabled/not found")
    print("Need sensors 0,1,2,3 with sweep 0 and 1 for both basestations.")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    buffer = {}
    last_values = {}
    lh2p_frames = []
    raw_lh2p_count = 0
    raw_lh2a_count = 0
    raw_lh2r_count = 0
    parsed_lh2p_count = 0
    wanted_lh2p_count = 0
    last_print = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            try:
                raw = ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as exc:
                print()
                print(f"Serial disconnected or reset: {exc}")
                print("Reset/replug the Pico, then rerun the command.")
                break
            if raw.startswith("LH2P;") or raw.startswith("LH2P2;"):
                raw_lh2p_count += 1
            if raw.startswith("LH2A,"):
                raw_lh2a_count += 1
            if raw.startswith("LH2R,"):
                raw_lh2r_count += 1
            frame = parse_lh2p_line(raw)
            if frame is not None:
                parsed_lh2p_count += 1
                if frame.basestation in basestations:
                    wanted_lh2p_count += 1
                    lh2p_frames.append(frame)
                data = None
            else:
                data = parse_lh2_line(raw)

            if data is None:
                now = time.time()
            else:
                if data["basestation"] not in basestations:
                    continue

                key = (
                    data["sensor"],
                    data["basestation"],
                    data["sweep"],
                )

                buffer.setdefault(key, []).append(data)
                now = time.time()

            if now - last_print < args.window:
                continue

            use_paired_frames = lh2p_frames and not (args.prefer_direct_lh2a and raw_lh2a_count > 0)
            if use_paired_frames:
                max_spread = args.max_sensor_spread_ticks if args.max_sensor_spread_ticks > 0 else None
                clean_frames = select_clean_lh2p_frames(lh2p_frames, max_sensor_spread_ticks=max_spread)
                buffer = {}
                for frame in clean_frames:
                    for item in frame_to_observations(frame, factory_calibs):
                        key = (item["sensor"], item["basestation"], item["sweep"])
                        buffer.setdefault(key, []).append(item)

            print()
            print(f"--- angles {now:.2f} ---")
            if args.debug:
                valid_poly_count = sum(1 for frame in lh2p_frames if frame_axes(frame) is not None)
                max_spread = args.max_sensor_spread_ticks if args.max_sensor_spread_ticks > 0 else None
                clean_count = len(select_clean_lh2p_frames(lh2p_frames, max_sensor_spread_ticks=max_spread)) if lh2p_frames else 0
                print(
                    "debug | "
                    f"raw_lh2p={raw_lh2p_count} | "
                    f"raw_lh2a={raw_lh2a_count} | "
                    f"raw_lh2r={raw_lh2r_count} | "
                    f"parsed={parsed_lh2p_count} | "
                    f"selected_bs={wanted_lh2p_count} | "
                    f"valid_poly={valid_poly_count} | "
                    f"clean={clean_count}"
                )

            summaries = summarize_observation_buffer(
                buffer,
                angle_outlier_deg=args.angle_outlier_deg,
                min_samples=args.min_channel_samples,
            )
            summary_by_key = {
                (item["sensor"], item["basestation"], item["sweep"]): item
                for item in summaries
            }

            if args.debug:
                expected_keys = [
                    (sensor, bs, sweep)
                    for sensor in range(4)
                    for bs in basestations
                    for sweep in range(2)
                ]
                present_count = sum(1 for key in expected_keys if key in summary_by_key)
                if args.prefer_direct_lh2a and raw_lh2a_count > 0:
                    stream_mode = "direct_lh2a"
                elif parsed_lh2p_count > 0:
                    stream_mode = "paired_lh2p"
                elif raw_lh2a_count > 0:
                    stream_mode = "direct_lh2a"
                elif raw_lh2r_count > 0:
                    stream_mode = "raw_lh2r"
                else:
                    stream_mode = "no_lh2"
                print(
                    "coverage | "
                    f"mode={stream_mode} | "
                    f"channels={present_count}/{len(expected_keys)}"
                )

            for key, item in summary_by_key.items():
                last_values[key] = (now, dict(item))

            for sensor in range(4):
                for bs in basestations:
                    for sweep in range(2):
                        key = (sensor, bs, sweep)
                        item = summary_by_key.get(key)
                        held_age = None

                        if item is None and key in last_values:
                            last_time, held_item = last_values[key]
                            age = now - last_time
                            if age <= args.hold_seconds:
                                item = held_item
                                held_age = age

                        if item is None:
                            print(f"sensor={sensor} | bs={bs} | sweep={sweep} | MISSING")
                            continue

                        med_lfsr = float(item.get("lfsr_location", item.get("median_lfsr_location", 0.0)))
                        if "calibrated_angle_rad" in item:
                            angle_deg = math.degrees(float(item["calibrated_angle_rad"]))
                        elif "raw_angle_rad" in item:
                            angle_deg = math.degrees(float(item["raw_angle_rad"]))
                        else:
                            angle_deg = lfsr_to_deg(med_lfsr, sweep)

                        print(
                            f"sensor={sensor} | "
                            f"bs={bs} | "
                            f"sweep={sweep} | "
                            f"lfsr={med_lfsr:.0f} | "
                            f"angle={angle_deg:+.3f} deg | "
                            f"n={int(item.get('sample_count', 1))}"
                            + (f"/{int(item['raw_sample_count'])}" if int(item.get("raw_sample_count", item.get("sample_count", 1))) != int(item.get("sample_count", 1)) else "")
                            + (f" | rej={int(item['rejected_count'])}" if int(item.get("rejected_count", 0)) else "")
                            + (f" | spread={float(item['angle_spread_deg']):.2f}deg" if "angle_spread_deg" in item else "")
                            + (f" | held={held_age:.1f}s" if held_age is not None else "")
                        )

            buffer.clear()
            lh2p_frames.clear()
            raw_lh2p_count = 0
            raw_lh2a_count = 0
            raw_lh2r_count = 0
            parsed_lh2p_count = 0
            wanted_lh2p_count = 0
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
