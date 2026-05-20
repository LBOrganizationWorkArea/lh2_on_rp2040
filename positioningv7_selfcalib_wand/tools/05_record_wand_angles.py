import argparse
import time

import serial

from wand_common import (
    load_angle_modes,
    load_latest_coefficients,
    new_state,
    parse_lh2_line,
    save_json,
    update_state_from_lh2,
)


def fresh_angles(state, basestations, sensor_order, max_age_s):
    now = time.time()
    angles = {}

    for bs in basestations:
        bs_angles = {}
        for sensor in sensor_order:
            item = state[sensor][bs]
            if item["az"] is None or item["el"] is None or now - item["age"] > max_age_s:
                return None

            bs_angles[str(sensor)] = {
                "az_deg": float(item["az"]),
                "el_deg": float(item["el"]),
                "mode": item["mode"],
            }

        angles[str(bs)] = bs_angles

    return angles


def main():
    parser = argparse.ArgumentParser(description="Record calibrated LH2 angle frames for direct wand self-calibration.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--modes", default="config/angle_modes.json")
    parser.add_argument("--output", default="data/wand_angles_record.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--period", type=float, default=0.10)
    parser.add_argument("--max-age", type=float, default=0.60)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    coeffs = load_latest_coefficients(args.history)
    angle_modes = load_angle_modes(args.modes)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensor_order = [int(x) for x in args.sensors.split(",")]
    state = new_state(sensor_order, basestations)

    frames = []
    start = time.time()
    last_frame = 0.0
    last_debug = 0.0
    updates = 0

    print("=" * 70)
    print("Record wand angle frames")
    print(f"Duration: {args.duration:.1f} s")
    print("Move in 3D and keep both Lighthouses seeing all 4 sensors.")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        while time.time() - start < args.duration:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is not None and update_state_from_lh2(state, data, coeffs, angle_modes):
                updates += 1

            now = time.time()
            if now - last_frame >= args.period:
                angles = fresh_angles(state, basestations, sensor_order, args.max_age)
                if angles is not None:
                    frames.append({
                        "pc_time_s": now,
                        "angles": angles,
                    })
                    print(f"\rframes={len(frames)}", end="", flush=True)
                last_frame = now

            if args.debug and now - last_debug >= 1.0:
                fresh = {}
                for bs in basestations:
                    fresh[str(bs)] = []
                    for sensor in sensor_order:
                        item = state[sensor][bs]
                        if item["az"] is not None and item["el"] is not None and now - item["age"] <= args.max_age:
                            fresh[str(bs)].append(sensor)
                print()
                print(f"debug updates={updates} frames={len(frames)} fresh={fresh}")
                last_debug = now

    save_json(args.output, {
        "description": "Calibrated LH2 azimuth/elevation frames for direct self-calibration.",
        "basestations": basestations,
        "sensors": sensor_order,
        "angle_modes": angle_modes,
        "duration_s": args.duration,
        "frames": frames,
    })

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
