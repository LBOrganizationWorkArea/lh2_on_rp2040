import argparse
import time

import serial

from lh2v4 import collect_window, load_json, median_observations, save_json


def main():
    parser = argparse.ArgumentParser(description="Capture LH2 measurements at known drone floor positions.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--poses", default="config/floor_poses.json")
    parser.add_argument("--output", default="config/floor_calibration.json")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--basestations", default="auto", help="Example: 4,10. Use auto to keep every detected base station.")
    parser.add_argument("--min-samples", type=int, default=2)
    args = parser.parse_args()

    pose_file = load_json(args.poses)
    basestations = None if args.basestations.lower() == "auto" else [int(x) for x in args.basestations.split(",")]

    output = {
        "description": "LH2 floor calibration captures. Known drone center positions produce known sensor floor positions.",
        "created_unix_time_s": time.time(),
        "input_poses": args.poses,
        "basestations": basestations if basestations is not None else "auto",
        "duration_s": args.duration,
        "poses": [],
    }

    print("=" * 70)
    print("positioningv4 floor calibration capture")
    print(f"Port: {args.port}")
    print(f"Poses: {args.poses}")
    print(f"Output: {args.output}")
    print("Keep the drone yaw fixed for this first 2D version.")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        time.sleep(0.3)

        for pose in pose_file["poses"]:
            print()
            print("=" * 70)
            print(f"Place drone at {pose['name']}")
            print(f"x={float(pose['x_m']):+.3f} m | y={float(pose['y_m']):+.3f} m | yaw={float(pose.get('yaw_deg', 0.0)):+.1f} deg")
            input("Press ENTER when still...")

            ser.reset_input_buffer()
            samples = collect_window(ser, args.duration, basestations)
            observations = median_observations(samples, min_samples=args.min_samples)

            print(f"Captured usable sensor observations: {len(observations)}")
            if not observations:
                print("  No usable observation. Check port, Lighthouse visibility, or run tools/00_check_serial.py.")
            for obs in observations:
                print(
                    f"  bs={obs['basestation']} sensor={obs['sensor']} "
                    f"n0={obs['samples0']} n1={obs['samples1']}"
                )

            output["poses"].append({
                "name": pose["name"],
                "x_m": float(pose["x_m"]),
                "y_m": float(pose["y_m"]),
                "yaw_deg": float(pose.get("yaw_deg", 0.0)),
                "observations": observations,
            })

    save_json(args.output, output)
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
