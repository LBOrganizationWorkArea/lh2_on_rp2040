import argparse
import time

import numpy as np
import serial

from lh2v4 import (
    apply_homography,
    collect_window,
    lfsr_pair_to_image,
    load_json,
    median_observations,
    robust_median_xy,
)


def estimate_center(observations, maps, reject_radius):
    candidates = []
    labels = []

    basestations = sorted({str(int(obs["basestation"])) for obs in observations})

    for bs in basestations:
        if bs not in maps["basestations"]:
            continue

        bs_map = maps["basestations"][bs]
        image_points = []

        for obs in observations:
            if str(int(obs["basestation"])) != bs:
                continue
            image_points.append(lfsr_pair_to_image(obs["lfsr0"], obs["lfsr1"], sweep_swap=bool(bs_map["sweep_swap"])))

        if not image_points:
            continue

        arr = np.array(image_points, dtype=float)
        u = float(np.median(arr[:, 0]))
        v = float(np.median(arr[:, 1]))
        candidates.append(apply_homography(bs_map["H_image_to_world"], u, v))
        labels.append(f"bs{bs}:n{len(image_points)}")

    center, kept = robust_median_xy(candidates, reject_radius)
    return center, kept, labels


def main():
    parser = argparse.ArgumentParser(description="Live drone center x,y from positioningv4 floor maps.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--maps", default="config/floor_maps.json")
    parser.add_argument("--window", type=float, default=0.25)
    parser.add_argument("--basestations", default="auto", help="Example: 4,10. Use auto to keep every detected base station.")
    parser.add_argument("--reject-radius", type=float, default=0.20)
    parser.add_argument("--alpha", type=float, default=0.45)
    args = parser.parse_args()

    maps = load_json(args.maps)
    basestations = None if args.basestations.lower() == "auto" else [int(x) for x in args.basestations.split(",")]

    filt = None

    print("=" * 70)
    print("positioningv4 live x,y")
    print(f"Port: {args.port}")
    print(f"Maps: {args.maps}")
    print("Assumption: drone yaw stays the same as during calibration.")
    print("=" * 70)

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        time.sleep(0.3)

        while True:
            samples = collect_window(ser, args.window, basestations)
            observations = median_observations(samples, min_samples=1)
            center, kept, labels = estimate_center(observations, maps, args.reject_radius)

            if center is None:
                print("waiting for valid LH2 position...")
                continue

            raw = np.array(center, dtype=float)
            if filt is None:
                filt = raw
            else:
                filt = args.alpha * raw + (1.0 - args.alpha) * filt

            print(
                f"x={filt[0]:+.3f} m | y={filt[1]:+.3f} m | "
                f"raw=({raw[0]:+.3f},{raw[1]:+.3f}) | "
                f"used={len(kept)}"
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
