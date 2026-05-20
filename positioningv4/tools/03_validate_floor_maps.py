import argparse
import math

import numpy as np

from lh2v4 import (
    apply_homography,
    lfsr_pair_to_image,
    load_json,
    robust_median_xy,
)


def estimate_pose_center(pose, maps, reject_radius):
    center_candidates = []
    details = []

    basestations = sorted({str(int(obs["basestation"])) for obs in pose["observations"]})

    for bs in basestations:
        if bs not in maps["basestations"]:
            continue

        bs_map = maps["basestations"][bs]
        image_points = []

        for obs in pose["observations"]:
            if str(int(obs["basestation"])) != bs:
                continue

            image_points.append(lfsr_pair_to_image(obs["lfsr0"], obs["lfsr1"], sweep_swap=bool(bs_map["sweep_swap"])))

        if not image_points:
            continue

        arr = np.array(image_points, dtype=float)
        u = float(np.median(arr[:, 0]))
        v = float(np.median(arr[:, 1]))

        cx, cy = apply_homography(bs_map["H_image_to_world"], u, v)

        center_candidates.append((cx, cy))
        details.append({"bs": int(bs), "sensor_count": int(len(image_points)), "center": [cx, cy]})

    center, kept = robust_median_xy(center_candidates, reject_radius)
    return center, kept, details


def main():
    parser = argparse.ArgumentParser(description="Validate positioningv4 floor maps on the captured calibration poses.")
    parser.add_argument("--calibration", default="config/floor_calibration.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--maps", default="config/floor_maps.json")
    parser.add_argument("--reject-radius", type=float, default=0.20)
    args = parser.parse_args()

    calibration = load_json(args.calibration)
    maps = load_json(args.maps)

    errors = []
    print("=" * 70)
    print("Validate positioningv4 floor maps")
    print("=" * 70)

    for pose in calibration["poses"]:
        center, kept, _details = estimate_pose_center(pose, maps, args.reject_radius)
        if center is None:
            print(f"{pose['name']}: no estimate")
            continue

        x = float(pose["x_m"])
        y = float(pose["y_m"])
        err = math.hypot(center[0] - x, center[1] - y)
        errors.append(err)
        print(
            f"{pose['name']}: "
            f"est=({center[0]:+.3f},{center[1]:+.3f}) m | "
            f"expected=({x:+.3f},{y:+.3f}) m | "
            f"err={err:.3f} m | used={len(kept)}"
        )

    if errors:
        arr = np.array(errors, dtype=float)
        print("=" * 70)
        print(
            f"mean={float(np.mean(arr)):.3f} m | "
            f"median={float(np.median(arr)):.3f} m | "
            f"max={float(np.max(arr)):.3f} m"
        )
        print("=" * 70)


if __name__ == "__main__":
    main()
