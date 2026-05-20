import argparse
import itertools
import math

import numpy as np

from lh2v4 import apply_homography, fit_homography, lfsr_pair_to_image, load_json, save_json


def build_points(calibration, basestation, sweep_swap):
    image_points = []
    world_points = []
    records = []

    for pose in calibration["poses"]:
        pose_images = []
        for obs in pose["observations"]:
            if int(obs["basestation"]) != int(basestation):
                continue

            u, v = lfsr_pair_to_image(obs["lfsr0"], obs["lfsr1"], sweep_swap=sweep_swap)
            pose_images.append((u, v))

        if not pose_images:
            continue

        # The sensors are very close together. For v4, use them as repeated
        # observations of the drone center instead of trying to calibrate each
        # 6.25 cm offset separately.
        arr = np.array(pose_images, dtype=float)
        u = float(np.median(arr[:, 0]))
        v = float(np.median(arr[:, 1]))
        x = float(pose["x_m"])
        y = float(pose["y_m"])

        image_points.append((u, v))
        world_points.append((x, y))
        records.append({
            "pose": pose["name"],
            "sensor_count": int(len(pose_images)),
            "image": [u, v],
            "world": [x, y],
        })

    return image_points, world_points, records


def residual_stats(H, image_points, world_points):
    errors = []
    for (u, v), (x, y) in zip(image_points, world_points):
        px, py = apply_homography(H, u, v)
        errors.append(math.hypot(px - x, py - y))

    if not errors:
        return {"rmse_m": float("nan"), "median_m": float("nan"), "max_m": float("nan")}

    arr = np.array(errors, dtype=float)
    return {
        "rmse_m": float(math.sqrt(np.mean(arr ** 2))),
        "median_m": float(np.median(arr)),
        "max_m": float(np.max(arr)),
    }


def fit_one_basestation(calibration, basestation):
    candidates = []

    for sweep_swap in (False, True):
        image_points, world_points, records = build_points(calibration, basestation, sweep_swap)
        if len(image_points) < 4:
            continue

        H = fit_homography(image_points, world_points)
        stats = residual_stats(H, image_points, world_points)
        candidates.append({
            "basestation": int(basestation),
            "model": "floor_homography_lh2_center_image_to_xy",
            "sweep_swap": bool(sweep_swap),
            "point_count": int(len(image_points)),
            "H_image_to_world": H.tolist(),
            "fit": stats,
            "records": records,
        })

    if not candidates:
        raise RuntimeError(f"Not enough data for basestation {basestation}.")

    candidates.sort(key=lambda item: item["fit"]["rmse_m"])
    return candidates[0]


def detected_basestations(calibration):
    out = set()
    for pose in calibration["poses"]:
        for obs in pose["observations"]:
            out.add(int(obs["basestation"]))
    return sorted(out)


def main():
    parser = argparse.ArgumentParser(description="Fit 2D floor maps from Lighthouse image coordinates to world x,y.")
    parser.add_argument("--calibration", default="config/floor_calibration.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--output", default="config/floor_maps.json")
    parser.add_argument("--basestations", help="Example: 4,10. Default: use detected basestations.")
    args = parser.parse_args()

    calibration = load_json(args.calibration)

    if args.basestations:
        basestations = [int(x) for x in args.basestations.split(",")]
    else:
        basestations = detected_basestations(calibration)

    print("=" * 70)
    print("Fit positioningv4 floor maps")
    print(f"Calibration: {args.calibration}")
    print(f"Layout:      {args.layout}")
    print(f"Basestations: {basestations}")
    print("=" * 70)

    maps = {}
    for bs in basestations:
        fitted = fit_one_basestation(calibration, bs)
        maps[str(bs)] = {
            key: value for key, value in fitted.items()
            if key != "records"
        }
        fit = fitted["fit"]
        print(
            f"BS{bs}: points={fitted['point_count']} "
            f"sweep_swap={fitted['sweep_swap']} "
            f"rmse={fit['rmse_m']:.3f} m median={fit['median_m']:.3f} m max={fit['max_m']:.3f} m"
        )

    save_json(args.output, {
        "description": "positioningv4 2D floor homographies, one map per Lighthouse base station.",
        "input_calibration": args.calibration,
        "input_layout": args.layout,
        "basestations": maps,
    })

    print("=" * 70)
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
