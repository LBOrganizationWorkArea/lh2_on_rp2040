import argparse
import itertools
import math

import numpy as np

from lh2v4 import (
    apply_homography,
    fit_homography,
    layout_by_sensor,
    lfsr_to_alpha_rad,
    load_json,
    sensor_world_xy,
)


def angle_pair_to_image(alpha0, alpha1, variant):
    if variant["swap"]:
        alpha0, alpha1 = alpha1, alpha0

    alpha0 = variant["sign0"] * alpha0 + variant["offset0"]
    alpha1 = variant["sign1"] * alpha1 + variant["offset1"]

    theta = (alpha0 + alpha1) / 2.0

    if variant["phi_formula"] == "paper_minus":
        numerator = math.sin(((alpha1 - alpha0) / 2.0) - (math.pi / 3.0))
    elif variant["phi_formula"] == "paper_plus":
        numerator = math.sin(((alpha1 - alpha0) / 2.0) + (math.pi / 3.0))
    elif variant["phi_formula"] == "reverse_minus":
        numerator = math.sin(((alpha0 - alpha1) / 2.0) - (math.pi / 3.0))
    else:
        numerator = math.sin(((alpha0 - alpha1) / 2.0) + (math.pi / 3.0))

    denominator = math.tan(math.pi / 6.0) * max(1e-9, math.cos(theta))
    phi = math.atan2(numerator, denominator)

    u = variant["u_sign"] * math.tan(theta)
    v = variant["v_sign"] * math.tan(phi) / max(1e-9, math.cos(theta))
    return float(u), float(v)


def build_correspondences(calibration, layout, basestation, variant):
    sensors = layout_by_sensor(layout)
    image_points = []
    world_points = []

    for pose in calibration["poses"]:
        for obs in pose["observations"]:
            if int(obs["basestation"]) != int(basestation):
                continue

            sensor = int(obs["sensor"])
            if sensor not in sensors:
                continue

            alpha0 = lfsr_to_alpha_rad(obs["lfsr0"])
            alpha1 = lfsr_to_alpha_rad(obs["lfsr1"])
            image_points.append(angle_pair_to_image(alpha0, alpha1, variant))

            xy = sensor_world_xy(pose, sensors[sensor])
            world_points.append((float(xy[0]), float(xy[1])))

    return image_points, world_points


def residual_stats(H, image_points, world_points):
    errors = []
    per_point = []

    for image, world in zip(image_points, world_points):
        pred = apply_homography(H, image[0], image[1])
        err = math.hypot(pred[0] - world[0], pred[1] - world[1])
        errors.append(err)
        per_point.append((err, image, world, pred))

    arr = np.array(errors, dtype=float)
    return {
        "rmse_m": float(math.sqrt(np.mean(arr ** 2))),
        "median_m": float(np.median(arr)),
        "max_m": float(np.max(arr)),
        "per_point": per_point,
    }


def candidate_variants():
    offsets = [0.0]
    formulas = ["paper_minus", "paper_plus", "reverse_minus", "reverse_plus"]

    for swap, sign0, sign1, u_sign, v_sign, formula, offset0, offset1 in itertools.product(
        [False, True],
        [1.0, -1.0],
        [1.0, -1.0],
        [1.0, -1.0],
        [1.0, -1.0],
        formulas,
        offsets,
        offsets,
    ):
        yield {
            "swap": swap,
            "sign0": sign0,
            "sign1": sign1,
            "u_sign": u_sign,
            "v_sign": v_sign,
            "phi_formula": formula,
            "offset0": offset0,
            "offset1": offset1,
        }


def detected_basestations(calibration):
    out = set()
    for pose in calibration["poses"]:
        for obs in pose["observations"]:
            out.add(int(obs["basestation"]))
    return sorted(out)


def main():
    parser = argparse.ArgumentParser(description="Validate LH2 sweep-to-image conversion using known floor calibration points.")
    parser.add_argument("--calibration", default="config/floor_calibration.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--basestations", help="Example: 4,10. Default: detected.")
    parser.add_argument("--top", type=int, default=8)
    args = parser.parse_args()

    calibration = load_json(args.calibration)
    layout = load_json(args.layout)
    basestations = [int(x) for x in args.basestations.split(",")] if args.basestations else detected_basestations(calibration)

    print("=" * 70)
    print("Validate LH2 conversion")
    print("Good conversion should fit the 16 known sensor points with low residual.")
    print("=" * 70)

    for bs in basestations:
        results = []

        for variant in candidate_variants():
            image_points, world_points = build_correspondences(calibration, layout, bs, variant)
            if len(image_points) < 4:
                continue

            try:
                H = fit_homography(image_points, world_points)
                stats = residual_stats(H, image_points, world_points)
            except Exception:
                continue

            results.append((stats["rmse_m"], stats, variant))

        results.sort(key=lambda item: item[0])

        print()
        print(f"BS{bs}: tested={len(results)} candidates")

        for rank, (_rmse, stats, variant) in enumerate(results[: args.top], start=1):
            print(
                f"  #{rank}: rmse={stats['rmse_m']:.3f} m | "
                f"median={stats['median_m']:.3f} m | max={stats['max_m']:.3f} m | "
                f"swap={variant['swap']} sign=({variant['sign0']:+.0f},{variant['sign1']:+.0f}) "
                f"uvsign=({variant['u_sign']:+.0f},{variant['v_sign']:+.0f}) "
                f"formula={variant['phi_formula']}"
            )

        if results:
            best_stats = results[0][1]
            worst = sorted(best_stats["per_point"], key=lambda item: item[0], reverse=True)[:4]
            print("  worst points for best candidate:")
            for err, _image, world, pred in worst:
                print(
                    f"    err={err:.3f} m | "
                    f"world=({world[0]:+.3f},{world[1]:+.3f}) | "
                    f"pred=({pred[0]:+.3f},{pred[1]:+.3f})"
                )


if __name__ == "__main__":
    main()
