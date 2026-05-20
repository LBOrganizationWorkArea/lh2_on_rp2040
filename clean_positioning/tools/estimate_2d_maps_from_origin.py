# -*- coding: utf-8 -*-

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


TICKS_PER_REV = 833333

# Sweep swap found with the sweep-swap PnP test
SWEEP_SWAP_BY_BASESTATION = {
    4: True,
    10: False
}


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def lfsr_to_alpha_rad(lfsr_location):
    deg = (((lfsr_location % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def alphas_to_theta_phi(alpha0, alpha1):
    theta = (alpha0 + alpha1) / 2.0
    numerator = math.sin(((alpha1 - alpha0) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha0 + alpha1) / 2.0)
    phi = math.atan2(numerator, denominator)
    return theta, phi


def angles_to_image_point(theta, phi):
    u = math.tan(theta)
    v = math.tan(phi) / math.cos(theta)
    return [u, v]


def measurement_to_image_point(bs, measurement):
    l0 = int(measurement["lfsr0_median"])
    l1 = int(measurement["lfsr1_median"])

    a0 = lfsr_to_alpha_rad(l0)
    a1 = lfsr_to_alpha_rad(l1)

    if SWEEP_SWAP_BY_BASESTATION.get(int(bs), False):
        a0, a1 = a1, a0

    theta, phi = alphas_to_theta_phi(a0, a1)
    return angles_to_image_point(theta, phi), theta, phi


def main():
    parser = argparse.ArgumentParser(description="Build 2D image-to-world maps from origin capture.")
    parser.add_argument("--layout", required=True)
    parser.add_argument("--origin", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--basestations", default="4,10")
    args = parser.parse_args()

    layout = load_json(args.layout)
    origin = load_json(args.origin)
    basestations = [int(x.strip()) for x in args.basestations.split(",") if x.strip()]

    result = {
        "type": "origin_2d_maps",
        "description": "2D homography maps from Lighthouse angular image coordinates to origin-plane coordinates.",
        "frame": "origin_drone_2d_frame",
        "layout_file": args.layout,
        "origin_file": args.origin,
        "basestations": {}
    }

    print("Estimate 2D maps from origin")
    print("=" * 60)

    for bs in basestations:
        bs_key = str(bs)

        if bs_key not in origin["measurements"]:
            print(f"Basestation {bs}: missing")
            continue

        image_points = []
        world_points = []
        used_sensors = []

        for sid in sorted(layout["sensors"].keys(), key=lambda x: int(x)):
            if sid not in origin["measurements"][bs_key]:
                continue

            sensor_info = layout["sensors"][sid]
            meas = origin["measurements"][bs_key][sid]

            image_pt, theta, phi = measurement_to_image_point(bs, meas)

            image_points.append(image_pt)
            world_points.append([float(sensor_info["x"]), float(sensor_info["y"])])
            used_sensors.append(int(sid))

        if len(image_points) < 4:
            print(f"Basestation {bs}: not enough sensors ({len(image_points)}/4)")
            continue

        image_np = np.array(image_points, dtype=np.float64)
        world_np = np.array(world_points, dtype=np.float64)

        H, mask = cv2.findHomography(image_np, world_np, method=0)

        if H is None:
            print(f"Basestation {bs}: homography failed")
            continue

        projected = cv2.perspectiveTransform(image_np.reshape(-1, 1, 2), H).reshape(-1, 2)
        err = projected - world_np
        rmse_m = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))

        result["basestations"][bs_key] = {
            "used_sensors": used_sensors,
            "sweep_swap": bool(SWEEP_SWAP_BY_BASESTATION.get(bs, False)),
            "H_image_to_world": H.tolist(),
            "fit_rmse_m": rmse_m
        }

        print()
        print(f"Basestation {bs}")
        print(f"  used sensors: {used_sensors}")
        print(f"  sweep swap: {SWEEP_SWAP_BY_BASESTATION.get(bs, False)}")
        print(f"  fit RMSE: {rmse_m * 100:.2f} cm")

    save_json(args.output, result)

    print()
    print("=" * 60)
    print(f"Saved: {args.output}")
    print("=" * 60)


if __name__ == "__main__":
    main()
