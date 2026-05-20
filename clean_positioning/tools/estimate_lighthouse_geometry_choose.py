# -*- coding: utf-8 -*-

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


TICKS_PER_REV = 833333

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


def parse_choose(value):
    out = {}
    for item in value.split(","):
        bs, sol = item.split(":")
        out[int(bs)] = int(sol)
    return out


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


def build_points_for_bs(bs, layout, origin):
    obj = []
    img = []
    used_sensors = []

    bs_key = str(bs)
    measurements = origin["measurements"][bs_key]

    for sid in sorted(layout["sensors"].keys(), key=lambda x: int(x)):
        if sid not in measurements:
            continue

        sinfo = layout["sensors"][sid]
        meas = measurements[sid]

        l0 = int(meas["lfsr0_median"])
        l1 = int(meas["lfsr1_median"])

        a0 = lfsr_to_alpha_rad(l0)
        a1 = lfsr_to_alpha_rad(l1)

        if SWEEP_SWAP_BY_BASESTATION.get(bs, False):
            a0, a1 = a1, a0

        theta, phi = alphas_to_theta_phi(a0, a1)

        obj.append([float(sinfo["x"]), float(sinfo["y"]), float(sinfo["z"])])
        img.append(angles_to_image_point(theta, phi))
        used_sensors.append(int(sid))

    return np.array(obj, dtype=np.float64), np.array(img, dtype=np.float64), used_sensors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", required=True)
    parser.add_argument("--origin", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--choose", required=True, help="Example: 4:1,10:0")
    args = parser.parse_args()

    layout = load_json(args.layout)
    origin = load_json(args.origin)
    chosen = parse_choose(args.choose)

    K = np.eye(3, dtype=np.float64)
    dist = np.zeros((4, 1), dtype=np.float64)

    result = {
        "type": "chosen_lighthouse_geometry",
        "frame": "origin_drone_frame",
        "layout_file": args.layout,
        "origin_file": args.origin,
        "chosen_solutions": chosen,
        "sweep_swap_by_basestation": SWEEP_SWAP_BY_BASESTATION,
        "basestations": {}
    }

    print("Estimate chosen Lighthouse geometry")
    print("=" * 60)
    print(f"Chosen: {chosen}")
    print("=" * 60)

    for bs, sol_idx in chosen.items():
        obj, img, used_sensors = build_points_for_bs(bs, layout, origin)

        retval, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
            obj,
            img,
            K,
            dist,
            flags=cv2.SOLVEPNP_IPPE
        )

        if len(rvecs) <= sol_idx:
            raise RuntimeError(f"Basestation {bs}: solution {sol_idx} does not exist")

        rvec = rvecs[sol_idx]
        tvec = tvecs[sol_idx]
        reproj_value = float(np.array(reproj).reshape(-1)[sol_idx])

        R, _ = cv2.Rodrigues(rvec)
        pos = (-R.T @ tvec).reshape(3)

        result["basestations"][str(bs)] = {
            "solution_index": sol_idx,
            "sweep_swap": bool(SWEEP_SWAP_BY_BASESTATION.get(bs, False)),
            "used_sensors": used_sensors,
            "rvec": rvec.reshape(3).tolist(),
            "tvec": tvec.reshape(3).tolist(),
            "R_lighthouse_from_origin": R.tolist(),
            "position_in_origin_frame_m": pos.tolist(),
            "reprojection_error": reproj_value
        }

        print()
        print(f"Basestation {bs}")
        print(f"  chosen solution: {sol_idx}")
        print(f"  sweep swap: {SWEEP_SWAP_BY_BASESTATION.get(bs, False)}")
        print(f"  position: x={pos[0]:+.3f}, y={pos[1]:+.3f}, z={pos[2]:+.3f} m")
        print(f"  reprojection: {reproj_value:.6f}")

    save_json(args.output, result)

    print()
    print("=" * 60)
    print(f"Saved: {args.output}")
    print("=" * 60)


if __name__ == "__main__":
    main()
