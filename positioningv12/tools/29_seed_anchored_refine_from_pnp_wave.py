#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def get_bs(data, bs_id):
    for item in data.get("basestations", []):
        if int(item.get("basestation")) == int(bs_id):
            return item
    return None


def room_to_bs4_seed(room_geometry_path):
    if not room_geometry_path:
        return np.zeros(3, dtype=float), np.array([0.7, -1.9, -0.7], dtype=float)
    path = Path(room_geometry_path)
    if not path.exists():
        return np.zeros(3, dtype=float), np.array([0.7, -1.9, -0.7], dtype=float)
    geom = load_json(path)
    bs4 = get_bs(geom, 4)
    if bs4 is None:
        return np.zeros(3, dtype=float), np.array([0.7, -1.9, -0.7], dtype=float)
    wt = bs4["world_to_lighthouse"]
    return (
        np.array(wt["rotation_vector"], dtype=float),
        np.array(wt["translation_m"], dtype=float),
    )


def main():
    parser = argparse.ArgumentParser(
        description="Build an anchored v12 refine command from LH2A wave PnP relative geometry."
    )
    parser.add_argument("--pnp", default="config/lighthouse_relative_pnp_oldwave_cluster_d14.json")
    parser.add_argument("--anchor-poses", default="config/vertical_pose_variants/vertical_x_up_face_plus_y.json")
    parser.add_argument("--room-geometry", default="config/lighthouse_geometry_lh2a_families.json")
    parser.add_argument("--output", default="config/anchored_refine_seed_from_pnp_wave.json")
    parser.add_argument("--distance-prior", type=float, default=1.4)
    args = parser.parse_args()

    pnp = load_json(args.pnp)
    T_world_from_other = np.array(pnp["transform_world_from_other"], dtype=float)

    # PnP output follows v7 naming:
    # p_bs4 = T_world_from_other * p_bs10.
    # The v12 predictor wants p_bs10 = R * (p_bs4 - t), so use the inverse
    # rotation and the BS10 origin position in the BS4 frame.
    R_bs4_from_bs10 = T_world_from_other[:3, :3]
    t_bs10_in_bs4 = T_world_from_other[:3, 3]
    R_bs10_from_bs4 = R_bs4_from_bs10.T
    bs10_rvec = Rotation.from_matrix(R_bs10_from_bs4).as_rotvec()

    room_rvec, room_t = room_to_bs4_seed(args.room_geometry)

    command = (
        "py .\\tools\\22_fit_relative_lighthouse_frame.py "
        f"--anchor-poses {args.anchor_poses} "
        "--max-frames 0 "
        f"--bs10-guess {bs10_rvec[0]:.6f},{bs10_rvec[1]:.6f},{bs10_rvec[2]:.6f},"
        f"{t_bs10_in_bs4[0]:.6f},{t_bs10_in_bs4[1]:.6f},{t_bs10_in_bs4[2]:.6f} "
        f"--bs-distance-prior {args.distance_prior:.6f} "
        "--bs-distance-sigma 0.05 --bs-distance-weight 200 "
        f"--room-to-bs4-guess {room_rvec[0]:.6f},{room_rvec[1]:.6f},{room_rvec[2]:.6f},"
        f"{room_t[0]:.6f},{room_t[1]:.6f},{room_t[2]:.6f} "
        "--starts 4 --workers 4 --max-nfev 700 --convention-search v10 --max-anchor-spread-deg 0.5 "
        "--output config\\lighthouse_relative_refined_from_pnp_anchors.json"
    )

    out = {
        "description": "Seed for anchored v12 refinement from LH2A wave PnP relative geometry.",
        "input_pnp": args.pnp,
        "input_anchor_poses": args.anchor_poses,
        "input_room_geometry": args.room_geometry,
        "bs10_in_bs4": {
            "rotation_vector": [float(v) for v in bs10_rvec],
            "translation_m": [float(v) for v in t_bs10_in_bs4],
            "distance_m": float(np.linalg.norm(t_bs10_in_bs4)),
        },
        "room_to_bs4": {
            "rotation_vector": [float(v) for v in room_rvec],
            "translation_m": [float(v) for v in room_t],
        },
        "suggested_command": command,
    }
    save_json(args.output, out)

    print("=" * 88)
    print("Anchored refine seed from PnP wave")
    print(f"PNP:     {args.pnp}")
    print(f"Anchors: {args.anchor_poses}")
    print(f"Output:  {args.output}")
    print("=" * 88)
    print(
        "BS10 seed in BS4 frame: "
        f"x={t_bs10_in_bs4[0]:+.3f}, y={t_bs10_in_bs4[1]:+.3f}, z={t_bs10_in_bs4[2]:+.3f} m | "
        f"distance={np.linalg.norm(t_bs10_in_bs4):.3f} m"
    )
    print(
        "Room origin seed in BS4 frame: "
        f"x={room_t[0]:+.3f}, y={room_t[1]:+.3f}, z={room_t[2]:+.3f} m"
    )
    print("=" * 88)
    print(command)


if __name__ == "__main__":
    main()
