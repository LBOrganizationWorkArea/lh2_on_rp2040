#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def get_bs(data, bs_id):
    for item in data.get("basestations", []):
        if int(item.get("basestation")) == int(bs_id):
            return item
    raise SystemExit(f"Missing basestation {bs_id} in geometry file.")


def load_transform(bs):
    wt = bs["world_to_lighthouse"]
    rvec = np.array(wt["rotation_vector"], dtype=float)
    t = np.array(wt["translation_m"], dtype=float)
    return Rotation.from_rotvec(rvec), t


def main():
    parser = argparse.ArgumentParser(
        description="Seed v12 relative fit from a v10 room-frame Lighthouse geometry."
    )
    parser.add_argument("--geometry", default="config/lighthouse_geometry_lh2a_families.json")
    parser.add_argument("--anchor-poses", default="config/vertical_pose_variants/vertical_x_up_face_plus_y.json")
    parser.add_argument("--output", default="config/lighthouse_relative_seed_from_room.json")
    args = parser.parse_args()

    with open(args.geometry, "r", encoding="utf-8") as f:
        data = json.load(f)

    bs4 = get_bs(data, 4)
    bs10 = get_bs(data, 10)
    r4, t4 = load_transform(bs4)
    r10, t10 = load_transform(bs10)

    # p_bs4 = R4 p_room + t4, p_bs10 = R10 p_room + t10.
    # Convert BS10 transform so it consumes points already in BS4 coordinates:
    # p_bs10 = R10 R4^-1 (p_bs4 - t4) + t10.
    r10_in_bs4 = r10 * r4.inv()
    t10_in_bs4 = t10 - r10_in_bs4.as_matrix() @ t4

    room_to_bs4_rvec = r4.as_rotvec()
    room_to_bs4_t = t4
    bs10_rvec = r10_in_bs4.as_rotvec()

    command = (
        "py .\\tools\\22_fit_relative_lighthouse_frame.py "
        f"--anchor-poses {args.anchor_poses} "
        "--max-frames 0 "
        f"--bs10-guess {bs10_rvec[0]:.6f},{bs10_rvec[1]:.6f},{bs10_rvec[2]:.6f},{t10_in_bs4[0]:.6f},{t10_in_bs4[1]:.6f},{t10_in_bs4[2]:.6f} "
        "--bs-distance-prior 1.4 --bs-distance-sigma 0.12 "
        f"--room-to-bs4-guess {room_to_bs4_rvec[0]:.6f},{room_to_bs4_rvec[1]:.6f},{room_to_bs4_rvec[2]:.6f},{room_to_bs4_t[0]:.6f},{room_to_bs4_t[1]:.6f},{room_to_bs4_t[2]:.6f} "
        "--starts 4 --workers 4 --max-nfev 500 --convention-search v10 --max-anchor-spread-deg 0.5"
    )

    out = {
        "description": "Initial v12 relative-frame seed derived from room-frame v10 geometry.",
        "input_geometry": args.geometry,
        "input_anchor_poses": args.anchor_poses,
        "room_to_bs4": {
            "rotation_vector": [float(v) for v in room_to_bs4_rvec],
            "translation_m": [float(v) for v in room_to_bs4_t],
        },
        "bs10_in_bs4": {
            "rotation_vector": [float(v) for v in bs10_rvec],
            "translation_m": [float(v) for v in t10_in_bs4],
            "distance_m": float(np.linalg.norm(t10_in_bs4)),
        },
        "suggested_command": command,
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")

    print("=" * 88)
    print("Relative seed from room geometry")
    print(f"Input:  {args.geometry}")
    print(f"Output: {args.output}")
    print("=" * 88)
    print(
        "BS10 in BS4 frame: "
        f"x={t10_in_bs4[0]:+.3f}, y={t10_in_bs4[1]:+.3f}, z={t10_in_bs4[2]:+.3f} m | "
        f"distance={np.linalg.norm(t10_in_bs4):.3f} m"
    )
    print(
        "Room origin in BS4 frame: "
        f"x={room_to_bs4_t[0]:+.3f}, y={room_to_bs4_t[1]:+.3f}, z={room_to_bs4_t[2]:+.3f} m"
    )
    print("=" * 88)
    print(command)


if __name__ == "__main__":
    main()
