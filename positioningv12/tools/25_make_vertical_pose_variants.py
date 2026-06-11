#!/usr/bin/env python3

import argparse
import copy
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


VARIANTS = {
    # Sensor 3 and 0 are the lower row, so local +x points upward.
    # The remaining ambiguity is the direction of the board normal.
    "vertical_x_up_face_plus_x": {
        "local_x_world": [0.0, 0.0, 1.0],
        "local_z_world": [1.0, 0.0, 0.0],
    },
    "vertical_x_up_face_minus_x": {
        "local_x_world": [0.0, 0.0, 1.0],
        "local_z_world": [-1.0, 0.0, 0.0],
    },
    "vertical_x_up_face_plus_y": {
        "local_x_world": [0.0, 0.0, 1.0],
        "local_z_world": [0.0, 1.0, 0.0],
    },
    "vertical_x_up_face_minus_y": {
        "local_x_world": [0.0, 0.0, 1.0],
        "local_z_world": [0.0, -1.0, 0.0],
    },
}


def rotation_for_variant(spec):
    local_x_world = np.array(spec["local_x_world"], dtype=float)
    local_z_world = np.array(spec["local_z_world"], dtype=float)
    local_y_world = np.cross(local_z_world, local_x_world)
    matrix = np.column_stack([local_x_world, local_y_world, local_z_world])
    return Rotation.from_matrix(matrix).as_euler("xyz", degrees=True)


def rounded_angle(value):
    value = 0.0 if abs(value) < 1e-9 else float(value)
    return round(value, 6)


def main():
    parser = argparse.ArgumentParser(
        description="Create pose-file copies for the possible vertical drone orientations."
    )
    parser.add_argument(
        "--input",
        default="config/wand_calibration_poses_3d_lh2a_families.json",
        help="Captured LH2A pose file.",
    )
    parser.add_argument(
        "--output-dir",
        default="config/vertical_pose_variants",
        help="Directory where variant pose files are written.",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with input_path.open("r", encoding="utf-8") as f:
        source = json.load(f)

    print("=" * 88)
    print("Create vertical pose variants")
    print(f"Input:  {input_path}")
    print(f"Output: {output_dir}")
    print("=" * 88)

    manifest = []
    for name, spec in VARIANTS.items():
        roll, pitch, yaw = rotation_for_variant(spec)
        data = copy.deepcopy(source)
        data["pose_orientation_variant"] = {
            "name": name,
            "note": "sensor 3 bottom-left and sensor 0 bottom-right; local +x points upward",
            "roll_deg": rounded_angle(roll),
            "pitch_deg": rounded_angle(pitch),
            "yaw_deg": rounded_angle(yaw),
            "local_x_world": spec["local_x_world"],
            "local_z_world": spec["local_z_world"],
        }
        for pose in data.get("poses", []):
            pose["roll_deg"] = rounded_angle(roll)
            pose["pitch_deg"] = rounded_angle(pitch)
            pose["yaw_deg"] = rounded_angle(yaw)

        output_path = output_dir / f"{name}.json"
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
            f.write("\n")

        manifest.append(
            {
                "name": name,
                "path": str(output_path),
                "roll_deg": rounded_angle(roll),
                "pitch_deg": rounded_angle(pitch),
                "yaw_deg": rounded_angle(yaw),
            }
        )
        print(
            f"{name}: roll={roll:+.1f} pitch={pitch:+.1f} yaw={yaw:+.1f} -> {output_path}"
        )

    manifest_path = output_dir / "manifest.json"
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump({"variants": manifest}, f, indent=2)
        f.write("\n")

    print("=" * 88)
    print(f"Saved manifest: {manifest_path}")


if __name__ == "__main__":
    main()
