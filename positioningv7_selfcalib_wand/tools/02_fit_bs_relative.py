import argparse
import math

import numpy as np

from wand_common import average_transforms, load_json, rt_to_matrix, save_json


def transform_error(T, ref):
    dt = float(np.linalg.norm(T[:3, 3] - ref[:3, 3]))
    dR = T[:3, :3] @ ref[:3, :3].T
    trace = np.trace(dR)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))
    return dt, angle


def main():
    parser = argparse.ArgumentParser(description="Fit relative transform between two Lighthouses from wand PnP frames.")
    parser.add_argument("--input", default="data/wand_pnp_record.json")
    parser.add_argument("--output", default="config/bs_relative.json")
    parser.add_argument("--world-bs", type=int, default=4)
    parser.add_argument("--other-bs", type=int, default=10)
    parser.add_argument("--max-reproj", type=float, default=0.01)
    args = parser.parse_args()

    record = load_json(args.input)
    transforms = []

    for frame in record["frames"]:
        poses = frame["poses"]
        world_key = str(args.world_bs)
        other_key = str(args.other_bs)

        if world_key not in poses or other_key not in poses:
            continue
        if poses[world_key]["reproj_rmse"] > args.max_reproj or poses[other_key]["reproj_rmse"] > args.max_reproj:
            continue

        T_world_obj = rt_to_matrix(poses[world_key]["rvec"], poses[world_key]["tvec"])
        T_other_obj = rt_to_matrix(poses[other_key]["rvec"], poses[other_key]["tvec"])

        # Object point in other frame = T_other_obj * object.
        # Object point in world frame = T_world_obj * object.
        # Therefore: p_world = T_world_obj * inv(T_other_obj) * p_other.
        T_world_other = T_world_obj @ np.linalg.inv(T_other_obj)
        transforms.append(T_world_other)

    if len(transforms) < 5:
        raise SystemExit(f"Need at least 5 valid frames, got {len(transforms)}.")

    rough = average_transforms(transforms)
    errors = [transform_error(T, rough) for T in transforms]
    trans_errors = np.array([e[0] for e in errors], dtype=float)
    rot_errors = np.array([e[1] for e in errors], dtype=float)

    keep = trans_errors <= np.percentile(trans_errors, 80)
    kept = [T for T, k in zip(transforms, keep) if k]
    fitted = average_transforms(kept)

    errors2 = [transform_error(T, fitted) for T in kept]
    trans2 = np.array([e[0] for e in errors2], dtype=float)
    rot2 = np.array([e[1] for e in errors2], dtype=float)

    output = {
        "description": "Relative Lighthouse calibration from drone wand PnP.",
        "world_bs": int(args.world_bs),
        "other_bs": int(args.other_bs),
        "transform_world_from_other": fitted.tolist(),
        "num_frames_total": int(len(transforms)),
        "num_frames_used": int(len(kept)),
        "quality": {
            "translation_median_error_m": float(np.median(trans2)),
            "translation_p95_error_m": float(np.percentile(trans2, 95)),
            "rotation_median_error_deg": float(np.median(rot2)),
            "rotation_p95_error_deg": float(np.percentile(rot2, 95)),
        },
    }

    save_json(args.output, output)

    t = fitted[:3, 3]
    q = output["quality"]
    print("=" * 70)
    print("Relative Lighthouse fit")
    print(f"Frames: {len(kept)} used / {len(transforms)} total")
    print(f"BS{args.other_bs} in BS{args.world_bs} frame: x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m")
    print(
        f"Quality: trans median={q['translation_median_error_m']:.3f} m "
        f"p95={q['translation_p95_error_m']:.3f} m | "
        f"rot median={q['rotation_median_error_deg']:.2f} deg "
        f"p95={q['rotation_p95_error_deg']:.2f} deg"
    )
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
