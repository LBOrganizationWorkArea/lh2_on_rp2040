import argparse
import itertools
import math

import numpy as np

from wand_common import average_transforms, load_json, rt_to_matrix


def transform_error(T, ref):
    dt = float(np.linalg.norm(T[:3, 3] - ref[:3, 3]))
    dR = T[:3, :3] @ ref[:3, :3].T
    trace = np.trace(dR)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))
    return dt, angle


def evaluate(frames, world_bs, other_bs):
    transforms = []

    for frame in frames:
        poses = frame["poses"]
        if str(world_bs) not in poses or str(other_bs) not in poses:
            continue

        T_world_obj = rt_to_matrix(poses[str(world_bs)]["rvec"], poses[str(world_bs)]["tvec"])
        T_other_obj = rt_to_matrix(poses[str(other_bs)]["rvec"], poses[str(other_bs)]["tvec"])
        transforms.append(T_world_obj @ np.linalg.inv(T_other_obj))

    if len(transforms) < 5:
        return None

    center = average_transforms(transforms)
    errors = [transform_error(T, center) for T in transforms]
    trans = np.array([e[0] for e in errors], dtype=float)
    rot = np.array([e[1] for e in errors], dtype=float)

    return {
        "n": len(transforms),
        "translation_median": float(np.median(trans)),
        "translation_p95": float(np.percentile(trans, 95)),
        "rotation_median": float(np.median(rot)),
        "rotation_p95": float(np.percentile(rot, 95)),
        "translation": center[:3, 3].tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description="Analyze wand PnP record quality.")
    parser.add_argument("--input", default="data/wand_pnp_record.json")
    parser.add_argument("--world-bs", type=int, default=4)
    parser.add_argument("--other-bs", type=int, default=10)
    args = parser.parse_args()

    data = load_json(args.input)
    result = evaluate(data["frames"], args.world_bs, args.other_bs)

    print("=" * 70)
    print("Analyze wand record")
    print("=" * 70)

    if result is None:
        print("Not enough frames.")
        return

    t = result["translation"]
    print(f"Frames: {result['n']}")
    print(f"Relative t: x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m")
    print(
        f"Translation median={result['translation_median']:.3f} m p95={result['translation_p95']:.3f} m | "
        f"Rotation median={result['rotation_median']:.2f} deg p95={result['rotation_p95']:.2f} deg"
    )

    print()
    print("Interpretation:")
    print("- If translation median is much larger than 0.1-0.2 m, PnP poses are not mutually consistent.")
    print("- Common causes: planar PnP ambiguity, wrong sensor order, wrong calibrated angle model, or moving too fast/occlusion.")


if __name__ == "__main__":
    main()
