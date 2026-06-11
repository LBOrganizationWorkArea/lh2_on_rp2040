#!/usr/bin/env python3

import argparse
import importlib.util
import json
import math
import tempfile
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
diag_path = HERE / "11_diagnose_wand_pose_data.py"
spec = importlib.util.spec_from_file_location("diag_wand_pose_data", diag_path)
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


def load_layout_data(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def transform_sensor(sensor, mode):
    x = float(sensor["x_m"])
    y = float(sensor.get("y_m", 0.0))
    z = float(sensor.get("z_m", 0.0))

    if mode == "identity":
        nx, ny, nz = x, y, z
    elif mode == "flip_x":
        nx, ny, nz = -x, y, z
    elif mode == "flip_z":
        nx, ny, nz = x, y, -z
    elif mode == "flip_xz":
        nx, ny, nz = -x, y, -z
    elif mode == "swap_xz":
        nx, ny, nz = z, y, x
    elif mode == "swap_xz_flip_x":
        nx, ny, nz = -z, y, x
    elif mode == "swap_xz_flip_z":
        nx, ny, nz = z, y, -x
    elif mode == "swap_xz_flip_xz":
        nx, ny, nz = -z, y, -x
    elif mode == "use_xy_from_xz":
        nx, ny, nz = x, z, y
    elif mode == "use_xy_from_xz_flip_x":
        nx, ny, nz = -x, z, y
    elif mode == "use_xy_from_xz_flip_y":
        nx, ny, nz = x, -z, y
    elif mode == "use_yz_from_xz":
        nx, ny, nz = y, x, z
    else:
        raise ValueError(mode)

    out = dict(sensor)
    out["x_m"] = nx
    out["y_m"] = ny
    out["z_m"] = nz
    return out


def make_layout(base, mode):
    data = dict(base)
    data["description"] = f"Generated diagnostic layout variant: {mode}"
    data["sensors"] = [transform_sensor(sensor, mode) for sensor in base["sensors"]]
    return data


def score_layout(layout_path, poses_path, angle_key):
    layout = diag.load_layout(layout_path)
    obs = diag.load_observations(poses_path, layout, angle_key)

    rmses = []
    pair_mid_rmses = []
    for bs in sorted({o["basestation"] for o in obs}):
        for sweep in sorted({o["sweep"] for o in obs if o["basestation"] == bs}):
            group = [o for o in obs if o["basestation"] == bs and o["sweep"] == sweep]
            if len(group) < 4:
                continue
            _, rmse, _, _ = diag.fit_affine(group)
            if math.isfinite(rmse):
                rmses.append(rmse)

    pairs = diag.paired_sweep_diagnostics(obs)
    for bs in sorted({p["basestation"] for p in pairs}):
        group = [p for p in pairs if p["basestation"] == bs]
        if len(group) < 4:
            continue
        mid_group = [{"p": p["p"], "angle": p["mid"]} for p in group]
        _, rmse, _, _ = diag.fit_affine(mid_group)
        if math.isfinite(rmse):
            pair_mid_rmses.append(rmse)

    return {
        "angle_rmse_mean": float(np.mean(rmses)) if rmses else float("inf"),
        "angle_rmse_best": float(np.min(rmses)) if rmses else float("inf"),
        "mid_rmse_mean": float(np.mean(pair_mid_rmses)) if pair_mid_rmses else float("inf"),
        "mid_rmse_best": float(np.min(pair_mid_rmses)) if pair_mid_rmses else float("inf"),
    }


def main():
    parser = argparse.ArgumentParser(description="Try simple sensor layout variants against captured wand poses.")
    parser.add_argument("--layout", default="config/sensors_layout_vertical_head_down.json")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d.json")
    parser.add_argument("--angle-key", default="raw_angle_rad", choices=["raw_angle_rad", "calibrated_angle_rad"])
    parser.add_argument("--keep-best", default=None, help="Optional output path for the best generated layout JSON.")
    args = parser.parse_args()

    modes = [
        "identity",
        "flip_x",
        "flip_z",
        "flip_xz",
        "swap_xz",
        "swap_xz_flip_x",
        "swap_xz_flip_z",
        "swap_xz_flip_xz",
        "use_xy_from_xz",
        "use_xy_from_xz_flip_x",
        "use_xy_from_xz_flip_y",
        "use_yz_from_xz",
    ]

    base = load_layout_data(args.layout)
    results = []

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for mode in modes:
            candidate = make_layout(base, mode)
            path = tmp / f"{mode}.json"
            with open(path, "w", encoding="utf-8") as f:
                json.dump(candidate, f, indent=2)
            score = score_layout(path, args.poses, args.angle_key)
            score["mode"] = mode
            score["layout"] = candidate
            results.append(score)

    results.sort(key=lambda item: (item["angle_rmse_mean"], item["mid_rmse_mean"]))

    print("=" * 86)
    print("Layout variant diagnostic")
    print(f"Base layout: {args.layout}")
    print(f"Poses:       {args.poses}")
    print(f"Angle:       {args.angle_key}")
    print("=" * 86)
    print("Lower is better. If all values stay near 50 deg, the issue is not just layout flips.")
    print()
    print(f"{'mode':26s} {'angle_mean':>12s} {'angle_best':>12s} {'mid_mean':>12s} {'mid_best':>12s}")
    for item in results:
        print(
            f"{item['mode']:26s} "
            f"{item['angle_rmse_mean']:12.3f} "
            f"{item['angle_rmse_best']:12.3f} "
            f"{item['mid_rmse_mean']:12.3f} "
            f"{item['mid_rmse_best']:12.3f}"
        )

    if args.keep_best:
        best = results[0]
        out = Path(args.keep_best)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(best["layout"], f, indent=2)
            f.write("\n")
        print()
        print(f"Saved best layout variant '{best['mode']}' to: {out}")


if __name__ == "__main__":
    main()
