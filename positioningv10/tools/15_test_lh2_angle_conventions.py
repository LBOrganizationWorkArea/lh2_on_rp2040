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


CYCLE_PERIODS = [
    959000 / 2, 957000 / 2, 953000 / 2, 949000 / 2,
    947000 / 2, 943000 / 2, 941000 / 2, 939000 / 2,
    937000 / 2, 929000 / 2, 919000 / 2, 911000 / 2,
    907000 / 2, 901000 / 2, 893000 / 2, 887000 / 2,
]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def circ_spread_deg(values):
    if not values:
        return float("nan")
    center = diag.circular_mean(values)
    return math.degrees(max(abs(diag.angle_diff(v, center)) for v in values))


def angle_from_offset(offset, bs, sweep, convention):
    period = convention["period"](int(bs))
    base = (float(offset) % period) * 2.0 * math.pi / period
    if convention["center"] == "minus_pi":
        base -= math.pi
    elif convention["center"] == "zero":
        pass
    elif convention["center"] == "plus_pi":
        base += math.pi

    if convention["sign"] < 0:
        base = -base

    tilt = math.pi / 3.0
    if convention["tilt"] == "current":
        base += tilt if int(sweep) == 0 else -tilt
    elif convention["tilt"] == "reversed":
        base += -tilt if int(sweep) == 0 else tilt
    elif convention["tilt"] == "none":
        pass

    base += math.radians(convention["offset_deg"])
    return wrap(base)


def build_conventions():
    periods = {
        "bs_period": lambda bs: CYCLE_PERIODS[bs],
        "bs_period_div4": lambda bs: CYCLE_PERIODS[bs] / 4.0,
        "120000": lambda bs: 120000.0,
        "240000": lambda bs: 240000.0,
        "480000": lambda bs: 480000.0,
    }
    conventions = []
    for period_name, period_fn in periods.items():
        for center in ["minus_pi", "zero"]:
            for sign in [1.0, -1.0]:
                for tilt in ["current", "reversed", "none"]:
                    for offset_deg in [0.0, 90.0, -90.0, 180.0]:
                        conventions.append({
                            "name": f"{period_name}|{center}|sign{int(sign):+d}|tilt_{tilt}|off{offset_deg:+.0f}",
                            "period": period_fn,
                            "period_name": period_name,
                            "center": center,
                            "sign": sign,
                            "tilt": tilt,
                            "offset_deg": offset_deg,
                        })
    return conventions


def converted_pose_file(input_path, output_path, convention):
    with open(input_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    out = dict(data)
    out["description"] = data.get("description", "") + f" Recomputed angles with convention {convention['name']}."
    poses = []
    for pose in data.get("poses", []):
        new_pose = dict(pose)
        measurements = []
        for m in pose.get("measurements", []):
            if "lfsr_location" not in m:
                continue
            nm = dict(m)
            nm["raw_angle_rad"] = angle_from_offset(
                float(m["lfsr_location"]),
                int(m["basestation"]),
                int(m["sweep"]),
                convention,
            )
            nm.pop("calibrated_angle_rad", None)
            measurements.append(nm)
        new_pose["measurements"] = measurements
        poses.append(new_pose)
    out["poses"] = poses

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(out, f)


def score_convention(layout_path, poses_path):
    layout = diag.load_layout(layout_path)
    obs = diag.load_observations(poses_path, layout, "raw_angle_rad")
    rmses = []
    mid_rmses = []

    for bs in sorted({o["basestation"] for o in obs}):
        for sweep in sorted({o["sweep"] for o in obs if o["basestation"] == bs}):
            group = [o for o in obs if o["basestation"] == bs and o["sweep"] == sweep]
            if len(group) < 4:
                continue
            _, rmse, _, _ = diag.fit_affine(group)
            if math.isfinite(rmse):
                rmses.append(rmse)

    pairs = diag.paired_sweep_diagnostics(obs)
    sep_spreads = []
    for bs in sorted({p["basestation"] for p in pairs}):
        group = [p for p in pairs if p["basestation"] == bs]
        if len(group) < 4:
            continue
        mid_group = [{"p": p["p"], "angle": p["mid"]} for p in group]
        _, rmse, _, _ = diag.fit_affine(mid_group)
        if math.isfinite(rmse):
            mid_rmses.append(rmse)
        sep_spreads.append(circ_spread_deg([p["sep"] for p in group]))

    return {
        "angle_mean": float(np.mean(rmses)) if rmses else float("inf"),
        "angle_best": float(np.min(rmses)) if rmses else float("inf"),
        "mid_mean": float(np.mean(mid_rmses)) if mid_rmses else float("inf"),
        "mid_best": float(np.min(mid_rmses)) if mid_rmses else float("inf"),
        "sep_spread_mean": float(np.mean(sep_spreads)) if sep_spreads else float("inf"),
    }


def main():
    parser = argparse.ArgumentParser(description="Test LH2 offset-to-angle conventions against known pose data.")
    parser.add_argument("--layout", default="config/sensors_layout_vertical_head_down.json")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d_filtered.json")
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--save-best", default=None, help="Optional output pose JSON with angles recomputed by the best convention.")
    args = parser.parse_args()

    conventions = build_conventions()
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for idx, convention in enumerate(conventions):
            tmp_pose = tmp / f"poses_{idx}.json"
            converted_pose_file(args.poses, tmp_pose, convention)
            score = score_convention(args.layout, tmp_pose)
            score["name"] = convention["name"]
            score["convention"] = convention
            results.append(score)

    results.sort(key=lambda x: (x["angle_mean"], x["mid_mean"], x["sep_spread_mean"]))

    print("=" * 110)
    print("LH2 offset-to-angle convention sweep")
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print("=" * 110)
    print("Lower angle_mean/mid_mean is better. sep_spread shows if sweep separation is stable.")
    print()
    print(f"{'rank':>4s} {'angle_mean':>11s} {'angle_best':>11s} {'mid_mean':>11s} {'sep_spread':>11s}  convention")
    for rank, item in enumerate(results[:args.top], start=1):
        print(
            f"{rank:4d} "
            f"{item['angle_mean']:11.3f} "
            f"{item['angle_best']:11.3f} "
            f"{item['mid_mean']:11.3f} "
            f"{item['sep_spread_mean']:11.3f}  "
            f"{item['name']}"
        )

    if args.save_best:
        best = results[0]
        converted_pose_file(args.poses, Path(args.save_best), best["convention"])
        print()
        print(f"Saved best convention pose file to: {args.save_best}")
        print(f"Best convention: {best['name']}")


if __name__ == "__main__":
    main()
