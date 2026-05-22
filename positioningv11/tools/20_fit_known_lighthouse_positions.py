#!/usr/bin/env python3

import argparse
import importlib.util
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from lh2_factory_model import load_factory_calibration_map, factory_axis_for_sweep


def load_geom_module():
    path = Path(__file__).resolve().parent / "04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py"
    spec = importlib.util.spec_from_file_location("geom_v11", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def load_lighthouse_positions(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    result = {}
    for item in data.get("basestations", []):
        bs = int(item["basestation"])
        result[bs] = np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item["z_m"]),
        ], dtype=float)
    return result


def corrected_raw(raw, sweep, params, signs):
    off0 = params[3]
    off1 = params[4]
    if int(sweep) == 0:
        return signs[0] * float(raw) + off0
    return signs[1] * float(raw) + off1


def predict_fixed_position_angle(geom, p_world, params, position, tilt, axis_calibration, model_variant):
    rx, ry, rz = params[:3]
    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    p_lh = R @ (p_world - position)
    return geom.factory_angle_variant(p_lh, tilt, axis_calibration, model_variant)


def residuals(params, bs, observations, layout, position, factory_entry, tilt_map, signs, axis_map, model_variant, geom):
    out = []
    for obs in observations:
        if int(obs["basestation"]) != int(bs):
            continue
        sensor = int(obs["sensor"])
        if sensor not in layout:
            continue

        sweep = int(obs["sweep"])
        p_world = geom.sensor_world(obs["pose_data"], 0.0, layout[sensor])
        pred = predict_fixed_position_angle(
            geom,
            p_world,
            params,
            position,
            tilt_map[sweep],
            None if obs.get("angle_is_calibrated") else factory_axis_for_sweep(factory_entry, axis_map[sweep]),
            model_variant,
        )

        candidates = obs.get("candidate_raw_angles")
        if candidates:
            diffs = [angle_diff(pred, corrected_raw(candidate, sweep, params, signs)) for candidate in candidates]
            out.append(min(diffs, key=abs))
        else:
            out.append(angle_diff(pred, corrected_raw(float(obs["raw_angle"]), sweep, params, signs)))

    return np.array(out, dtype=float)


def fit_one_task(task):
    (
        x0,
        bs,
        observations,
        layout,
        position,
        factory_entry,
        tilt_map,
        signs,
        axis_map,
        model_variant,
        max_nfev,
    ) = task
    geom = load_geom_module()
    lower = np.array([-math.pi, -math.pi, -math.pi, -math.pi, -math.pi], dtype=float)
    upper = np.array([+math.pi, +math.pi, +math.pi, +math.pi, +math.pi], dtype=float)
    result = least_squares(
        residuals,
        x0,
        bounds=(lower, upper),
        args=(bs, observations, layout, position, factory_entry, tilt_map, signs, axis_map, model_variant, geom),
        loss="soft_l1",
        f_scale=math.radians(1.0),
        max_nfev=max_nfev,
    )
    err = residuals(result.x, bs, observations, layout, position, factory_entry, tilt_map, signs, axis_map, model_variant, geom)
    if len(err) == 0:
        return None
    rmse = float(np.sqrt(np.mean(err ** 2)))
    return {
        "params": result.x,
        "rmse_rad": rmse,
        "rmse_deg": float(math.degrees(rmse)),
        "num_residuals": int(len(err)),
        "success": bool(result.success),
        "tilt_map": dict(tilt_map),
        "signs": dict(signs),
        "axis_map": dict(axis_map),
        "model_variant": model_variant,
    }


def run_tasks(tasks, workers):
    best = None
    workers = max(1, int(workers or 1))
    if workers == 1:
        for index, task in enumerate(tasks, start=1):
            candidate = fit_one_task(task)
            if candidate is not None and (best is None or candidate["rmse_rad"] < best["rmse_rad"]):
                best = candidate
            if best is not None and (index == len(tasks) or index % max(1, len(tasks) // 10) == 0):
                print(f"  hypotheses {index}/{len(tasks)} | best RMSE={best['rmse_deg']:.3f} deg", flush=True)
        return best

    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(fit_one_task, task) for task in tasks]
        done = 0
        for future in as_completed(futures):
            done += 1
            candidate = future.result()
            if candidate is not None and (best is None or candidate["rmse_rad"] < best["rmse_rad"]):
                best = candidate
            if best is not None and (done == len(tasks) or done % max(1, len(tasks) // 10) == 0):
                print(f"  hypotheses {done}/{len(tasks)} | best RMSE={best['rmse_deg']:.3f} deg", flush=True)
    return best


def fit_basestation(bs, observations, layout, position, factory_entry, model_variants, max_nfev, workers):
    tilt_maps = [
        {0: math.pi / 6.0, 1: -math.pi / 6.0},
        {0: -math.pi / 6.0, 1: math.pi / 6.0},
    ]
    sign_maps = [
        {0: +1.0, 1: +1.0},
        {0: +1.0, 1: -1.0},
        {0: -1.0, 1: +1.0},
        {0: -1.0, 1: -1.0},
    ]
    axis_maps = [
        {0: 0, 1: 1},
        {0: 1, 1: 0},
    ]
    rotation_guesses = [
        [0.0, 0.0, 0.0],
        [0.0, math.pi / 2.0, 0.0],
        [0.0, -math.pi / 2.0, 0.0],
        [0.0, 0.0, math.pi],
    ]

    tasks = []
    for model_variant in model_variants:
        for tilt_map in tilt_maps:
            for signs in sign_maps:
                for axis_map in axis_maps:
                    for rot0 in rotation_guesses:
                        tasks.append((
                            np.array([rot0[0], rot0[1], rot0[2], 0.0, 0.0], dtype=float),
                            bs,
                            observations,
                            layout,
                            position,
                            factory_entry,
                            tilt_map,
                            signs,
                            axis_map,
                            model_variant,
                            max_nfev,
                        ))

    best = run_tasks(tasks, workers)
    if best is None:
        raise RuntimeError(f"No solution for BS{bs}")

    rx, ry, rz, off0, off1 = best["params"]
    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    return {
        "basestation": int(bs),
        "model": "v11_known_position_orientation_fit",
        "rmse_deg": float(best["rmse_deg"]),
        "num_residuals": int(best["num_residuals"]),
        "position_fixed": True,
        "model_variant": best["model_variant"],
        "factory_calibration": None if factory_entry is None else factory_entry.get("path"),
        "sweep_tilts": {
            "sweep_0_deg": float(math.degrees(best["tilt_map"][0])),
            "sweep_1_deg": float(math.degrees(best["tilt_map"][1])),
            "sweep_0_rad": float(best["tilt_map"][0]),
            "sweep_1_rad": float(best["tilt_map"][1]),
        },
        "angle_correction": {
            "sign_sweep_0": float(best["signs"][0]),
            "sign_sweep_1": float(best["signs"][1]),
            "offset_sweep_0_rad": float(off0),
            "offset_sweep_1_rad": float(off1),
            "offset_sweep_0_deg": float(math.degrees(off0)),
            "offset_sweep_1_deg": float(math.degrees(off1)),
        },
        "factory_axis_map": {
            "sweep_0_axis": int(best["axis_map"][0]),
            "sweep_1_axis": int(best["axis_map"][1]),
        },
        "world_to_lighthouse": {
            "rotation_vector": [float(rx), float(ry), float(rz)],
            "rotation_matrix": R.tolist(),
            "translation_m": [float(position[0]), float(position[1]), float(position[2])],
        },
    }


def parse_model_variants(value):
    value = str(value).strip()
    if value.lower() == "all":
        return ["factory_xy_z", "xy_z", "xz_y", "yz_x", "yx_z"]
    return [item.strip() for item in value.split(",") if item.strip()] or ["factory_xy_z"]


def main():
    parser = argparse.ArgumentParser(description="Fit Lighthouse orientations with measured Lighthouse positions fixed.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d_lh2a_families.json")
    parser.add_argument("--positions", default="config/lighthouse_positions.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_known_positions.json")
    parser.add_argument("--model-variants", default="factory_xy_z")
    parser.add_argument("--max-nfev", type=int, default=600)
    parser.add_argument("--workers", type=int, default=max(1, os.cpu_count() or 1))
    parser.add_argument("--factory-calibs", default="auto")
    args = parser.parse_args()

    geom = load_geom_module()
    layout = geom.load_layout(args.layout)
    observations = geom.load_poses(args.poses, 360.0, prefer_raw_angles=True)
    positions = load_lighthouse_positions(args.positions)
    factory_calibs = load_factory_calibration_map(args.factory_calibs)
    model_variants = parse_model_variants(args.model_variants)

    print("=" * 88)
    print("v11 fit: known Lighthouse positions, solve orientations")
    print(f"Poses:     {args.poses}")
    print(f"Positions: {args.positions}")
    print(f"Models:    {','.join(model_variants)}")
    print(f"Obs:       {len(observations)}")
    print("=" * 88)

    results = []
    for bs, position in sorted(positions.items()):
        print()
        print(f"Fitting BS{bs} fixed at x={position[0]:+.3f}, y={position[1]:+.3f}, z={position[2]:+.3f}")
        result = fit_basestation(
            bs,
            observations,
            layout,
            position,
            factory_calibs.get(bs),
            model_variants,
            args.max_nfev,
            args.workers,
        )
        results.append(result)
        print(f"BS{bs}: RMSE={result['rmse_deg']:.3f} deg | residuals={result['num_residuals']}")

    output = {
        "description": "v11 geometry with measured Lighthouse positions fixed and orientations fitted from LH2A family candidates.",
        "input_layout": args.layout,
        "input_poses": args.poses,
        "input_positions": args.positions,
        "basestations": results,
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
        f.write("\n")

    print()
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
