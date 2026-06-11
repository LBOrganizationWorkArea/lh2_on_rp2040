#!/usr/bin/env python3

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def load_fit22():
    path = Path(__file__).with_name("22_fit_relative_lighthouse_frame.py")
    spec = importlib.util.spec_from_file_location("fit22", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fit22 = load_fit22()


def parse_csv_floats(text, expected, name):
    values = [float(v) for v in str(text).split(",")]
    if len(values) != expected:
        raise SystemExit(f"{name} must contain {expected} comma-separated numbers.")
    return np.array(values, dtype=float)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def measured_bs10_from_geometry(path):
    data = load_json(path)
    if "transform_world_from_other" in data:
        transform = np.array(data["transform_world_from_other"], dtype=float)
        r_bs4_from_bs10 = transform[:3, :3]
        t_bs10_in_bs4 = transform[:3, 3]
        r_bs10_from_bs4 = r_bs4_from_bs10.T
        rvec = Rotation.from_matrix(r_bs10_from_bs4).as_rotvec()
        return rvec, t_bs10_in_bs4

    for item in data.get("basestations", []):
        if int(item.get("basestation")) == 10:
            wt = item["world_to_lighthouse"]
            return (
                np.array(wt["rotation_vector"], dtype=float),
                np.array(wt["translation_m"], dtype=float),
            )

    raise SystemExit(f"Could not read BS10 relative transform from {path}")


def room_seed_from_geometry(path):
    if not path or not Path(path).exists():
        return np.array([0.0, 0.0, 0.0, 0.3, 1.8, 0.7], dtype=float)
    data = load_json(path)
    if "room_to_bs4" in data:
        rt = data["room_to_bs4"]
        return np.array(rt["rotation_vector"] + rt["translation_m"], dtype=float)
    for item in data.get("basestations", []):
        if int(item.get("basestation")) == 4:
            wt = item["world_to_lighthouse"]
            return np.array(wt["rotation_vector"] + wt["translation_m"], dtype=float)
    return np.array([0.0, 0.0, 0.0, 0.3, 1.8, 0.7], dtype=float)


def params_to_full(params, bs10_translation):
    full = np.zeros(16, dtype=float)
    full[:6] = params[:6]
    full[6:9] = params[6:9]
    full[9:12] = bs10_translation
    full[12:16] = params[9:13]
    return full


def convention_from_params(params, hypothesis):
    return {
        "bs4_tilts": hypothesis["bs4_tilts"],
        "bs4_axis_map": hypothesis["bs4_axis_map"],
        "bs10_tilts": hypothesis["bs10_tilts"],
        "bs10_axis_map": hypothesis["bs10_axis_map"],
        "signs": hypothesis["signs"],
        "bs4_offsets": {0: params[9], 1: params[10]},
        "bs10_offsets": {0: params[11], 1: params[12]},
    }


def residual_vector(params, anchors, layout, factory_calibs, robust_scale, hypothesis, bs10_translation):
    convention = convention_from_params(params, hypothesis)
    bs10 = np.concatenate([params[6:9], bs10_translation])
    full = params_to_full(params, bs10_translation)
    out = []

    for anchor in anchors:
        for m in anchor.get("measurements", []):
            sensor = int(m["sensor"])
            if sensor not in layout:
                continue
            p_room = fit22.sensor_anchor_room(anchor, layout[sensor])
            p_bs4 = fit22.room_to_bs4(full, p_room)
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            if bs == 4:
                pred = fit22.predict_bs4(p_bs4, sweep, factory_calibs.get(4), convention)
            elif bs == 10:
                pred = fit22.predict_bs10(p_bs4, sweep, bs10, factory_calibs.get(10), convention)
            else:
                continue
            sign = convention["signs"][(bs, sweep)]
            out.append(fit22.best_family_residual(pred, m.get("candidate_families", []), sign) / robust_scale)

    return np.array(out, dtype=float)


def rmse_deg(params, anchors, layout, factory_calibs, hypothesis, bs10_translation):
    r = residual_vector(params, anchors, layout, factory_calibs, 1.0, hypothesis, bs10_translation)
    return math.degrees(float(np.sqrt(np.mean(r * r))))


def make_start(room_guess, bs10_rvec_guess, starts):
    base = np.zeros(13, dtype=float)
    base[:6] = room_guess
    base[6:9] = bs10_rvec_guess
    shifts = [
        np.zeros(6),
        np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, -0.4, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, -0.3, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.0, 0.3, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.0, -0.3, 0.0]),
        np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
    out = []
    for shift in shifts[:max(1, int(starts))]:
        item = base.copy()
        item[:6] += shift
        out.append(item)
    return out


def fit_task(task):
    index, x0, lower, upper, anchors, layout, factory_calibs, robust_scale, max_nfev, hypothesis, bs10_translation = task
    result = least_squares(
        residual_vector,
        x0,
        bounds=(lower, upper),
        args=(anchors, layout, factory_calibs, robust_scale, hypothesis, bs10_translation),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=max_nfev,
        verbose=0,
    )
    return {
        "index": int(index),
        "x": result.x,
        "fun": result.fun,
        "success": bool(result.success),
        "message": str(result.message),
        "nfev": int(result.nfev),
        "hypothesis": hypothesis,
        "rmse_deg": rmse_deg(result.x, anchors, layout, factory_calibs, hypothesis, bs10_translation),
    }


def residual_report(params, anchors, layout, factory_calibs, hypothesis, bs10_translation):
    convention = convention_from_params(params, hypothesis)
    bs10 = np.concatenate([params[6:9], bs10_translation])
    full = params_to_full(params, bs10_translation)
    rows = []
    for anchor in anchors:
        name = anchor.get("name", "?")
        for m in anchor.get("measurements", []):
            sensor = int(m["sensor"])
            if sensor not in layout:
                continue
            p_room = fit22.sensor_anchor_room(anchor, layout[sensor])
            p_bs4 = fit22.room_to_bs4(full, p_room)
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            if bs == 4:
                pred = fit22.predict_bs4(p_bs4, sweep, factory_calibs.get(4), convention)
            elif bs == 10:
                pred = fit22.predict_bs10(p_bs4, sweep, bs10, factory_calibs.get(10), convention)
            else:
                continue
            sign = convention["signs"][(bs, sweep)]
            res = fit22.best_family_residual(pred, m.get("candidate_families", []), sign)
            rows.append({
                "pose": name,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "residual_deg": math.degrees(float(res)),
            })
    return sorted(rows, key=lambda row: abs(row["residual_deg"]), reverse=True)


def save_output(path, args, anchors, best, bs10_translation):
    params = best["x"]
    full = params_to_full(params, bs10_translation)
    summary = fit22.room_frame_lighthouse_summary(full)
    out = {
        "description": "v12 anchor fit with measured/fixed BS10 translation in the BS4 frame.",
        "input_anchor_poses": args.anchor_poses,
        "input_relative_geometry": args.relative_geometry,
        "reference_basestation": 4,
        "fit": {
            "anchor_poses_used": len(anchors),
            "residuals": int(best["fun"].size),
            "rmse_deg": float(best["rmse_deg"]),
            "success": bool(best["success"]),
            "message": str(best["message"]),
            "hypothesis": best["hypothesis"]["label"],
            "bs10_translation_fixed": True,
        },
        "basestations": [
            {
                "basestation": 4,
                "role": "reference",
                "world_to_lighthouse": {
                    "rotation_vector": [0.0, 0.0, 0.0],
                    "translation_m": [0.0, 0.0, 0.0],
                },
                "angle_offsets_deg": {
                    "sweep_0": float(math.degrees(params[9])),
                    "sweep_1": float(math.degrees(params[10])),
                },
            },
            {
                "basestation": 10,
                "role": "fixed_translation_solved_orientation",
                "world_to_lighthouse": {
                    "rotation_vector": [float(v) for v in params[6:9]],
                    "translation_m": [float(v) for v in bs10_translation],
                },
                "angle_offsets_deg": {
                    "sweep_0": float(math.degrees(params[11])),
                    "sweep_1": float(math.degrees(params[12])),
                },
            },
        ],
        "room_to_bs4": {
            "rotation_vector": [float(v) for v in params[:3]],
            "translation_m": [float(v) for v in params[3:6]],
        },
        "room_summary": {
            "bs4_room_position_m": [float(v) for v in summary["bs4_room_position"]],
            "bs10_room_position_m": [float(v) for v in summary["bs10_room_position"]],
            "bs10_room_offset_from_bs4_m": [float(v) for v in summary["bs10_room_offset"]],
            "bs4_orientation_xyz_deg": [float(v) for v in summary["bs4_euler_xyz_deg"]],
            "bs10_orientation_xyz_deg": [float(v) for v in summary["bs10_euler_xyz_deg"]],
        },
        "anchor_summary": {
            "poses_used": [anchor.get("name") for anchor in anchors],
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")


def main():
    fit22.configure_numeric_threads()
    parser = argparse.ArgumentParser(description="Anchor a measured/fixed BS4-BS10 block to room points.")
    parser.add_argument("--relative-geometry", default="config/lighthouse_relative_pnp_oldwave_cluster_d14.json")
    parser.add_argument("--anchor-poses", default="config/vertical_pose_variants/vertical_x_up_face_minus_y.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--factory-calibs", default="auto")
    parser.add_argument("--bs10-translation", default="", help="Fixed BS10 position in BS4 frame: x,y,z. Overrides --relative-geometry.")
    parser.add_argument("--bs10-rotation-guess", default="", help="Initial BS10 world_to_lighthouse rotation vector: rx,ry,rz.")
    parser.add_argument("--room-to-bs4-guess", default="", help="Initial room->BS4 rx,ry,rz,x,y,z.")
    parser.add_argument("--max-anchor-spread-deg", type=float, default=0.5)
    parser.add_argument("--position-bound", type=float, default=4.0)
    parser.add_argument("--max-nfev", type=int, default=500)
    parser.add_argument("--starts", type=int, default=4)
    parser.add_argument("--workers", type=int, default=fit22.cpu_count_default())
    parser.add_argument("--convention-search", choices=["v10", "simple", "all"], default="all")
    parser.add_argument("--output", default="config/lighthouse_anchor_from_measured_block.json")
    args = parser.parse_args()

    if args.bs10_translation:
        bs10_translation = parse_csv_floats(args.bs10_translation, 3, "--bs10-translation")
        bs10_rvec_guess = np.zeros(3, dtype=float)
    else:
        bs10_rvec_guess, bs10_translation = measured_bs10_from_geometry(args.relative_geometry)

    if args.bs10_rotation_guess:
        bs10_rvec_guess = parse_csv_floats(args.bs10_rotation_guess, 3, "--bs10-rotation-guess")

    if args.room_to_bs4_guess:
        room_guess = parse_csv_floats(args.room_to_bs4_guess, 6, "--room-to-bs4-guess")
    else:
        room_guess = room_seed_from_geometry("config/lighthouse_relative_refined_from_pnp_anchors_d127_all.json")

    _anchor_meta, anchors = fit22.load_anchor_poses(args.anchor_poses, args.max_anchor_spread_deg)
    if not anchors:
        raise SystemExit("No usable anchor poses after filtering.")
    layout = fit22.load_layout(args.layout)
    factory_calibs = fit22.load_factory_calibration_map(args.factory_calibs)
    hypotheses = fit22.make_hypotheses(args.convention_search)
    starts = make_start(room_guess, bs10_rvec_guess, args.starts)

    lower = np.array([-math.pi, -math.pi, -math.pi, -args.position_bound, -args.position_bound, -args.position_bound,
                      -math.pi, -math.pi, -math.pi, -math.pi, -math.pi, -math.pi, -math.pi], dtype=float)
    upper = np.array([math.pi, math.pi, math.pi, args.position_bound, args.position_bound, args.position_bound,
                      math.pi, math.pi, math.pi, math.pi, math.pi, math.pi, math.pi], dtype=float)
    robust_scale = math.radians(1.0)

    tasks = []
    index = 0
    for hypothesis in hypotheses:
        for start in starts:
            tasks.append((index, np.clip(start, lower + 1e-6, upper - 1e-6), lower, upper,
                          anchors, layout, factory_calibs, robust_scale, args.max_nfev, hypothesis, bs10_translation))
            index += 1

    print("=" * 88)
    print("v12 anchor measured Lighthouse block to room")
    print(f"Anchor poses: {args.anchor_poses}")
    print(f"Anchor poses used: {len(anchors)}")
    print(f"Fixed BS10 in BS4: x={bs10_translation[0]:+.3f}, y={bs10_translation[1]:+.3f}, z={bs10_translation[2]:+.3f} m")
    print(f"Distance: {np.linalg.norm(bs10_translation):.3f} m")
    print(f"starts={len(starts)} | hypotheses={len(hypotheses)} | tasks={len(tasks)} | workers={args.workers} | max_nfev={args.max_nfev}")
    print("=" * 88)

    best = None
    workers = max(1, int(args.workers or 1))
    if workers == 1 or len(tasks) == 1:
        for task in tasks:
            candidate = fit_task(task)
            if best is None or candidate["rmse_deg"] < best["rmse_deg"]:
                best = candidate
            print(f"  task {candidate['index'] + 1}/{len(tasks)} rmse={candidate['rmse_deg']:.3f} deg | best={best['rmse_deg']:.3f} deg | {candidate['hypothesis']['label']}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as executor:
            futures = [executor.submit(fit_task, task) for task in tasks]
            done = 0
            for future in as_completed(futures):
                done += 1
                candidate = future.result()
                if best is None or candidate["rmse_deg"] < best["rmse_deg"]:
                    best = candidate
                print(f"  task {done}/{len(tasks)} rmse={candidate['rmse_deg']:.3f} deg | best={best['rmse_deg']:.3f} deg | {candidate['hypothesis']['label']}", flush=True)

    save_output(args.output, args, anchors, best, bs10_translation)
    full = params_to_full(best["x"], bs10_translation)
    summary = fit22.room_frame_lighthouse_summary(full)
    worst = residual_report(best["x"], anchors, layout, factory_calibs, best["hypothesis"], bs10_translation)[:8]

    print("=" * 88)
    print(f"RMSE: {best['rmse_deg']:.3f} deg")
    print(f"BS4 room position: x={summary['bs4_room_position'][0]:+.3f}, y={summary['bs4_room_position'][1]:+.3f}, z={summary['bs4_room_position'][2]:+.3f} m")
    print(f"BS10 room position: x={summary['bs10_room_position'][0]:+.3f}, y={summary['bs10_room_position'][1]:+.3f}, z={summary['bs10_room_position'][2]:+.3f} m")
    print(f"BS10 room offset from BS4: x={summary['bs10_room_offset'][0]:+.3f}, y={summary['bs10_room_offset'][1]:+.3f}, z={summary['bs10_room_offset'][2]:+.3f} m")
    print(f"BS4 room orientation xyz: roll={summary['bs4_euler_xyz_deg'][0]:+.1f}, pitch={summary['bs4_euler_xyz_deg'][1]:+.1f}, yaw={summary['bs4_euler_xyz_deg'][2]:+.1f} deg")
    print(f"BS10 room orientation xyz: roll={summary['bs10_euler_xyz_deg'][0]:+.1f}, pitch={summary['bs10_euler_xyz_deg'][1]:+.1f}, yaw={summary['bs10_euler_xyz_deg'][2]:+.1f} deg")
    print(f"Best convention: {best['hypothesis']['label']}")
    print("Worst residuals:")
    for row in worst:
        print(f"  {row['pose']} sensor={row['sensor']} bs={row['basestation']} sweep={row['sweep']} residual={row['residual_deg']:+.2f} deg")
    print(f"Saved: {args.output}")
    print("=" * 88)


if __name__ == "__main__":
    main()
