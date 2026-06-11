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


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


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


def room_sensor_point(anchor, local):
    return fit22.sensor_anchor_room(anchor, local)


def group_anchor_sensor_observations(anchors, layout, min_channels):
    groups = []
    for anchor in anchors:
        by_sensor = {}
        for m in anchor.get("measurements", []):
            sensor = int(m["sensor"])
            if sensor in layout:
                by_sensor.setdefault(sensor, []).append(m)
        for sensor, measurements in by_sensor.items():
            channels = {(int(m["basestation"]), int(m["sweep"])) for m in measurements}
            if len(channels) < min_channels:
                continue
            groups.append({
                "pose": anchor.get("name", "?"),
                "sensor": sensor,
                "measurements": measurements,
                "room_point": room_sensor_point(anchor, layout[sensor]).tolist(),
            })
    return groups


def point_angle_residuals(p_bs4, measurements, factory_calibs, hypothesis, bs10, offsets):
    out = []
    convention = {
        "bs4_tilts": hypothesis["bs4_tilts"],
        "bs4_axis_map": hypothesis["bs4_axis_map"],
        "bs10_tilts": hypothesis["bs10_tilts"],
        "bs10_axis_map": hypothesis["bs10_axis_map"],
        "signs": hypothesis["signs"],
        "bs4_offsets": offsets[4],
        "bs10_offsets": offsets[10],
    }
    for m in measurements:
        bs = int(m["basestation"])
        sweep = int(m["sweep"])
        if bs == 4:
            pred = fit22.predict_bs4(p_bs4, sweep, factory_calibs.get(4), convention)
        elif bs == 10:
            pred = fit22.predict_bs10(p_bs4, sweep, bs10, factory_calibs.get(10), convention)
        else:
            continue
        sign = convention["signs"][(bs, sweep)]
        out.append(fit22.best_family_residual(pred, m.get("candidate_families", []), sign))
    return np.array(out, dtype=float)


def solve_sensor_point(task):
    index, group, factory_calibs, hypothesis, bs10, offsets, starts, max_nfev, max_angle_rmse_deg = task
    measurements = group["measurements"]
    best = None

    def residual(x):
        return point_angle_residuals(x, measurements, factory_calibs, hypothesis, bs10, offsets)

    for x0 in starts:
        result = least_squares(
            residual,
            np.array(x0, dtype=float),
            bounds=([-4.0, -4.0, -1.0], [4.0, 4.0, 4.0]),
            loss="soft_l1",
            f_scale=math.radians(1.0),
            max_nfev=max_nfev,
        )
        raw = residual(result.x)
        rmse_deg = math.degrees(float(np.sqrt(np.mean(raw * raw)))) if raw.size else float("inf")
        candidate = {
            "index": index,
            "pose": group["pose"],
            "sensor": int(group["sensor"]),
            "point_bs4": result.x.tolist(),
            "point_room": group["room_point"],
            "angle_rmse_deg": rmse_deg,
            "channels": len(measurements),
            "success": bool(result.success),
        }
        if best is None or candidate["angle_rmse_deg"] < best["angle_rmse_deg"]:
            best = candidate

    if best is None or best["angle_rmse_deg"] > max_angle_rmse_deg:
        return None
    return best


def rigid_transform_3d(a_points, b_points):
    a = np.asarray(a_points, dtype=float)
    b = np.asarray(b_points, dtype=float)
    ca = np.mean(a, axis=0)
    cb = np.mean(b, axis=0)
    aa = a - ca
    bb = b - cb
    h = aa.T @ bb
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = cb - r @ ca
    return r, t


def transform_points(r, t, points):
    p = np.asarray(points, dtype=float)
    return (r @ p.T).T + t


def fit_hypothesis(task):
    hyp_index, hypothesis, groups, factory_calibs, bs10, offsets, point_starts, args = task
    point_tasks = [
        (idx, group, factory_calibs, hypothesis, bs10, offsets, point_starts, args.point_max_nfev, args.max_point_angle_rmse_deg)
        for idx, group in enumerate(groups)
    ]
    solved = []
    for item in point_tasks:
        solved_item = solve_sensor_point(item)
        if solved_item is not None:
            solved.append(solved_item)

    if len(solved) < args.min_points:
        return None

    a = np.array([item["point_bs4"] for item in solved], dtype=float)
    b = np.array([item["point_room"] for item in solved], dtype=float)
    r, t = rigid_transform_3d(a, b)
    aligned = transform_points(r, t, a)
    errors = np.linalg.norm(aligned - b, axis=1)
    keep = np.ones(len(solved), dtype=bool)

    for _ in range(max(0, int(args.trim_rounds))):
        if int(np.sum(keep)) <= args.min_points:
            break
        kept_errors = errors[keep]
        cutoff = max(args.max_align_error_m, float(np.percentile(kept_errors, args.trim_percentile)))
        new_keep = keep & (errors <= cutoff)
        if np.array_equal(new_keep, keep) or int(np.sum(new_keep)) < args.min_points:
            break
        keep = new_keep
        r, t = rigid_transform_3d(a[keep], b[keep])
        aligned = transform_points(r, t, a)
        errors = np.linalg.norm(aligned - b, axis=1)

    kept = [item for item, ok in zip(solved, keep) if ok]
    rejected = [item for item, ok in zip(solved, keep) if not ok]
    kept_errors = errors[keep]
    rmse_m = float(np.sqrt(np.mean(kept_errors * kept_errors)))
    median_m = float(np.median(kept_errors))
    p95_m = float(np.percentile(kept_errors, 95))

    bs10_room = r @ bs10[3:6] + t
    bs4_room = t
    bs4_rot_room = r
    bs10_rot_room = r @ Rotation.from_rotvec(bs10[:3]).as_matrix().T

    return {
        "hypothesis_index": int(hyp_index),
        "hypothesis": hypothesis["label"],
        "rmse_m": rmse_m,
        "median_m": median_m,
        "p95_m": p95_m,
        "points_used": len(kept),
        "points_solved": len(solved),
        "points_rejected": len(rejected),
        "room_from_bs4": {
            "rotation_matrix": bs4_rot_room.tolist(),
            "rotation_vector": Rotation.from_matrix(bs4_rot_room).as_rotvec().tolist(),
            "translation_m": bs4_room.tolist(),
        },
        "basestations": {
            "bs4": {
                "position_m": bs4_room.tolist(),
                "orientation_xyz_deg": Rotation.from_matrix(bs4_rot_room).as_euler("xyz", degrees=True).tolist(),
            },
            "bs10": {
                "position_m": bs10_room.tolist(),
                "orientation_xyz_deg": Rotation.from_matrix(bs10_rot_room).as_euler("xyz", degrees=True).tolist(),
            },
            "bs10_offset_from_bs4_room_m": (bs10_room - bs4_room).tolist(),
        },
        "points": [
            {
                **item,
                "align_error_m": float(err),
            }
            for item, err, ok in zip(solved, errors, keep)
            if ok
        ],
        "rejected_points": [
            {
                **item,
                "align_error_m": float(err),
            }
            for item, err, ok in zip(solved, errors, keep)
            if not ok
        ],
    }


def main():
    fit22.configure_numeric_threads()
    parser = argparse.ArgumentParser(description="Anchor a relative Lighthouse block by reconstructing fixed sensor points and rigid-aligning them to room points.")
    parser.add_argument("--relative-geometry", default="config/lighthouse_relative_from_wave_d11.json")
    parser.add_argument("--anchor-poses", default="config/vertical_pose_variants/vertical_x_up_face_minus_y.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--factory-calibs", default="auto")
    parser.add_argument("--angle-geometry", default="", help="Optional fitted geometry to reuse angle offsets and, when possible, the winning convention.")
    parser.add_argument("--max-anchor-spread-deg", type=float, default=0.5)
    parser.add_argument("--min-channels", type=int, default=3)
    parser.add_argument("--min-points", type=int, default=8)
    parser.add_argument("--point-max-nfev", type=int, default=120)
    parser.add_argument("--max-point-angle-rmse-deg", type=float, default=2.0)
    parser.add_argument("--trim-rounds", type=int, default=2)
    parser.add_argument("--trim-percentile", type=float, default=85.0)
    parser.add_argument("--max-align-error-m", type=float, default=0.08)
    parser.add_argument("--workers", type=int, default=fit22.cpu_count_default())
    parser.add_argument("--convention-search", choices=["v10", "simple", "all"], default="all")
    parser.add_argument("--output", default="config/lighthouse_rigid_point_anchor.json")
    args = parser.parse_args()

    bs10_rvec, bs10_t = measured_bs10_from_geometry(args.relative_geometry)
    bs10 = np.concatenate([bs10_rvec, bs10_t])
    _anchor_meta, anchors = fit22.load_anchor_poses(args.anchor_poses, args.max_anchor_spread_deg)
    layout = fit22.load_layout(args.layout)
    factory_calibs = fit22.load_factory_calibration_map(args.factory_calibs)
    groups = group_anchor_sensor_observations(anchors, layout, args.min_channels)
    hypotheses = fit22.make_hypotheses(args.convention_search)
    offsets = {4: {0: 0.0, 1: 0.0}, 10: {0: 0.0, 1: 0.0}}

    if args.angle_geometry:
        angle_data = load_json(args.angle_geometry)
        fit_hypothesis_label = angle_data.get("fit", {}).get("hypothesis")
        if fit_hypothesis_label:
            filtered = [h for h in hypotheses if h["label"] == fit_hypothesis_label]
            if filtered:
                hypotheses = filtered
        for item in angle_data.get("basestations", []):
            bs = int(item.get("basestation"))
            angle_offsets = item.get("angle_offsets_deg", {})
            if bs in offsets and angle_offsets:
                offsets[bs] = {
                    0: math.radians(float(angle_offsets.get("sweep_0", 0.0))),
                    1: math.radians(float(angle_offsets.get("sweep_1", 0.0))),
                }

    point_starts = [
        [0.0, 0.5, 0.5],
        [0.0, 1.0, 0.5],
        [0.0, 1.5, 0.5],
        [0.5, 1.5, 0.5],
        [-0.5, 1.5, 0.5],
        [0.0, 2.0, 0.8],
        [0.0, 1.0, 1.2],
    ]

    tasks = [
        (idx, hypothesis, groups, factory_calibs, bs10, offsets, point_starts, args)
        for idx, hypothesis in enumerate(hypotheses)
    ]

    print("=" * 88)
    print("Rigid point cloud Lighthouse anchor")
    print(f"Relative geometry: {args.relative_geometry}")
    print(f"Anchor poses:      {args.anchor_poses}")
    print(f"Anchor poses used: {len(anchors)}")
    print(f"Sensor point groups: {len(groups)}")
    print(f"BS10 in BS4: x={bs10_t[0]:+.3f}, y={bs10_t[1]:+.3f}, z={bs10_t[2]:+.3f} m")
    if args.angle_geometry:
        print(f"Angle geometry: {args.angle_geometry}")
        print(
            "Offsets deg: "
            f"BS4=({math.degrees(offsets[4][0]):+.2f},{math.degrees(offsets[4][1]):+.2f}) "
            f"BS10=({math.degrees(offsets[10][0]):+.2f},{math.degrees(offsets[10][1]):+.2f})"
        )
    print(f"hypotheses={len(hypotheses)} | workers={args.workers} | point max_nfev={args.point_max_nfev}")
    print("=" * 88)

    best = None
    candidates = []
    workers = max(1, int(args.workers or 1))
    if workers == 1 or len(tasks) == 1:
        for done, task in enumerate(tasks, start=1):
            candidate = fit_hypothesis(task)
            if candidate is not None:
                candidates.append(candidate)
                if best is None or candidate["rmse_m"] < best["rmse_m"]:
                    best = candidate
            if done % max(1, len(tasks) // 10) == 0 or done == len(tasks):
                label = "none" if best is None else f"{best['rmse_m'] * 1000.0:.1f} mm"
                print(f"  task {done}/{len(tasks)} | best={label}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as executor:
            futures = [executor.submit(fit_hypothesis, task) for task in tasks]
            done = 0
            for future in as_completed(futures):
                done += 1
                candidate = future.result()
                if candidate is not None:
                    candidates.append(candidate)
                    if best is None or candidate["rmse_m"] < best["rmse_m"]:
                        best = candidate
                if done % max(1, len(tasks) // 10) == 0 or done == len(tasks):
                    label = "none" if best is None else f"{best['rmse_m'] * 1000.0:.1f} mm"
                    print(f"  task {done}/{len(tasks)} | best={label}", flush=True)

    if best is None:
        raise SystemExit("No hypothesis produced enough reconstructed points.")

    candidates.sort(key=lambda item: item["rmse_m"])
    output = {
        "description": "Rigid SVD anchor from reconstructed fixed sensor points. Experimental diagnostic geometry.",
        "input_relative_geometry": args.relative_geometry,
        "input_anchor_poses": args.anchor_poses,
        "reference_basestation": 4,
        "fit": best,
        "top_candidates": [
            {
                "hypothesis": item["hypothesis"],
                "rmse_m": item["rmse_m"],
                "median_m": item["median_m"],
                "p95_m": item["p95_m"],
                "points_used": item["points_used"],
                "points_solved": item["points_solved"],
                "points_rejected": item["points_rejected"],
            }
            for item in candidates[:10]
        ],
    }
    save_json(args.output, output)

    bs = best["basestations"]
    worst = sorted(best["points"], key=lambda item: abs(item["align_error_m"]), reverse=True)[:8]
    print("=" * 88)
    print(f"Rigid align RMSE: {best['rmse_m'] * 1000.0:.1f} mm | median={best['median_m'] * 1000.0:.1f} mm | p95={best['p95_m'] * 1000.0:.1f} mm")
    print(f"Points: used={best['points_used']} solved={best['points_solved']} rejected={best['points_rejected']}")
    print(f"BS4 room position: x={bs['bs4']['position_m'][0]:+.3f}, y={bs['bs4']['position_m'][1]:+.3f}, z={bs['bs4']['position_m'][2]:+.3f} m")
    print(f"BS10 room position: x={bs['bs10']['position_m'][0]:+.3f}, y={bs['bs10']['position_m'][1]:+.3f}, z={bs['bs10']['position_m'][2]:+.3f} m")
    off = bs["bs10_offset_from_bs4_room_m"]
    print(f"BS10 room offset from BS4: x={off[0]:+.3f}, y={off[1]:+.3f}, z={off[2]:+.3f} m")
    print(f"BS4 room orientation xyz: roll={bs['bs4']['orientation_xyz_deg'][0]:+.1f}, pitch={bs['bs4']['orientation_xyz_deg'][1]:+.1f}, yaw={bs['bs4']['orientation_xyz_deg'][2]:+.1f} deg")
    print(f"BS10 room orientation xyz: roll={bs['bs10']['orientation_xyz_deg'][0]:+.1f}, pitch={bs['bs10']['orientation_xyz_deg'][1]:+.1f}, yaw={bs['bs10']['orientation_xyz_deg'][2]:+.1f} deg")
    print(f"Best convention: {best['hypothesis']}")
    print("Worst aligned points:")
    for row in worst:
        print(f"  {row['pose']} sensor={row['sensor']} align={row['align_error_m'] * 1000.0:.1f} mm angle_rmse={row['angle_rmse_deg']:.2f} deg")
    if best["rejected_points"]:
        print("Rejected points:")
        for row in sorted(best["rejected_points"], key=lambda item: abs(item["align_error_m"]), reverse=True)[:8]:
            print(f"  {row['pose']} sensor={row['sensor']} align={row['align_error_m'] * 1000.0:.1f} mm angle_rmse={row['angle_rmse_deg']:.2f} deg")
    print(f"Saved: {args.output}")
    print("=" * 88)


if __name__ == "__main__":
    main()
