#!/usr/bin/env python3

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import inspect
import json
import math
import os
from pathlib import Path
from statistics import median


def configure_numeric_threads():
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(name, "1")


configure_numeric_threads()

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from lh2_factory_model import (
    factory_axis_for_sweep,
    lh2_factory_angle,
    load_factory_calibration_map,
    serialize_factory_for_geometry,
)


TICKS_PER_REV = 120000
TILT_POS = math.pi / 6.0
TILT_NEG = -math.pi / 6.0


def cpu_count_default():
    return max(1, os.cpu_count() or 1)


def parse_float_list(value):
    if value is None or value == "":
        return None
    return [float(x) for x in value.split(",") if x.strip()]


def parse_str_list(value):
    if value is None or value == "":
        return None
    return [x.strip() for x in value.split(",") if x.strip()]


def least_squares_compat(*args, workers=1, **kwargs):
    if workers and workers != 1 and "workers" in inspect.signature(least_squares).parameters:
        kwargs["workers"] = workers
    return least_squares(*args, **kwargs)


def lfsr_to_raw_rad(lfsr, sweep, degrees_per_cycle):
    half_span = degrees_per_cycle / 2.0
    angle_deg = (((float(lfsr) % TICKS_PER_REV) / TICKS_PER_REV) * degrees_per_cycle) - half_span
    angle_rad = math.radians(angle_deg)

    if int(sweep) == 0:
        return angle_rad + math.pi / 3.0

    return angle_rad - math.pi / 3.0


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def load_layout(path):
    with open(path, "r") as f:
        data = json.load(f)

    sensors = {}
    for s in data["sensors"]:
        sensors[int(s["sensor"])] = np.array([
            float(s["x_m"]),
            float(s["y_m"]),
            float(s.get("z_m", 0.0)),
        ], dtype=float)
    return sensors


def load_poses(path, degrees_per_cycle, prefer_raw_angles=False):
    with open(path, "r") as f:
        data = json.load(f)

    obs = []

    for pose in data["poses"]:
        grouped = {}

        for m in pose["measurements"]:
            sensor = int(m["sensor"])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            angle_is_calibrated = False
            candidate_raw_angles = None
            if "candidate_families" in m:
                candidate_raw_angles = [
                    float(item["raw_angle_rad"])
                    for item in m.get("candidate_families", [])
                    if "raw_angle_rad" in item
                ]
            if not prefer_raw_angles and "calibrated_angle_rad" in m:
                raw = float(m["calibrated_angle_rad"])
                angle_is_calibrated = True
            elif "raw_angle_rad" in m:
                raw = float(m["raw_angle_rad"])
            else:
                raw = lfsr_to_raw_rad(float(m["median_lfsr_location"]), sweep, degrees_per_cycle)
            grouped.setdefault((sensor, bs, sweep, angle_is_calibrated), []).append(raw)
            if candidate_raw_angles and not angle_is_calibrated:
                grouped.setdefault((sensor, bs, sweep, "candidate_raw_angles"), []).extend(candidate_raw_angles)

        for (sensor, bs, sweep, angle_is_calibrated), values in grouped.items():
            if angle_is_calibrated == "candidate_raw_angles":
                continue
            candidate_values = grouped.get((sensor, bs, sweep, "candidate_raw_angles"))
            obs.append({
                "pose": pose["name"],
                "pose_data": pose,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "raw_angle": float(median(values)),
                "angle_is_calibrated": bool(angle_is_calibrated),
                "candidate_raw_angles": [float(value) for value in candidate_values] if candidate_values else None,
            })

    return obs


def sensor_world(pose, default_drone_z, local):
    roll = math.radians(float(pose.get("roll_deg", 0.0)))
    pitch = math.radians(float(pose.get("pitch_deg", 0.0)))
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))

    R = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    drone_pos = np.array([
        float(pose["x_m"]),
        float(pose["y_m"]),
        float(pose.get("z_m", default_drone_z)),
    ], dtype=float)

    return drone_pos + R @ local


def nominal_angle_variant(p_lh, tilt, variant):
    x, y, z = p_lh
    if variant == "xy_z":
        u, v, w = x, y, z
    elif variant == "xz_y":
        u, v, w = x, z, y
    elif variant == "yz_x":
        u, v, w = y, z, x
    elif variant == "yx_z":
        u, v, w = y, x, z
    else:
        raise ValueError(f"Unknown model variant: {variant}")

    r = math.sqrt(u * u + v * v)
    if r < 1e-9:
        r = 1e-9
    value = (w * math.tan(tilt)) / r
    value = max(-0.999999, min(0.999999, value))
    return math.atan2(v, u) + math.asin(value)


def factory_angle_variant(p_lh, nominal_tilt, axis_calibration, variant):
    if variant == "factory_xy_z":
        return lh2_factory_angle(p_lh, nominal_tilt, axis_calibration)

    tilt = nominal_tilt
    if axis_calibration is not None:
        tilt += float(axis_calibration["tilt"])
    angle = nominal_angle_variant(p_lh, tilt, variant)
    if axis_calibration is not None:
        angle += float(axis_calibration["phase"])
        angle += float(axis_calibration["gibmag"]) * math.cos(angle + float(axis_calibration["gibphase"]))
    return angle


def predict_angle(p_world, params, lighthouse_z, tilt, axis_calibration=None, solve_lighthouse_z=False, model_variant="factory_xy_z"):
    # params = rx, ry, rz, tx, ty, off0, off1
    # or, with solve_lighthouse_z: rx, ry, rz, tx, ty, tz, off0, off1
    if solve_lighthouse_z:
        rx, ry, rz, tx, ty, tz, off0, off1 = params
    else:
        rx, ry, rz, tx, ty, off0, off1 = params
        tz = lighthouse_z

    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, tz], dtype=float)

    p_lh = R @ (p_world - t)
    return factory_angle_variant(p_lh, tilt, axis_calibration, model_variant)


def corrected_raw(raw, sweep, params, signs, solve_lighthouse_z=False):
    offset_base = 6 if solve_lighthouse_z else 5
    off0 = params[offset_base]
    off1 = params[offset_base + 1]

    if sweep == 0:
        return signs[0] * raw + off0
    return signs[1] * raw + off1


def residuals(params, bs, observations, layout, lighthouse_z, drone_z, factory_entry, tilt_map, signs, axis_map, solve_lighthouse_z, model_variant):
    out = []

    for o in observations:
        if int(o["basestation"]) != int(bs):
            continue

        sensor = int(o["sensor"])
        if sensor not in layout:
            continue

        p_world = sensor_world(o["pose_data"], drone_z, layout[sensor])

        sweep = int(o["sweep"])
        pred = predict_angle(
            p_world,
            params,
            lighthouse_z,
            tilt_map[sweep],
            None if o.get("angle_is_calibrated") else factory_axis_for_sweep(factory_entry, axis_map[sweep]),
            solve_lighthouse_z,
            model_variant,
        )
        candidate_raw_angles = o.get("candidate_raw_angles")
        if candidate_raw_angles:
            diffs = [
                angle_diff(pred, corrected_raw(float(candidate), sweep, params, signs, solve_lighthouse_z))
                for candidate in candidate_raw_angles
            ]
            out.append(min(diffs, key=abs))
        else:
            meas = corrected_raw(float(o["raw_angle"]), sweep, params, signs, solve_lighthouse_z)
            out.append(angle_diff(pred, meas))

    return np.array(out, dtype=float)


def fit_hypothesis(task):
    (
        x0,
        lower,
        upper,
        bs,
        observations,
        layout,
        lighthouse_z,
        drone_z,
        factory_entry,
        tilt_map,
        signs,
        axis_map,
        solve_lighthouse_z,
        model_variant,
        max_nfev,
        scipy_workers,
    ) = task

    result = least_squares_compat(
        residuals,
        x0,
        bounds=(lower, upper),
        args=(bs, observations, layout, lighthouse_z, drone_z, factory_entry, tilt_map, signs, axis_map, solve_lighthouse_z, model_variant),
        loss="soft_l1",
        f_scale=math.radians(1.0),
        max_nfev=max_nfev,
        workers=scipy_workers,
    )

    err = residuals(
        result.x,
        bs,
        observations,
        layout,
        lighthouse_z,
        drone_z,
        factory_entry,
        tilt_map,
        signs,
        axis_map,
        solve_lighthouse_z,
        model_variant,
    )

    if len(err) == 0:
        return None

    rmse_rad = float(np.sqrt(np.mean(err ** 2)))
    return {
        "params": result.x,
        "rmse_rad": rmse_rad,
        "rmse_deg": float(math.degrees(rmse_rad)),
        "cost": float(result.cost),
        "success": bool(result.success),
        "num_residuals": int(len(err)),
        "tilt_map": dict(tilt_map),
        "signs": dict(signs),
        "axis_map": dict(axis_map),
        "model_variant": model_variant,
    }


def run_tasks(tasks, workers, label):
    best = None
    candidates = []
    workers = max(1, int(workers or 1))
    if workers == 1 or len(tasks) <= 1:
        for done, task in enumerate(tasks, start=1):
            cand = fit_hypothesis(task)
            if cand is not None:
                candidates.append(cand)
                if best is None or cand["rmse_rad"] < best["rmse_rad"]:
                    best = cand
            if done % max(1, len(tasks) // 10) == 0 or done == len(tasks):
                if best is not None:
                    print(f"  {label} {done}/{len(tasks)} | best RMSE={best['rmse_deg']:.3f} deg", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=workers) as executor:
            futures = [executor.submit(fit_hypothesis, task) for task in tasks]
            done = 0
            for future in as_completed(futures):
                done += 1
                cand = future.result()
                if cand is not None:
                    candidates.append(cand)
                    if best is None or cand["rmse_rad"] < best["rmse_rad"]:
                        best = cand
                if done % max(1, len(tasks) // 10) == 0 or done == len(tasks):
                    if best is not None:
                        print(f"  {label} {done}/{len(tasks)} | best RMSE={best['rmse_deg']:.3f} deg", flush=True)
    candidates.sort(key=lambda c: c["rmse_rad"])
    return best, candidates


def fit_bs(
    bs,
    observations,
    layout,
    lighthouse_z,
    drone_z,
    guess_x,
    guess_y,
    factory_entry=None,
    max_nfev=300,
    workers=1,
    coarse_nfev=0,
    refine_top_k=0,
    solve_lighthouse_z=False,
    xy_bound=4.0,
    z_min=0.6,
    z_max=2.5,
    broad_search=False,
    grid_x=None,
    grid_y=None,
    grid_z=None,
    model_variants=None,
):
    has_calibrated_measurements = any(
        int(o["basestation"]) == int(bs) and o.get("angle_is_calibrated")
        for o in observations
    )
    tilt_maps = [
        {0: TILT_POS, 1: TILT_NEG},
        {0: TILT_NEG, 1: TILT_POS},
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
    model_variants = model_variants or ["factory_xy_z"]

    rotation_guesses = [
        [0.0, 0.0, 0.0],
        [0.0, math.pi / 2.0, 0.0],
        [0.0, -math.pi / 2.0, 0.0],
    ]

    if broad_search:
        xs = grid_x or [-2.0, -1.0, 0.0, 1.0, 2.0]
        ys = grid_y or [-1.0, 0.5, 2.0, 3.5]
        z_guesses = grid_z or [1.0, 1.5, 2.0]
        position_guesses = [[x, y, z] for x in xs for y in ys for z in z_guesses]
    else:
        z_guesses = grid_z or [lighthouse_z]
        position_guesses = [
            [guess_x, guess_y, z_guesses[0]],
            [guess_x + 0.5, guess_y, z_guesses[0]],
            [guess_x - 0.5, guess_y, z_guesses[0]],
            [guess_x, guess_y + 0.5, z_guesses[0]],
            [guess_x, guess_y - 0.5, z_guesses[0]],
        ]

    if solve_lighthouse_z:
        lower = np.array([
            -math.pi, -math.pi, -math.pi,
            -xy_bound, -xy_bound, z_min,
            -math.pi, -math.pi,
        ], dtype=float)

        upper = np.array([
            +math.pi, +math.pi, +math.pi,
            +xy_bound, +xy_bound, z_max,
            +math.pi, +math.pi,
        ], dtype=float)
    else:
        lower = np.array([
            -math.pi, -math.pi, -math.pi,
            -xy_bound, -xy_bound,
            -math.pi, -math.pi,
        ], dtype=float)

        upper = np.array([
            +math.pi, +math.pi, +math.pi,
            +xy_bound, +xy_bound,
            +math.pi, +math.pi,
        ], dtype=float)

    tasks = []
    for model_variant in model_variants:
        for tilt_map in tilt_maps:
            for signs in sign_maps:
                for axis_map in axis_maps:
                    for rot0 in rotation_guesses:
                        for tx0, ty0, tz0 in position_guesses:
                            if solve_lighthouse_z:
                                x0 = np.array([
                                    rot0[0], rot0[1], rot0[2],
                                    tx0, ty0, tz0,
                                    0.0, 0.0,
                                ], dtype=float)
                            else:
                                x0 = np.array([
                                    rot0[0], rot0[1], rot0[2],
                                    tx0, ty0,
                                    0.0, 0.0,
                                ], dtype=float)

                            x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)
                            tasks.append((
                                x0,
                                lower,
                                upper,
                                bs,
                                observations,
                                layout,
                                lighthouse_z,
                                drone_z,
                                factory_entry,
                                tilt_map,
                                signs,
                                axis_map,
                                solve_lighthouse_z,
                                model_variant,
                                max_nfev,
                                1,
                            ))

    if coarse_nfev and coarse_nfev > 0 and coarse_nfev < max_nfev and refine_top_k and refine_top_k > 0:
        coarse_tasks = []
        for task in tasks:
            coarse_tasks.append((*task[:-2], int(coarse_nfev), task[-1]))

        print(
            f"  coarse pass: {len(coarse_tasks)} hypotheses x {coarse_nfev} evals, "
            f"then refine top {min(refine_top_k, len(coarse_tasks))}",
            flush=True,
        )
        coarse_best, coarse_candidates = run_tasks(coarse_tasks, workers, "coarse hypotheses")
        if not coarse_candidates:
            best = coarse_best
        else:
            refine_tasks = []
            for cand in coarse_candidates[:refine_top_k]:
                refine_tasks.append((
                    cand["params"],
                    lower,
                    upper,
                    bs,
                    observations,
                    layout,
                    lighthouse_z,
                    drone_z,
                    factory_entry,
                    cand["tilt_map"],
                    cand["signs"],
                    cand["axis_map"],
                    solve_lighthouse_z,
                    cand["model_variant"],
                    max_nfev,
                    1,
                ))
            best, _ = run_tasks(refine_tasks, workers, "refine hypotheses")
    else:
        best, _ = run_tasks(tasks, workers, "hypotheses")

    if best is None:
        raise RuntimeError(f"No solution for BS{bs}")

    if solve_lighthouse_z:
        rx, ry, rz, tx, ty, tz, off0, off1 = best["params"]
    else:
        rx, ry, rz, tx, ty, off0, off1 = best["params"]
        tz = lighthouse_z
    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    tilt_map = best["tilt_map"]
    signs = best["signs"]
    axis_map = best["axis_map"]
    model_variant = best["model_variant"]

    return {
        "basestation": int(bs),
        "model": "lh2_ultrafast_fixed_hypothesis_factory_corrected_measurements" if has_calibrated_measurements else ("lh2_ultrafast_fixed_hypothesis_with_factory_calibration" if factory_entry else "lh2_ultrafast_fixed_hypothesis"),
        "rmse_deg": float(best["rmse_deg"]),
        "num_residuals": int(best["num_residuals"]),
        "lighthouse_z_m": float(tz),
        "drone_z_m": float(drone_z),
        "measurements_factory_corrected": bool(has_calibrated_measurements),
        "model_variant": model_variant,
        "factory_calibration": None if has_calibrated_measurements else serialize_factory_for_geometry(factory_entry),
        "sweep_tilts": {
            "sweep_0_deg": float(math.degrees(tilt_map[0])),
            "sweep_1_deg": float(math.degrees(tilt_map[1])),
            "sweep_0_rad": float(tilt_map[0]),
            "sweep_1_rad": float(tilt_map[1]),
        },
        "angle_correction": {
            "sign_sweep_0": float(signs[0]),
            "sign_sweep_1": float(signs[1]),
            "offset_sweep_0_rad": float(off0),
            "offset_sweep_1_rad": float(off1),
            "offset_sweep_0_deg": float(math.degrees(off0)),
            "offset_sweep_1_deg": float(math.degrees(off1)),
        },
        "factory_axis_map": {
            "sweep_0_axis": int(axis_map[0]),
            "sweep_1_axis": int(axis_map[1]),
        },
        "world_to_lighthouse": {
            "rotation_vector": [float(rx), float(ry), float(rz)],
            "rotation_matrix": R.tolist(),
                "translation_m": [float(tx), float(ty), float(tz)],
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Ultra-fast guided LH2 geometry calibration.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_lh2_guided_ultrafast.json")
    parser.add_argument("--lighthouse-z", type=float, default=1.20)
    parser.add_argument("--drone-z", type=float, default=0.00)
    parser.add_argument(
        "--lfsr-degrees-per-cycle",
        type=float,
        default=360.0,
        help="Angle span represented by one LH2 LFSR cycle. Try 120, 240, or 360 when diagnosing raw RP2040 data.",
    )
    parser.add_argument("--bs4-guess", default="-0.50,1.20", help="Initial x,y guess for BS4. Default: left/front.")
    parser.add_argument("--bs10-guess", default="0.50,1.20", help="Initial x,y guess for BS10. Default: right/front.")
    parser.add_argument("--solve-lighthouse-z", action="store_true", help="Optimize Lighthouse z instead of using --lighthouse-z as a fixed height.")
    parser.add_argument("--xy-bound", type=float, default=4.0, help="Absolute x/y search bound in meters when fitting Lighthouse position.")
    parser.add_argument("--z-min", type=float, default=0.60, help="Minimum Lighthouse z when --solve-lighthouse-z is enabled.")
    parser.add_argument("--z-max", type=float, default=2.50, help="Maximum Lighthouse z when --solve-lighthouse-z is enabled.")
    parser.add_argument("--broad-search", action="store_true", help="Use a room-wide grid of initial x/y/z guesses instead of only guesses near BS4/BS10 defaults.")
    parser.add_argument("--grid-x", default=None, help="Comma-separated x initial guesses for --broad-search, example: -2,-1,0,1,2")
    parser.add_argument("--grid-y", default=None, help="Comma-separated y initial guesses for --broad-search, example: -1,0.5,2,3.5")
    parser.add_argument("--grid-z", default=None, help="Comma-separated z initial guesses for --solve-lighthouse-z, example: 1.0,1.5,2.0")
    parser.add_argument(
        "--prefer-raw-angles",
        action="store_true",
        help="Ignore calibrated_angle_rad in pose files and fit from raw_angle_rad plus factory model. Useful to diagnose factory correction direction/convention.",
    )
    parser.add_argument(
        "--model-variants",
        default="factory_xy_z",
        help=(
            "Comma-separated LH2 model variants to test. "
            "Use 'all' to test factory_xy_z,xy_z,xz_y,yz_x,yx_z."
        ),
    )
    parser.add_argument("--max-nfev", type=int, default=300, help="Max optimizer evaluations per hypothesis.")
    parser.add_argument(
        "--coarse-nfev",
        type=int,
        default=0,
        help="Optional fast first pass evaluations per hypothesis. Use with --refine-top-k for quicker fits.",
    )
    parser.add_argument(
        "--refine-top-k",
        type=int,
        default=0,
        help="After --coarse-nfev, refine only the best K hypotheses with --max-nfev. 0 disables two-stage fitting.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=cpu_count_default(),
        help="Parallel worker processes for independent fit hypotheses. Use 1 to disable. Default: all CPU cores.",
    )
    parser.add_argument(
        "--factory-calibs",
        default="auto",
        help=(
            "Factory calibration JSON map. Default 'auto' loads "
            "config/lighthouse_factory_calibration_bs4.json and bs10.json when present. "
            "Use 'none' to disable or '4=path,10=path' to set explicit files."
        ),
    )
    args = parser.parse_args()

    layout = load_layout(args.layout)
    obs = load_poses(args.poses, args.lfsr_degrees_per_cycle, args.prefer_raw_angles)
    factory_calibs = load_factory_calibration_map(args.factory_calibs)

    bs4_guess = [float(x) for x in args.bs4_guess.split(",")]
    bs10_guess = [float(x) for x in args.bs10_guess.split(",")]
    grid_x = parse_float_list(args.grid_x)
    grid_y = parse_float_list(args.grid_y)
    grid_z = parse_float_list(args.grid_z)
    if str(args.model_variants).strip().lower() == "all":
        model_variants = ["factory_xy_z", "xy_z", "xz_y", "yz_x", "yx_z"]
    else:
        model_variants = parse_str_list(args.model_variants) or ["factory_xy_z"]

    print("=" * 70)
    print("Ultra-fast guided LH2 geometry calibration")
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print(f"BS4 guess:  {bs4_guess}")
    print(f"BS10 guess: {bs10_guess}")
    print(f"Search: xy_bound=+/-{args.xy_bound:.2f} m")
    if args.solve_lighthouse_z:
        print(f"Solve Lighthouse z: {args.z_min:.2f}..{args.z_max:.2f} m")
    else:
        print(f"Fixed Lighthouse z: {args.lighthouse_z:.2f} m")
    if args.broad_search:
        print("Broad search: enabled")
    print(f"Model variants: {','.join(model_variants)}")
    print(f"LFSR scale: {args.lfsr_degrees_per_cycle:.1f} deg/cycle")
    print(f"Workers: {args.workers}")
    if args.coarse_nfev and args.refine_top_k:
        print(f"Two-stage fit: coarse_nfev={args.coarse_nfev}, refine_top_k={args.refine_top_k}")
    if factory_calibs:
        loaded = ", ".join(f"BS{bs}:{entry['path']}" for bs, entry in sorted(factory_calibs.items()))
        print(f"Factory: {loaded}")
    else:
        print("Factory: disabled/not found")
    print(f"Pose angles: {'raw_angle_rad + factory model' if args.prefer_raw_angles else 'calibrated_angle_rad when present'}")
    print(f"Observations: {len(obs)}")
    print("=" * 70)

    results = []

    for bs, guess in [(4, bs4_guess), (10, bs10_guess)]:
        print()
        print(f"Fitting BS{bs} around x={guess[0]:+.2f}, y={guess[1]:+.2f} ...")

        geom = fit_bs(
            bs,
            obs,
            layout,
            args.lighthouse_z,
            args.drone_z,
            guess[0],
            guess[1],
            factory_calibs.get(bs),
            args.max_nfev,
            args.workers,
            args.coarse_nfev,
            args.refine_top_k,
            args.solve_lighthouse_z,
            args.xy_bound,
            args.z_min,
            args.z_max,
            args.broad_search,
            grid_x,
            grid_y,
            grid_z,
            model_variants,
        )

        results.append(geom)

        t = geom["world_to_lighthouse"]["translation_m"]
        corr = geom["angle_correction"]
        tilts = geom["sweep_tilts"]
        axis_map = geom["factory_axis_map"]
        factory_status = "on" if geom.get("factory_calibration") else "off"

        print(f"BS{bs}")
        print(f"  RMSE: {geom['rmse_deg']:.4f} deg")
        print(f"  translation: x={t[0]:+.3f}, y={t[1]:+.3f}, z={t[2]:+.3f} m")
        print(f"  tilts: sweep0={tilts['sweep_0_deg']:+.1f} deg, sweep1={tilts['sweep_1_deg']:+.1f} deg")
        print(f"  signs: sweep0={corr['sign_sweep_0']:+.0f}, sweep1={corr['sign_sweep_1']:+.0f}")
        print(f"  model variant: {geom.get('model_variant', 'unknown')}")
        print(f"  offsets: sweep0={corr['offset_sweep_0_deg']:+.2f} deg, sweep1={corr['offset_sweep_1_deg']:+.2f} deg")
        print(
            "  factory correction: "
            f"{factory_status}, sweep0->axis{axis_map['sweep_0_axis']}, sweep1->axis{axis_map['sweep_1_axis']}"
        )

    output = {
        "description": "Ultra-fast guided LH2 geometry calibration.",
        "input_layout": args.layout,
        "input_poses": args.poses,
        "lighthouse_z_m": args.lighthouse_z,
        "drone_z_m": args.drone_z,
        "lfsr_degrees_per_cycle": args.lfsr_degrees_per_cycle,
        "factory_calibs": args.factory_calibs,
        "basestations": results,
    }

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    with open(out, "w") as f:
        json.dump(output, f, indent=2)

    print()
    print("=" * 70)
    print(f"Saved: {out}")
    print("=" * 70)


if __name__ == "__main__":
    main()
