#!/usr/bin/env python3

import argparse
import inspect
import json
import math
import os
from pathlib import Path


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

import importlib.util

import lh2_factory_model
import lh2v10


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HERE = Path(__file__).resolve().parent
geom_fit = load_module("geom_fit_v10", HERE / "04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py")
live = load_module("live_position_v10", HERE / "05_live_position.py")


def cpu_count_default():
    return max(1, os.cpu_count() or 1)


def least_squares_compat(*args, workers=1, **kwargs):
    if workers and workers != 1 and "workers" in inspect.signature(least_squares).parameters:
        kwargs["workers"] = workers
    return least_squares(*args, **kwargs)


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def pose_matrix_from_known_pose(pose):
    roll = math.radians(float(pose.get("roll_deg", 0.0)))
    pitch = math.radians(float(pose.get("pitch_deg", 0.0)))
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
    return Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()


def sensor_world_known(pose, sensor_local, default_z):
    pos = np.array([
        float(pose["x_m"]),
        float(pose["y_m"]),
        float(pose.get("z_m", default_z)),
    ], dtype=float)
    return pos + pose_matrix_from_known_pose(pose) @ sensor_local


def sensor_world_wave(params, sensor_local):
    x, y, z, rx, ry, rz = params
    return np.array([x, y, z], dtype=float) + Rotation.from_rotvec([rx, ry, rz]).as_matrix() @ sensor_local


def bs_params_from_geometry(geometry):
    out = {}
    for bs, item in geometry.items():
        rotvec = Rotation.from_matrix(item["rotation"]).as_rotvec()
        t = item["translation"]
        out[int(bs)] = np.array([
            rotvec[0],
            rotvec[1],
            rotvec[2],
            t[0],
            t[1],
            t[2],
            item["offsets"][0],
            item["offsets"][1],
        ], dtype=float)
    return out


def geometry_from_bs_params(bs_order, params, template):
    out = {}
    for idx, bs in enumerate(bs_order):
        base = idx * 8
        rx, ry, rz, tx, ty, tz, off0, off1 = params[base:base + 8]
        prev = template[int(bs)]
        item = {
            "rotation": Rotation.from_rotvec([rx, ry, rz]).as_matrix(),
            "translation": np.array([tx, ty, tz], dtype=float),
            "tilts": prev["tilts"],
            "signs": prev["signs"],
            "offsets": {0: float(off0), 1: float(off1)},
            "factory_axes": prev.get("factory_axes"),
            "lfsr_degrees_per_cycle": prev.get("lfsr_degrees_per_cycle", 360.0),
        }
        out[int(bs)] = item
    return out


def measured_observation_angle(obs, bs_geom, sweep):
    if "calibrated_angle_rad" in obs:
        raw = float(obs["calibrated_angle_rad"])
    elif "raw_angle_rad" in obs:
        raw = float(obs["raw_angle_rad"])
    else:
        raw = live.lfsr_to_raw_rad(
            float(obs["lfsr_location"]),
            int(sweep),
            bs_geom.get("lfsr_degrees_per_cycle", 360.0),
        )
    return bs_geom["signs"][int(sweep)] * raw + bs_geom["offsets"][int(sweep)]


def predict_angle(p_world, bs_geom, sweep):
    sweep = int(sweep)
    p_lh = bs_geom["rotation"] @ (p_world - bs_geom["translation"])
    factory_axes = bs_geom.get("factory_axes")
    axis_calibration = factory_axes.get(sweep) if factory_axes else None
    return lh2_factory_model.lh2_factory_angle(p_lh, bs_geom["tilts"][sweep], axis_calibration)


def residuals(all_params, ctx):
    bs_count = len(ctx["bs_order"])
    bs_params = all_params[:bs_count * 8]
    wave_params = all_params[bs_count * 8:]
    geometry = geometry_from_bs_params(ctx["bs_order"], bs_params, ctx["geometry_template"])
    out = []

    point_weight = ctx["point_weight"]
    for obs in ctx["point_observations"]:
        sensor = int(obs["sensor"])
        bs = int(obs["basestation"])
        sweep = int(obs["sweep"])
        if sensor not in ctx["layout"] or bs not in geometry:
            continue
        p_world = sensor_world_known(obs["pose_data"], ctx["layout"][sensor], ctx["default_drone_z"])
        pred = predict_angle(p_world, geometry[bs], sweep)
        meas = measured_observation_angle(obs, geometry[bs], sweep)
        out.append(point_weight * angle_diff(pred, meas))

    wave_weight = ctx["wave_weight"]
    for frame_idx, frame in enumerate(ctx["wave_frames"]):
        pose = wave_params[frame_idx * 6:frame_idx * 6 + 6]
        for obs in frame["measurements"]:
            sensor = int(obs["sensor"])
            bs = int(obs["basestation"])
            sweep = int(obs["sweep"])
            if sensor not in ctx["layout"] or bs not in geometry:
                continue
            p_world = sensor_world_wave(pose, ctx["layout"][sensor])
            pred = predict_angle(p_world, geometry[bs], sweep)
            meas = measured_observation_angle(obs, geometry[bs], sweep)
            out.append(wave_weight * angle_diff(pred, meas))

    if ctx["geometry_prior_weight"] > 0.0:
        out.extend((all_params[:bs_count * 8] - ctx["initial_bs_params"]) * ctx["geometry_prior_weight"])

    return np.array(out, dtype=float)


def load_wave_frames(path, max_frames, min_observations, min_sensors, min_basestations):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    frames = []
    for frame in data.get("frames", []):
        measurements = frame.get("measurements", [])
        counts = lh2v10.observation_quality_counts(measurements)
        if counts["channels"] < min_observations:
            continue
        if counts["sensors"] < min_sensors:
            continue
        if counts["basestations"] < min_basestations:
            continue
        frames.append({
            "pc_time_s": frame.get("pc_time_s"),
            "measurements": measurements,
            "quality": counts,
        })

    if max_frames > 0 and len(frames) > max_frames:
        idx = np.linspace(0, len(frames) - 1, max_frames).round().astype(int)
        frames = [frames[int(i)] for i in idx]

    return frames


def initialize_wave_poses(frames, layout, geometry, max_rmse_deg):
    poses = []
    kept = []
    previous = None
    for frame in frames:
        pose, rmse_deg, used, success = live.solve_pose(
            frame["measurements"],
            layout,
            geometry,
            previous,
            solve_attitude=True,
            bounds_xy=5.0,
            bounds_z=(-0.50, 3.00),
        )
        if success and used >= 6 and rmse_deg <= max_rmse_deg:
            poses.append(pose)
            kept.append(frame)
            previous = pose
    return kept, poses


def serialize_geometry(output_path, source_geometry_path, layout_path, points_path, wave_path, bs_order, params, template, ctx, result):
    geometry = geometry_from_bs_params(bs_order, params[:len(bs_order) * 8], template)
    errors = residuals(params, ctx)
    rmse_deg = float(math.degrees(math.sqrt(float(np.mean(errors ** 2))))) if len(errors) else float("nan")

    basestations = []
    for bs in bs_order:
        item = geometry[bs]
        rotvec = Rotation.from_matrix(item["rotation"]).as_rotvec()
        source = template[bs]
        basestations.append({
            "basestation": int(bs),
            "model": "lh2_points_plus_wave_refined",
            "rmse_deg": rmse_deg,
            "num_residuals": int(len(errors)),
            "measurements_factory_corrected": bool(source.get("factory_axes") is None),
            "factory_calibration": None,
            "sweep_tilts": {
                "sweep_0_deg": float(math.degrees(item["tilts"][0])),
                "sweep_1_deg": float(math.degrees(item["tilts"][1])),
                "sweep_0_rad": float(item["tilts"][0]),
                "sweep_1_rad": float(item["tilts"][1]),
            },
            "angle_correction": {
                "sign_sweep_0": float(item["signs"][0]),
                "sign_sweep_1": float(item["signs"][1]),
                "offset_sweep_0_rad": float(item["offsets"][0]),
                "offset_sweep_1_rad": float(item["offsets"][1]),
                "offset_sweep_0_deg": float(math.degrees(item["offsets"][0])),
                "offset_sweep_1_deg": float(math.degrees(item["offsets"][1])),
            },
            "factory_axis_map": {
                "sweep_0_axis": 0,
                "sweep_1_axis": 1,
            },
            "world_to_lighthouse": {
                "rotation_vector": [float(v) for v in rotvec],
                "rotation_matrix": item["rotation"].tolist(),
                "translation_m": [float(v) for v in item["translation"]],
            },
        })

    out = {
        "description": "Lighthouse geometry refined with anchored known points plus moving wand-wave rigid-body constraints.",
        "source_geometry": str(source_geometry_path),
        "input_layout": str(layout_path),
        "input_points": str(points_path),
        "input_wave": str(wave_path),
        "wave_frames_used": len(ctx["wave_frames"]),
        "point_observations": len(ctx["point_observations"]),
        "weighted_rmse_deg": rmse_deg,
        "optimizer_success": bool(result.success),
        "optimizer_message": str(result.message),
        "lfsr_degrees_per_cycle": 360.0,
        "basestations": basestations,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    return rmse_deg


def main():
    parser = argparse.ArgumentParser(description="Refine Lighthouse geometry using both known points and wand-wave motion.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--points", default="config/wand_calibration_poses_3d.json")
    parser.add_argument("--wave", default="config/wand_wave_record.json")
    parser.add_argument("--input-geometry", default="config/lighthouse_geometry_wand_3d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_points_plus_wave.json")
    parser.add_argument("--max-wave-frames", type=int, default=80)
    parser.add_argument("--max-wave-init-rmse-deg", type=float, default=6.0)
    parser.add_argument("--min-wave-observations", type=int, default=6)
    parser.add_argument("--min-wave-sensors", type=int, default=2)
    parser.add_argument("--min-wave-basestations", type=int, default=1)
    parser.add_argument("--point-weight", type=float, default=1.0)
    parser.add_argument("--wave-weight", type=float, default=0.35)
    parser.add_argument("--geometry-prior-weight", type=float, default=0.02)
    parser.add_argument("--max-nfev", type=int, default=250)
    parser.add_argument(
        "--workers",
        type=int,
        default=cpu_count_default(),
        help="Parallel workers for SciPy finite-difference Jacobian when supported. Use 1 to disable. Default: all CPU cores.",
    )
    parser.add_argument("--drone-z", type=float, default=0.0)
    args = parser.parse_args()

    layout = geom_fit.load_layout(args.layout)
    point_obs = geom_fit.load_poses(args.points, 360.0)
    geometry = live.load_geometry(args.input_geometry)
    bs_order = sorted(geometry)
    wave_candidates = load_wave_frames(
        args.wave,
        args.max_wave_frames,
        args.min_wave_observations,
        args.min_wave_sensors,
        args.min_wave_basestations,
    )
    wave_frames, wave_poses = initialize_wave_poses(wave_candidates, layout, geometry, args.max_wave_init_rmse_deg)

    if not wave_frames:
        raise SystemExit("No usable wave frames. Record again or relax --max-wave-init-rmse-deg / min wave settings.")

    initial_bs = bs_params_from_geometry(geometry)
    bs0 = np.concatenate([initial_bs[bs] for bs in bs_order])
    wave0 = np.concatenate(wave_poses)
    x0 = np.concatenate([bs0, wave0])

    lower_bs = []
    upper_bs = []
    for bs in bs_order:
        p = initial_bs[bs]
        lower_bs.extend([-math.pi, -math.pi, -math.pi, p[3] - 2.0, p[4] - 2.0, p[5] - 1.5, -math.pi, -math.pi])
        upper_bs.extend([+math.pi, +math.pi, +math.pi, p[3] + 2.0, p[4] + 2.0, p[5] + 1.5, +math.pi, +math.pi])

    lower_wave = []
    upper_wave = []
    for _ in wave_frames:
        lower_wave.extend([-5.0, -5.0, -0.50, -math.pi, -math.pi, -math.pi])
        upper_wave.extend([+5.0, +5.0, +3.00, +math.pi, +math.pi, +math.pi])

    lower = np.array(lower_bs + lower_wave, dtype=float)
    upper = np.array(upper_bs + upper_wave, dtype=float)
    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    ctx = {
        "bs_order": bs_order,
        "geometry_template": geometry,
        "layout": layout,
        "point_observations": point_obs,
        "wave_frames": wave_frames,
        "default_drone_z": args.drone_z,
        "point_weight": args.point_weight,
        "wave_weight": args.wave_weight,
        "geometry_prior_weight": args.geometry_prior_weight,
        "initial_bs_params": bs0,
    }

    print("=" * 70)
    print("Refine geometry: points + wand wave")
    print(f"Layout:          {args.layout}")
    print(f"Known points:    {args.points} ({len(point_obs)} observations)")
    print(f"Wave:            {args.wave} ({len(wave_frames)}/{len(wave_candidates)} frames used)")
    print(f"Input geometry:  {args.input_geometry}")
    print(f"Output geometry: {args.output}")
    print(f"Workers:         {args.workers}")
    print("=" * 70)

    before = residuals(x0, ctx)
    before_rmse = float(math.degrees(math.sqrt(float(np.mean(before ** 2))))) if len(before) else float("nan")
    print(f"Initial weighted RMSE: {before_rmse:.3f} deg")

    result = least_squares_compat(
        residuals,
        x0,
        bounds=(lower, upper),
        args=(ctx,),
        loss="soft_l1",
        f_scale=math.radians(1.0),
        max_nfev=args.max_nfev,
        verbose=1,
        workers=args.workers,
    )

    after = residuals(result.x, ctx)
    after_rmse = float(math.degrees(math.sqrt(float(np.mean(after ** 2))))) if len(after) else float("nan")
    out_path = Path(args.output)
    serialize_geometry(
        out_path,
        args.input_geometry,
        args.layout,
        args.points,
        args.wave,
        bs_order,
        result.x,
        geometry,
        ctx,
        result,
    )

    print()
    print(f"Final weighted RMSE: {after_rmse:.3f} deg")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
