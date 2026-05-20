#!/usr/bin/env python3
"""Fit Lighthouse geometry from known floor anchor points."""

from __future__ import annotations

import argparse
import math

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from dynamic_lh2_common import (
    angle_residual,
    load_json,
    load_sensors_layout,
    pose_look_at,
    predict_angles,
    residual_quality,
    save_json,
    sensor_world_position,
)


TAN_30 = math.tan(math.radians(30.0))


def expected_lh2_sweeps_from_point(point_lighthouse):
    x, y, z = [float(v) for v in point_lighthouse]
    if z <= 1e-6:
        return None

    theta = math.atan2(x, z)
    image_v = y / z
    phi = math.atan(image_v * math.cos(theta))
    value = math.tan(phi) * TAN_30 * math.cos(theta)
    value = max(-1.0, min(1.0, value))
    half_delta = (math.pi / 3.0) + math.asin(value)
    return np.array([theta - half_delta, theta + half_delta], dtype=float)


def pose_look_at_lh2(origin, target=(0.0, 0.0, 0.0)):
    """Return BS-from-world rotation with Lighthouse local +Z looking at target."""
    origin = np.asarray(origin, dtype=float)
    target = np.asarray(target, dtype=float)
    forward = target - origin
    norm = np.linalg.norm(forward)
    if norm < 1e-9:
        return np.zeros(3)
    forward /= norm

    up = np.array([0.0, 0.0, 1.0], dtype=float)
    right = np.cross(up, forward)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0], dtype=float)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    down /= max(np.linalg.norm(down), 1e-12)

    rotation_bs_from_world = np.vstack([right, down, forward])
    return Rotation.from_matrix(rotation_bs_from_world).as_rotvec()


def load_anchored_sweeps(path, sweep_source):
    df = pd.read_csv(path)
    sweep_columns = ("sweep0_deg", "sweep1_deg")
    if sweep_source == "model":
        sweep_columns = ("model_sweep0_deg", "model_sweep1_deg")
    required = [
        "point_id",
        "point_x",
        "point_y",
        "point_z",
        "point_yaw_deg",
        "sensor_id",
        "lighthouse_id",
        sweep_columns[0],
        sweep_columns[1],
    ]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"Missing anchored CSV columns: {missing}")
    df = df.dropna(subset=list(sweep_columns)).copy()
    df = df[df[sweep_columns[0]].astype(str) != ""]
    df = df[df[sweep_columns[1]].astype(str) != ""]
    for col in ("point_x", "point_y", "point_z", "point_yaw_deg", sweep_columns[0], sweep_columns[1]):
        df[col] = df[col].astype(float)
    df["sensor_id"] = df["sensor_id"].astype(int)
    df["lighthouse_id"] = df["lighthouse_id"].astype(int)
    df["measure_sweep0_deg"] = df[sweep_columns[0]]
    df["measure_sweep1_deg"] = df[sweep_columns[1]]
    return df.reset_index(drop=True), sweep_columns


def predict_sweeps(
    sensor_position_world,
    lighthouse_translation,
    lighthouse_rotvec,
    model,
    lh2_sweep_order,
    lh2_elevation_sign,
):
    if model == "angular":
        azimuth, elevation = predict_angles(sensor_position_world, lighthouse_translation, lighthouse_rotvec)
        return np.array([
            azimuth + TAN_30 * elevation,
            azimuth - TAN_30 * elevation,
        ], dtype=float)

    rotation = Rotation.from_rotvec(lighthouse_rotvec).as_matrix()
    point_lighthouse = rotation @ (np.asarray(sensor_position_world) - np.asarray(lighthouse_translation))
    predicted = expected_lh2_sweeps_from_point(point_lighthouse)
    if predicted is not None and lh2_sweep_order == "reversed":
        predicted = predicted[[1, 0]]
    if predicted is not None and lh2_elevation_sign < 0.0:
        predicted = predicted[[1, 0]]
    return predicted


def parse_ids(text):
    if not text:
        return None
    return {int(part.strip()) for part in text.split(",") if part.strip()}


def parse_initial_guesses(text):
    guesses = {}
    if not text:
        return guesses
    for chunk in text.split(";"):
        if not chunk.strip():
            continue
        lh_text, xyz_text = chunk.split("=", 1)
        xyz = [float(part.strip()) for part in xyz_text.split(",")]
        if len(xyz) != 3:
            raise ValueError(f"Bad initial guess '{chunk}', expected id=x,y,z")
        guesses[int(lh_text.strip())] = np.array(xyz, dtype=float)
    return guesses


def pack_initial(lighthouse_ids, settings, args):
    z_guess = float(settings.get("lighthouse_z_guess", 1.5))
    default_guesses = [
        np.array([1.2, 1.5, z_guess], dtype=float),
        np.array([-1.2, 1.5, z_guess], dtype=float),
    ]
    guesses = {lh_id: default_guesses[index % len(default_guesses)] for index, lh_id in enumerate(lighthouse_ids)}
    guesses.update(parse_initial_guesses(args.initial))
    values = []
    lower = []
    upper = []
    xy_bound = float(args.xy_bound)
    z_min = float(args.z_min)
    z_max = float(args.z_max)
    for lh_id in lighthouse_ids:
        t = guesses.get(lh_id, np.array([0.0, 1.5, z_guess], dtype=float))
        target = (0.0, 0.0, float(settings.get("drone_z", 0.0)))
        if args.geometry_model == "lh2-plane":
            r = pose_look_at_lh2(t, target=target)
        else:
            r = pose_look_at(t, target=target)
        values.extend(t.tolist())
        values.extend(r.tolist())
        lower.extend([-xy_bound, -xy_bound, z_min, -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi])
        upper.extend([+xy_bound, +xy_bound, z_max, +2.0 * math.pi, +2.0 * math.pi, +2.0 * math.pi])
        if args.fit_sweep_phase:
            values.extend([0.0, 0.0])
            phase_bound = math.radians(float(args.phase_bound_deg))
            lower.extend([-phase_bound, -phase_bound])
            upper.extend([+phase_bound, +phase_bound])
    return np.asarray(values), (np.asarray(lower), np.asarray(upper))


def unpack(params, lighthouse_ids, fit_sweep_phase=False):
    offset = 0
    lighthouses = {}
    for lh_id in lighthouse_ids:
        lighthouses[lh_id] = {
            "translation": params[offset:offset + 3],
            "rotation_vector": params[offset + 3:offset + 6],
            "sweep_phase": np.zeros(2, dtype=float),
        }
        offset += 6
        if fit_sweep_phase:
            lighthouses[lh_id]["sweep_phase"] = params[offset:offset + 2]
            offset += 2
    return lighthouses


def residuals(params, lighthouse_ids, df, sensors, model, lh2_sweep_order, lh2_elevation_sign, fit_sweep_phase):
    lighthouses = unpack(params, lighthouse_ids, fit_sweep_phase)
    out = []
    for row in df.itertuples(index=False):
        sensor_body = sensors.get(int(row.sensor_id))
        lighthouse = lighthouses.get(int(row.lighthouse_id))
        if sensor_body is None or lighthouse is None:
            continue
        pose = np.array([float(row.point_x), float(row.point_y), math.radians(float(row.point_yaw_deg))])
        p_world = sensor_world_position(pose, sensor_body, float(row.point_z))
        predicted = predict_sweeps(
            p_world,
            lighthouse["translation"],
            lighthouse["rotation_vector"],
            model,
            lh2_sweep_order,
            lh2_elevation_sign,
        )
        if predicted is None:
            out.extend([math.radians(120.0), math.radians(120.0)])
            continue
        predicted = predicted + lighthouse["sweep_phase"]
        measured = np.radians([float(row.measure_sweep0_deg), float(row.measure_sweep1_deg)])
        out.extend(angle_residual(measured, predicted).tolist())
    return np.asarray(out, dtype=float)


def point_errors(params, lighthouse_ids, df, sensors, model, lh2_sweep_order, lh2_elevation_sign, fit_sweep_phase):
    lighthouses = unpack(params, lighthouse_ids, fit_sweep_phase)
    errors = []
    for row in df.itertuples(index=False):
        sensor_body = sensors.get(int(row.sensor_id))
        lighthouse = lighthouses.get(int(row.lighthouse_id))
        if sensor_body is None or lighthouse is None:
            continue
        pose = np.array([float(row.point_x), float(row.point_y), math.radians(float(row.point_yaw_deg))])
        p_world = sensor_world_position(pose, sensor_body, float(row.point_z))
        predicted = predict_sweeps(
            p_world,
            lighthouse["translation"],
            lighthouse["rotation_vector"],
            model,
            lh2_sweep_order,
            lh2_elevation_sign,
        )
        if predicted is None:
            errors.append((row.point_id, int(row.sensor_id), int(row.lighthouse_id), 120.0))
            continue
        predicted = predicted + lighthouse["sweep_phase"]
        measured = np.radians([float(row.measure_sweep0_deg), float(row.measure_sweep1_deg)])
        err = np.degrees(np.max(np.abs(angle_residual(measured, predicted))))
        errors.append((row.point_id, int(row.sensor_id), int(row.lighthouse_id), float(err)))
    return errors


def print_worst_errors(params, lighthouse_ids, df, sensors, model, lh2_sweep_order, lh2_elevation_sign, fit_sweep_phase, limit):
    if not limit:
        return
    errors = sorted(
        point_errors(params, lighthouse_ids, df, sensors, model, lh2_sweep_order, lh2_elevation_sign, fit_sweep_phase),
        key=lambda item: item[3],
        reverse=True,
    )
    if not errors:
        return
    print(f"Worst anchored observations before rejection/top {min(limit, len(errors))}:")
    for point_id, sensor_id, lighthouse_id, err in errors[:limit]:
        print(f"  {point_id} S{sensor_id} BS{lighthouse_id}: {err:.3f} deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--settings", default="config/calibration_settings.json")
    parser.add_argument("--input", default="data/captures/anchored_floor9_sweeps.csv")
    parser.add_argument("--output", default="config/lighthouse_geometry_anchored_floor.json")
    parser.add_argument("--sweep-source", choices=["ordered", "model"], default="model")
    parser.add_argument("--geometry-model", choices=["lh2-plane", "angular"], default="lh2-plane")
    parser.add_argument("--lh2-sweep-order", choices=["normal", "reversed"], default="normal")
    parser.add_argument("--lh2-elevation-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--fit-sweep-phase", action="store_true")
    parser.add_argument("--phase-bound-deg", type=float, default=80.0)
    parser.add_argument("--lighthouses", default=None, help="Comma-separated lighthouse ids to fit, e.g. 4 or 4,10.")
    parser.add_argument("--sensors", default=None, help="Comma-separated sensor ids to keep.")
    parser.add_argument("--exclude-sensors", default=None, help="Comma-separated sensor ids to drop.")
    parser.add_argument("--points", default=None, help="Comma-separated point ids to keep.")
    parser.add_argument("--exclude-points", default=None, help="Comma-separated point ids to drop.")
    parser.add_argument("--mode", choices=["all", "normal", "swapped"], default="all")
    parser.add_argument("--initial", default=None, help="Initial lighthouse translations, e.g. '4=0,2,1.8;10=-2,1,1.8'.")
    parser.add_argument("--xy-bound", type=float, default=8.0)
    parser.add_argument("--z-min", type=float, default=0.2)
    parser.add_argument("--z-max", type=float, default=4.0)
    parser.add_argument("--print-errors", type=int, default=0)
    parser.add_argument("--max-nfev", type=int, default=2000)
    parser.add_argument("--reject-outliers-deg", type=float, default=None)
    parser.add_argument("--reject-rounds", type=int, default=1)
    args = parser.parse_args()

    _, sensors = load_sensors_layout(args.layout)
    settings = load_json(args.settings)
    df, sweep_columns = load_anchored_sweeps(args.input, args.sweep_source)
    if args.lighthouses:
        expected = parse_ids(args.lighthouses)
    else:
        expected = {int(x) for x in settings.get("expected_lighthouses", [4, 10])}
    df = df[df["lighthouse_id"].isin(expected)].copy()
    sensors_keep = parse_ids(args.sensors)
    sensors_drop = parse_ids(args.exclude_sensors)
    if sensors_keep is not None:
        df = df[df["sensor_id"].isin(sensors_keep)].copy()
    if sensors_drop is not None:
        df = df[~df["sensor_id"].isin(sensors_drop)].copy()
    if args.points:
        points_keep = {part.strip() for part in args.points.split(",") if part.strip()}
        df = df[df["point_id"].isin(points_keep)].copy()
    if args.exclude_points:
        points_drop = {part.strip() for part in args.exclude_points.split(",") if part.strip()}
        df = df[~df["point_id"].isin(points_drop)].copy()
    if args.mode != "all":
        if "mode" not in df.columns:
            raise ValueError("--mode requires a CSV with a mode column.")
        df = df[df["mode"] == args.mode].copy()
    lighthouse_ids = sorted(int(x) for x in df["lighthouse_id"].unique())
    if not lighthouse_ids:
        raise ValueError("Need observations from at least one lighthouse.")

    x0, bounds = pack_initial(lighthouse_ids, settings, args)
    f_scale = math.radians(float(settings.get("max_angle_error_deg", 5.0)))

    result = least_squares(
        residuals,
        x0,
        bounds=bounds,
        args=(
            lighthouse_ids,
            df,
            sensors,
            args.geometry_model,
            args.lh2_sweep_order,
            args.lh2_elevation_sign,
            args.fit_sweep_phase,
        ),
        loss=settings.get("robust_loss", "soft_l1"),
        f_scale=f_scale,
        max_nfev=args.max_nfev,
    )
    print_worst_errors(
        result.x,
        lighthouse_ids,
        df,
        sensors,
        args.geometry_model,
        args.lh2_sweep_order,
        args.lh2_elevation_sign,
        args.fit_sweep_phase,
        args.print_errors,
    )

    removed_total = 0
    if args.reject_outliers_deg is not None:
        rounds = max(1, int(args.reject_rounds))
        for round_index in range(rounds):
            errs = point_errors(
                result.x,
                lighthouse_ids,
                df,
                sensors,
                args.geometry_model,
                args.lh2_sweep_order,
                args.lh2_elevation_sign,
                args.fit_sweep_phase,
            )
            bad = {(p, s, b) for p, s, b, e in errs if e > args.reject_outliers_deg}
            if not bad:
                break
            before = len(df)
            df = df[
                ~df.apply(lambda row: (row["point_id"], int(row["sensor_id"]), int(row["lighthouse_id"])) in bad, axis=1)
            ].copy()
            removed = before - len(df)
            removed_total += removed
            print(f"Removed outlier anchored observations round {round_index + 1}: {removed}")
            if len(df) < 4:
                raise ValueError("Too few observations left after outlier rejection.")
            result = least_squares(
                residuals,
                result.x,
                bounds=bounds,
                args=(
                    lighthouse_ids,
                    df,
                    sensors,
                    args.geometry_model,
                    args.lh2_sweep_order,
                    args.lh2_elevation_sign,
                    args.fit_sweep_phase,
                ),
                loss=settings.get("robust_loss", "soft_l1"),
                f_scale=f_scale,
                max_nfev=args.max_nfev,
            )

    res = residuals(
        result.x,
        lighthouse_ids,
        df,
        sensors,
        args.geometry_model,
        args.lh2_sweep_order,
        args.lh2_elevation_sign,
        args.fit_sweep_phase,
    )
    lighthouses = unpack(result.x, lighthouse_ids, args.fit_sweep_phase)
    quality = residual_quality(res)
    quality.update({
        "num_observations": int(len(df)),
        "num_points": int(df["point_id"].nunique()),
        "sweep_source": args.sweep_source,
        "sweep_columns": list(sweep_columns),
        "geometry_model": args.geometry_model,
        "lh2_sweep_order": args.lh2_sweep_order,
        "lh2_elevation_sign": float(args.lh2_elevation_sign),
        "fit_sweep_phase": bool(args.fit_sweep_phase),
        "outliers_removed": int(removed_total),
    })

    output = {
        "version": 1,
        "model": f"anchored_floor_{args.geometry_model}",
        "units": "meters",
        "angle_units": "radians",
        "drone_z_assumed": float(settings.get("drone_z", 0.0)),
        "anchors": {
            "yaw_fixed_deg": 0.0,
            "note": "Drone anchor point x/y/yaw are fixed by floor marks.",
        },
        "lighthouses": [
            {
                "id": int(lh_id),
                "translation": lighthouses[lh_id]["translation"].tolist(),
                "rotation_vector": lighthouses[lh_id]["rotation_vector"].tolist(),
                "sweep_phase": lighthouses[lh_id]["sweep_phase"].tolist(),
                "sweep_phase_deg": np.degrees(lighthouses[lh_id]["sweep_phase"]).tolist(),
            }
            for lh_id in sorted(lighthouses)
        ],
        "calibration_quality": quality,
    }
    save_json(args.output, output)

    print("=" * 70)
    print("Fit anchored floor Lighthouse geometry")
    print(f"Input: {args.input}")
    print(f"Sweep source: {args.sweep_source} columns={sweep_columns[0]},{sweep_columns[1]}")
    print(f"Geometry model: {args.geometry_model}")
    if args.geometry_model == "lh2-plane":
        print(f"LH2 sweep order: {args.lh2_sweep_order}")
        print(f"LH2 elevation sign: {args.lh2_elevation_sign:+.0f}")
        print(f"Fit sweep phase: {args.fit_sweep_phase}")
    print(f"points={quality['num_points']} observations={quality['num_observations']}")
    print(f"Success: {result.success} | cost={result.cost:.6g} | nfev={result.nfev}")
    for item in output["lighthouses"]:
        t = item["translation"]
        r = item["rotation_vector"]
        print(
            f"BS{item['id']}: "
            f"translation=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m "
            f"rotvec=({r[0]:+.3f},{r[1]:+.3f},{r[2]:+.3f}) "
            f"phase_deg=({item['sweep_phase_deg'][0]:+.3f},{item['sweep_phase_deg'][1]:+.3f})"
        )
    print(
        f"RMSE={quality['rmse_deg']:.3f} deg | "
        f"median={quality['median_error_deg']:.3f} deg | "
        f"max={quality['max_error_deg']:.3f} deg"
    )
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
