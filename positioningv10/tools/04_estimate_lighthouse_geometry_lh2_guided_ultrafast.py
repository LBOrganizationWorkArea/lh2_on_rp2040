#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
from statistics import median

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


def load_poses(path, degrees_per_cycle):
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
            if "calibrated_angle_rad" in m:
                raw = float(m["calibrated_angle_rad"])
                angle_is_calibrated = True
            elif "raw_angle_rad" in m:
                raw = float(m["raw_angle_rad"])
            else:
                raw = lfsr_to_raw_rad(float(m["median_lfsr_location"]), sweep, degrees_per_cycle)
            grouped.setdefault((sensor, bs, sweep, angle_is_calibrated), []).append(raw)

        for (sensor, bs, sweep, angle_is_calibrated), values in grouped.items():
            obs.append({
                "pose": pose["name"],
                "pose_data": pose,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "raw_angle": float(median(values)),
                "angle_is_calibrated": bool(angle_is_calibrated),
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


def predict_angle(p_world, params, lighthouse_z, tilt, axis_calibration=None):
    # params = rx, ry, rz, tx, ty, off0, off1
    rx, ry, rz, tx, ty, off0, off1 = params

    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, lighthouse_z], dtype=float)

    p_lh = R @ (p_world - t)
    return lh2_factory_angle(p_lh, tilt, axis_calibration)


def corrected_raw(raw, sweep, params, signs):
    off0 = params[5]
    off1 = params[6]

    if sweep == 0:
        return signs[0] * raw + off0
    return signs[1] * raw + off1


def residuals(params, bs, observations, layout, lighthouse_z, drone_z, factory_entry, tilt_map, signs, axis_map):
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
        )
        meas = corrected_raw(float(o["raw_angle"]), sweep, params, signs)

        out.append(angle_diff(pred, meas))

    return np.array(out, dtype=float)


def fit_bs(bs, observations, layout, lighthouse_z, drone_z, guess_x, guess_y, factory_entry=None, max_nfev=300):
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

    rotation_guesses = [
        [0.0, 0.0, 0.0],
        [0.0, math.pi / 2.0, 0.0],
        [0.0, -math.pi / 2.0, 0.0],
    ]

    position_guesses = [
        [guess_x, guess_y],
        [guess_x + 0.5, guess_y],
        [guess_x - 0.5, guess_y],
        [guess_x, guess_y + 0.5],
        [guess_x, guess_y - 0.5],
    ]

    best = None

    lower = np.array([
        -math.pi, -math.pi, -math.pi,
        guess_x - 3.0, guess_y - 3.0,
        -math.pi, -math.pi,
    ], dtype=float)

    upper = np.array([
        +math.pi, +math.pi, +math.pi,
        guess_x + 3.0, guess_y + 3.0,
        +math.pi, +math.pi,
    ], dtype=float)

    for tilt_map in tilt_maps:
        for signs in sign_maps:
            for axis_map in axis_maps:
                for rot0 in rotation_guesses:
                    for tx0, ty0 in position_guesses:
                        x0 = np.array([
                            rot0[0], rot0[1], rot0[2],
                            tx0, ty0,
                            0.0, 0.0,
                        ], dtype=float)

                        x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

                        result = least_squares(
                            residuals,
                            x0,
                            bounds=(lower, upper),
                            args=(bs, observations, layout, lighthouse_z, drone_z, factory_entry, tilt_map, signs, axis_map),
                            loss="soft_l1",
                            f_scale=math.radians(1.0),
                            max_nfev=max_nfev,
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
                        )

                        if len(err) == 0:
                            continue

                        rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                        rmse_deg = float(math.degrees(rmse_rad))

                        cand = {
                            "params": result.x,
                            "rmse_rad": rmse_rad,
                            "rmse_deg": rmse_deg,
                            "cost": float(result.cost),
                            "success": bool(result.success),
                            "num_residuals": int(len(err)),
                            "tilt_map": dict(tilt_map),
                            "signs": dict(signs),
                            "axis_map": dict(axis_map),
                        }

                        if best is None or cand["rmse_rad"] < best["rmse_rad"]:
                            best = cand

    if best is None:
        raise RuntimeError(f"No solution for BS{bs}")

    rx, ry, rz, tx, ty, off0, off1 = best["params"]
    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    tilt_map = best["tilt_map"]
    signs = best["signs"]
    axis_map = best["axis_map"]

    return {
        "basestation": int(bs),
        "model": "lh2_ultrafast_fixed_hypothesis_factory_corrected_measurements" if has_calibrated_measurements else ("lh2_ultrafast_fixed_hypothesis_with_factory_calibration" if factory_entry else "lh2_ultrafast_fixed_hypothesis"),
        "rmse_deg": float(best["rmse_deg"]),
        "num_residuals": int(best["num_residuals"]),
        "lighthouse_z_m": float(lighthouse_z),
        "drone_z_m": float(drone_z),
        "measurements_factory_corrected": bool(has_calibrated_measurements),
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
            "translation_m": [float(tx), float(ty), float(lighthouse_z)],
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
    parser.add_argument("--max-nfev", type=int, default=300, help="Max optimizer evaluations per hypothesis.")
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
    obs = load_poses(args.poses, args.lfsr_degrees_per_cycle)
    factory_calibs = load_factory_calibration_map(args.factory_calibs)

    bs4_guess = [float(x) for x in args.bs4_guess.split(",")]
    bs10_guess = [float(x) for x in args.bs10_guess.split(",")]

    print("=" * 70)
    print("Ultra-fast guided LH2 geometry calibration")
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print(f"BS4 guess:  {bs4_guess}")
    print(f"BS10 guess: {bs10_guess}")
    print(f"LFSR scale: {args.lfsr_degrees_per_cycle:.1f} deg/cycle")
    if factory_calibs:
        loaded = ", ".join(f"BS{bs}:{entry['path']}" for bs, entry in sorted(factory_calibs.items()))
        print(f"Factory: {loaded}")
    else:
        print("Factory: disabled/not found")
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
