#!/usr/bin/env python3

import argparse
import itertools
import json
import math
from pathlib import Path
from statistics import median

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


TICKS_PER_REV = 833333
TILT_NEG = -math.pi / 6.0
TILT_POS = +math.pi / 6.0


def lfsr_to_raw_rad(lfsr_location):
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def load_sensor_layout(path):
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


def raw_angle_from_measurement(m):
    if "raw_angle_rad" in m:
        return float(m["raw_angle_rad"])
    if "angle_rad" in m:
        return float(m["angle_rad"])
    if "angle_deg" in m:
        return math.radians(float(m["angle_deg"]))
    if "median_lfsr_location" in m:
        return lfsr_to_raw_rad(float(m["median_lfsr_location"]))
    if "lfsr_location" in m:
        return lfsr_to_raw_rad(float(m["lfsr_location"]))

    raise ValueError(f"Cannot convert measurement to raw angle: {m}")


def load_calibration_poses(path):
    with open(path, "r") as f:
        data = json.load(f)

    observations = []
    detected_bs = set()

    for pose in data["poses"]:
        grouped = {}

        for m in pose["measurements"]:
            sensor = int(m["sensor"])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            raw_angle = raw_angle_from_measurement(m)

            detected_bs.add(bs)
            grouped.setdefault((sensor, bs, sweep), []).append(raw_angle)

        for (sensor, bs, sweep), values in grouped.items():
            observations.append({
                "pose": pose,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "raw_angle_rad": float(median(values)),
                "sample_count": len(values),
            })

    return observations, sorted(detected_bs)


def pose_rotation(roll_rad, pitch_rad, yaw_rad):
    return Rotation.from_euler("xyz", [roll_rad, pitch_rad, yaw_rad]).as_matrix()


def sensor_world_position(pose, default_drone_z, sensor_local):
    roll = math.radians(float(pose.get("roll_deg", 0.0)))
    pitch = math.radians(float(pose.get("pitch_deg", 0.0)))
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))

    drone_pos = np.array([
        float(pose["x_m"]),
        float(pose["y_m"]),
        float(pose.get("z_m", default_drone_z)),
    ], dtype=float)

    return drone_pos + pose_rotation(roll, pitch, yaw) @ sensor_local


def lh2_angle_model(p_lh, tilt):
    x, y, z = p_lh

    r = math.sqrt(x * x + y * y)
    if r < 1e-9:
        r = 1e-9

    value = (z * math.tan(tilt)) / r
    value = max(-0.999999, min(0.999999, value))

    return math.atan2(y, x) + math.asin(value)


def predict_lh2_angle(sensor_world, params, lighthouse_z, tilt):
    rx, ry, rz, tx, ty, offset0, offset1 = params

    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, lighthouse_z], dtype=float)

    p_lh = R @ (sensor_world - t)

    return lh2_angle_model(p_lh, tilt)


def corrected_measured_angle(raw_angle, sweep, params, signs):
    offset0 = params[5]
    offset1 = params[6]

    if sweep == 0:
        return signs[0] * raw_angle + offset0
    else:
        return signs[1] * raw_angle + offset1


def residuals_for_bs(params, bs, observations, sensors_layout, sweep_tilts, signs, lighthouse_z, drone_z):
    res = []

    for o in observations:
        if int(o["basestation"]) != int(bs):
            continue

        sensor_id = int(o["sensor"])
        if sensor_id not in sensors_layout:
            continue

        sensor_local = sensors_layout[sensor_id]

        p_world = sensor_world_position(o["pose"], drone_z, sensor_local)

        sweep = int(o["sweep"])
        tilt = sweep_tilts[sweep]

        predicted = predict_lh2_angle(p_world, params, lighthouse_z, tilt)
        measured = corrected_measured_angle(
            float(o["raw_angle_rad"]),
            sweep,
            params,
            signs
        )

        res.append(angle_diff(predicted, measured))

    return np.array(res, dtype=float)


def fit_basestation(bs, observations, sensors_layout, lighthouse_z, drone_z, xy_bound):
    sweep_tilt_maps = [
        {0: TILT_NEG, 1: TILT_POS},
        {0: TILT_POS, 1: TILT_NEG},
    ]

    sign_maps = [
        {0: +1.0, 1: +1.0},
        {0: +1.0, 1: -1.0},
        {0: -1.0, 1: +1.0},
        {0: -1.0, 1: -1.0},
    ]

    # params = rx, ry, rz, tx, ty, offset0, offset1
    lower = np.array([
        -math.pi, -math.pi, -math.pi,
        -xy_bound, -xy_bound,
        -math.pi, -math.pi
    ], dtype=float)

    upper = np.array([
        +math.pi, +math.pi, +math.pi,
        +xy_bound, +xy_bound,
        +math.pi, +math.pi
    ], dtype=float)

    initial_positions = [
        [+0.5, +0.5],
        [+0.5, -0.5],
        [-0.5, +0.5],
        [-0.5, -0.5],

        [+1.0, +0.0],
        [-1.0, +0.0],
        [+0.0, +1.0],
        [+0.0, -1.0],

        [+1.5, +1.5],
        [+1.5, -1.5],
        [-1.5, +1.5],
        [-1.5, -1.5],

        [+2.0, +0.0],
        [-2.0, +0.0],
        [+0.0, +2.0],
        [+0.0, -2.0],
    ]

    initial_rotations = [
        [0.0, 0.0, 0.0],
        [0.0, math.pi / 2, 0.0],
        [0.0, -math.pi / 2, 0.0],
        [math.pi / 2, 0.0, 0.0],
        [-math.pi / 2, 0.0, 0.0],
        [0.0, 0.0, math.pi / 2],
        [0.0, 0.0, -math.pi / 2],
        [math.pi, 0.0, 0.0],
        [0.0, math.pi, 0.0],
    ]

    initial_offsets = [
        [0.0, 0.0],
        [math.radians(+30), math.radians(-30)],
        [math.radians(-30), math.radians(+30)],
        [math.radians(+60), math.radians(-60)],
        [math.radians(-60), math.radians(+60)],
    ]

    candidates = []
    tested = 0

    for sweep_tilts in sweep_tilt_maps:
        for signs in sign_maps:
            for tx0, ty0 in initial_positions:
                for r0 in initial_rotations:
                    for off0, off1 in initial_offsets:
                        tested += 1

                        x0 = np.array([
                            r0[0], r0[1], r0[2],
                            tx0, ty0,
                            off0, off1,
                        ], dtype=float)

                        x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

                        result = least_squares(
                            residuals_for_bs,
                            x0,
                            bounds=(lower, upper),
                            args=(bs, observations, sensors_layout, sweep_tilts, signs, lighthouse_z, drone_z),
                            loss="soft_l1",
                            f_scale=math.radians(1.0),
                            max_nfev=800,
                            xtol=1e-9,
                            ftol=1e-9,
                            gtol=1e-9,
                        )

                        err = residuals_for_bs(
                            result.x,
                            bs,
                            observations,
                            sensors_layout,
                            sweep_tilts,
                            signs,
                            lighthouse_z,
                            drone_z,
                        )

                        if len(err) == 0:
                            continue

                        rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                        rmse_deg = float(math.degrees(rmse_rad))

                        candidates.append({
                            "params": result.x,
                            "sweep_tilts": sweep_tilts,
                            "signs": signs,
                            "rmse_rad": rmse_rad,
                            "rmse_deg": rmse_deg,
                            "cost": float(result.cost),
                            "success": bool(result.success),
                            "num_residuals": int(len(err)),
                        })

    if not candidates:
        raise RuntimeError(f"No candidate found for basestation {bs}")

    candidates.sort(key=lambda c: c["rmse_rad"])
    best = candidates[0]

    rx, ry, rz, tx, ty, offset0, offset1 = best["params"]
    rotvec = np.array([rx, ry, rz], dtype=float)
    R = Rotation.from_rotvec(rotvec).as_matrix()

    return {
        "basestation": int(bs),
        "model": "lighthouse_v2_fixed_height_with_lfsr_angle_correction",
        "lighthouse_z_m": float(lighthouse_z),
        "drone_z_m": float(drone_z),
        "xy_bound_m": float(xy_bound),
        "rmse_deg": float(best["rmse_deg"]),
        "rmse_rad": float(best["rmse_rad"]),
        "cost": float(best["cost"]),
        "success": bool(best["success"]),
        "num_residuals": int(best["num_residuals"]),
        "sweep_tilts": {
            "sweep_0_rad": float(best["sweep_tilts"][0]),
            "sweep_1_rad": float(best["sweep_tilts"][1]),
            "sweep_0_deg": float(math.degrees(best["sweep_tilts"][0])),
            "sweep_1_deg": float(math.degrees(best["sweep_tilts"][1])),
        },
        "angle_correction": {
            "sign_sweep_0": float(best["signs"][0]),
            "sign_sweep_1": float(best["signs"][1]),
            "offset_sweep_0_rad": float(offset0),
            "offset_sweep_1_rad": float(offset1),
            "offset_sweep_0_deg": float(math.degrees(offset0)),
            "offset_sweep_1_deg": float(math.degrees(offset1)),
        },
        "world_to_lighthouse": {
            "rotation_vector": rotvec.tolist(),
            "rotation_matrix": R.tolist(),
            "translation_m": [float(tx), float(ty), float(lighthouse_z)],
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Estimate LH2 geometry with LFSR angle correction.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_lh2_corrected.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--lighthouse-z", type=float, default=1.20)
    parser.add_argument("--drone-z", type=float, default=0.00)
    parser.add_argument("--xy-bound", type=float, default=3.0)
    args = parser.parse_args()

    basestations = [int(x) for x in args.basestations.split(",")]

    sensors_layout = load_sensor_layout(args.layout)
    observations, detected_bs = load_calibration_poses(args.poses)

    print("=" * 70)
    print("Estimate LH2 geometry with LFSR angle correction")
    print(f"Layout:       {args.layout}")
    print(f"Poses:        {args.poses}")
    print(f"Output:       {args.output}")
    print(f"Detected BS:  {detected_bs}")
    print(f"Using BS:     {basestations}")
    print(f"Lighthouse z: {args.lighthouse_z:.3f} m")
    print(f"Drone z:      {args.drone_z:.3f} m")
    print(f"XY bound:     +/- {args.xy_bound:.2f} m")
    print(f"Observations: {len(observations)}")
    print("=" * 70)

    results = []

    for bs in basestations:
        print()
        print(f"Fitting basestation {bs}...")
        geom = fit_basestation(
            bs,
            observations,
            sensors_layout,
            lighthouse_z=args.lighthouse_z,
            drone_z=args.drone_z,
            xy_bound=args.xy_bound,
        )

        results.append(geom)

        t = geom["world_to_lighthouse"]["translation_m"]
        rv = geom["world_to_lighthouse"]["rotation_vector"]
        tilts = geom["sweep_tilts"]
        corr = geom["angle_correction"]

        print(f"Basestation {bs}")
        print(f"  RMSE: {geom['rmse_deg']:.4f} deg")
        print(f"  residuals: {geom['num_residuals']}")
        print(f"  sweep tilts: sweep0={tilts['sweep_0_deg']:+.1f} deg | sweep1={tilts['sweep_1_deg']:+.1f} deg")
        print(f"  signs: sweep0={corr['sign_sweep_0']:+.0f} | sweep1={corr['sign_sweep_1']:+.0f}")
        print(f"  offsets: sweep0={corr['offset_sweep_0_deg']:+.2f} deg | sweep1={corr['offset_sweep_1_deg']:+.2f} deg")
        print(f"  translation: x={t[0]:+.3f}, y={t[1]:+.3f}, z={t[2]:+.3f} m")
        print(f"  rotvec: rx={rv[0]:+.3f}, ry={rv[1]:+.3f}, rz={rv[2]:+.3f}")

    output = {
        "description": "Estimated LH2 geometry with corrected raw LFSR angles.",
        "input_layout": args.layout,
        "input_poses": args.poses,
        "lighthouse_z_m": args.lighthouse_z,
        "drone_z_m": args.drone_z,
        "basestations": results,
    }

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)

    print()
    print("=" * 70)
    print(f"Saved: {out_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
