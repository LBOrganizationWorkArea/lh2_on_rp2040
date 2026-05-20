#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
from statistics import median

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


TICKS_PER_REV = 833333


def lfsr_to_rad(lfsr_location):
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


def angle_diff(a, b):
    d = a - b
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def load_sensor_layout(path):
    with open(path, "r") as f:
        data = json.load(f)

    sensors = {}

    for s in data["sensors"]:
        sensors[int(s["sensor"])] = np.array([
            float(s["x_m"]),
            float(s["y_m"]),
            float(s["z_m"]),
        ], dtype=float)

    return sensors


def measurement_angle_rad(m):
    if "angle_rad" in m:
        return float(m["angle_rad"])

    if "angle_deg" in m:
        return math.radians(float(m["angle_deg"]))

    if "median_lfsr_location" in m:
        return lfsr_to_rad(float(m["median_lfsr_location"]))

    if "lfsr_location" in m:
        return lfsr_to_rad(float(m["lfsr_location"]))

    raise ValueError(f"Cannot convert measurement to angle: {m}")


def load_calibration_poses(path):
    with open(path, "r") as f:
        data = json.load(f)

    observations = []
    basestations = set()

    for pose in data["poses"]:
        pose_name = pose["name"]
        pose_x = float(pose["x_m"])
        pose_y = float(pose["y_m"])
        pose_yaw = math.radians(float(pose.get("yaw_deg", 0.0)))

        grouped = {}

        for m in pose["measurements"]:
            sensor = int(m["sensor"])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            angle = measurement_angle_rad(m)

            basestations.add(bs)
            key = (sensor, bs, sweep)
            grouped.setdefault(key, []).append(angle)

        for (sensor, bs, sweep), values in grouped.items():
            observations.append({
                "pose_name": pose_name,
                "pose_x": pose_x,
                "pose_y": pose_y,
                "pose_yaw": pose_yaw,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "angle_rad": float(median(values)),
                "sample_count": len(values),
            })

    return observations, sorted(basestations)


def sensor_world_position(pose_x, pose_y, pose_yaw, sensor_local):
    c = math.cos(pose_yaw)
    s = math.sin(pose_yaw)

    Rz = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)

    t = np.array([pose_x, pose_y, 0.0], dtype=float)

    return t + Rz @ sensor_local


def predict_lh_angles(sensor_world, params):
    """
    params = [rx, ry, rz, tx, ty, tz]

    This models world -> lighthouse:
        p_lh = R @ (p_world - t)

    Then predicted lighthouse angles:
        h = atan2(x_lh, z_lh)
        v = atan2(y_lh, z_lh)
    """
    rotvec = params[0:3]
    t = params[3:6]

    R = Rotation.from_rotvec(rotvec).as_matrix()
    p_lh = R @ (sensor_world - t)

    x, y, z = p_lh

    if z <= 1e-6:
        z = 1e-6

    h = math.atan2(x, z)
    v = math.atan2(y, z)

    return h, v


def residuals_for_bs(params, bs, observations, sensors_layout, sweep_map):
    res = []

    for o in observations:
        if int(o["basestation"]) != int(bs):
            continue

        sensor_id = int(o["sensor"])
        if sensor_id not in sensors_layout:
            continue

        sensor_local = sensors_layout[sensor_id]

        p_world = sensor_world_position(
            o["pose_x"],
            o["pose_y"],
            o["pose_yaw"],
            sensor_local
        )

        pred_h, pred_v = predict_lh_angles(p_world, params)

        if sweep_map[int(o["sweep"])] == "h":
            pred = pred_h
        else:
            pred = pred_v

        res.append(angle_diff(pred, float(o["angle_rad"])))

    return np.array(res, dtype=float)


def fit_basestation(bs, observations, sensors_layout):
    candidates = []

    sweep_maps = [
        {0: "h", 1: "v"},
        {0: "v", 1: "h"},
    ]

    lower = np.array([
        -math.pi, -math.pi, -math.pi,
        -8.0, -8.0, 0.10
    ], dtype=float)

    upper = np.array([
        math.pi, math.pi, math.pi,
        8.0, 8.0, 4.00
    ], dtype=float)

    initial_positions = [
        np.array([+1.0,  0.0, +1.0]),
        np.array([-1.0,  0.0, +1.0]),
        np.array([ 0.0, +1.0, +1.0]),
        np.array([ 0.0, -1.0, +1.0]),

        np.array([+2.0,  0.0, +1.5]),
        np.array([-2.0,  0.0, +1.5]),
        np.array([ 0.0, +2.0, +1.5]),
        np.array([ 0.0, -2.0, +1.5]),

        np.array([+2.0, +2.0, +1.5]),
        np.array([+2.0, -2.0, +1.5]),
        np.array([-2.0, +2.0, +1.5]),
        np.array([-2.0, -2.0, +1.5]),

        np.array([+3.0,  0.0, +2.0]),
        np.array([-3.0,  0.0, +2.0]),
        np.array([ 0.0, +3.0, +2.0]),
        np.array([ 0.0, -3.0, +2.0]),
    ]

    initial_rotations = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, math.pi / 2, 0.0]),
        np.array([0.0, -math.pi / 2, 0.0]),
        np.array([0.0, 0.0, math.pi / 2]),
        np.array([0.0, 0.0, -math.pi / 2]),
        np.array([math.pi / 2, 0.0, 0.0]),
        np.array([-math.pi / 2, 0.0, 0.0]),
    ]

    for sweep_map in sweep_maps:
        for t0 in initial_positions:
            for r0 in initial_rotations:
                x0 = np.array([
                    r0[0], r0[1], r0[2],
                    t0[0], t0[1], t0[2]
                ], dtype=float)

                x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

                result = least_squares(
                    residuals_for_bs,
                    x0,
                    bounds=(lower, upper),
                    args=(bs, observations, sensors_layout, sweep_map),
                    loss="soft_l1",
                    f_scale=math.radians(1.0),
                    max_nfev=5000,
                    xtol=1e-11,
                    ftol=1e-11,
                    gtol=1e-11,
                )

                err = residuals_for_bs(result.x, bs, observations, sensors_layout, sweep_map)

                if len(err) == 0:
                    continue

                rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                rmse_deg = float(math.degrees(rmse_rad))

                candidates.append({
                    "params": result.x,
                    "sweep_map": sweep_map,
                    "rmse_rad": rmse_rad,
                    "rmse_deg": rmse_deg,
                    "cost": float(result.cost),
                    "success": bool(result.success),
                    "num_residuals": int(len(err)),
                })

    if not candidates:
        raise RuntimeError(f"No valid candidate for basestation {bs}")

    candidates.sort(key=lambda c: c["rmse_rad"])
    best = candidates[0]

    params = best["params"]
    rotvec = params[0:3]
    t = params[3:6]
    R = Rotation.from_rotvec(rotvec).as_matrix()

    return {
        "basestation": int(bs),
        "model": "world_to_lighthouse_angles",
        "rmse_deg": best["rmse_deg"],
        "rmse_rad": best["rmse_rad"],
        "cost": best["cost"],
        "success": best["success"],
        "num_residuals": best["num_residuals"],
        "sweep_map": {
            "sweep_0": best["sweep_map"][0],
            "sweep_1": best["sweep_map"][1],
        },
        "world_to_lighthouse": {
            "rotation_vector": rotvec.tolist(),
            "rotation_matrix": R.tolist(),
            "translation_m": t.tolist(),
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Estimate Lighthouse geometry from known 2D drone poses.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry.json")
    parser.add_argument("--basestations", default="4,10")
    args = parser.parse_args()

    layout_path = Path(args.layout)
    poses_path = Path(args.poses)
    output_path = Path(args.output)

    requested_bs = [int(x) for x in args.basestations.split(",")]

    sensors_layout = load_sensor_layout(layout_path)
    observations, detected_bs = load_calibration_poses(poses_path)

    print("=" * 70)
    print("Estimate Lighthouse geometry from 5 known drone poses")
    print(f"Layout: {layout_path}")
    print(f"Poses:  {poses_path}")
    print(f"Output: {output_path}")
    print(f"Detected basestations in file: {detected_bs}")
    print(f"Requested basestations: {requested_bs}")
    print(f"Sensors: {sorted(sensors_layout.keys())}")
    print(f"Observations: {len(observations)}")
    print("=" * 70)

    results = []

    for bs in requested_bs:
        print()
        print(f"Fitting basestation {bs} ...")
        geom = fit_basestation(bs, observations, sensors_layout)
        results.append(geom)

        t = geom["world_to_lighthouse"]["translation_m"]
        rv = geom["world_to_lighthouse"]["rotation_vector"]

        print(f"Basestation {bs}")
        print(f"  RMSE: {geom['rmse_deg']:.4f} deg")
        print(f"  residuals: {geom['num_residuals']}")
        print(f"  sweep map: {geom['sweep_map']}")
        print(f"  translation: x={t[0]:+.3f}, y={t[1]:+.3f}, z={t[2]:+.3f} m")
        print(f"  rotvec: rx={rv[0]:+.3f}, ry={rv[1]:+.3f}, rz={rv[2]:+.3f}")

    output = {
        "description": "Estimated Lighthouse geometry from known 2D drone poses and known sensor layout.",
        "input_layout": str(layout_path),
        "input_poses": str(poses_path),
        "basestations": results,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        json.dump(output, f, indent=2)

    print()
    print("=" * 70)
    print(f"Saved: {output_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()