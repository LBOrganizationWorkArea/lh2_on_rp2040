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
STABLE_BASESTATIONS = [4, 10]


def lfsr_to_rad(lfsr_location):
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


def load_sensor_layout(path):
    with open(path, "r") as f:
        data = json.load(f)

    sensors = {}

    for s in data["sensors"]:
        sensors[int(s["sensor"])] = np.array([
            float(s["x_m"]),
            float(s["y_m"]),
            float(s["z_m"]),
        ])

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

    raise ValueError(f"Cannot find angle/lfsr field in measurement: {m}")


def load_init_poses(path):
    with open(path, "r") as f:
        data = json.load(f)

    observations = []

    for p in data["points"]:
        pose_x = float(p["x_m"])
        pose_y = float(p["y_m"])
        pose_name = p["name"]

        grouped = {}

        for m in p["measurements"]:
            sensor = int(m["sensor"])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])

            if bs not in STABLE_BASESTATIONS:
                continue

            key = (pose_name, pose_x, pose_y, sensor, bs, sweep)
            grouped.setdefault(key, []).append(measurement_angle_rad(m))

        for key, values in grouped.items():
            pose_name, pose_x, pose_y, sensor, bs, sweep = key
            observations.append({
                "pose_name": pose_name,
                "pose_x": pose_x,
                "pose_y": pose_y,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "angle_rad": float(median(values)),
                "sample_count": len(values),
            })

    basestations = sorted(set(o["basestation"] for o in observations))
    return observations, basestations


def predict_angles(sensor_world, params):
    """
    params = [rx, ry, rz, tx, ty, tz]
    world -> lighthouse transform.
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


def residuals_for_bs(params, bs, sensors_layout, observations, sweep_map):
    res = []

    for o in observations:
        if o["basestation"] != bs:
            continue

        sensor_id = o["sensor"]
        if sensor_id not in sensors_layout:
            continue

        sensor_local = sensors_layout[sensor_id]

        # 2D drone pose: translation only. Orientation fixed.
        sensor_world = np.array([
            o["pose_x"] + sensor_local[0],
            o["pose_y"] + sensor_local[1],
            sensor_local[2],
        ])

        pred_h, pred_v = predict_angles(sensor_world, params)

        if sweep_map[o["sweep"]] == "h":
            pred = pred_h
        else:
            pred = pred_v

        res.append(pred - o["angle_rad"])

    return np.array(res, dtype=float)


def fit_basestation(bs, sensors_layout, observations):
    candidates = []

    sweep_maps = [
        {0: "h", 1: "v"},
        {0: "v", 1: "h"},
    ]

    lower_bounds = np.array([
        -math.pi, -math.pi, -math.pi,
        -5.0, -5.0, 0.20
    ])

    upper_bounds = np.array([
        math.pi, math.pi, math.pi,
        5.0, 5.0, 3.00
    ])

    initial_positions = [
        np.array([+1.0,  0.0, +0.8]),
        np.array([-1.0,  0.0, +0.8]),
        np.array([ 0.0, +1.0, +0.8]),
        np.array([ 0.0, -1.0, +0.8]),

        np.array([+1.5, +1.0, +1.0]),
        np.array([+1.5, -1.0, +1.0]),
        np.array([-1.5, +1.0, +1.0]),
        np.array([-1.5, -1.0, +1.0]),

        np.array([+2.5,  0.0, +1.2]),
        np.array([-2.5,  0.0, +1.2]),
        np.array([ 0.0, +2.5, +1.2]),
        np.array([ 0.0, -2.5, +1.2]),
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
        for rot0 in initial_rotations:
            for t0 in initial_positions:
                x0 = np.array([
                    rot0[0], rot0[1], rot0[2],
                    t0[0], t0[1], t0[2]
                ])

                x0 = np.clip(x0, lower_bounds + 1e-6, upper_bounds - 1e-6)

                result = least_squares(
                    residuals_for_bs,
                    x0,
                    bounds=(lower_bounds, upper_bounds),
                    args=(bs, sensors_layout, observations, sweep_map),
                    max_nfev=20000,
                    xtol=1e-12,
                    ftol=1e-12,
                    gtol=1e-12,
                )

                err = residuals_for_bs(
                    result.x,
                    bs,
                    sensors_layout,
                    observations,
                    sweep_map
                )

                rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                rmse_deg = float(math.degrees(rmse_rad))

                candidates.append({
                    "params": result.x,
                    "sweep_map": sweep_map,
                    "rmse_rad": rmse_rad,
                    "rmse_deg": rmse_deg,
                    "success": bool(result.success),
                    "cost": float(result.cost),
                })

    candidates.sort(key=lambda c: c["rmse_rad"])
    best = candidates[0]

    params = best["params"]
    rotvec = params[0:3]
    t = params[3:6]
    R = Rotation.from_rotvec(rotvec).as_matrix()

    return {
        "basestation": bs,
        "rmse_deg": best["rmse_deg"],
        "rmse_rad": best["rmse_rad"],
        "cost": best["cost"],
        "success": best["success"],
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
    parser = argparse.ArgumentParser(description="Estimate Lighthouse geometry from 2D initialization poses.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/init_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_2d.json")
    args = parser.parse_args()

    layout_path = Path(args.layout)
    poses_path = Path(args.poses)
    output_path = Path(args.output)

    sensors_layout = load_sensor_layout(layout_path)
    observations, basestations = load_init_poses(poses_path)

    print("Initialize Lighthouse geometry from 2D poses")
    print("=" * 60)
    print(f"Layout: {layout_path}")
    print(f"Poses:  {poses_path}")
    print(f"Observations: {len(observations)}")
    print(f"Basestations: {basestations}")
    print("=" * 60)

    results = []

    for bs in basestations:
        geometry = fit_basestation(bs, sensors_layout, observations)
        results.append(geometry)

        t = geometry["world_to_lighthouse"]["translation_m"]

        print()
        print(f"Basestation {bs}")
        print(f"  RMSE: {geometry['rmse_deg']:.4f} deg")
        print(f"  sweep map: {geometry['sweep_map']}")
        print(f"  translation: x={t[0]:+.3f}, y={t[1]:+.3f}, z={t[2]:+.3f} m")

    output = {
        "description": "Estimated Lighthouse geometry using known 2D drone poses.",
        "input_layout": str(layout_path),
        "input_poses": str(poses_path),
        "basestations": results,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        json.dump(output, f, indent=2)

    print()
    print("=" * 60)
    print(f"Saved: {output_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
