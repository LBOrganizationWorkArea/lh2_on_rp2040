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
    detected_bs = set()

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

            detected_bs.add(bs)
            grouped.setdefault((sensor, bs, sweep), []).append(angle)

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

    return observations, sorted(detected_bs)


def sensor_world_position(pose_x, pose_y, pose_yaw, drone_z, sensor_local):
    c = math.cos(pose_yaw)
    s = math.sin(pose_yaw)

    Rz = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)

    drone_pos = np.array([pose_x, pose_y, drone_z], dtype=float)
    return drone_pos + Rz @ sensor_local


def predict_angles(sensor_world, params, lighthouse_z):
    """
    params = [rx, ry, rz, tx, ty]
    lighthouse translation is [tx, ty, lighthouse_z]
    world -> lighthouse frame:
        p_lh = R @ (p_world - t)
    simple angular model:
        h = atan2(x_lh, z_lh)
        v = atan2(y_lh, z_lh)
    """
    rx, ry, rz, tx, ty = params

    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, lighthouse_z], dtype=float)

    p_lh = R @ (sensor_world - t)
    x, y, z = p_lh

    if z <= 1e-6:
        z = 1e-6

    h = math.atan2(x, z)
    v = math.atan2(y, z)

    return h, v


def residuals_for_bs(params, bs, observations, sensors_layout, sweep_map, lighthouse_z, drone_z):
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
            drone_z,
            sensor_local
        )

        pred_h, pred_v = predict_angles(p_world, params, lighthouse_z)

        if sweep_map[int(o["sweep"])] == "h":
            pred = pred_h
        else:
            pred = pred_v

        measured = float(o["angle_rad"])
        res.append(angle_diff(pred, measured))

    return np.array(res, dtype=float)


def fit_basestation(bs, observations, sensors_layout, lighthouse_z, drone_z):
    sweep_maps = [
        {0: "h", 1: "v"},
        {0: "v", 1: "h"},
    ]

    # params = rx, ry, rz, tx, ty
    lower = np.array([-math.pi, -math.pi, -math.pi, -5.0, -5.0], dtype=float)
    upper = np.array([+math.pi, +math.pi, +math.pi, +5.0, +5.0], dtype=float)

    initial_positions = [
        [+0.8, +0.0],
        [-0.8, +0.0],
        [+0.0, +0.8],
        [+0.0, -0.8],
        [+1.5, +0.0],
        [-1.5, +0.0],
        [+0.0, +1.5],
        [+0.0, -1.5],
        [+1.5, +1.5],
        [+1.5, -1.5],
        [-1.5, +1.5],
        [-1.5, -1.5],
        [+2.5, +0.0],
        [-2.5, +0.0],
        [+0.0, +2.5],
        [+0.0, -2.5],
    ]

    initial_rotations = [
        [0.0, 0.0, 0.0],
        [0.0, math.pi / 2, 0.0],
        [0.0, -math.pi / 2, 0.0],
        [0.0, 0.0, math.pi / 2],
        [0.0, 0.0, -math.pi / 2],
        [math.pi / 2, 0.0, 0.0],
        [-math.pi / 2, 0.0, 0.0],
    ]

    candidates = []

    for sweep_map in sweep_maps:
        for tx0, ty0 in initial_positions:
            for r0 in initial_rotations:
                x0 = np.array([
                    r0[0], r0[1], r0[2],
                    tx0, ty0,
                ], dtype=float)

                x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

                result = least_squares(
                    residuals_for_bs,
                    x0,
                    bounds=(lower, upper),
                    args=(bs, observations, sensors_layout, sweep_map, lighthouse_z, drone_z),
                    loss="soft_l1",
                    f_scale=math.radians(1.0),
                    max_nfev=1500,
                    xtol=1e-10,
                    ftol=1e-10,
                    gtol=1e-10,
                )

                err = residuals_for_bs(
                    result.x,
                    bs,
                    observations,
                    sensors_layout,
                    sweep_map,
                    lighthouse_z,
                    drone_z,
                )

                if len(err) == 0:
                    continue

                rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                rmse_deg = float(math.degrees(rmse_rad))

                candidates.append({
                    "params": result.x,
                    "sweep_map": sweep_map,
                    "rmse_rad": rmse_rad,
                    "rmse_deg": rmse_deg,
                    "success": bool(result.success),
                    "cost": float(result.cost),
                    "num_residuals": int(len(err)),
                })

    if not candidates:
        raise RuntimeError(f"No candidate found for basestation {bs}")

    candidates.sort(key=lambda c: c["rmse_rad"])
    best = candidates[0]

    rx, ry, rz, tx, ty = best["params"]
    rotvec = np.array([rx, ry, rz], dtype=float)
    R = Rotation.from_rotvec(rotvec).as_matrix()

    return {
        "basestation": int(bs),
        "model": "constrained_height_world_to_lighthouse",
        "lighthouse_z_m": float(lighthouse_z),
        "drone_z_m": float(drone_z),
        "rmse_deg": float(best["rmse_deg"]),
        "rmse_rad": float(best["rmse_rad"]),
        "cost": float(best["cost"]),
        "success": bool(best["success"]),
        "num_residuals": int(best["num_residuals"]),
        "sweep_map": {
            "sweep_0": best["sweep_map"][0],
            "sweep_1": best["sweep_map"][1],
        },
        "world_to_lighthouse": {
            "rotation_vector": rotvec.tolist(),
            "rotation_matrix": R.tolist(),
            "translation_m": [float(tx), float(ty), float(lighthouse_z)],
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Constrained Lighthouse geometry estimation.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_constrained.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--lighthouse-z", type=float, default=1.20)
    parser.add_argument("--drone-z", type=float, default=0.05)
    args = parser.parse_args()

    basestations = [int(x) for x in args.basestations.split(",")]

    sensors_layout = load_sensor_layout(args.layout)
    observations, detected_bs = load_calibration_poses(args.poses)

    print("=" * 70)
    print("Constrained Lighthouse geometry estimation")
    print(f"Layout:       {args.layout}")
    print(f"Poses:        {args.poses}")
    print(f"Output:       {args.output}")
    print(f"Detected BS:  {detected_bs}")
    print(f"Using BS:     {basestations}")
    print(f"Lighthouse z: {args.lighthouse_z:.3f} m")
    print(f"Drone z:      {args.drone_z:.3f} m")
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
        )

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
        "description": "Constrained Lighthouse geometry with fixed lighthouse height.",
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