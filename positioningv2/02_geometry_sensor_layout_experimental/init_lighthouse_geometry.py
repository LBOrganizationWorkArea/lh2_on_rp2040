#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def load_sensor_layout(path):
    with open(path, "r") as f:
        data = json.load(f)

    points = {}

    for s in data["sensors"]:
        points[int(s["sensor"])] = np.array([
            float(s["x_m"]),
            float(s["y_m"]),
            float(s["z_m"]),
        ])

    return points


def load_t0_angles(path):
    with open(path, "r") as f:
        data = json.load(f)

    angles = {}

    for m in data["measurements"]:
        bs = int(m["basestation"])
        sensor = int(m["sensor"])
        sweep = int(m["sweep"])
        angle_rad = float(m["angle_rad"])

        angles[(bs, sensor, sweep)] = angle_rad

    basestations = sorted(set(k[0] for k in angles.keys()))
    return angles, basestations


def predict_angles(sensor_world, params):
    """
    params = [rx, ry, rz, tx, ty, tz]

    We estimate the transform world -> lighthouse.

    sensor_world is a 3D point in the drone/world frame at t=0.

    In lighthouse frame:
      h = atan2(x, z)
      v = atan2(y, z)
    """
    rotvec = params[0:3]
    t = params[3:6]

    R = Rotation.from_rotvec(rotvec).as_matrix()

    p_lh = R @ (sensor_world - t)

    x, y, z = p_lh

    if z <= 1e-6:
        z = 1e-6

    angle_h = math.atan2(x, z)
    angle_v = math.atan2(y, z)

    return angle_h, angle_v


def residuals_for_bs(params, bs, sensor_points, angles, sweep_map):
    res = []

    for sensor, p_world in sensor_points.items():
        key0 = (bs, sensor, 0)
        key1 = (bs, sensor, 1)

        if key0 not in angles or key1 not in angles:
            continue

        pred_h, pred_v = predict_angles(p_world, params)

        observed = {
            sweep_map[0]: angles[key0],
            sweep_map[1]: angles[key1],
        }

        res.append(pred_h - observed["h"])
        res.append(pred_v - observed["v"])

    return np.array(res, dtype=float)


def fit_basestation(bs, sensor_points, angles):
    candidates = []

    # Two possible sweep mappings.
    sweep_maps = [
        {0: "h", 1: "v"},
        {0: "v", 1: "h"},
    ]

    # Realistic bounds.
    # rotation vector: -pi..pi
    # translation: x/y in -5..5 m, z in 0.20..3.00 m
    lower_bounds = np.array([
        -math.pi, -math.pi, -math.pi,
        -5.0, -5.0, 0.20
    ])

    upper_bounds = np.array([
        math.pi, math.pi, math.pi,
        5.0, 5.0, 3.00
    ])

    # Initial guesses around the drone.
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

        np.array([+3.0, +2.0, +1.5]),
        np.array([+3.0, -2.0, +1.5]),
        np.array([-3.0, +2.0, +1.5]),
        np.array([-3.0, -2.0, +1.5]),
    ]

    # Some rotation initial guesses.
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

                x0 = np.clip(
                    x0,
                    lower_bounds + 1e-6,
                    upper_bounds - 1e-6
                )

                result = least_squares(
                    residuals_for_bs,
                    x0,
                    bounds=(lower_bounds, upper_bounds),
                    args=(bs, sensor_points, angles, sweep_map),
                    max_nfev=10000,
                    xtol=1e-12,
                    ftol=1e-12,
                    gtol=1e-12,
                )

                err = residuals_for_bs(
                    result.x,
                    bs,
                    sensor_points,
                    angles,
                    sweep_map
                )

                rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                rmse_deg = float(rmse_rad * 180.0 / math.pi)

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
    parser = argparse.ArgumentParser(
        description="Initialize Lighthouse geometry from t=0 with realistic bounds."
    )
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--t0", default="config/t0_angles.json")
    parser.add_argument("--output", default="config/lighthouse_geometry.json")
    args = parser.parse_args()

    layout_path = Path(args.layout)
    t0_path = Path(args.t0)
    output_path = Path(args.output)

    sensor_points = load_sensor_layout(layout_path)
    angles, basestations = load_t0_angles(t0_path)

    print("Initialize Lighthouse geometry")
    print("=" * 60)
    print(f"Layout: {layout_path}")
    print(f"t=0:    {t0_path}")
    print(f"Basestations: {basestations}")
    print("Bounds:")
    print("  x/y: -5 m to +5 m")
    print("  z:   +0.20 m to +3.00 m")
    print("=" * 60)

    results = []

    for bs in basestations:
        geometry = fit_basestation(bs, sensor_points, angles)
        results.append(geometry)

        t = geometry["world_to_lighthouse"]["translation_m"]

        print()
        print(f"Basestation {bs}")
        print(f"  RMSE: {geometry['rmse_deg']:.4f} deg")
        print(f"  sweep map: {geometry['sweep_map']}")
        print(
            "  estimated translation: "
            f"x={t[0]:+.3f}, y={t[1]:+.3f}, z={t[2]:+.3f} m"
        )

    output = {
        "description": (
            "Estimated Lighthouse geometry from drone pose at t=0. "
            "World origin is drone center at t=0. "
            "This version uses bounded optimization."
        ),
        "input_layout": str(layout_path),
        "input_t0": str(t0_path),
        "bounds": {
            "x_m": [-5.0, 5.0],
            "y_m": [-5.0, 5.0],
            "z_m": [0.20, 3.00],
            "rotvec_rad": [-math.pi, math.pi]
        },
        "basestations": results
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
