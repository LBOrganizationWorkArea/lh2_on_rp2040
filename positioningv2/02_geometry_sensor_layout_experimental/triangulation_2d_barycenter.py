#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path
from statistics import median

import numpy as np
import serial
from scipy.optimize import least_squares


TICKS_PER_REV = 833333
DEFAULT_BASESTATIONS = [4, 10]


def lfsr_to_rad(lfsr_location):
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


def angle_diff(a, b):
    d = a - b
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def parse_lh2_line(line):
    line = line.strip()
    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")
    if len(parts) != 7:
        return None

    try:
        return {
            "time_us": int(parts[1]),
            "sensor": int(parts[2]),
            "sweep": int(parts[3]),
            "basestation": int(parts[4]),
            "polynomial": int(parts[5]),
            "lfsr_location": int(parts[6]),
        }
    except ValueError:
        return None


def measurement_to_rad(m):
    if "angle_rad" in m:
        return float(m["angle_rad"])
    if "angle_deg" in m:
        return math.radians(float(m["angle_deg"]))
    if "median_lfsr_location" in m:
        return lfsr_to_rad(float(m["median_lfsr_location"]))
    if "lfsr_location" in m:
        return lfsr_to_rad(float(m["lfsr_location"]))
    raise ValueError(f"Cannot convert measurement to angle: {m}")


def load_sensor_layout(path):
    with open(path, "r") as f:
        data = json.load(f)

    sensors = {}
    for s in data["sensors"]:
        sensor_id = int(s["sensor"])
        sensors[sensor_id] = np.array([
            float(s["x_m"]),
            float(s["y_m"]),
            float(s["z_m"]),
        ], dtype=float)

    return sensors


def load_calibration_points(path, basestations):
    with open(path, "r") as f:
        data = json.load(f)

    points = []

    for p in data["points"]:
        point = {
            "name": p["name"],
            "x_m": float(p["x_m"]),
            "y_m": float(p["y_m"]),
            "measurements": {},
        }

        grouped = {}

        for m in p["measurements"]:
            sensor = int(m["sensor"])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])

            if bs not in basestations:
                continue

            key = (sensor, bs, sweep)
            grouped.setdefault(key, []).append(measurement_to_rad(m))

        for key, values in grouped.items():
            point["measurements"][key] = float(median(values))

        points.append(point)

    return points


def feature_from_sweeps(s0, s1, feature_type):
    if feature_type == "sweep0":
        return s0
    if feature_type == "sweep1":
        return s1
    if feature_type == "average":
        return 0.5 * (s0 + s1)
    if feature_type == "diff01":
        return angle_diff(s0, s1)
    if feature_type == "diff10":
        return angle_diff(s1, s0)
    raise ValueError(feature_type)


def build_rows_for_bs(points, sensors_layout, bs, feature_type):
    """
    Each row corresponds to one physical sensor at one known drone pose.

    Known:
      drone pose on ground: point x,y
      sensor local offset: sensors_layout
      measured Lighthouse angles for that sensor

    Target:
      world bearing from Lighthouse to this sensor.
    """
    rows = []

    for p in points:
        pose_x = float(p["x_m"])
        pose_y = float(p["y_m"])

        for sensor_id, local in sensors_layout.items():
            key0 = (sensor_id, bs, 0)
            key1 = (sensor_id, bs, 1)

            if key0 not in p["measurements"]:
                continue
            if key1 not in p["measurements"]:
                continue

            s0 = p["measurements"][key0]
            s1 = p["measurements"][key1]

            u = feature_from_sweeps(s0, s1, feature_type)

            sensor_world_x = pose_x + float(local[0])
            sensor_world_y = pose_y + float(local[1])

            rows.append({
                "pose_name": p["name"],
                "sensor": sensor_id,
                "world_x": sensor_world_x,
                "world_y": sensor_world_y,
                "u": u,
                "sweep0": s0,
                "sweep1": s1,
            })

    return rows


def residuals_bearing_model(params, rows):
    """
    params = [bs_x, bs_y, yaw, scale]

    bearing_pred = yaw + scale * measured_feature

    bearing_geom = atan2(sensor_y - bs_y, sensor_x - bs_x)
    """
    bs_x, bs_y, yaw, scale = params
    res = []

    for r in rows:
        bearing_pred = yaw + scale * r["u"]
        bearing_geom = math.atan2(r["world_y"] - bs_y, r["world_x"] - bs_x)
        res.append(angle_diff(bearing_pred, bearing_geom))

    return np.array(res, dtype=float)


def fit_one_basestation(points, sensors_layout, bs):
    feature_types = ["sweep0", "sweep1", "average", "diff01", "diff10"]

    all_candidates = []

    # Initial guesses around the calibration area.
    initial_positions = [
        (-3.0, -3.0), (-3.0, 0.0), (-3.0, 3.0),
        (0.0, -3.0),              (0.0, 3.0),
        (3.0, -3.0),  (3.0, 0.0), (3.0, 3.0),

        (-2.0, -2.0), (-2.0, 0.0), (-2.0, 2.0),
        (0.0, -2.0),               (0.0, 2.0),
        (2.0, -2.0),  (2.0, 0.0),  (2.0, 2.0),

        (-1.0, -1.0), (-1.0, 0.0), (-1.0, 1.0),
        (0.0, -1.0),               (0.0, 1.0),
        (1.0, -1.0),  (1.0, 0.0),  (1.0, 1.0),
    ]

    initial_yaws = [
        -math.pi,
        -math.pi / 2,
        0.0,
        math.pi / 2,
        math.pi,
    ]

    initial_scales = [-3.0, -2.0, -1.0, -0.5, 0.5, 1.0, 2.0, 3.0]

    lower = np.array([-10.0, -10.0, -math.pi, -20.0], dtype=float)
    upper = np.array([+10.0, +10.0, +math.pi, +20.0], dtype=float)

    for feature_type in feature_types:
        rows = build_rows_for_bs(points, sensors_layout, bs, feature_type)

        if len(rows) < 8:
            continue

        best_for_feature = None

        for x0, y0 in initial_positions:
            for yaw0 in initial_yaws:
                for scale0 in initial_scales:
                    init = np.array([x0, y0, yaw0, scale0], dtype=float)

                    result = least_squares(
                        residuals_bearing_model,
                        init,
                        bounds=(lower, upper),
                        args=(rows,),
                        loss="soft_l1",
                        f_scale=math.radians(2.0),
                        max_nfev=5000,
                        xtol=1e-12,
                        ftol=1e-12,
                        gtol=1e-12,
                    )

                    err = residuals_bearing_model(result.x, rows)
                    rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                    rmse_deg = float(math.degrees(rmse_rad))

                    candidate = {
                        "basestation": bs,
                        "feature_type": feature_type,
                        "params": result.x,
                        "rmse_rad": rmse_rad,
                        "rmse_deg": rmse_deg,
                        "cost": float(result.cost),
                        "success": bool(result.success),
                        "num_rows": len(rows),
                    }

                    if best_for_feature is None or candidate["rmse_rad"] < best_for_feature["rmse_rad"]:
                        best_for_feature = candidate

        if best_for_feature is not None:
            all_candidates.append(best_for_feature)

    if not all_candidates:
        raise RuntimeError(f"Could not fit basestation {bs}")

    all_candidates.sort(key=lambda c: c["rmse_rad"])
    best = all_candidates[0]

    bs_x, bs_y, yaw, scale = best["params"]

    return {
        "basestation": int(bs),
        "model": "2d_bearing_from_lighthouse_feature",
        "feature_type": best["feature_type"],
        "position_m": [float(bs_x), float(bs_y)],
        "yaw_rad": float(wrap_angle(yaw)),
        "scale": float(scale),
        "rmse_deg": float(best["rmse_deg"]),
        "rmse_rad": float(best["rmse_rad"]),
        "num_rows": int(best["num_rows"]),
        "success": bool(best["success"]),
    }


def calibrate(args):
    basestations = [int(x) for x in args.basestations.split(",")]

    sensors_layout = load_sensor_layout(args.layout)
    points = load_calibration_points(args.poses, basestations)

    print("2D Lighthouse bearing calibration")
    print("=" * 70)
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print(f"Basestations: {basestations}")
    print(f"Points: {[p['name'] for p in points]}")
    print(f"Sensors: {sorted(sensors_layout.keys())}")
    print("=" * 70)

    results = []

    for bs in basestations:
        model = fit_one_basestation(points, sensors_layout, bs)
        results.append(model)

        print()
        print(f"Basestation {bs}")
        print(f"  feature: {model['feature_type']}")
        print(f"  position: x={model['position_m'][0]:+.3f} m | y={model['position_m'][1]:+.3f} m")
        print(f"  yaw: {math.degrees(model['yaw_rad']):+.2f} deg")
        print(f"  scale: {model['scale']:+.6f}")
        print(f"  RMSE: {model['rmse_deg']:.4f} deg")
        print(f"  rows used: {model['num_rows']}")

    output = {
        "description": "2D Lighthouse bearing calibration from known drone positions and 4 known sensor offsets.",
        "layout": args.layout,
        "poses": args.poses,
        "basestations": results,
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        json.dump(output, f, indent=2)

    print()
    print("=" * 70)
    print(f"Saved: {output_path}")
    print("=" * 70)


def compute_feature_from_live_pair(pair_sweeps, feature_type):
    if 0 not in pair_sweeps or 1 not in pair_sweeps:
        return None
    return feature_from_sweeps(pair_sweeps[0], pair_sweeps[1], feature_type)


def line_intersection(p1, theta1, p2, theta2):
    d1 = np.array([math.cos(theta1), math.sin(theta1)], dtype=float)
    d2 = np.array([math.cos(theta2), math.sin(theta2)], dtype=float)

    A = np.column_stack([d1, -d2])
    b = np.array(p2, dtype=float) - np.array(p1, dtype=float)

    det = abs(np.linalg.det(A))
    if det < 1e-6:
        return None

    sol = np.linalg.solve(A, b)
    t1 = sol[0]
    t2 = sol[1]

    q1 = np.array(p1, dtype=float) + t1 * d1
    q2 = np.array(p2, dtype=float) + t2 * d2

    point = 0.5 * (q1 + q2)
    intersection_error = float(np.linalg.norm(q1 - q2))

    return point, intersection_error


def load_geometry(path):
    with open(path, "r") as f:
        data = json.load(f)

    models = {}
    for m in data["basestations"]:
        bs = int(m["basestation"])
        models[bs] = m

    return models


def bearing_from_model(model, u):
    yaw = float(model["yaw_rad"])
    scale = float(model["scale"])
    return wrap_angle(yaw + scale * u)


def live(args):
    sensors_layout = load_sensor_layout(args.layout)
    models = load_geometry(args.geometry)

    if len(models) < 2:
        raise RuntimeError("Need at least 2 basestations for triangulation.")

    basestations = sorted(models.keys())

    print("Live 2D triangulation with sensor barycenter")
    print("=" * 70)
    print(f"Geometry: {args.geometry}")
    print(f"Layout:   {args.layout}")
    print(f"Basestations: {basestations}")
    print(f"Port: {args.port}")
    print("=" * 70)

    buffer = {}
    last_print = time.time()

    filtered_x = None
    filtered_y = None

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            data = parse_lh2_line(raw)

            if data is None:
                continue

            bs = data["basestation"]
            if bs not in models:
                continue

            key = (
                int(data["sensor"]),
                int(data["basestation"]),
                int(data["sweep"]),
            )

            buffer.setdefault(key, []).append(lfsr_to_rad(data["lfsr_location"]))

            now = time.time()
            if now - last_print < args.window:
                continue

            # Build median live sweeps:
            live = {}
            for key, values in buffer.items():
                sensor, bs, sweep = key
                live.setdefault((sensor, bs), {})[sweep] = float(median(values))

            sensor_positions = {}
            sensor_errors = {}

            for sensor_id in sorted(sensors_layout.keys()):
                bearings = []

                for bs in basestations:
                    pair_key = (sensor_id, bs)
                    if pair_key not in live:
                        continue

                    model = models[bs]
                    u = compute_feature_from_live_pair(live[pair_key], model["feature_type"])
                    if u is None:
                        continue

                    theta = bearing_from_model(model, u)
                    pos = np.array(model["position_m"], dtype=float)

                    bearings.append((bs, pos, theta))

                if len(bearings) < 2:
                    continue

                # Use first two basestations.
                b1, p1, th1 = bearings[0]
                b2, p2, th2 = bearings[1]

                result = line_intersection(p1, th1, p2, th2)
                if result is None:
                    continue

                sensor_pos, err = result

                if err > args.max_intersection_error:
                    continue

                sensor_positions[sensor_id] = sensor_pos
                sensor_errors[sensor_id] = err

            if not sensor_positions:
                print("Waiting for valid triangulated sensors...")
                buffer.clear()
                last_print = now
                continue

            corrected_centers = []

            for sensor_id, sensor_world_xy in sensor_positions.items():
                local_xy = sensors_layout[sensor_id][0:2]
                center_xy = sensor_world_xy - local_xy
                corrected_centers.append(center_xy)

            center = np.mean(np.vstack(corrected_centers), axis=0)

            raw_x = float(center[0])
            raw_y = float(center[1])

            if filtered_x is None:
                filtered_x = raw_x
                filtered_y = raw_y
            else:
                alpha = args.alpha
                filtered_x = (1.0 - alpha) * filtered_x + alpha * raw_x
                filtered_y = (1.0 - alpha) * filtered_y + alpha * raw_y

            avg_err = sum(sensor_errors.values()) / len(sensor_errors)

            print(
                f"x={filtered_x:+.3f} m | "
                f"y={filtered_y:+.3f} m | "
                f"raw=({raw_x:+.3f},{raw_y:+.3f}) | "
                f"sensors={len(sensor_positions)} | "
                f"ray_err={avg_err:.3f} m"
            )

            buffer.clear()
            last_print = now


def main():
    parser = argparse.ArgumentParser(description="2D Lighthouse calibration and triangulation using 4 sensors.")
    sub = parser.add_subparsers(dest="command", required=True)

    p_cal = sub.add_parser("calibrate", help="Calibrate 2D Lighthouse bearing geometry.")
    p_cal.add_argument("--layout", default="config/sensors_layout.json")
    p_cal.add_argument("--poses", default="config/calibration_relative_2d.json")
    p_cal.add_argument("--output", default="config/lighthouse_bearing_geometry_2d.json")
    p_cal.add_argument("--basestations", default="4,10")
    p_cal.set_defaults(func=calibrate)

    p_live = sub.add_parser("live", help="Live 2D triangulation.")
    p_live.add_argument("--port", required=True)
    p_live.add_argument("--baudrate", type=int, default=115200)
    p_live.add_argument("--layout", default="config/sensors_layout.json")
    p_live.add_argument("--geometry", default="config/lighthouse_bearing_geometry_2d.json")
    p_live.add_argument("--window", type=float, default=0.5)
    p_live.add_argument("--alpha", type=float, default=0.25)
    p_live.add_argument("--max-intersection-error", type=float, default=0.50)
    p_live.set_defaults(func=live)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
