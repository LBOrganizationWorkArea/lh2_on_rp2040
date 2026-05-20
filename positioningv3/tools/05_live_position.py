#!/usr/bin/env python3

import argparse
import json
import math
import time
from statistics import median

import numpy as np

from lh2_factory_model import lh2_factory_angle

try:
    import serial
except ImportError:
    serial = None

try:
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
except ImportError:
    least_squares = None
    Rotation = None


TICKS_PER_REV = 120000


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def lfsr_to_raw_rad(lfsr_location, sweep, degrees_per_cycle=360.0):
    half_span = degrees_per_cycle / 2.0
    angle_deg = (((float(lfsr_location) % TICKS_PER_REV) / TICKS_PER_REV) * degrees_per_cycle) - half_span
    angle_rad = math.radians(angle_deg)

    if int(sweep) == 0:
        return angle_rad + math.pi / 3.0

    return angle_rad - math.pi / 3.0


def parse_lh2_line(line):
    line = line.strip()

    if line.startswith("LH2A,"):
        parts = line.split(",")
        try:
            if len(parts) == 7:
                return {
                    "time_us": None,
                    "sensor": int(parts[1]),
                    "sweep": int(parts[2]),
                    "basestation": int(parts[3]),
                    "polynomial": int(parts[4]),
                    "lfsr_location": int(parts[5]),
                    "raw_angle_rad": int(parts[6]) / 1000000.0,
                }
            if len(parts) == 8:
                return {
                    "time_us": int(parts[1]),
                    "sensor": int(parts[2]),
                    "sweep": int(parts[3]),
                    "basestation": int(parts[4]),
                    "polynomial": int(parts[5]),
                    "lfsr_location": int(parts[6]),
                    "raw_angle_rad": int(parts[7]) / 1000000.0,
                }
        except ValueError:
            return None

        return None

    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    try:
        if len(parts) == 6:
            return {
                "time_us": None,
                "sensor": int(parts[1]),
                "sweep": int(parts[2]),
                "basestation": int(parts[3]),
                "polynomial": int(parts[4]),
                "lfsr_location": int(parts[5]),
            }
        if len(parts) == 7:
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

    return None


def load_layout(path):
    with open(path, "r") as f:
        data = json.load(f)

    return {
        int(s["sensor"]): np.array([
            float(s["x_m"]),
            float(s["y_m"]),
            float(s.get("z_m", 0.0)),
        ], dtype=float)
        for s in data["sensors"]
    }


def load_geometry(path):
    with open(path, "r") as f:
        data = json.load(f)

    degrees_per_cycle = float(data.get("lfsr_degrees_per_cycle", 360.0))
    basestations = {}
    for item in data["basestations"]:
        bs = int(item["basestation"])
        corr = item.get("angle_correction", {})
        tilts = item.get("sweep_tilts", {})
        world_to_lh = item["world_to_lighthouse"]

        basestations[bs] = {
            "rotation": np.array(world_to_lh["rotation_matrix"], dtype=float),
            "translation": np.array(world_to_lh["translation_m"], dtype=float),
            "tilts": {
                0: float(tilts.get("sweep_0_rad", math.radians(tilts.get("sweep_0_deg", 30.0)))),
                1: float(tilts.get("sweep_1_rad", math.radians(tilts.get("sweep_1_deg", -30.0)))),
            },
            "signs": {
                0: float(corr.get("sign_sweep_0", 1.0)),
                1: float(corr.get("sign_sweep_1", 1.0)),
            },
            "offsets": {
                0: float(corr.get("offset_sweep_0_rad", 0.0)),
                1: float(corr.get("offset_sweep_1_rad", 0.0)),
            },
            "factory_axes": None,
            "lfsr_degrees_per_cycle": degrees_per_cycle,
        }

        factory = item.get("factory_calibration")
        if isinstance(factory, dict):
            axis_map = item.get("factory_axis_map", {})
            sweep0_axis = int(axis_map.get("sweep_0_axis", 0))
            sweep1_axis = int(axis_map.get("sweep_1_axis", 1))
            axes = {
                0: factory.get("axis0"),
                1: factory.get("axis1"),
            }
            basestations[bs]["factory_axes"] = {
                0: axes.get(sweep0_axis),
                1: axes.get(sweep1_axis),
            }

    return basestations


def sensor_world_from_pose(params, sensor_local, solve_attitude):
    if solve_attitude:
        x, y, z, rx, ry, rz = params
        body_to_world = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    else:
        x, y, z = params
        body_to_world = np.eye(3)

    return np.array([x, y, z], dtype=float) + body_to_world @ sensor_local


def sensor_world_from_planar_pose(params, sensor_local, fixed_z, solve_yaw):
    if solve_yaw:
        x, y, yaw = params
        c = math.cos(yaw)
        s = math.sin(yaw)
        body_to_world = np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
    else:
        x, y = params
        body_to_world = np.eye(3)

    return np.array([x, y, fixed_z], dtype=float) + body_to_world @ sensor_local


def predict_angle(p_world, bs_geom, sweep):
    sweep = int(sweep)
    p_lh = bs_geom["rotation"] @ (p_world - bs_geom["translation"])
    factory_axes = bs_geom.get("factory_axes")
    axis_calibration = factory_axes.get(sweep) if factory_axes else None
    return lh2_factory_angle(p_lh, bs_geom["tilts"][sweep], axis_calibration)


def measured_angle(raw_lfsr, bs_geom, sweep):
    sweep = int(sweep)
    raw = lfsr_to_raw_rad(raw_lfsr, sweep, bs_geom.get("lfsr_degrees_per_cycle", 360.0))
    return bs_geom["signs"][sweep] * raw + bs_geom["offsets"][sweep]


def measured_observation_angle(obs, bs_geom, sweep):
    if "raw_angle_rad" in obs:
        raw = float(obs["raw_angle_rad"])
        sweep = int(sweep)
        return bs_geom["signs"][sweep] * raw + bs_geom["offsets"][sweep]

    return measured_angle(float(obs["lfsr_location"]), bs_geom, sweep)


def residuals(params, observations, layout, geometry, solve_attitude):
    out = []

    for obs in observations:
        sensor = int(obs["sensor"])
        bs = int(obs["basestation"])
        sweep = int(obs["sweep"])

        if sensor not in layout or bs not in geometry:
            continue

        p_world = sensor_world_from_pose(params, layout[sensor], solve_attitude)
        pred = predict_angle(p_world, geometry[bs], sweep)
        meas = measured_observation_angle(obs, geometry[bs], sweep)
        out.append(angle_diff(pred, meas))

    return np.array(out, dtype=float)


def residuals_planar(params, observations, layout, geometry, fixed_z, solve_yaw):
    out = []

    for obs in observations:
        sensor = int(obs["sensor"])
        bs = int(obs["basestation"])
        sweep = int(obs["sweep"])

        if sensor not in layout or bs not in geometry:
            continue

        p_world = sensor_world_from_planar_pose(params, layout[sensor], fixed_z, solve_yaw)
        pred = predict_angle(p_world, geometry[bs], sweep)
        meas = measured_observation_angle(obs, geometry[bs], sweep)
        out.append(angle_diff(pred, meas))

    return np.array(out, dtype=float)


def solve_pose(observations, layout, geometry, previous, solve_attitude, bounds_xy, bounds_z):
    if solve_attitude:
        if previous is None:
            x0 = np.array([0.0, 0.0, 0.15, 0.0, 0.0, 0.0], dtype=float)
        else:
            x0 = previous

        lower = np.array([-bounds_xy, -bounds_xy, bounds_z[0], -math.pi, -math.pi, -math.pi], dtype=float)
        upper = np.array([+bounds_xy, +bounds_xy, bounds_z[1], +math.pi, +math.pi, +math.pi], dtype=float)
    else:
        if previous is None:
            x0 = np.array([0.0, 0.0, 0.15], dtype=float)
        else:
            x0 = previous[:3]

        lower = np.array([-bounds_xy, -bounds_xy, bounds_z[0]], dtype=float)
        upper = np.array([+bounds_xy, +bounds_xy, bounds_z[1]], dtype=float)

    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    result = least_squares(
        residuals,
        x0,
        bounds=(lower, upper),
        args=(observations, layout, geometry, solve_attitude),
        loss="soft_l1",
        f_scale=math.radians(1.0),
        max_nfev=80,
    )

    err = residuals(result.x, observations, layout, geometry, solve_attitude)
    rmse_deg = float(math.degrees(math.sqrt(float(np.mean(err ** 2))))) if len(err) else float("nan")
    return result.x, rmse_deg, len(err), bool(result.success)


def solve_planar_pose(observations, layout, geometry, previous, fixed_z, solve_yaw, bounds_xy):
    if solve_yaw:
        if previous is None:
            x0 = np.array([0.0, 0.0, 0.0], dtype=float)
        else:
            x0 = previous

        lower = np.array([-bounds_xy, -bounds_xy, -math.pi], dtype=float)
        upper = np.array([+bounds_xy, +bounds_xy, +math.pi], dtype=float)
    else:
        if previous is None:
            x0 = np.array([0.0, 0.0], dtype=float)
        else:
            x0 = previous[:2]

        lower = np.array([-bounds_xy, -bounds_xy], dtype=float)
        upper = np.array([+bounds_xy, +bounds_xy], dtype=float)

    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    result = least_squares(
        residuals_planar,
        x0,
        bounds=(lower, upper),
        args=(observations, layout, geometry, fixed_z, solve_yaw),
        loss="soft_l1",
        f_scale=math.radians(1.0),
        max_nfev=80,
    )

    err = residuals_planar(result.x, observations, layout, geometry, fixed_z, solve_yaw)
    rmse_deg = float(math.degrees(math.sqrt(float(np.mean(err ** 2))))) if len(err) else float("nan")
    return result.x, rmse_deg, len(err), bool(result.success)


def capture_window(ser, duration_s, basestations):
    buffer = {}
    start = time.time()

    while time.time() - start < duration_s:
        raw = ser.readline().decode(errors="ignore").strip()
        data = parse_lh2_line(raw)
        if data is None or data["basestation"] not in basestations:
            continue

        key = (data["sensor"], data["basestation"], data["sweep"])
        buffer.setdefault(key, []).append(data)

    observations = []
    for (sensor, bs, sweep), values in sorted(buffer.items()):
        lfsr_values = [item["lfsr_location"] for item in values]
        raw_angle_values = [
            item["raw_angle_rad"]
            for item in values
            if "raw_angle_rad" in item
        ]
        obs = {
            "sensor": int(sensor),
            "basestation": int(bs),
            "sweep": int(sweep),
            "lfsr_location": float(median(lfsr_values)),
            "sample_count": int(len(values)),
        }
        if raw_angle_values:
            obs["raw_angle_rad"] = float(median(raw_angle_values))
        observations.append(obs)

    return observations


def format_pose(params, solve_attitude):
    if solve_attitude:
        x, y, z, rx, ry, rz = params
        roll, pitch, yaw = Rotation.from_rotvec([rx, ry, rz]).as_euler("xyz", degrees=True)
        return (
            f"x={x:+.3f} y={y:+.3f} z={z:+.3f} m | "
            f"roll={roll:+.1f} pitch={pitch:+.1f} yaw={yaw:+.1f} deg"
        )

    x, y, z = params
    return f"x={x:+.3f} y={y:+.3f} z={z:+.3f} m"


def format_planar_pose(params, fixed_z, solve_yaw):
    if solve_yaw:
        x, y, yaw = params
        return f"x={x:+.3f} y={y:+.3f} z={fixed_z:+.3f} m | yaw={math.degrees(yaw):+.1f} deg"

    x, y = params
    return f"x={x:+.3f} y={y:+.3f} z={fixed_z:+.3f} m"


def main():
    parser = argparse.ArgumentParser(description="Live indoor drone positioning from calibrated Lighthouse V2 geometry.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--geometry", default="config/lighthouse_geometry_lh2_guided_ultrafast.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--window", type=float, default=0.20)
    parser.add_argument("--position-only", action="store_true", help="Estimate only x,y,z. Default estimates 6D pose.")
    parser.add_argument("--planar-2d", action="store_true", help="Estimate x,y at a fixed z. Add --solve-yaw for x,y,yaw.")
    parser.add_argument("--fixed-z", type=float, default=0.0, help="Fixed z used with --planar-2d.")
    parser.add_argument("--solve-yaw", action="store_true", help="With --planar-2d, estimate x,y,yaw instead of x,y only.")
    parser.add_argument("--xy-bound", type=float, default=5.0)
    parser.add_argument("--z-min", type=float, default=-0.20)
    parser.add_argument("--z-max", type=float, default=3.00)
    args = parser.parse_args()

    if serial is None:
        raise SystemExit("Missing dependency: install pyserial in the Python environment used for this script.")

    if least_squares is None or Rotation is None:
        raise SystemExit("Missing dependency: install scipy in the Python environment used for this script.")

    layout = load_layout(args.layout)
    geometry = load_geometry(args.geometry)
    basestations = [int(x) for x in args.basestations.split(",")]
    solve_attitude = not args.position_only
    planar_2d = bool(args.planar_2d)
    previous = None

    print("=" * 70)
    print("Live LH2 drone position")
    print(f"Layout:   {args.layout}")
    print(f"Geometry: {args.geometry}")
    if planar_2d:
        mode = "planar 2D + yaw" if args.solve_yaw else "planar 2D"
    else:
        mode = "6D pose" if solve_attitude else "position only"
    print(f"Mode:     {mode}")
    print(f"BS:       {basestations}")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    with serial.Serial(args.port, args.baudrate, timeout=0.05) as ser:
        while True:
            observations = capture_window(ser, args.window, basestations)
            if len(observations) < 6:
                print(f"waiting for enough LH2 measurements... got {len(observations)}")
                continue

            if planar_2d:
                previous, rmse_deg, used, success = solve_planar_pose(
                    observations,
                    layout,
                    geometry,
                    previous,
                    args.fixed_z,
                    args.solve_yaw,
                    args.xy_bound,
                )
                pose_text = format_planar_pose(previous, args.fixed_z, args.solve_yaw)
            else:
                previous, rmse_deg, used, success = solve_pose(
                    observations,
                    layout,
                    geometry,
                    previous,
                    solve_attitude,
                    args.xy_bound,
                    (args.z_min, args.z_max),
                )
                pose_text = format_pose(previous, solve_attitude)

            quality = "OK" if success and rmse_deg < 3.0 else "CHECK"
            print(f"{pose_text} | rmse={rmse_deg:.2f} deg | obs={used} | {quality}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
