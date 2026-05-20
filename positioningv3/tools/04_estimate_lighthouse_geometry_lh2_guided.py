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
TILT_NEG = -math.pi / 6.0
TILT_POS = +math.pi / 6.0


def lfsr_to_raw_rad(lfsr):
    angle_deg = (((float(lfsr) % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(angle_deg)


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


def load_poses(path):
    with open(path, "r") as f:
        data = json.load(f)

    obs = []

    for pose in data["poses"]:
        px = float(pose["x_m"])
        py = float(pose["y_m"])
        yaw = math.radians(float(pose.get("yaw_deg", 0.0)))

        grouped = {}

        for m in pose["measurements"]:
            sensor = int(m["sensor"])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            raw = lfsr_to_raw_rad(float(m["median_lfsr_location"]))
            grouped.setdefault((sensor, bs, sweep), []).append(raw)

        for (sensor, bs, sweep), values in grouped.items():
            obs.append({
                "pose": pose["name"],
                "pose_x": px,
                "pose_y": py,
                "pose_yaw": yaw,
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "raw_angle": float(median(values)),
            })

    return obs


def sensor_world(pose_x, pose_y, pose_yaw, drone_z, local):
    c = math.cos(pose_yaw)
    s = math.sin(pose_yaw)

    R = np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ], dtype=float)

    return np.array([pose_x, pose_y, drone_z], dtype=float) + R @ local


def lh2_model(p_lh, tilt):
    x, y, z = p_lh
    r = math.sqrt(x*x + y*y)
    if r < 1e-9:
        r = 1e-9

    v = (z * math.tan(tilt)) / r
    v = max(-0.999999, min(0.999999, v))

    return math.atan2(y, x) + math.asin(v)


def predict_angle(p_world, params, lighthouse_z, tilt):
    # params = rx, ry, rz, tx, ty, off0, off1
    rx, ry, rz, tx, ty, off0, off1 = params

    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, lighthouse_z], dtype=float)

    p_lh = R @ (p_world - t)
    return lh2_model(p_lh, tilt)


def corrected_raw(raw, sweep, params, signs):
    off0 = params[5]
    off1 = params[6]

    if sweep == 0:
        return signs[0] * raw + off0
    return signs[1] * raw + off1


def residuals(params, bs, observations, layout, lighthouse_z, drone_z, tilt_map, signs):
    out = []

    for o in observations:
        if o["basestation"] != bs:
            continue

        sensor = o["sensor"]
        if sensor not in layout:
            continue

        p_world = sensor_world(
            o["pose_x"],
            o["pose_y"],
            o["pose_yaw"],
            drone_z,
            layout[sensor],
        )

        sweep = o["sweep"]
        tilt = tilt_map[sweep]

        pred = predict_angle(p_world, params, lighthouse_z, tilt)
        meas = corrected_raw(o["raw_angle"], sweep, params, signs)

        out.append(angle_diff(pred, meas))

    return np.array(out, dtype=float)


def fit_bs(bs, observations, layout, lighthouse_z, drone_z, guess_x, guess_y):
    tilt_maps = [
        {0: TILT_NEG, 1: TILT_POS},
        {0: TILT_POS, 1: TILT_NEG},
    ]

    sign_maps = [
        {0: +1.0, 1: +1.0},
        {0: +1.0, 1: -1.0},
        {0: -1.0, 1: +1.0},
        {0: -1.0, 1: -1.0},
    ]

    # Very small set of rotations around plausible orientations.
    rotation_guesses = [
        [0.0, 0.0, 0.0],
        [0.0, math.pi / 2, 0.0],
        [0.0, -math.pi / 2, 0.0],
        [math.pi / 2, 0.0, 0.0],
        [-math.pi / 2, 0.0, 0.0],
    ]

    position_guesses = [
        [guess_x, guess_y],
        [guess_x + 0.5, guess_y],
        [guess_x - 0.5, guess_y],
        [guess_x, guess_y + 0.5],
        [guess_x, guess_y - 0.5],
    ]

    offset_guesses = [
        [0.0, 0.0],
        [math.radians(30), math.radians(-30)],
        [math.radians(-30), math.radians(30)],
    ]

    candidates = []

    for tilt_map in tilt_maps:
        for signs in sign_maps:
            for tx0, ty0 in position_guesses:
                for rot0 in rotation_guesses:
                    for off0, off1 in offset_guesses:
                        x0 = np.array([
                            rot0[0], rot0[1], rot0[2],
                            tx0, ty0,
                            off0, off1,
                        ], dtype=float)

                        lower = np.array([
                            -math.pi, -math.pi, -math.pi,
                            guess_x - 1.5, guess_y - 1.5,
                            -math.pi, -math.pi,
                        ], dtype=float)

                        upper = np.array([
                            +math.pi, +math.pi, +math.pi,
                            guess_x + 1.5, guess_y + 1.5,
                            +math.pi, +math.pi,
                        ], dtype=float)

                        x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

                        res = least_squares(
                            residuals,
                            x0,
                            bounds=(lower, upper),
                            args=(bs, observations, layout, lighthouse_z, drone_z, tilt_map, signs),
                            loss="soft_l1",
                            f_scale=math.radians(1.0),
                            max_nfev=300,
                        )

                        err = residuals(res.x, bs, observations, layout, lighthouse_z, drone_z, tilt_map, signs)
                        rmse = float(np.sqrt(np.mean(err**2)))
                        candidates.append((rmse, res.x, tilt_map, signs, len(err)))

    candidates.sort(key=lambda x: x[0])
    rmse, params, tilt_map, signs, n = candidates[0]

    rx, ry, rz, tx, ty, off0, off1 = params
    R = Rotation.from_rotvec([rx, ry, rz]).as_matrix()

    return {
        "basestation": int(bs),
        "model": "lh2_corrected_guided",
        "rmse_deg": float(math.degrees(rmse)),
        "num_residuals": int(n),
        "lighthouse_z_m": float(lighthouse_z),
        "drone_z_m": float(drone_z),
        "sweep_tilts": {
            "sweep_0_rad": float(tilt_map[0]),
            "sweep_1_rad": float(tilt_map[1]),
            "sweep_0_deg": float(math.degrees(tilt_map[0])),
            "sweep_1_deg": float(math.degrees(tilt_map[1])),
        },
        "angle_correction": {
            "sign_sweep_0": float(signs[0]),
            "sign_sweep_1": float(signs[1]),
            "offset_sweep_0_rad": float(off0),
            "offset_sweep_1_rad": float(off1),
            "offset_sweep_0_deg": float(math.degrees(off0)),
            "offset_sweep_1_deg": float(math.degrees(off1)),
        },
        "world_to_lighthouse": {
            "rotation_vector": [float(rx), float(ry), float(rz)],
            "rotation_matrix": R.tolist(),
            "translation_m": [float(tx), float(ty), float(lighthouse_z)],
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Guided fast LH2 geometry calibration.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_lh2_guided.json")
    parser.add_argument("--lighthouse-z", type=float, default=1.20)
    parser.add_argument("--drone-z", type=float, default=0.00)

    parser.add_argument("--bs4-guess", default="-1.0,1.0", help="approx x,y for BS4")
    parser.add_argument("--bs10-guess", default="1.0,1.0", help="approx x,y for BS10")

    args = parser.parse_args()

    layout = load_layout(args.layout)
    obs = load_poses(args.poses)

    bs4_guess = [float(x) for x in args.bs4_guess.split(",")]
    bs10_guess = [float(x) for x in args.bs10_guess.split(",")]

    print("=" * 70)
    print("Guided fast LH2 geometry calibration")
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print(f"BS4 guess:  {bs4_guess}")
    print(f"BS10 guess: {bs10_guess}")
    print("=" * 70)

    results = []

    for bs, guess in [(4, bs4_guess), (10, bs10_guess)]:
        print()
        print(f"Fitting BS{bs} around guess x={guess[0]:+.2f}, y={guess[1]:+.2f} ...")

        geom = fit_bs(
            bs,
            obs,
            layout,
            args.lighthouse_z,
            args.drone_z,
            guess[0],
            guess[1],
        )

        results.append(geom)

        t = geom["world_to_lighthouse"]["translation_m"]
        corr = geom["angle_correction"]
        tilts = geom["sweep_tilts"]

        print(f"BS{bs}")
        print(f"  RMSE: {geom['rmse_deg']:.4f} deg")
        print(f"  translation: x={t[0]:+.3f}, y={t[1]:+.3f}, z={t[2]:+.3f} m")
        print(f"  tilts: sweep0={tilts['sweep_0_deg']:+.1f} deg, sweep1={tilts['sweep_1_deg']:+.1f} deg")
        print(f"  signs: {corr['sign_sweep_0']:+.0f}, {corr['sign_sweep_1']:+.0f}")
        print(f"  offsets: {corr['offset_sweep_0_deg']:+.2f} deg, {corr['offset_sweep_1_deg']:+.2f} deg")

    output = {
        "description": "Guided LH2 geometry calibration with LFSR correction.",
        "input_layout": args.layout,
        "input_poses": args.poses,
        "lighthouse_z_m": args.lighthouse_z,
        "drone_z_m": args.drone_z,
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