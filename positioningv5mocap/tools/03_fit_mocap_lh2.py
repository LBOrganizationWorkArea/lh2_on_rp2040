import argparse
import math
from collections import defaultdict

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from mocap_lh2 import (
    MocapInterpolator,
    TICKS_PER_REV,
    angle_wrap,
    default_lfsr_to_alpha,
    lh2_sweep_angle_from_point,
    load_lh2_csv,
    load_mocap_csv,
    load_sensor_layout,
    save_json,
    sensor_world_position,
)


TILT_POS = math.radians(30.0)
TILT_NEG = math.radians(-30.0)


def build_observations(lh2_rows, mocap, layout, basestations, time_offset_s, max_rows_per_bs):
    per_bs = defaultdict(list)
    counts = defaultdict(int)

    for row in lh2_rows:
        bs = int(row["basestation"])
        if basestations is not None and bs not in basestations:
            continue
        if int(row["sweep"]) not in (0, 1):
            continue
        if int(row["sensor"]) not in layout:
            continue
        if counts[bs] >= max_rows_per_bs:
            continue

        t = float(row["pc_time_s"]) + time_offset_s
        if not mocap.contains(t):
            continue

        pos, rot = mocap.pose_at(t)
        p_world = sensor_world_position(pos, rot, layout[int(row["sensor"])])

        per_bs[bs].append({
            "p_world": p_world,
            "sweep": int(row["sweep"]),
            "lfsr": float(row["lfsr_location"]),
            "raw_alpha": default_lfsr_to_alpha(float(row["lfsr_location"])),
            "polynomial": int(row["polynomial"]),
            "sensor": int(row["sensor"]),
            "pc_time_s": float(row["pc_time_s"]),
        })
        counts[bs] += 1

    return per_bs


def alpha_from_lfsr(lfsr, sweep, params, calibrate_lfsr):
    if not calibrate_lfsr:
        return default_lfsr_to_alpha(lfsr)

    # params tail: a0_scale, b0, a1_scale, b1
    a0_scale, b0, a1_scale, b1 = params[6:10]
    raw = default_lfsr_to_alpha(lfsr)

    if sweep == 0:
        return a0_scale * raw + b0
    return a1_scale * raw + b1


def residuals(params, observations, sweep_tilts, calibrate_lfsr):
    rotvec = params[0:3]
    trans = params[3:6]
    world_to_lh = Rotation.from_rotvec(rotvec).as_matrix()

    out = []

    for obs in observations:
        p_lh = world_to_lh @ (obs["p_world"] - trans)
        pred = lh2_sweep_angle_from_point(p_lh, sweep_tilts[int(obs["sweep"])])
        meas = alpha_from_lfsr(obs["lfsr"], int(obs["sweep"]), params, calibrate_lfsr)
        out.append(angle_wrap(pred - meas))

    return np.array(out, dtype=float)


def initial_params(observations, calibrate_lfsr):
    points = np.array([obs["p_world"] for obs in observations], dtype=float)
    center = np.mean(points, axis=0)
    guess = np.array([
        0.0, 0.0, 0.0,
        center[0], center[1] - 1.5, center[2] + 1.2,
    ], dtype=float)

    if calibrate_lfsr:
        guess = np.concatenate([guess, np.array([1.0, 0.0, 1.0, 0.0], dtype=float)])

    return guess


def fit_basestation(bs, observations, calibrate_lfsr):
    sweep_maps = [
        {0: TILT_POS, 1: TILT_NEG},
        {0: TILT_NEG, 1: TILT_POS},
    ]

    best = None

    for sweep_tilts in sweep_maps:
        x0 = initial_params(observations, calibrate_lfsr)

        if calibrate_lfsr:
            lower = np.array([-math.pi, -math.pi, -math.pi, -5.0, -5.0, -1.0, 0.5, -math.pi, 0.5, -math.pi])
            upper = np.array([+math.pi, +math.pi, +math.pi, +5.0, +5.0, +4.0, 1.5, +math.pi, 1.5, +math.pi])
        else:
            lower = np.array([-math.pi, -math.pi, -math.pi, -5.0, -5.0, -1.0])
            upper = np.array([+math.pi, +math.pi, +math.pi, +5.0, +5.0, +4.0])

        x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

        result = least_squares(
            residuals,
            x0,
            bounds=(lower, upper),
            args=(observations, sweep_tilts, calibrate_lfsr),
            loss="soft_l1",
            f_scale=math.radians(1.0),
            max_nfev=2000,
        )

        err = residuals(result.x, observations, sweep_tilts, calibrate_lfsr)
        rmse_rad = float(math.sqrt(np.mean(err ** 2)))
        candidate = {
            "params": result.x,
            "sweep_tilts": sweep_tilts,
            "rmse_rad": rmse_rad,
            "rmse_deg": float(math.degrees(rmse_rad)),
            "median_abs_deg": float(math.degrees(np.median(np.abs(err)))),
            "max_abs_deg": float(math.degrees(np.max(np.abs(err)))),
            "num_observations": int(len(observations)),
            "success": bool(result.success),
            "cost": float(result.cost),
        }

        if best is None or candidate["rmse_rad"] < best["rmse_rad"]:
            best = candidate

    params = best["params"]
    R = Rotation.from_rotvec(params[0:3]).as_matrix()

    output = {
        "basestation": int(bs),
        "model": "mocap_lh2_pose_and_angle_fit",
        "rmse_deg": best["rmse_deg"],
        "median_abs_deg": best["median_abs_deg"],
        "max_abs_deg": best["max_abs_deg"],
        "num_observations": best["num_observations"],
        "success": best["success"],
        "sweep_tilts": {
            "sweep_0_rad": float(best["sweep_tilts"][0]),
            "sweep_1_rad": float(best["sweep_tilts"][1]),
            "sweep_0_deg": float(math.degrees(best["sweep_tilts"][0])),
            "sweep_1_deg": float(math.degrees(best["sweep_tilts"][1])),
        },
        "world_to_lighthouse": {
            "rotation_vector": [float(x) for x in params[0:3]],
            "rotation_matrix": R.tolist(),
            "translation_m": [float(x) for x in params[3:6]],
        },
        "angle_conversion": {
            "type": "raw_alpha_scale_offset" if calibrate_lfsr else "raw_default",
            "raw_default": "alpha_rad = radians(((lfsr % 833333) / 833333) * 120 - 60)",
        },
    }

    if calibrate_lfsr:
        output["angle_conversion"].update({
            "sweep_0_scale": float(params[6]),
            "sweep_0_offset_rad": float(params[7]),
            "sweep_0_offset_deg": float(math.degrees(params[7])),
            "sweep_1_scale": float(params[8]),
            "sweep_1_offset_rad": float(params[9]),
            "sweep_1_offset_deg": float(math.degrees(params[9])),
        })

    return output


def main():
    parser = argparse.ArgumentParser(description="Fit Lighthouse poses and LH2 angle conversion from mocap.")
    parser.add_argument("--lh2", default="data/lh2_record.csv")
    parser.add_argument("--mocap", default="data/mocap.csv")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--output", default="config/mocap_lh2_calibration.json")
    parser.add_argument("--basestations", help="Example: 4,10. Default: detected.")
    parser.add_argument("--time-offset", type=float, default=0.0, help="Added to LH2 pc_time_s before mocap lookup.")
    parser.add_argument("--max-rows-per-bs", type=int, default=4000)
    parser.add_argument("--no-angle-calibration", action="store_true")
    args = parser.parse_args()

    lh2_rows = load_lh2_csv(args.lh2)
    mocap_rows = load_mocap_csv(args.mocap)
    layout = load_sensor_layout(args.layout)
    mocap = MocapInterpolator(mocap_rows)
    basestations = None if args.basestations is None else {int(x) for x in args.basestations.split(",")}

    per_bs = build_observations(
        lh2_rows,
        mocap,
        layout,
        basestations,
        args.time_offset,
        args.max_rows_per_bs,
    )

    print("=" * 70)
    print("Fit mocap LH2 calibration")
    print(f"LH2:    {args.lh2}")
    print(f"Mocap:  {args.mocap}")
    print(f"Output: {args.output}")
    print(f"Angle calibration: {not args.no_angle_calibration}")
    print("=" * 70)

    results = []
    for bs in sorted(per_bs):
        observations = per_bs[bs]
        if len(observations) < 50:
            print(f"BS{bs}: skipped, only {len(observations)} observations")
            continue

        print(f"BS{bs}: fitting {len(observations)} observations...")
        fitted = fit_basestation(bs, observations, calibrate_lfsr=not args.no_angle_calibration)
        results.append(fitted)
        print(
            f"  rmse={fitted['rmse_deg']:.3f} deg | "
            f"median={fitted['median_abs_deg']:.3f} deg | "
            f"max={fitted['max_abs_deg']:.3f} deg"
        )

    save_json(args.output, {
        "description": "Mocap-based Lighthouse/LH2 calibration.",
        "input_lh2": args.lh2,
        "input_mocap": args.mocap,
        "input_layout": args.layout,
        "time_offset_s": args.time_offset,
        "basestations": results,
    })

    print("=" * 70)
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
