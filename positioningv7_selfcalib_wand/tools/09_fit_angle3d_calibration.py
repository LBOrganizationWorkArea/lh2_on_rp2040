import argparse
import itertools
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from wand_common import (
    TAN_30,
    load_angle_modes,
    load_json,
    load_latest_coefficients,
    save_json,
)


def load_layout(path):
    data = load_json(path)
    return {
        int(item["sensor"]): np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ], dtype=float)
        for item in data["sensors"]
    }


def permute_layout(layout, permutation):
    sensor_ids = sorted(layout)
    positions = [layout[sensor] for sensor in sensor_ids]
    return {
        sensor: positions[permutation[idx]]
        for idx, sensor in enumerate(sensor_ids)
    }


def pose_rotation(pose):
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
    pitch = math.radians(float(pose.get("pitch_deg", 0.0)))
    roll = math.radians(float(pose.get("roll_deg", 0.0)))
    return Rotation.from_euler("zyx", [yaw, pitch, roll]).as_matrix()


def known_sensor_points(record, layout, anchor_sensor=None):
    points = {}
    for frame_idx, frame in enumerate(record["frames"]):
        pose = frame["pose"]
        center = np.array([float(pose["x_m"]), float(pose["y_m"]), float(pose["z_m"])], dtype=float)
        R = pose_rotation(pose)
        for sensor, offset in layout.items():
            if anchor_sensor is not None and sensor == anchor_sensor:
                points[(frame_idx, sensor)] = center
            else:
                points[(frame_idx, sensor)] = center + R @ offset
    return points


def filter_record_by_pose_z(record, max_pose_z):
    if max_pose_z is None:
        return record

    filtered = dict(record)
    filtered["frames"] = [
        frame for frame in record["frames"]
        if float(frame.get("pose", {}).get("z_m", 0.0)) <= max_pose_z
    ]
    return filtered


def collect_observations(record, bs, modes, points):
    obs = []
    for frame_idx, frame in enumerate(record["frames"]):
        bs_obs = frame.get("observations", {}).get(str(bs), {})
        for sensor_key, item in bs_obs.items():
            sensor = int(sensor_key)
            if (frame_idx, sensor) not in points:
                continue
            mode = modes.get(bs, {}).get(sensor, "normal")
            obs.append({
                "point": points[(frame_idx, sensor)],
                "frame": frame_idx,
                "pose": frame.get("pose", {}).get("name", str(frame_idx)),
                "sensor": sensor,
                "mode": mode,
                "lfsr0": float(item["lfsr0"]),
                "lfsr1": float(item["lfsr1"]),
            })
    return obs


def wrap_error_deg(value, target):
    err = value - target
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def measured_sweeps(obs, coeffs):
    A0, B0, A1, B1 = coeffs
    if obs["mode"] == "swapped":
        lfsr0 = obs["lfsr1"]
        lfsr1 = obs["lfsr0"]
    else:
        lfsr0 = obs["lfsr0"]
        lfsr1 = obs["lfsr1"]
    return A0 * lfsr0 + B0, A1 * lfsr1 + B1


def expected_sweeps_from_point(p):
    theta = math.atan2(p[0], p[2])
    image_v = p[1] / max(1e-9, p[2])
    phi = math.atan(image_v * math.cos(theta))

    # Inverse of positioningv4/tools/lh2v4.py alphas_to_theta_phi().
    value = math.tan(phi) * math.tan(math.pi / 6.0) * math.cos(theta)
    value = max(-1.0, min(1.0, value))
    half_delta = (math.pi / 3.0) + math.asin(value)

    alpha0 = theta - half_delta
    alpha1 = theta + half_delta
    return math.degrees(alpha0), math.degrees(alpha1)


def measured_image_abs(obs, coeffs):
    sweep0_deg, sweep1_deg = measured_sweeps(obs, coeffs)
    while sweep1_deg - sweep0_deg > 180.0:
        sweep1_deg -= 360.0
    while sweep1_deg - sweep0_deg < -180.0:
        sweep1_deg += 360.0

    alpha0 = math.radians(sweep0_deg)
    alpha1 = math.radians(sweep1_deg)
    azimuth = (alpha0 + alpha1) / 2.0
    half_delta = abs((alpha1 - alpha0) / 2.0)
    cos_azimuth = math.cos(azimuth)
    if abs(cos_azimuth) < 1e-6:
        cos_azimuth = math.copysign(1e-6, cos_azimuth)

    u = -math.tan(azimuth)
    v = (
        -math.sin(half_delta - math.pi / 3.0)
        / math.tan(math.pi / 6.0)
        / cos_azimuth
    )
    return u, v


def expected_image_from_point(p):
    return -p[0] / p[2], p[1] / p[2]


def coeff_array(coeff):
    if isinstance(coeff, dict):
        return np.array([coeff["A0"], coeff["B0"], coeff["A1"], coeff["B1"]], dtype=float)
    return np.asarray(coeff, dtype=float)


def split_params(params, fixed_coeff=None):
    rvec = params[0:3]
    tvec = params[3:6]
    if fixed_coeff is None:
        coeffs = params[6:10]
    else:
        coeffs = coeff_array(fixed_coeff)
    return rvec, tvec, coeffs


def residuals_sweep(params, obs, elevation_sign=1.0, fixed_coeff=None):
    rvec, tvec, coeffs = split_params(params, fixed_coeff)
    R = Rotation.from_rotvec(rvec).as_matrix()

    out = []
    for item in obs:
        p = R @ item["point"] + tvec
        if p[2] <= 0.02:
            out.extend([10.0, 10.0])
            continue

        expected0, expected1 = expected_sweeps_from_point(p)
        if elevation_sign < 0.0:
            expected0, expected1 = expected1, expected0
        sweep0, sweep1 = measured_sweeps(item, coeffs)

        out.append(wrap_error_deg(sweep0, expected0))
        out.append(wrap_error_deg(sweep1, expected1))

    return np.array(out, dtype=float)


def residuals_image_abs(params, obs, elevation_sign=1.0, fixed_coeff=None):
    del elevation_sign

    rvec, tvec, coeffs = split_params(params, fixed_coeff)
    R = Rotation.from_rotvec(rvec).as_matrix()

    out = []
    for item in obs:
        p = R @ item["point"] + tvec
        if p[2] <= 0.02:
            out.extend([10.0, 10.0])
            continue

        expected_u, expected_v = expected_image_from_point(p)
        measured_u, measured_v = measured_image_abs(item, coeffs)
        out.append(measured_u - expected_u)
        out.append(measured_v - expected_v)

    return np.array(out, dtype=float)


RESIDUAL_MODELS = {
    "sweep": {
        "fn": residuals_sweep,
        "unit": "deg",
        "f_scale": 2.0,
        "label": "sweep_rmse",
    },
    "image-abs": {
        "fn": residuals_image_abs,
        "unit": "image",
        "f_scale": 0.05,
        "label": "image_rmse",
    },
}


def residuals(params, obs, elevation_sign=1.0, model="sweep", fixed_coeff=None):
    return RESIDUAL_MODELS[model]["fn"](params, obs, elevation_sign, fixed_coeff)


def residual_pairs(params, obs, elevation_sign=1.0, model="sweep", fixed_coeff=None):
    values = residuals(params, obs, elevation_sign, model, fixed_coeff).reshape((-1, 2))
    out = []
    for item, pair in zip(obs, values):
        out.append({
            "frame": item["frame"],
            "pose": item["pose"],
            "sensor": item["sensor"],
            "e0": float(pair[0]),
            "e1": float(pair[1]),
            "rmse": float(np.sqrt(np.mean(pair ** 2))),
        })
    return out


def look_at_pose(center, target=np.zeros(3, dtype=float)):
    center = np.asarray(center, dtype=float)
    target = np.asarray(target, dtype=float)
    forward = target - center
    forward /= np.linalg.norm(forward)

    up = np.array([0.0, 0.0, 1.0], dtype=float)
    right = np.cross(up, forward)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0], dtype=float)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    down /= np.linalg.norm(down)

    R = np.vstack([right, down, forward])
    t = -R @ center
    rvec = Rotation.from_matrix(R).as_rotvec()
    return rvec, t


def initial_params(coeff, pose_center=None, fit_coefficients=True):
    if pose_center is None:
        rvec = np.array([0.0, 0.0, 0.0], dtype=float)
        tvec = np.array([0.0, 0.0, 1.5], dtype=float)
    else:
        rvec, tvec = look_at_pose(pose_center)

    if not fit_coefficients:
        return np.concatenate([rvec, tvec]).astype(float)

    return np.concatenate([rvec, tvec, coeff_array(coeff)]).astype(float)


def initial_candidates(coeff, fit_coefficients=True):
    centers = [
        None,
        np.array([0.0, -2.0, 1.5]),
        np.array([0.0, 2.0, 1.5]),
        np.array([2.0, 0.0, 1.5]),
        np.array([-2.0, 0.0, 1.5]),
        np.array([2.0, -2.0, 1.8]),
        np.array([-2.0, 2.0, 1.8]),
    ]
    return [initial_params(coeff, center, fit_coefficients) for center in centers]


def parameter_bounds(coeff, fit_coefficients=True):
    pose_lower = np.array([-math.pi, -math.pi, -math.pi, -8.0, -8.0, -8.0], dtype=float)
    pose_upper = np.array([math.pi, math.pi, math.pi, 8.0, 8.0, 8.0], dtype=float)
    if not fit_coefficients:
        return pose_lower, pose_upper

    a0 = float(coeff["A0"])
    a1 = float(coeff["A1"])
    return (
        np.concatenate([pose_lower, np.array([
            0.5 * a0, coeff["B0"] - 90.0, 0.5 * a1, coeff["B1"] - 90.0,
        ], dtype=float)]),
        np.concatenate([pose_upper, np.array([
            1.5 * a0, coeff["B0"] + 90.0, 1.5 * a1, coeff["B1"] + 90.0,
        ], dtype=float)]),
    )


def with_modes(obs, modes_by_sensor):
    out = []
    for item in obs:
        copy = dict(item)
        copy["mode"] = modes_by_sensor.get(item["sensor"], item["mode"])
        out.append(copy)
    return out


def fit_observations(obs, coeff, max_nfev, elevation_sign=1.0, model="sweep", fit_coefficients=True):
    fixed_coeff = None if fit_coefficients else coeff
    bounds = parameter_bounds(coeff, fit_coefficients)
    best_result = None
    best_after = None
    best_before = None

    for x0 in initial_candidates(coeff, fit_coefficients):
        before = residuals(x0, obs, elevation_sign, model, fixed_coeff)
        result = least_squares(
            residuals,
            x0,
            args=(obs, elevation_sign, model, fixed_coeff),
            bounds=bounds,
            loss="soft_l1",
            f_scale=RESIDUAL_MODELS[model]["f_scale"],
            max_nfev=max_nfev,
            verbose=0,
        )
        after = residuals(result.x, obs, elevation_sign, model, fixed_coeff)
        after_rmse = float(np.sqrt(np.mean(after ** 2)))
        if best_after is None or after_rmse < float(np.sqrt(np.mean(best_after ** 2))):
            best_result = result
            best_after = after
            best_before = before

    result = best_result
    before = best_before
    after = best_after
    before_rmse = float(np.sqrt(np.mean(before ** 2)))
    after_rmse = float(np.sqrt(np.mean(after ** 2)))
    return result, before_rmse, after_rmse


def find_best_convention(obs, coeff, max_nfev, model="sweep", fit_coefficients=True):
    sensors = sorted({item["sensor"] for item in obs})
    best = None

    elevation_signs = (1.0, -1.0) if model == "sweep" else (1.0,)
    for elevation_sign in elevation_signs:
        for mask in range(1 << len(sensors)):
            modes_by_sensor = {
                sensor: ("swapped" if mask & (1 << idx) else "normal")
                for idx, sensor in enumerate(sensors)
            }
            trial_obs = with_modes(obs, modes_by_sensor)
            result, before_rmse, after_rmse = fit_observations(
                trial_obs,
                coeff,
                max_nfev,
                elevation_sign,
                model,
                fit_coefficients,
            )
            if best is None or after_rmse < best["after_rmse"]:
                best = {
                    "obs": trial_obs,
                    "result": result,
                    "before_rmse": before_rmse,
                    "after_rmse": after_rmse,
                    "elevation_sign": elevation_sign,
                    "modes_by_sensor": modes_by_sensor,
                }

    return best


def diagnose_layout(record, layout, coeffs, modes, basestations, max_nfev, model, fit_coefficients):
    sensor_ids = sorted(layout)
    candidates = []

    for permutation in itertools.permutations(range(len(sensor_ids))):
        trial_layout = permute_layout(layout, permutation)
        points = known_sensor_points(record, trial_layout)
        bs_results = {}
        score = 0.0

        for bs in basestations:
            obs = collect_observations(record, bs, modes, points)
            if len(obs) < 12:
                score = float("inf")
                break

            best = find_best_convention(obs, coeffs[bs], max_nfev, model, fit_coefficients)
            score += best["after_rmse"]
            bs_results[bs] = best

        candidates.append({
            "score": score,
            "permutation": permutation,
            "bs_results": bs_results,
        })

    candidates.sort(key=lambda item: item["score"])

    print("=" * 70)
    print("Sensor layout permutation diagnostic")
    print("Mapping format: sensor_id -> source layout position sensor_id")
    print("=" * 70)
    for candidate in candidates[:8]:
        mapping = {
            sensor_ids[idx]: sensor_ids[candidate["permutation"][idx]]
            for idx in range(len(sensor_ids))
        }
        print(f"score={candidate['score']:.3f} deg mapping={mapping}")
        for bs in basestations:
            best = candidate["bs_results"][bs]
            modes_text = ", ".join(f"{sensor}:{mode}" for sensor, mode in best["modes_by_sensor"].items())
            unit = RESIDUAL_MODELS[model]["unit"]
            print(
                f"  BS{bs}: rmse={best['after_rmse']:.3f} {unit} "
                f"elevation_sign={best['elevation_sign']:+.0f} modes={{{modes_text}}}"
            )


def fit_bs_observations(obs, coeff, max_nfev, model, fit_coefficients, try_conventions):
    elevation_sign = 1.0
    modes_by_sensor = {
        sensor: obs_item["mode"]
        for sensor in sorted({item["sensor"] for item in obs})
        for obs_item in obs
        if obs_item["sensor"] == sensor
    }

    if try_conventions:
        best = find_best_convention(obs, coeff, max_nfev, model, fit_coefficients)
        return (
            best["obs"],
            best["result"],
            best["before_rmse"],
            best["after_rmse"],
            best["elevation_sign"],
            best["modes_by_sensor"],
        )

    result, before_rmse, after_rmse = fit_observations(
        obs,
        coeff,
        max_nfev,
        elevation_sign,
        model,
        fit_coefficients,
    )
    return obs, result, before_rmse, after_rmse, elevation_sign, modes_by_sensor


def result_to_output(result, coeff, fit_coefficients, observations, before_rmse, after_rmse, model, elevation_sign, modes_by_sensor):
    rvec = result.x[0:3]
    tvec = result.x[3:6]
    c = result.x[6:10] if fit_coefficients else coeff_array(coeff)
    R = Rotation.from_rotvec(rvec).as_matrix()
    T_bs_from_world = np.eye(4, dtype=float)
    T_bs_from_world[:3, :3] = R
    T_bs_from_world[:3, 3] = tvec
    T_world_from_bs = np.linalg.inv(T_bs_from_world)
    metric_key = model.replace("-", "_")

    return {
        "observations": observations,
        f"rmse_{metric_key}_initial": before_rmse,
        f"rmse_{metric_key}_final": after_rmse,
        "coefficients": {
            "A0": float(c[0]),
            "B0": float(c[1]),
            "A1": float(c[2]),
            "B1": float(c[3]),
        },
        "elevation_sign": float(elevation_sign),
        "modes_by_sensor": {str(sensor): mode for sensor, mode in modes_by_sensor.items()},
        "T_bs_from_world": T_bs_from_world.tolist(),
        "T_world_from_bs": T_world_from_bs.tolist(),
    }, T_world_from_bs


def main():
    parser = argparse.ArgumentParser(description="Fit 3D LH2 angle calibration from known drone poses.")
    parser.add_argument("--input", default="data/angle3d_calibration.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--modes", default="config/angle_modes.json")
    parser.add_argument("--output", default="config/angle3d_calibration.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--max-nfev", type=int, default=2000)
    parser.add_argument("--model", choices=sorted(RESIDUAL_MODELS), default="image-abs")
    parser.add_argument("--fit-coefficients", action="store_true")
    parser.add_argument("--try-conventions", action="store_true")
    parser.add_argument("--per-sensor", action="store_true")
    parser.add_argument("--anchor-sensor", type=int, default=None)
    parser.add_argument("--only-sensor", type=int, default=None)
    parser.add_argument("--max-pose-z", type=float, default=None)
    parser.add_argument("--diagnose-layout", action="store_true")
    args = parser.parse_args()
    if args.per_sensor and args.output == "config/angle3d_calibration.json":
        args.output = "config/angle3d_calibration_per_sensor.json"

    record = load_json(args.input)
    record = filter_record_by_pose_z(record, args.max_pose_z)
    layout = load_layout(args.layout)
    coeffs = load_latest_coefficients(args.history)
    modes = load_angle_modes(args.modes)
    points = known_sensor_points(record, layout, args.anchor_sensor)
    basestations = [int(x) for x in args.basestations.split(",")]
    fit_coefficients = args.fit_coefficients or args.model == "sweep"

    if args.diagnose_layout:
        diagnose_layout(record, layout, coeffs, modes, basestations, args.max_nfev, args.model, fit_coefficients)
        return

    output = {
        "description": "3D-fitted LH2 calibration and basestation poses. World is the measured calibration-point frame.",
        "model": args.model,
        "basestations": {},
    }
    if args.per_sensor:
        output["description"] = "Per-sensor 3D-fitted LH2 calibration. Each sensor is fitted independently."
        output["sensors"] = {}

    print("=" * 70)
    print("Fit LH2 3D angle calibration")
    print(f"Input: {args.input}")
    print(f"Model: {args.model}")
    if not fit_coefficients:
        print("Coefficients: fixed from history_calibration.txt")
    if args.max_pose_z is not None:
        print(f"Using only frames with pose z <= {args.max_pose_z:.3f} m ({len(record['frames'])} frames)")
    if args.per_sensor:
        print("Mode: per-sensor independent fits")
    if args.anchor_sensor is not None:
        print(f"Anchor sensor: sensor {args.anchor_sensor} is treated as the measured point")
    print("=" * 70)

    if args.per_sensor:
        label = RESIDUAL_MODELS[args.model]["label"]
        unit = RESIDUAL_MODELS[args.model]["unit"]
        sensor_ids = sorted(layout)
        if args.only_sensor is not None:
            sensor_ids = [args.only_sensor]

        for sensor in sensor_ids:
            output["sensors"][str(sensor)] = {"basestations": {}}
            print(f"sensor={sensor}")

            for bs in basestations:
                all_obs = collect_observations(record, bs, modes, points)
                obs = [item for item in all_obs if item["sensor"] == sensor]
                if len(obs) < 6:
                    print(f"  BS{bs}: not enough observations ({len(obs)})")
                    continue

                obs, result, before_rmse, after_rmse, elevation_sign, modes_by_sensor = fit_bs_observations(
                    obs,
                    coeffs[bs],
                    args.max_nfev,
                    args.model,
                    fit_coefficients,
                    args.try_conventions,
                )
                packaged, T_world_from_bs = result_to_output(
                    result,
                    coeffs[bs],
                    fit_coefficients,
                    len(obs),
                    before_rmse,
                    after_rmse,
                    args.model,
                    elevation_sign,
                    modes_by_sensor,
                )
                output["sensors"][str(sensor)]["basestations"][str(bs)] = packaged
                print(
                    f"  BS{bs}: obs={len(obs)} {label} {before_rmse:.3f} {unit} -> {after_rmse:.3f} {unit} | "
                    f"pos_world=({T_world_from_bs[0, 3]:+.3f},{T_world_from_bs[1, 3]:+.3f},{T_world_from_bs[2, 3]:+.3f}) m | "
                    f"mode={modes_by_sensor.get(sensor, 'normal')}"
                )

        save_json(args.output, output)
        print("=" * 70)
        print(f"Saved: {args.output}")
        print("=" * 70)
        return

    for bs in basestations:
        obs = collect_observations(record, bs, modes, points)
        if len(obs) < 12:
            print(f"BS{bs}: not enough observations ({len(obs)})")
            continue

        elevation_sign = 1.0
        modes_by_sensor = {sensor: modes.get(bs, {}).get(sensor, "normal") for sensor in sorted({item["sensor"] for item in obs})}
        if args.try_conventions:
            best = find_best_convention(obs, coeffs[bs], args.max_nfev, args.model, fit_coefficients)
            obs = best["obs"]
            result = best["result"]
            before_rmse = best["before_rmse"]
            after_rmse = best["after_rmse"]
            elevation_sign = best["elevation_sign"]
            modes_by_sensor = best["modes_by_sensor"]
        else:
            result, before_rmse, after_rmse = fit_observations(
                obs,
                coeffs[bs],
                args.max_nfev,
                elevation_sign,
                args.model,
                fit_coefficients,
            )

        rvec = result.x[0:3]
        tvec = result.x[3:6]
        c = result.x[6:10] if fit_coefficients else coeff_array(coeffs[bs])
        R = Rotation.from_rotvec(rvec).as_matrix()
        T_bs_from_world = np.eye(4, dtype=float)
        T_bs_from_world[:3, :3] = R
        T_bs_from_world[:3, 3] = tvec
        T_world_from_bs = np.linalg.inv(T_bs_from_world)

        label = RESIDUAL_MODELS[args.model]["label"]
        unit = RESIDUAL_MODELS[args.model]["unit"]
        print(
            f"BS{bs}: obs={len(obs)} {label} {before_rmse:.3f} {unit} -> {after_rmse:.3f} {unit} | "
            f"pos_world=({T_world_from_bs[0, 3]:+.3f},{T_world_from_bs[1, 3]:+.3f},{T_world_from_bs[2, 3]:+.3f}) m"
        )
        print(
            f"  convention: elevation_sign={elevation_sign:+.0f} "
            f"modes={{{', '.join(f'{sensor}:{mode}' for sensor, mode in modes_by_sensor.items())}}}"
        )
        worst = sorted(
            residual_pairs(
                result.x,
                obs,
                elevation_sign,
                args.model,
                None if fit_coefficients else coeffs[bs],
            ),
            key=lambda item: item["rmse"],
            reverse=True,
        )[:8]
        print("  worst residuals:")
        for item in worst:
            print(
                f"    frame={item['frame']:02d} {item['pose']} sensor={item['sensor']} "
                f"e0={item['e0']:+.3f} {unit} e1={item['e1']:+.3f} {unit} rmse={item['rmse']:.3f} {unit}"
            )

        metric_key = args.model.replace("-", "_")
        output["basestations"][str(bs)] = {
            "observations": len(obs),
            f"rmse_{metric_key}_initial": before_rmse,
            f"rmse_{metric_key}_final": after_rmse,
            "coefficients": {
                "A0": float(c[0]),
                "B0": float(c[1]),
                "A1": float(c[2]),
                "B1": float(c[3]),
            },
            "elevation_sign": float(elevation_sign),
            "modes_by_sensor": {str(sensor): mode for sensor, mode in modes_by_sensor.items()},
            "T_bs_from_world": T_bs_from_world.tolist(),
            "T_world_from_bs": T_world_from_bs.tolist(),
        }

    save_json(args.output, output)
    print("=" * 70)
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
