#!/usr/bin/env python3

import argparse
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


TAN_30 = math.tan(math.radians(30.0))


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_object_points(path, sensor_order):
    data = load_json(path)
    by_sensor = {
        int(item["sensor"]): [
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ]
        for item in data["sensors"]
    }
    return np.array([by_sensor[s] for s in sensor_order], dtype=np.float32).reshape((-1, 1, 3))


def angle_diff_rad(a, b):
    return (float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_near_rad(angle, reference):
    return reference + angle_diff_rad(angle, reference)


def compute_az_el_rad(sweep0, sweep1):
    sweep1 = unwrap_near_rad(sweep1, sweep0)
    az = 0.5 * (sweep0 + sweep1)
    el = (sweep0 - sweep1) / (2.0 * TAN_30)
    return az, el


def rt_to_matrix(rvec, tvec):
    out = np.eye(4, dtype=float)
    out[:3, :3] = Rotation.from_rotvec(np.asarray(rvec, dtype=float).reshape(3)).as_matrix()
    out[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return out


def transform_error(T, ref):
    dt = float(np.linalg.norm(T[:3, 3] - ref[:3, 3]))
    dR = T[:3, :3] @ ref[:3, :3].T
    trace = float(np.trace(dR))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))
    return dt, angle


def average_transforms(transforms):
    translations = np.array([T[:3, 3] for T in transforms], dtype=float)
    rotations = Rotation.from_matrix(np.array([T[:3, :3] for T in transforms], dtype=float))
    out = np.eye(4, dtype=float)
    out[:3, :3] = rotations.mean().as_matrix()
    out[:3, 3] = np.median(translations, axis=0)
    return out


def transform_cluster_score(T, transforms, trans_radius, rot_radius_deg):
    count = 0
    trans_sum = 0.0
    rot_sum = 0.0
    for other in transforms:
        dt, da = transform_error(other, T)
        if dt <= trans_radius and da <= rot_radius_deg:
            count += 1
            trans_sum += dt
            rot_sum += da
    return count, trans_sum, rot_sum


def select_best_cluster(transforms, trans_radius, rot_radius_deg):
    if not transforms:
        return []
    scored = [
        (transform_cluster_score(T, transforms, trans_radius, rot_radius_deg), idx)
        for idx, T in enumerate(transforms)
    ]
    scored.sort(key=lambda item: (-item[0][0], item[0][1], item[0][2]))
    center = transforms[scored[0][1]]
    return [
        T for T in transforms
        if transform_error(T, center)[0] <= trans_radius
        and transform_error(T, center)[1] <= rot_radius_deg
    ]


def group_measurements(frame, max_family_spread_deg, max_families):
    grouped = {}
    for m in frame.get("measurements", []):
        sensor = int(m["sensor"])
        bs = int(m["basestation"])
        sweep = int(m["sweep"])
        families = [
            f for f in m.get("candidate_families", [])
            if float(f.get("angle_spread_deg", 0.0)) <= max_family_spread_deg
        ][:max_families]
        if families:
            grouped[(sensor, bs, sweep)] = families
    return grouped


def sensor_pair_candidates(grouped, sensor, bs, max_pairs):
    f0 = grouped.get((sensor, bs, 0), [])
    f1 = grouped.get((sensor, bs, 1), [])
    pairs = []
    for a, b in itertools.product(f0, f1):
        az, el = compute_az_el_rad(float(a["raw_angle_rad"]), float(b["raw_angle_rad"]))
        if not (math.isfinite(az) and math.isfinite(el)):
            continue
        if abs(az) > math.radians(88.0) or abs(el) > math.radians(88.0):
            continue
        spread = float(a.get("angle_spread_deg", 0.0)) + float(b.get("angle_spread_deg", 0.0))
        pairs.append({
            "image": [math.tan(az), math.tan(el)],
            "az_rad": az,
            "el_rad": el,
            "spread_deg": spread,
            "families": [a.get("rank"), b.get("rank")],
        })
    pairs.sort(key=lambda item: item["spread_deg"])
    return pairs[:max_pairs]


def choose_pnp_solution(rvecs, tvecs, reprojection_errors, object_points, image_points, previous_T):
    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)
    choices = []
    for idx, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        rvec = np.asarray(rvec, dtype=float).reshape(3)
        tvec = np.asarray(tvec, dtype=float).reshape(3)
        if tvec[2] <= 0.0:
            continue
        if reprojection_errors is not None and len(reprojection_errors) > idx:
            rmse = float(np.asarray(reprojection_errors[idx]).reshape(-1)[0])
        else:
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
            residual = image_points.reshape((-1, 2)) - projected.reshape((-1, 2))
            rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        continuity = 0.0
        if previous_T is not None:
            T = rt_to_matrix(rvec, tvec)
            dt, da = transform_error(T, previous_T)
            continuity = dt + 0.01 * da
        choices.append((rmse + 0.05 * continuity, rmse, rvec, tvec))
    if not choices:
        return None
    choices.sort(key=lambda item: item[0])
    _score, rmse, rvec, tvec = choices[0]
    return rvec, tvec, rmse


def solve_pnp_for_bs(grouped, bs, sensor_order, object_points, previous_T, max_pairs, y_sign, max_combos):
    per_sensor = []
    for sensor in sensor_order:
        pairs = sensor_pair_candidates(grouped, sensor, bs, max_pairs)
        if not pairs:
            return None
        per_sensor.append(pairs)

    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)
    best = None
    combos = itertools.product(*per_sensor)
    for combo_index, combo in enumerate(combos):
        if combo_index >= max_combos:
            break
        image = np.array(
            [[item["image"][0], y_sign * item["image"][1]] for item in combo],
            dtype=np.float32,
        ).reshape((-1, 1, 2))
        try:
            success, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                object_points,
                image,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error:
            success = False

        chosen = choose_pnp_solution(rvecs, tvecs, errors, object_points, image, previous_T) if success else None
        if chosen is None:
            try:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
            except cv2.error:
                continue
            if not success:
                continue
            rvec = np.asarray(rvec, dtype=float).reshape(3)
            tvec = np.asarray(tvec, dtype=float).reshape(3)
            if tvec[2] <= 0.0:
                continue
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
            residual = image.reshape((-1, 2)) - projected.reshape((-1, 2))
            rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        else:
            rvec, tvec, rmse = chosen

        spread = sum(float(item["spread_deg"]) for item in combo)
        score = rmse + 0.002 * spread
        if best is None or score < best["score"]:
            best = {
                "score": float(score),
                "rvec": rvec,
                "tvec": tvec,
                "reproj_rmse": float(rmse),
                "families": [item["families"] for item in combo],
                "az_el_rad": [[float(item["az_rad"]), float(item["el_rad"])] for item in combo],
                "spread_sum_deg": float(spread),
            }
    return best


def main():
    parser = argparse.ArgumentParser(description="Fit BS10 relative to BS4 using LH2A wave frames via per-Lighthouse PnP.")
    parser.add_argument("--wave", default="config/lh2a_wave_record.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--output", default="config/lighthouse_relative_pnp_from_lh2a_wave.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--max-family-spread-deg", type=float, default=0.8)
    parser.add_argument("--max-families", type=int, default=2)
    parser.add_argument("--max-pairs-per-sensor", type=int, default=4)
    parser.add_argument("--max-combos-per-bs", type=int, default=256)
    parser.add_argument("--max-reproj", type=float, default=0.04)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all usable frames.")
    parser.add_argument("--y-sign", type=float, default=1.0)
    parser.add_argument("--distance-prior", type=float, default=1.4)
    parser.add_argument("--distance-tolerance", type=float, default=0.6)
    parser.add_argument("--cluster-trans-radius", type=float, default=0.35)
    parser.add_argument("--cluster-rot-radius-deg", type=float, default=12.0)
    args = parser.parse_args()

    wave = load_json(args.wave)
    sensor_order = [int(x) for x in args.sensors.split(",")]
    bs_world, bs_other = [int(x) for x in args.basestations.split(",")]
    object_points = load_object_points(args.layout, sensor_order)

    transforms = []
    frame_outputs = []
    previous = {bs_world: None, bs_other: None}
    counters = {
        "frames_total": 0,
        "missing_or_unsolved": 0,
        "reproj_rejected": 0,
        "distance_rejected": 0,
        "used": 0,
    }

    frames = wave.get("frames", [])
    if args.max_frames and len(frames) > args.max_frames:
        idx = np.linspace(0, len(frames) - 1, args.max_frames).round().astype(int)
        frames = [frames[int(i)] for i in idx]

    print("=" * 88)
    print("LH2A wave PnP relative fit")
    print(f"Wave: {args.wave}")
    print(f"Frames considered: {len(frames)}")
    print(f"Family spread <= {args.max_family_spread_deg:.2f} deg | max reproj <= {args.max_reproj:.4f}")
    print("=" * 88)

    for frame in frames:
        counters["frames_total"] += 1
        grouped = group_measurements(frame, args.max_family_spread_deg, args.max_families)
        pose_world = solve_pnp_for_bs(
            grouped, bs_world, sensor_order, object_points, previous[bs_world],
            args.max_pairs_per_sensor, args.y_sign, args.max_combos_per_bs,
        )
        pose_other = solve_pnp_for_bs(
            grouped, bs_other, sensor_order, object_points, previous[bs_other],
            args.max_pairs_per_sensor, args.y_sign, args.max_combos_per_bs,
        )
        if pose_world is None or pose_other is None:
            counters["missing_or_unsolved"] += 1
            continue
        if pose_world["reproj_rmse"] > args.max_reproj or pose_other["reproj_rmse"] > args.max_reproj:
            counters["reproj_rejected"] += 1
            continue

        T_world_obj = rt_to_matrix(pose_world["rvec"], pose_world["tvec"])
        T_other_obj = rt_to_matrix(pose_other["rvec"], pose_other["tvec"])
        T_world_other = T_world_obj @ np.linalg.inv(T_other_obj)
        distance = float(np.linalg.norm(T_world_other[:3, 3]))
        if args.distance_prior > 0.0 and abs(distance - args.distance_prior) > args.distance_tolerance:
            counters["distance_rejected"] += 1
            continue

        previous[bs_world] = T_world_obj
        previous[bs_other] = T_other_obj
        transforms.append(T_world_other)
        counters["used"] += 1
        frame_outputs.append({
            "frame_index": int(frame.get("frame_index", counters["frames_total"] - 1)),
            "pc_time_s": float(frame.get("pc_time_s", 0.0)),
            "bs4_reproj_rmse": float(pose_world["reproj_rmse"]),
            "bs10_reproj_rmse": float(pose_other["reproj_rmse"]),
            "relative_distance_m": distance,
            "transform_world_from_other": T_world_other.tolist(),
        })
        print(
            f"\rused={counters['used']} / considered={counters['frames_total']} "
            f"last reproj=({pose_world['reproj_rmse']:.4f},{pose_other['reproj_rmse']:.4f}) "
            f"dist={distance:.3f}m",
            end="",
            flush=True,
        )

    print()
    if len(transforms) < 5:
        save_json(args.output, {
            "description": "LH2A wave PnP relative fit failed: not enough usable frames.",
            "input_wave": args.wave,
            "counters": counters,
            "frames": frame_outputs,
        })
        raise SystemExit(f"Need at least 5 usable frames, got {len(transforms)}. Saved diagnostics to {args.output}.")

    clustered = select_best_cluster(transforms, args.cluster_trans_radius, args.cluster_rot_radius_deg)
    if len(clustered) >= 5:
        kept = clustered
    else:
        rough = average_transforms(transforms)
        errors = [transform_error(T, rough) for T in transforms]
        trans_errors = np.array([item[0] for item in errors], dtype=float)
        keep = trans_errors <= np.percentile(trans_errors, 80)
        kept = [T for T, k in zip(transforms, keep) if k]
    fitted = average_transforms(kept)
    errors2 = [transform_error(T, fitted) for T in kept]
    trans2 = np.array([item[0] for item in errors2], dtype=float)
    rot2 = np.array([item[1] for item in errors2], dtype=float)

    output = {
        "description": "Relative Lighthouse calibration from LH2A wave PnP. BS4 is the world/reference frame.",
        "input_wave": args.wave,
        "world_bs": bs_world,
        "other_bs": bs_other,
        "transform_world_from_other": fitted.tolist(),
        "counters": counters,
        "num_transforms_total": int(len(transforms)),
        "num_transforms_used": int(len(kept)),
        "cluster": {
            "translation_radius_m": float(args.cluster_trans_radius),
            "rotation_radius_deg": float(args.cluster_rot_radius_deg),
            "clustered_count": int(len(clustered)),
        },
        "quality": {
            "translation_median_error_m": float(np.median(trans2)),
            "translation_p95_error_m": float(np.percentile(trans2, 95)),
            "rotation_median_error_deg": float(np.median(rot2)),
            "rotation_p95_error_deg": float(np.percentile(rot2, 95)),
            "distance_m": float(np.linalg.norm(fitted[:3, 3])),
        },
        "frames": frame_outputs,
    }
    save_json(args.output, output)

    t = fitted[:3, 3]
    q = output["quality"]
    print("=" * 88)
    print(f"Frames used: {len(kept)} / {len(transforms)} accepted / {counters['frames_total']} considered")
    print(f"BS{bs_other} in BS{bs_world} frame: x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m")
    print(f"Distance: {q['distance_m']:.3f} m")
    print(
        f"Quality: trans median={q['translation_median_error_m']:.3f} m "
        f"p95={q['translation_p95_error_m']:.3f} m | "
        f"rot median={q['rotation_median_error_deg']:.2f} deg "
        f"p95={q['rotation_p95_error_deg']:.2f} deg"
    )
    print(f"Saved: {args.output}")
    print("=" * 88)


if __name__ == "__main__":
    main()
