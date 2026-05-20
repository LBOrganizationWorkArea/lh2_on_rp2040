import argparse
import math

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from wand_common import average_transforms, load_json, rt_to_matrix, save_json


def load_layout(path, sensor_order):
    data = load_json(path)
    by_sensor = {
        int(item["sensor"]): np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ], dtype=float)
        for item in data["sensors"]
    }
    return np.array([by_sensor[sensor] for sensor in sensor_order], dtype=float)


def frame_measurements(frame, basestations, sensor_order):
    out = {}
    for bs in basestations:
        bs_key = str(bs)
        if bs_key not in frame["angles"]:
            return None
        out[bs] = []
        for sensor in sensor_order:
            s_key = str(sensor)
            if s_key not in frame["angles"][bs_key]:
                return None
            item = frame["angles"][bs_key][s_key]
            out[bs].append([
                math.tan(math.radians(float(item["az_deg"]))),
                math.tan(math.radians(float(item["el_deg"]))),
            ])
        out[bs] = np.array(out[bs], dtype=float)
    return out


def pnp_initial_pose(object_points, image_points):
    obj = np.asarray(object_points, dtype=np.float32).reshape((-1, 1, 3))
    img = np.asarray(image_points, dtype=np.float32).reshape((-1, 1, 2))
    K = np.eye(3, dtype=np.float32)
    dist = np.zeros((4, 1), dtype=np.float32)

    try:
        success, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if success:
            best = None
            for rvec, tvec, err in zip(rvecs, tvecs, errors):
                rvec = np.asarray(rvec, dtype=float).reshape(3)
                tvec = np.asarray(tvec, dtype=float).reshape(3)
                if tvec[2] <= 0:
                    continue
                score = float(np.asarray(err).reshape(-1)[0])
                if best is None or score < best[0]:
                    best = (score, rvec, tvec)
            if best is not None:
                return best[1], best[2]
    except cv2.error:
        pass

    success, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not success:
        return None
    rvec = rvec.reshape(3)
    tvec = tvec.reshape(3)
    if tvec[2] <= 0:
        return None
    return rvec, tvec


def select_frames(record, basestations, sensor_order, max_frames):
    valid = []
    for frame in record["frames"]:
        meas = frame_measurements(frame, basestations, sensor_order)
        if meas is not None:
            valid.append((frame, meas))

    if max_frames is not None and len(valid) > max_frames:
        idx = np.linspace(0, len(valid) - 1, max_frames).round().astype(int)
        valid = [valid[i] for i in idx]

    return valid


def pack_params(T_4_from_10, frame_poses):
    r10 = Rotation.from_matrix(T_4_from_10[:3, :3]).as_rotvec()
    t10 = T_4_from_10[:3, 3]
    values = [*r10, *t10]
    for rvec, tvec in frame_poses:
        values.extend(rvec)
        values.extend(tvec)
    return np.array(values, dtype=float)


def unpack_params(params, num_frames):
    r10 = params[0:3]
    t10 = params[3:6]
    R10 = Rotation.from_rotvec(r10).as_matrix()

    frame_poses = []
    offset = 6
    for _ in range(num_frames):
        rvec = params[offset:offset + 3]
        tvec = params[offset + 3:offset + 6]
        frame_poses.append((Rotation.from_rotvec(rvec).as_matrix(), tvec, rvec))
        offset += 6

    return R10, t10, frame_poses


def project_points(points_cam):
    z = points_cam[:, 2]
    z_safe = np.where(np.abs(z) < 1e-6, 1e-6, z)
    return np.column_stack([points_cam[:, 0] / z_safe, points_cam[:, 1] / z_safe])


def residuals(params, object_points, measurements, world_bs, other_bs):
    R10, t10, frame_poses = unpack_params(params, len(measurements))
    out = []

    for (_frame, meas), (R4_obj, t4_obj, _rvec) in zip(measurements, frame_poses):
        p4 = (R4_obj @ object_points.T).T + t4_obj

        pred4 = project_points(p4)
        out.extend((pred4 - meas[world_bs]).reshape(-1))

        p10 = (R10.T @ (p4 - t10).T).T
        pred10 = project_points(p10)
        out.extend((pred10 - meas[other_bs]).reshape(-1))

        # Fixed-size soft penalty against points behind either camera.
        out.extend(np.maximum(0.02 - p4[:, 2], 0.0) * 10.0)
        out.extend(np.maximum(0.02 - p10[:, 2], 0.0) * 10.0)

    return np.array(out, dtype=float)


def initial_solution(measurements, object_points, world_bs, other_bs):
    frame_poses = []
    rel_transforms = []
    kept_measurements = []

    for _frame, meas in measurements:
        init4 = pnp_initial_pose(object_points, meas[world_bs])
        init10 = pnp_initial_pose(object_points, meas[other_bs])

        if init4 is None:
            continue

        r4, t4 = init4
        frame_poses.append((r4, t4))
        kept_measurements.append((_frame, meas))

        if init10 is not None:
            r10_obj, t10_obj = init10
            T4_obj = rt_to_matrix(r4, t4)
            T10_obj = rt_to_matrix(r10_obj, t10_obj)
            rel_transforms.append(T4_obj @ np.linalg.inv(T10_obj))

    if len(frame_poses) < 10:
        raise RuntimeError(f"Only {len(frame_poses)} / {len(measurements)} frames have a BS{world_bs} PnP initial pose.")

    if rel_transforms:
        T_4_from_10 = average_transforms(rel_transforms)
    else:
        T_4_from_10 = np.eye(4, dtype=float)
        T_4_from_10[:3, 3] = np.array([1.0, 0.0, 0.0], dtype=float)

    return T_4_from_10, frame_poses, kept_measurements


def bounds(num_frames):
    lower = [-math.pi, -math.pi, -math.pi, -5.0, -5.0, -5.0]
    upper = [+math.pi, +math.pi, +math.pi, +5.0, +5.0, +5.0]

    for _ in range(num_frames):
        lower.extend([-math.pi, -math.pi, -math.pi, -5.0, -5.0, 0.05])
        upper.extend([+math.pi, +math.pi, +math.pi, +5.0, +5.0, 10.0])

    return np.array(lower, dtype=float), np.array(upper, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Direct self-calibration from calibrated LH2 angle frames.")
    parser.add_argument("--input", default="data/wand_angles_record.json")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--output", default="config/direct_selfcalib.json")
    parser.add_argument("--world-bs", type=int, default=4)
    parser.add_argument("--other-bs", type=int, default=10)
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--max-frames", type=int, default=200)
    parser.add_argument("--max-nfev", type=int, default=300)
    args = parser.parse_args()

    record = load_json(args.input)
    sensor_order = [int(x) for x in args.sensors.split(",")]
    object_points = load_layout(args.layout, sensor_order)
    measurements = select_frames(record, [args.world_bs, args.other_bs], sensor_order, args.max_frames)

    if len(measurements) < 10:
        raise SystemExit(f"Need at least 10 valid frames, got {len(measurements)}.")

    print("=" * 70)
    print("Direct self-calibration")
    print(f"Frames used: {len(measurements)}")
    print("Initializing from per-Lighthouse PnP, then optimizing raw angle projections.")
    print("=" * 70)

    T_init, frame_init, measurements = initial_solution(measurements, object_points, args.world_bs, args.other_bs)
    print(f"Frames initialized: {len(measurements)}")
    x0 = pack_params(T_init, frame_init)
    lo, hi = bounds(len(measurements))
    x0 = np.clip(x0, lo + 1e-6, hi - 1e-6)

    before = residuals(x0, object_points, measurements, args.world_bs, args.other_bs)
    print(f"Initial residual RMSE in image space: {float(np.sqrt(np.mean(before ** 2))):.6f}")

    result = least_squares(
        residuals,
        x0,
        bounds=(lo, hi),
        args=(object_points, measurements, args.world_bs, args.other_bs),
        loss="soft_l1",
        f_scale=0.01,
        max_nfev=args.max_nfev,
        verbose=1,
    )

    after = residuals(result.x, object_points, measurements, args.world_bs, args.other_bs)
    R10, t10, frame_poses = unpack_params(result.x, len(measurements))

    T10 = np.eye(4, dtype=float)
    T10[:3, :3] = R10
    T10[:3, 3] = t10

    frame_outputs = []
    for (frame, _meas), (_R, tvec, rvec) in zip(measurements, frame_poses):
        frame_outputs.append({
            "pc_time_s": float(frame["pc_time_s"]),
            "rvec_world_from_drone": [float(x) for x in rvec],
            "tvec_world_from_drone": [float(x) for x in tvec],
        })

    output = {
        "description": "Direct self-calibration from wand angle frames. BS4 is fixed as world frame by default.",
        "world_bs": int(args.world_bs),
        "other_bs": int(args.other_bs),
        "sensor_order": sensor_order,
        "num_frames_used": int(len(measurements)),
        "rmse_image_initial": float(np.sqrt(np.mean(before ** 2))),
        "rmse_image_final": float(np.sqrt(np.mean(after ** 2))),
        "transform_world_from_other": T10.tolist(),
        "frames": frame_outputs,
    }
    save_json(args.output, output)

    print("=" * 70)
    print(f"Final residual RMSE in image space: {output['rmse_image_final']:.6f}")
    print(f"BS{args.other_bs} in BS{args.world_bs}: x={t10[0]:+.3f} y={t10[1]:+.3f} z={t10[2]:+.3f} m")
    print(f"Saved: {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
