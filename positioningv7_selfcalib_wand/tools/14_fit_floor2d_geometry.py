import argparse
import math

import numpy as np
from scipy.optimize import least_squares

from wand_common import load_json, save_json


def wrap_angle(rad):
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def feature_from_frame(frame, sensor, bs):
    item = frame.get("observations", {}).get(str(bs), {}).get(str(sensor))
    if item is None:
        return None
    return [float(item["lfsr0"]), float(item["lfsr1"])]


def normalize_features(features):
    arr = np.asarray(features, dtype=float)
    mean = arr.mean(axis=0)
    scale = arr.std(axis=0)
    scale[scale < 1.0] = 1.0
    return arr, mean, scale


def bearing_from_feature(norm_feature, coeff):
    return coeff[0] + coeff[1] * norm_feature[0] + coeff[2] * norm_feature[1]


def residuals(params, bs_data):
    out = []
    offset = 0
    for data in bs_data:
        bx, by, yaw = params[offset:offset + 3]
        coeff = params[offset + 3:offset + 6]
        offset += 6

        for feat, point in zip(data["features_norm"], data["points"]):
            measured = yaw + bearing_from_feature(feat, coeff)
            expected = math.atan2(point[1] - by, point[0] - bx)
            out.append(wrap_angle(measured - expected))
    return np.asarray(out, dtype=float)


def initial_params(bs_data, station_guesses):
    params = []
    for data, guess in zip(bs_data, station_guesses):
        points = data["points"]
        bx, by = guess
        bearings = np.unwrap([
            math.atan2(point[1] - by, point[0] - bx)
            for point in points
        ])
        A = np.column_stack([np.ones(len(points)), data["features_norm"]])
        coeff, *_ = np.linalg.lstsq(A, bearings, rcond=None)
        params.extend([bx, by, 0.0, coeff[0], coeff[1], coeff[2]])
    return np.asarray(params, dtype=float)


def parameter_bounds(count, station_limit):
    lo = []
    hi = []
    for _ in range(count):
        lo.extend([-station_limit, -station_limit, -4.0 * math.pi, -4.0 * math.pi, -4.0 * math.pi, -4.0 * math.pi])
        hi.extend([station_limit, station_limit, +4.0 * math.pi, +4.0 * math.pi, +4.0 * math.pi, +4.0 * math.pi])
    return np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)


def station_guess_sets(count, station_limit):
    positions = [
        (-2.0, +2.0),
        (+2.0, +2.0),
        (-2.0, -2.0),
        (+2.0, -2.0),
        (0.0, +3.0),
        (0.0, -3.0),
        (-3.0, 0.0),
        (+3.0, 0.0),
    ]
    clipped = [
        (
            max(-station_limit * 0.9, min(station_limit * 0.9, x)),
            max(-station_limit * 0.9, min(station_limit * 0.9, y)),
        )
        for x, y in positions
    ]
    if count == 1:
        return [[pos] for pos in clipped]
    if count == 2:
        return [[a, b] for a in clipped for b in clipped if a != b]
    return [clipped[:count]]


def intersect_rays(stations):
    rows = []
    rhs = []
    for station in stations:
        x, y, theta = station
        # Point lies on ray, so dot(point - origin, normal) = 0.
        normal = np.array([-math.sin(theta), math.cos(theta)], dtype=float)
        rows.append(normal)
        rhs.append(float(normal @ np.array([x, y], dtype=float)))
    A = np.vstack(rows)
    b = np.asarray(rhs)
    point, *_ = np.linalg.lstsq(A, b, rcond=None)
    return point


def predict_xy(calibration, frame, sensor):
    stations = []
    for bs_item in calibration["basestations"]:
        bs = int(bs_item["id"])
        feat = feature_from_frame(frame, sensor, bs)
        if feat is None:
            return None
        norm = (np.asarray(feat, dtype=float) - np.asarray(bs_item["feature_mean"])) / np.asarray(bs_item["feature_scale"])
        bearing = bs_item["yaw_rad"] + bearing_from_feature(norm, np.asarray(bs_item["bearing_coefficients"]))
        stations.append([bs_item["x_m"], bs_item["y_m"], bearing])
    return intersect_rays(stations)


def main():
    parser = argparse.ArgumentParser(description="Fit an effective 2D Lighthouse geometry on floor points.")
    parser.add_argument("--input", default="data/angle3d_calibration_floor9.json")
    parser.add_argument("--output", default="config/floor2d_geometry.json")
    parser.add_argument("--sensor", type=int, default=2)
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--max-nfev", type=int, default=5000)
    parser.add_argument("--station-limit", type=float, default=8.0)
    args = parser.parse_args()

    record = load_json(args.input)
    bs_ids = [int(x) for x in args.basestations.split(",")]

    bs_data = []
    for bs in bs_ids:
        features = []
        points = []
        names = []
        for frame in record.get("frames", []):
            feat = feature_from_frame(frame, args.sensor, bs)
            if feat is None:
                continue
            pose = frame["pose"]
            features.append(feat)
            points.append([float(pose["x_m"]), float(pose["y_m"])])
            names.append(pose.get("name", str(len(names))))

        if len(features) < 4:
            raise ValueError(f"Not enough points for BS{bs}: {len(features)}")
        raw, mean, scale = normalize_features(features)
        bs_data.append({
            "id": bs,
            "features_norm": (raw - mean) / scale,
            "feature_mean": mean,
            "feature_scale": scale,
            "points": np.asarray(points, dtype=float),
            "names": names,
        })

    bounds = parameter_bounds(len(bs_data), args.station_limit)
    best_result = None
    best_cost = None
    for guesses in station_guess_sets(len(bs_data), args.station_limit):
        x0 = initial_params(bs_data, guesses)
        result = least_squares(
            residuals,
            x0,
            args=(bs_data,),
            bounds=bounds,
            loss="soft_l1",
            f_scale=math.radians(2.0),
            max_nfev=args.max_nfev,
        )
        cost = float(np.mean(residuals(result.x, bs_data) ** 2))
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_result = result

    result = best_result

    calibration = {
        "description": "Effective 2D Lighthouse geometry fitted on floor calibration points.",
        "model": "floor2d_geometry_bearing",
        "input": args.input,
        "sensor": args.sensor,
        "basestations": [],
    }

    offset = 0
    for data in bs_data:
        bx, by, yaw = result.x[offset:offset + 3]
        coeff = result.x[offset + 3:offset + 6]
        offset += 6
        calibration["basestations"].append({
            "id": data["id"],
            "x_m": float(bx),
            "y_m": float(by),
            "yaw_rad": float(yaw),
            "bearing_coefficients": coeff.tolist(),
            "feature_order": ["lfsr0", "lfsr1"],
            "feature_mean": data["feature_mean"].tolist(),
            "feature_scale": data["feature_scale"].tolist(),
        })

    errors = []
    point_rows = []
    for frame in record.get("frames", []):
        pred = predict_xy(calibration, frame, args.sensor)
        if pred is None:
            continue
        pose = frame["pose"]
        ref = np.asarray([float(pose["x_m"]), float(pose["y_m"])], dtype=float)
        err = float(np.linalg.norm(pred - ref))
        errors.append(err)
        point_rows.append({
            "name": pose.get("name", str(len(point_rows))),
            "ref": ref.tolist(),
            "est": pred.tolist(),
            "err_m": err,
        })

    calibration["train_error_m"] = {
        "median": float(np.median(errors)),
        "mean": float(np.mean(errors)),
        "max": float(np.max(errors)),
    }
    calibration["points"] = point_rows
    save_json(args.output, calibration)

    print("=" * 70)
    print("Fit effective floor 2D Lighthouse geometry")
    print(f"Input: {args.input}")
    print(f"Output: {args.output}")
    print(f"sensor={args.sensor} points={len(errors)} basestations={bs_ids}")
    print(
        f"train_err median={np.median(errors):.3f} m "
        f"mean={np.mean(errors):.3f} m max={np.max(errors):.3f} m"
    )
    for bs in calibration["basestations"]:
        print(f"  BS{bs['id']}: x={bs['x_m']:+.3f} y={bs['y_m']:+.3f} yaw={math.degrees(bs['yaw_rad']):+.1f} deg")
    for item in point_rows:
        print(
            f"  {item['name']}: err={item['err_m']:.3f} m "
            f"est=({item['est'][0]:+.3f},{item['est'][1]:+.3f}) "
            f"ref=({item['ref'][0]:+.3f},{item['ref'][1]:+.3f})"
        )
    print("=" * 70)


if __name__ == "__main__":
    main()
