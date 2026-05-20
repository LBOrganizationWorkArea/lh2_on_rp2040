#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
from statistics import median

import numpy as np
from scipy.optimize import least_squares


TICKS_PER_REV = 833333


def lfsr_to_rad(lfsr):
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
            float(s["y_m"])
        ], dtype=float)
    return sensors


def measurement_angle(m):
    if "angle_rad" in m:
        return float(m["angle_rad"])
    if "angle_deg" in m:
        return math.radians(float(m["angle_deg"]))
    return lfsr_to_rad(float(m["median_lfsr_location"]))


def load_rows(poses_path, layout, basestation, sweep):
    with open(poses_path, "r") as f:
        data = json.load(f)

    rows = []

    for pose in data["poses"]:
        px = float(pose["x_m"])
        py = float(pose["y_m"])

        values = {}

        for m in pose["measurements"]:
            if int(m["basestation"]) != basestation:
                continue
            if int(m["sweep"]) != sweep:
                continue

            sensor = int(m["sensor"])
            values.setdefault(sensor, []).append(measurement_angle(m))

        for sensor, vals in values.items():
            if sensor not in layout:
                continue

            local = layout[sensor]
            wx = px + local[0]
            wy = py + local[1]

            rows.append({
                "x": float(wx),
                "y": float(wy),
                "angle": float(median(vals)),
                "sensor": sensor,
                "pose": pose["name"],
            })

    return rows


def residuals(params, rows):
    """
    params = [bs_x, bs_y, yaw, scale]

    We fit:
      world_bearing = yaw + scale * measured_angle

    with:
      world_bearing = atan2(sensor_y - bs_y, sensor_x - bs_x)
    """
    bx, by, yaw, scale = params
    out = []

    for r in rows:
        predicted_bearing = yaw + scale * r["angle"]
        geometric_bearing = math.atan2(r["y"] - by, r["x"] - bx)
        out.append(angle_diff(predicted_bearing, geometric_bearing))

    return np.array(out, dtype=float)


def fit_bs_sweep(rows):
    guesses = [
        (-3, -3), (-3, 0), (-3, 3),
        (0, -3),          (0, 3),
        (3, -3),  (3, 0), (3, 3),
        (-1.5, -1.5), (-1.5, 1.5),
        (1.5, -1.5), (1.5, 1.5),
    ]

    best = None

    for gx, gy in guesses:
        for yaw0 in [0.0, math.pi / 2, -math.pi / 2, math.pi]:
            for scale0 in [1.0, -1.0]:
                x0 = np.array([gx, gy, yaw0, scale0], dtype=float)

                res = least_squares(
                    residuals,
                    x0,
                    args=(rows,),
                    loss="soft_l1",
                    f_scale=math.radians(2.0),
                    max_nfev=500,
                )

                err = residuals(res.x, rows)
                rmse_rad = float(np.sqrt(np.mean(err ** 2)))
                rmse_deg = float(math.degrees(rmse_rad))

                candidate = {
                    "params": res.x,
                    "rmse_deg": rmse_deg,
                    "rmse_rad": rmse_rad,
                    "cost": float(res.cost),
                    "success": bool(res.success),
                }

                if best is None or candidate["rmse_rad"] < best["rmse_rad"]:
                    best = candidate

    return best


def main():
    parser = argparse.ArgumentParser(description="Fast 2D Lighthouse geometry calibration.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--poses", default="config/calibration_poses_2d.json")
    parser.add_argument("--output", default="config/lighthouse_geometry_fast.json")
    parser.add_argument("--basestations", default="4,10")
    args = parser.parse_args()

    layout = load_layout(args.layout)
    basestations = [int(x) for x in args.basestations.split(",")]

    output = {
        "description": "Fast 2D Lighthouse bearing geometry. Uses one sweep per Lighthouse.",
        "layout": args.layout,
        "poses": args.poses,
        "basestations": []
    }

    print("=" * 70)
    print("Fast 2D Lighthouse geometry calibration")
    print(f"Layout: {args.layout}")
    print(f"Poses:  {args.poses}")
    print(f"BS:     {basestations}")
    print("=" * 70)

    for bs in basestations:
        best_for_bs = None

        for sweep in [0, 1]:
            rows = load_rows(args.poses, layout, bs, sweep)

            print()
            print(f"BS {bs} sweep {sweep}: rows={len(rows)}")

            if len(rows) < 8:
                print("  Not enough rows, skipped.")
                continue

            result = fit_bs_sweep(rows)
            bx, by, yaw, scale = result["params"]

            print(f"  RMSE={result['rmse_deg']:.4f} deg")
            print(f"  pos=({bx:+.3f}, {by:+.3f}) m | yaw={math.degrees(yaw):+.2f} deg | scale={scale:+.4f}")

            candidate = {
                "basestation": bs,
                "sweep": sweep,
                "position_m": [float(bx), float(by)],
                "yaw_rad": float(yaw),
                "scale": float(scale),
                "rmse_deg": float(result["rmse_deg"]),
                "rmse_rad": float(result["rmse_rad"]),
                "rows": len(rows),
            }

            if best_for_bs is None or candidate["rmse_rad"] < best_for_bs["rmse_rad"]:
                best_for_bs = candidate

        if best_for_bs is None:
            raise RuntimeError(f"Could not fit basestation {bs}")

        print()
        print(f"SELECTED BS {bs}: sweep={best_for_bs['sweep']} | RMSE={best_for_bs['rmse_deg']:.4f} deg")

        output["basestations"].append(best_for_bs)

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