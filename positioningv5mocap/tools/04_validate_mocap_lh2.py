import argparse
import math
from collections import defaultdict

import numpy as np
from scipy.spatial.transform import Rotation

from mocap_lh2 import (
    MocapInterpolator,
    angle_wrap,
    default_lfsr_to_alpha,
    lh2_sweep_angle_from_point,
    load_json,
    load_lh2_csv,
    load_mocap_csv,
    load_sensor_layout,
    sensor_world_position,
)


def calibrated_alpha(lfsr, sweep, conversion):
    raw = default_lfsr_to_alpha(lfsr)
    if conversion.get("type") != "raw_alpha_scale_offset":
        return raw

    if int(sweep) == 0:
        return float(conversion["sweep_0_scale"]) * raw + float(conversion["sweep_0_offset_rad"])
    return float(conversion["sweep_1_scale"]) * raw + float(conversion["sweep_1_offset_rad"])


def main():
    parser = argparse.ArgumentParser(description="Validate mocap LH2 calibration residuals.")
    parser.add_argument("--calibration", default="config/mocap_lh2_calibration.json")
    parser.add_argument("--lh2", help="Override LH2 CSV path.")
    parser.add_argument("--mocap", help="Override mocap CSV path.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--max-rows-per-bs", type=int, default=8000)
    args = parser.parse_args()

    calibration = load_json(args.calibration)
    lh2_path = args.lh2 or calibration["input_lh2"]
    mocap_path = args.mocap or calibration["input_mocap"]
    time_offset = float(calibration.get("time_offset_s", 0.0))

    lh2_rows = load_lh2_csv(lh2_path)
    mocap_rows = load_mocap_csv(mocap_path)
    layout = load_sensor_layout(args.layout)
    mocap = MocapInterpolator(mocap_rows)

    by_bs = {int(item["basestation"]): item for item in calibration["basestations"]}
    residuals_by_bs = defaultdict(list)
    counts_by_bs = defaultdict(int)

    for row in lh2_rows:
        bs = int(row["basestation"])
        if bs not in by_bs:
            continue
        if counts_by_bs[bs] >= args.max_rows_per_bs:
            continue
        if int(row["sensor"]) not in layout or int(row["sweep"]) not in (0, 1):
            continue

        t = float(row["pc_time_s"]) + time_offset
        if not mocap.contains(t):
            continue

        item = by_bs[bs]
        pos, rot = mocap.pose_at(t)
        p_world = sensor_world_position(pos, rot, layout[int(row["sensor"])])

        R = np.array(item["world_to_lighthouse"]["rotation_matrix"], dtype=float)
        trans = np.array(item["world_to_lighthouse"]["translation_m"], dtype=float)
        p_lh = R @ (p_world - trans)

        sweep = int(row["sweep"])
        tilt = float(item["sweep_tilts"][f"sweep_{sweep}_rad"])
        pred = lh2_sweep_angle_from_point(p_lh, tilt)
        meas = calibrated_alpha(float(row["lfsr_location"]), sweep, item["angle_conversion"])
        residuals_by_bs[bs].append(angle_wrap(pred - meas))
        counts_by_bs[bs] += 1

    print("=" * 70)
    print("Validate mocap LH2 calibration")
    print("=" * 70)

    for bs in sorted(by_bs):
        residuals = np.array(residuals_by_bs.get(bs, []), dtype=float)
        if len(residuals) == 0:
            print(f"BS{bs}: no validation rows")
            continue

        abs_deg = np.degrees(np.abs(residuals))
        rmse_deg = math.degrees(math.sqrt(float(np.mean(residuals ** 2))))
        print(
            f"BS{bs}: n={len(residuals)} | "
            f"rmse={rmse_deg:.3f} deg | "
            f"median={float(np.median(abs_deg)):.3f} deg | "
            f"p95={float(np.percentile(abs_deg, 95)):.3f} deg | "
            f"max={float(np.max(abs_deg)):.3f} deg"
        )


if __name__ == "__main__":
    main()
