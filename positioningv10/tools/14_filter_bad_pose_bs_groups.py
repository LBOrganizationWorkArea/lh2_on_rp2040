#!/usr/bin/env python3

import argparse
import importlib.util
import json
from collections import defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
diag_path = HERE / "13_inspect_pose_angle_families.py"
spec = importlib.util.spec_from_file_location("inspect_pose_angle_families", diag_path)
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


def group_metrics(pose, measurements, angle_key):
    grouped = defaultdict(dict)
    for m in measurements:
        if angle_key not in m:
            continue
        key = (int(m["sensor"]), int(m["basestation"]))
        grouped[key][int(m["sweep"])] = float(m[angle_key])

    by_bs = defaultdict(list)
    for (sensor, bs), sweeps in grouped.items():
        if 0 not in sweeps or 1 not in sweeps:
            continue
        a0 = sweeps[0]
        a1 = sweeps[1]
        by_bs[bs].append({
            "sensor": sensor,
            "mid": diag.midpoint(a0, a1),
            "sep": diag.angle_diff(a0, a1),
        })

    out = {}
    for bs, pairs in by_bs.items():
        mids = [p["mid"] for p in pairs]
        seps = [p["sep"] for p in pairs]
        out[int(bs)] = {
            "paired_sensors": len(pairs),
            "sensor_spread_deg": diag.circ_spread_deg(mids),
            "sep_spread_deg": diag.circ_spread_deg(seps),
            "sep_median_deg": diag.math.degrees(diag.circ_mean(seps)),
        }
    return out


def main():
    parser = argparse.ArgumentParser(description="Filter bad pose/base-station groups from wand calibration poses.")
    parser.add_argument("--input", default="config/wand_calibration_poses_3d.json")
    parser.add_argument("--output", default="config/wand_calibration_poses_3d_filtered.json")
    parser.add_argument("--angle-key", default="raw_angle_rad", choices=["raw_angle_rad", "calibrated_angle_rad"])
    parser.add_argument("--max-sensor-spread-deg", type=float, default=15.0)
    parser.add_argument("--max-sep-spread-deg", type=float, default=5.0)
    parser.add_argument("--min-paired-sensors", type=int, default=3)
    args = parser.parse_args()

    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)

    kept_measurements = 0
    dropped_measurements = 0
    kept_groups = 0
    dropped_groups = 0
    report = []
    new_poses = []

    for pose in data.get("poses", []):
        metrics = group_metrics(pose, pose.get("measurements", []), args.angle_key)
        keep_bs = set()
        for bs, item in sorted(metrics.items()):
            ok = (
                item["paired_sensors"] >= args.min_paired_sensors
                and item["sensor_spread_deg"] <= args.max_sensor_spread_deg
                and item["sep_spread_deg"] <= args.max_sep_spread_deg
            )
            report.append({
                "pose": pose["name"],
                "basestation": bs,
                "kept": bool(ok),
                **item,
            })
            if ok:
                keep_bs.add(bs)
                kept_groups += 1
            else:
                dropped_groups += 1

        filtered = []
        for m in pose.get("measurements", []):
            if int(m["basestation"]) in keep_bs:
                filtered.append(m)
                kept_measurements += 1
            else:
                dropped_measurements += 1

        if filtered:
            new_pose = dict(pose)
            new_pose["measurements"] = filtered
            new_poses.append(new_pose)

    out = dict(data)
    out["description"] = data.get("description", "") + " Filtered by pose/base-station consistency."
    out["source_file"] = args.input
    out["filter"] = {
        "angle_key": args.angle_key,
        "max_sensor_spread_deg": args.max_sensor_spread_deg,
        "max_sep_spread_deg": args.max_sep_spread_deg,
        "min_paired_sensors": args.min_paired_sensors,
        "kept_groups": kept_groups,
        "dropped_groups": dropped_groups,
        "kept_measurements": kept_measurements,
        "dropped_measurements": dropped_measurements,
        "report": report,
    }
    out["poses"] = new_poses

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")

    print("=" * 70)
    print("Filtered wand calibration poses")
    print(f"Input:  {args.input}")
    print(f"Output: {args.output}")
    print(f"Kept groups:    {kept_groups}")
    print(f"Dropped groups: {dropped_groups}")
    print(f"Kept measurements:    {kept_measurements}")
    print(f"Dropped measurements: {dropped_measurements}")
    print("=" * 70)
    print("Dropped pose/base-station groups:")
    for item in report:
        if not item["kept"]:
            print(
                f"  {item['pose']} BS{item['basestation']}: "
                f"sensors={item['paired_sensors']} "
                f"sensor_spread={item['sensor_spread_deg']:.2f} deg "
                f"sep_spread={item['sep_spread_deg']:.2f} deg"
            )


if __name__ == "__main__":
    main()
