#!/usr/bin/env python3

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path
from statistics import median

import serial


def load_live_angles_module():
    path = Path(__file__).resolve().parent / "02_live_angles.py"
    spec = importlib.util.spec_from_file_location("live_angles_v10", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def angle_diff_deg(a, b):
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def circular_median_deg(values):
    values = [float(value) for value in values]
    if not values:
        return 0.0
    return min(values, key=lambda candidate: sum(abs(angle_diff_deg(value, candidate)) for value in values))


def circular_spread_deg(values, center):
    if not values:
        return 0.0
    return max(abs(angle_diff_deg(value, center)) for value in values)


def quantile(values, q):
    if not values:
        return float("nan")
    values = sorted(values)
    return values[int((len(values) - 1) * float(q))]


def cluster_angles(samples, cluster_deg):
    clusters = []
    for sample in samples:
        angle = float(sample["angle_deg"])
        best_cluster = None
        best_distance = None
        for cluster in clusters:
            distance = abs(angle_diff_deg(angle, cluster["center_deg"]))
            if distance <= cluster_deg and (best_distance is None or distance < best_distance):
                best_cluster = cluster
                best_distance = distance

        if best_cluster is None:
            clusters.append({"center_deg": angle, "samples": [sample]})
        else:
            best_cluster["samples"].append(sample)
            best_cluster["center_deg"] = circular_median_deg(
                [item["angle_deg"] for item in best_cluster["samples"]]
            )

    for cluster in clusters:
        angles = [item["angle_deg"] for item in cluster["samples"]]
        lfsrs = [item["lfsr_location"] for item in cluster["samples"]]
        center = circular_median_deg(angles)
        cluster.update({
            "count": len(cluster["samples"]),
            "center_deg": center,
            "spread_deg": circular_spread_deg(angles, center),
            "q25_deg": quantile(angles, 0.25),
            "q75_deg": quantile(angles, 0.75),
            "median_lfsr": float(median(lfsrs)) if lfsrs else None,
            "polynomials": sorted({int(item["polynomial"]) for item in cluster["samples"]}),
        })

    clusters.sort(key=lambda item: item["count"], reverse=True)
    return clusters


def compact_clusters(clusters, max_clusters):
    result = []
    for index, cluster in enumerate(clusters[:max_clusters], start=1):
        result.append({
            "rank": index,
            "count": int(cluster["count"]),
            "fraction": float(cluster.get("fraction", 0.0)),
            "center_deg": float(cluster["center_deg"]),
            "spread_deg": float(cluster["spread_deg"]),
            "q25_deg": float(cluster["q25_deg"]),
            "q75_deg": float(cluster["q75_deg"]),
            "median_lfsr": cluster["median_lfsr"],
            "polynomials": cluster["polynomials"],
        })
    return result


def main():
    parser = argparse.ArgumentParser(description="Cluster direct LH2A angle families by sensor/base-station/sweep.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--cluster-deg", type=float, default=8.0, help="Maximum angular distance used to merge samples into one family.")
    parser.add_argument("--max-clusters", type=int, default=4)
    parser.add_argument("--min-samples", type=int, default=4)
    parser.add_argument("--output", help="Optional JSON output path for the family report.")
    args = parser.parse_args()

    live_angles = load_live_angles_module()
    basestations = {int(item) for item in args.basestations.split(",")}
    samples_by_key = {}
    counts = {"lh2a": 0, "lh2p": 0, "lh2r": 0, "other": 0}

    print("=" * 88)
    print("LH2A family diagnostic")
    print(f"Port: {args.port}")
    print(f"Duration: {args.duration:.1f}s")
    print(f"Cluster: {args.cluster_deg:.1f} deg")
    print("=" * 88)

    start = time.time()
    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while time.time() - start < args.duration:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue
            if raw.startswith("LH2A,"):
                counts["lh2a"] += 1
            elif raw.startswith("LH2P"):
                counts["lh2p"] += 1
            elif raw.startswith("LH2R,"):
                counts["lh2r"] += 1
            else:
                counts["other"] += 1
                continue

            data = live_angles.parse_lh2_line(raw)
            if data is None or "raw_angle_rad" not in data:
                continue
            if int(data["basestation"]) not in basestations:
                continue

            key = (int(data["sensor"]), int(data["basestation"]), int(data["sweep"]))
            samples_by_key.setdefault(key, []).append({
                "time_us": data.get("time_us"),
                "angle_deg": math.degrees(float(data["raw_angle_rad"])),
                "lfsr_location": int(data.get("lfsr_location", 0)),
                "polynomial": int(data.get("polynomial", -1)),
            })

    expected_keys = [
        (sensor, bs, sweep)
        for sensor in range(4)
        for bs in sorted(basestations)
        for sweep in range(2)
    ]

    report = {
        "description": "Direct LH2A angle family clustering.",
        "duration_s": float(args.duration),
        "cluster_deg": float(args.cluster_deg),
        "counts": counts,
        "channels": [],
    }

    print()
    print(
        "raw counts | "
        f"LH2A={counts['lh2a']} | LH2P={counts['lh2p']} | "
        f"LH2R={counts['lh2r']} | other={counts['other']}"
    )
    print()

    for key in expected_keys:
        samples = samples_by_key.get(key, [])
        sensor, bs, sweep = key
        if len(samples) < args.min_samples:
            print(f"sensor={sensor} | bs={bs} | sweep={sweep} | samples={len(samples)} | MISSING/LOW")
            report["channels"].append({
                "sensor": sensor,
                "basestation": bs,
                "sweep": sweep,
                "sample_count": len(samples),
                "families": [],
            })
            continue

        clusters = cluster_angles(samples, args.cluster_deg)
        for cluster in clusters:
            cluster["fraction"] = cluster["count"] / len(samples)

        compact = compact_clusters(clusters, args.max_clusters)
        report["channels"].append({
            "sensor": sensor,
            "basestation": bs,
            "sweep": sweep,
            "sample_count": len(samples),
            "families": compact,
        })

        family_text = " ; ".join(
            f"#{item['rank']} n={item['count']} ({item['fraction']*100:.0f}%) "
            f"angle={item['center_deg']:+.3f}deg spread={item['spread_deg']:.2f}deg "
            f"lfsr={item['median_lfsr']:.0f} poly={','.join(str(p) for p in item['polynomials'])}"
            for item in compact
        )
        print(
            f"sensor={sensor} | bs={bs} | sweep={sweep} | "
            f"samples={len(samples)} | families={len(clusters)} | {family_text}"
        )

    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with open(output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print()
        print(f"Saved: {output}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
