#!/usr/bin/env python3

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path
from statistics import median

import serial


def load_module(filename, name):
    path = Path(__file__).resolve().parent / filename
    spec = importlib.util.spec_from_file_location(name, path)
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
            best_cluster["center_deg"] = circular_median_deg([item["angle_deg"] for item in best_cluster["samples"]])

    for cluster in clusters:
        angles = [item["angle_deg"] for item in cluster["samples"]]
        lfsrs = [item["lfsr_location"] for item in cluster["samples"]]
        center = circular_median_deg(angles)
        cluster.update({
            "count": len(cluster["samples"]),
            "center_deg": center,
            "raw_angle_rad": math.radians(center),
            "spread_deg": circular_spread_deg(angles, center),
            "median_lfsr_location": float(median(lfsrs)) if lfsrs else 0.0,
            "polynomials": sorted({int(item["polynomial"]) for item in cluster["samples"]}),
        })
    clusters.sort(key=lambda item: item["count"], reverse=True)
    total = sum(cluster["count"] for cluster in clusters) or 1
    for cluster in clusters:
        cluster["fraction"] = cluster["count"] / total
    return clusters


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def capture_window(ser, live_angles, duration, basestations, cluster_deg, max_families, min_family_samples):
    samples_by_key = {}
    counts = {"lh2a": 0, "lh2p": 0, "lh2r": 0, "other": 0}
    end = time.time() + duration
    while time.time() < end:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue
        if raw.startswith("LH2A,"):
            counts["lh2a"] += 1
        elif raw.startswith("LH2P"):
            counts["lh2p"] += 1
            continue
        elif raw.startswith("LH2R,"):
            counts["lh2r"] += 1
            continue
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
            "angle_deg": math.degrees(float(data["raw_angle_rad"])),
            "lfsr_location": int(data.get("lfsr_location", 0)),
            "polynomial": int(data.get("polynomial", -1)),
        })

    measurements = []
    for key, samples in sorted(samples_by_key.items()):
        sensor, bs, sweep = key
        clusters = [
            cluster
            for cluster in cluster_angles(samples, cluster_deg)
            if int(cluster["count"]) >= int(min_family_samples)
        ][:max_families]
        if not clusters:
            continue

        measurements.append({
            "sensor": sensor,
            "basestation": bs,
            "sweep": sweep,
            "raw_sample_count": int(len(samples)),
            "candidate_families": [
                {
                    "rank": rank,
                    "sample_count": int(cluster["count"]),
                    "fraction": float(cluster["fraction"]),
                    "raw_angle_rad": float(cluster["raw_angle_rad"]),
                    "angle_deg": float(cluster["center_deg"]),
                    "angle_spread_deg": float(cluster["spread_deg"]),
                    "lfsr_location": float(cluster["median_lfsr_location"]),
                    "polynomials": cluster["polynomials"],
                }
                for rank, cluster in enumerate(clusters, start=1)
            ],
        })

    return measurements, counts


def main():
    parser = argparse.ArgumentParser(description="Record v12 LH2A wand wave frames without known point positions.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--window", type=float, default=0.20)
    parser.add_argument("--period", type=float, default=0.25)
    parser.add_argument("--output", default="config/lh2a_wave_record.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--cluster-deg", type=float, default=2.0)
    parser.add_argument("--max-families", type=int, default=3)
    parser.add_argument("--min-family-samples", type=int, default=2)
    parser.add_argument("--min-channels", type=int, default=8)
    args = parser.parse_args()

    live_angles = load_module("02_live_angles.py", "live_angles_v12")
    basestations = {int(item) for item in args.basestations.split(",")}
    frames = []
    start = time.time()
    next_frame = start

    print("=" * 88)
    print("Record v12 LH2A wave")
    print(f"Output: {args.output}")
    print(f"Duration: {args.duration:.1f}s | window={args.window:.2f}s | period={args.period:.2f}s")
    print("Move through the flight volume. This records angle families only; no room points are required.")
    print("=" * 88)

    with serial.Serial(args.port, args.baudrate, timeout=0.05) as ser:
        while time.time() - start < args.duration:
            now = time.time()
            if now < next_frame:
                time.sleep(0.005)
                continue

            measurements, counts = capture_window(
                ser,
                live_angles,
                args.window,
                basestations,
                args.cluster_deg,
                args.max_families,
                args.min_family_samples,
            )
            channels = len({(m["sensor"], m["basestation"], m["sweep"]) for m in measurements})
            if channels >= args.min_channels:
                frames.append({
                    "frame_index": len(frames),
                    "pc_time_s": float(now),
                    "measurements": measurements,
                    "raw_counts": counts,
                    "channels": channels,
                })

            print(
                f"\rframes={len(frames)} channels={channels}/16 "
                f"LH2A={counts['lh2a']} LH2R={counts['lh2r']}",
                end="",
                flush=True,
            )
            next_frame = now + args.period

    save_json(args.output, {
        "description": "v12 LH2A wand wave record. BS4 will be the reference Lighthouse frame; BS10 and moving poses are solved later.",
        "created_unix_time_s": time.time(),
        "basestations": sorted(basestations),
        "duration_s": float(args.duration),
        "window_s": float(args.window),
        "period_s": float(args.period),
        "cluster_deg": float(args.cluster_deg),
        "frames": frames,
    })

    print()
    print("=" * 88)
    print(f"Saved {len(frames)} frames to: {args.output}")
    print("=" * 88)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
