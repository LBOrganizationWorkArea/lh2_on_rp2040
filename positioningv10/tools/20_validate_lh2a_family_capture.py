#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


EXPECTED_CHANNELS = 16


def main():
    parser = argparse.ArgumentParser(description="Validate an LH2A family pose capture before fitting.")
    parser.add_argument("--poses", default="config/wand_calibration_poses_3d_lh2a_families.json")
    parser.add_argument("--min-channels", type=int, default=16)
    parser.add_argument("--min-lh2a", type=int, default=250)
    parser.add_argument("--min-two-family-channels", type=int, default=12)
    parser.add_argument("--max-spread-deg", type=float, default=2.0)
    args = parser.parse_args()

    path = Path(args.poses)
    if not path.exists():
        raise SystemExit(f"Missing capture file: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    poses = data.get("poses", [])
    print("=" * 88)
    print("LH2A family capture validation")
    print(f"File: {path}")
    print(f"Poses: {len(poses)}")
    print("=" * 88)

    good = 0
    warnings = 0
    for pose in poses:
        name = pose.get("name", "(unnamed)")
        measurements = pose.get("measurements", [])
        counts = pose.get("raw_counts", {})
        lh2a_count = int(counts.get("lh2a", 0))
        keys = {
            (int(m.get("sensor", -1)), int(m.get("basestation", -1)), int(m.get("sweep", -1)))
            for m in measurements
        }
        channels = len(keys)
        two_family_channels = sum(1 for m in measurements if len(m.get("candidate_families", [])) >= 2)
        max_spread = 0.0
        for m in measurements:
            for family in m.get("candidate_families", []):
                max_spread = max(max_spread, float(family.get("angle_spread_deg", 0.0)))

        issues = []
        if channels < args.min_channels:
            issues.append(f"channels {channels}/{EXPECTED_CHANNELS}")
        if lh2a_count < args.min_lh2a:
            issues.append(f"LH2A low {lh2a_count}")
        if two_family_channels < args.min_two_family_channels:
            issues.append(f"two-family {two_family_channels}/{EXPECTED_CHANNELS}")
        if max_spread > args.max_spread_deg:
            issues.append(f"spread {max_spread:.2f}deg")

        if issues:
            warnings += 1
            status = "WARN"
            suffix = " | " + "; ".join(issues)
        else:
            good += 1
            status = "OK"
            suffix = ""

        print(
            f"{status} | {name} | channels={channels}/{EXPECTED_CHANNELS} | "
            f"two-family={two_family_channels}/{EXPECTED_CHANNELS} | "
            f"LH2A={lh2a_count} | max_spread={max_spread:.2f}deg{suffix}"
        )

    print("=" * 88)
    print(f"Summary: OK={good} | WARN={warnings}")
    if warnings:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
