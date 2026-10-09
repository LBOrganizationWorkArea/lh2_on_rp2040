#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_synthetic_measurements.py — fabricate a wand capture from known lighthouse poses.

Writes the same JSON format as `calibrate_lighthouse.py --udp` (including
angle_ages_ms), so the solvers can be checked against a known ground truth:

    # Wizard session (origin / x-axis / floor / sweep), like capture_lh2.py:
    python make_synthetic_measurements.py --session synth_session
    python calibrate_bitcraze.py synth_session --baseline 2.26 --truth synth_session/truth.json

    # Single free-motion file:
    python make_synthetic_measurements.py -o synth.json --truth synth_truth.json
    python calibrate_bitcraze.py synth.json --auto-frame --baseline 2.26 --truth synth_truth.json

Angle model is the firmware / Bitcraze one: BS-local +X is the boresight and
    horiz = atan2(y_local, x_local),  vert = atan2(z_local, x_local).
"""

import argparse
import json
import math
import os

import numpy as np
from scipy.spatial.transform import Rotation


def bs_rotation(yaw_deg: float, pitch_down_deg: float) -> np.ndarray:
    """Local->world R for a station facing `yaw_deg`, tilted down by `pitch_down_deg`."""
    return Rotation.from_euler("ZY", [yaw_deg, pitch_down_deg], degrees=True).as_matrix()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-o", "--output", default="synthetic_measurements.json")
    parser.add_argument("--truth", help="also write the ground-truth poses to this JSON file")
    parser.add_argument("--sensor-positions", default=None,
                        help="wand layout used to fabricate the angles (default: 4x4 cm square)")
    parser.add_argument("--samples", type=int, default=600)
    parser.add_argument("--noise-deg", type=float, default=0.02, help="1-sigma angle noise [deg]")
    parser.add_argument("--baseline", type=float, default=2.26)
    parser.add_argument("--height", type=float, default=3.45)
    parser.add_argument("--pitch-down", type=float, default=70.0,
                        help="station tilt below horizontal [deg]; 90 = straight down")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--session", help="write a capture_lh2.py-style wizard session directory instead")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    if args.sensor_positions:
        with open(args.sensor_positions, encoding="utf-8") as source:
            sensors = np.asarray(json.load(source), dtype=float)
    else:
        sensors = np.array([[0, 0, 0], [0.04, 0, 0], [0.04, 0.04, 0], [0, 0.04, 0]], dtype=float)

    stations = {
        0: (np.array([0.0, 0.0, args.height]), bs_rotation(0.0, args.pitch_down)),
        1: (np.array([args.baseline, 0.0, args.height]), bs_rotation(180.0, args.pitch_down)),
    }

    noise = math.radians(args.noise_deg)

    def observe(world_sensors, t):
        """Records for both stations, or [] if a station cannot see every sensor."""
        sample = []
        for bs_id, (origin, R) in stations.items():
            local = (R.T @ (world_sensors - origin).T).T
            if np.any(local[:, 0] <= 0.1):
                return []
            angles = np.column_stack((np.arctan2(local[:, 1], local[:, 0]),
                                      np.arctan2(local[:, 2], local[:, 0])))
            if np.any(np.abs(angles) > math.radians(60)):
                return []
            angles += rng.normal(0, noise, angles.shape)
            sample.append({
                "timestamp": round(t, 6),
                "base_station_id": bs_id,
                "angles": angles.tolist(),
                "angle_ages_ms": rng.integers(0, 30, 4).tolist(),
            })
        return sample

    centred = sensors - sensors.mean(axis=0)
    static_t = [1000.0]

    def static_capture(mark, count=50):
        """Drone flat on the floor, sensor centre on `mark`, random heading."""
        heading = Rotation.from_euler("z", rng.uniform(-180, 180), degrees=True).as_matrix()
        world_sensors = (heading @ centred.T).T + mark
        records = []
        for _ in range(count):
            static_t[0] += 0.1
            sample = observe(world_sensors, static_t[0])
            if not sample:
                raise SystemExit(f"mark {mark} is not visible to both stations")
            records.extend(sample)
        return records

    records = []
    t = 0.0
    while len(records) < 2 * args.samples:
        t += 0.1
        position = rng.uniform([0.2, -0.8, 0.2], [args.baseline - 0.2, 0.8, 1.6])
        # Wand mostly face-up, with up to ~25 deg tilt and any yaw.
        wand_R = Rotation.from_euler("zyx", [rng.uniform(-180, 180), rng.normal(0, 12), rng.normal(0, 12)],
                                     degrees=True).as_matrix()
        world_sensors = (wand_R @ sensors.T).T + position

        records.extend(observe(world_sensors, t))

    if args.session:
        # Marks on the floor (z = 0); the origin mark becomes world (0, 0, 0).
        origin_mark = np.array([0.4, 0.1, 0.0])
        marks = {
            "origin.json": origin_mark,
            "x_axis.json": origin_mark + [1.0, 0.0, 0.0],
            "xy_plane_1.json": np.array([1.0, 0.7, 0.0]),
            "xy_plane_2.json": np.array([1.8, -0.5, 0.0]),
            "xy_plane_3.json": np.array([0.6, -0.6, 0.0]),
        }
        os.makedirs(args.session, exist_ok=True)
        for name, mark in marks.items():
            with open(os.path.join(args.session, name), "w", encoding="utf-8") as output:
                json.dump(static_capture(mark), output)
        with open(os.path.join(args.session, "sweep.json"), "w", encoding="utf-8") as output:
            json.dump(records, output)
        with open(os.path.join(args.session, "session.json"), "w", encoding="utf-8") as output:
            json.dump({"type": "lh2_bitcraze_calibration_session", "origin": "origin.json",
                       "x_axis": ["x_axis.json"], "x_axis_dist": 1.0,
                       "xy_plane": ["xy_plane_1.json", "xy_plane_2.json", "xy_plane_3.json"],
                       "sweep": "sweep.json"}, output, indent=2)
        truth = {str(bs_id): {"origin": (origin - origin_mark).tolist(), "R": R.tolist()}
                 for bs_id, (origin, R) in stations.items()}
        with open(os.path.join(args.session, "truth.json"), "w", encoding="utf-8") as output:
            json.dump(truth, output, indent=2)
        print(f"Wrote wizard session ({len(records) // 2} sweep poses, 5 static captures) to {args.session}/")
        return

    with open(args.output, "w", encoding="utf-8") as output:
        json.dump(records, output, indent=1)
    print(f"Wrote {len(records)} measurements ({len(records) // 2} wand poses) to {args.output}")

    if args.truth:
        truth = {str(bs_id): {"origin": origin.tolist(), "R": R.tolist()}
                 for bs_id, (origin, R) in stations.items()}
        with open(args.truth, "w", encoding="utf-8") as output:
            json.dump(truth, output, indent=2)
        print(f"Wrote ground truth to {args.truth}")


if __name__ == "__main__":
    main()
