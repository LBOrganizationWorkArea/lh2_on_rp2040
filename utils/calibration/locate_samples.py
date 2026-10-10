#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
locate_samples.py — place a session's samples in the world with a solved geometry.

Each static capture (origin, x_axis, xy_plane_*, verify_*) is averaged like the solver
does and triangulated from both stations (calibrate_bitcraze.triangulate_wand); the
board layout is then rigidly fitted to the four sensor points. The fit error says how
well the geometry explains that sample — for verify_* captures, which the solver never
saw, it is the Bitcraze wizard's "verification sample error". A thinned cloud of the
sweep (XYZ-space samples) is included for display.

Usage:
    python locate_samples.py calib_runs/run_20261009_180300
    python locate_samples.py <session> --geometry <session>_factory/geometry.yaml --json
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import calibrate_bitcraze as cb

STATIC_ORDER = ("origin", "x_axis", "xy_plane", "verify")


def load_geometry(path: Path) -> dict[int, cb.Pose]:
    import yaml
    with open(path, encoding="utf-8") as stream:
        geos = yaml.safe_load(stream)["geos"]
    return {int(bs): cb.Pose(R_matrix=Rotation.from_quat(g["rotation_quat"]).as_matrix(), t_vec=g["position"])
            for bs, g in geos.items()}


def _sample(per_bs: dict[int, list]) -> cb.LhCfPoseSample:
    sample = cb.LhCfPoseSample()
    for bs_id, angles in per_bs.items():
        sample.angles_calibrated[bs_id] = cb.LighthouseBsVectors(
            [cb.LighthouseBsVector(float(h), float(v)) for h, v in angles])
    return sample


def _locate(sample, poses, sensors) -> dict | None:
    result = cb.triangulate_wand(sample, poses, sensors)
    if result is None:
        return None
    pose, error = result
    return {"position": pose.translation.tolist(), "error_mm": error * 1000.0}


def locate(session: Path, geometry: Path, sensors_path: str, max_cloud: int) -> dict:
    poses = load_geometry(geometry)
    sensors = cb.load_sensor_positions(sensors_path)
    statics = []
    for path in sorted(session.glob("*.json"), key=lambda p: (
            next((i for i, k in enumerate(STATIC_ORDER) if p.stem.startswith(k)), 99), p.stem)):
        kind = next((k for k in STATIC_ORDER if path.stem.startswith(k)), None)
        if kind is None:
            continue
        try:
            sample = cb.average_static_capture(str(path))
        except ValueError as error:
            statics.append({"name": path.stem, "kind": kind, "error": str(error)})
            continue
        located = _locate(sample, poses, sensors) or {"error": "rays do not intersect"}
        statics.append({"name": path.stem, "kind": kind, **located})

    cloud = []
    sweep = session / "sweep.json"
    if sweep.exists():
        grouped: dict[float, dict[int, list]] = {}
        for record in cb.load_records(str(sweep)):
            grouped.setdefault(float(record["timestamp"]), {})[int(record["base_station_id"])] = record["angles"]
        complete = [g for g in grouped.values() if len(g) >= 2]
        step = max(1, len(complete) // max_cloud)
        for per_bs in complete[::step]:
            located = _locate(_sample(per_bs), poses, sensors)
            if located and located["error_mm"] < 20.0:  # drop mis-decoded snapshots from the picture
                cloud.append([round(c, 4) for c in located["position"]])
    return {"geometry": str(geometry), "statics": statics, "cloud": cloud}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="capture_lh2.py / pose editor session directory")
    parser.add_argument("--geometry", type=Path, help="solved geometry (default <session>/geometry.yaml)")
    parser.add_argument("--sensor-positions", default=str(Path(__file__).with_name("wand_sensors.json")))
    parser.add_argument("--max-cloud", type=int, default=400, help="sweep points to keep (default 400)")
    parser.add_argument("--json", action="store_true", help="print JSON (for the pose editor bridge)")
    args = parser.parse_args()
    geometry = args.geometry or args.session / "geometry.yaml"
    if not geometry.exists():
        print(f"Error: {geometry} not found — solve the session first", file=sys.stderr)
        return 1
    if args.json:
        with contextlib.redirect_stdout(sys.stderr):  # solver warnings must not corrupt the JSON
            result = locate(args.session, geometry, args.sensor_positions, args.max_cloud)
        print(json.dumps(result))
        return 0
    result = locate(args.session, geometry, args.sensor_positions, args.max_cloud)
    for item in result["statics"]:
        if "position" in item:
            x, y, z = item["position"]
            print(f"{item['name']:<12} [{x:+.3f}, {y:+.3f}, {z:+.3f}] m   fit error {item['error_mm']:.2f} mm")
        else:
            print(f"{item['name']:<12} {item['error']}")
    print(f"sweep cloud: {len(result['cloud'])} points")
    return 0


if __name__ == "__main__":
    sys.exit(main())
