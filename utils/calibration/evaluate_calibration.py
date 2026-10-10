#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
evaluate_calibration.py — how good is a calibration, measured on its own real data.

The solver's 1-sigma (in geometry.yaml) is a lower bound. This script measures the
real thing by resampling (RESULTS.md, finding 6):

  1. Repeat the solve --runs times, each on a random --fraction of the sweep snapshots
     (static captures always kept), with calibrate_bitcraze.py exactly as the wizard runs it.
  2. Repeatability: spread of the base-station positions, tilts and baseline over the runs.
  3. Hold-out error: every sweep snapshot left out of a run is triangulated with that run's
     geometry; the board-fit error (how far the four triangulated sensors are from the real
     board shape) says how well the geometry explains data it never saw.
  4. Per-point error, averaged over the runs that held each snapshot out, for a 3D map of
     where the calibration is weak.

Usage:
    python evaluate_calibration.py calib_runs/run_20261009_180300 --baseline 2.26
    python evaluate_calibration.py <session> --runs 12 --fraction 0.7 --jobs 4 --json
"""

from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import calibrate_bitcraze as cb
from locate_samples import load_geometry

OUTLIER_MM = 50.0  # hold-out snapshots worse than this are mis-decoded, not calibration error


def _snapshots(sweep: Path, max_age_ms: int, max_skew_ms: int) -> list[dict[int, dict]]:
    records, _ = cb.filter_by_age(cb.load_records(str(sweep)), max_age_ms, max_skew_ms)
    grouped: dict[float, dict[int, dict]] = {}
    for record in records:
        grouped.setdefault(float(record["timestamp"]), {})[int(record["base_station_id"])] = record
    return [g for _, g in sorted(grouped.items()) if len(g) >= 2]


def _solve_subset(session: Path, work: Path, keep: list[dict[int, dict]], solver_args: list[str]) -> Path | None:
    """Copy the session with only `keep` as sweep and solve it; returns geometry.yaml or None."""
    work.mkdir(parents=True)
    for item in session.iterdir():
        if item.suffix == ".json" and item.name != "sweep.json":
            shutil.copy2(item, work / item.name)
    with open(work / "sweep.json", "w", encoding="utf-8") as stream:
        json.dump([record for snapshot in keep for record in snapshot.values()], stream)
    geometry = work / "geometry.yaml"
    result = subprocess.run([sys.executable, str(Path(cb.__file__)), str(work), "-o", str(geometry), *solver_args],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
    (work / "solve.log").write_text(result.stdout, encoding="utf-8")
    return geometry if result.returncode == 0 and geometry.exists() else None


def _tilt_deg(pose) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, -pose.rot_matrix[2, 0]))))


def _stats(values) -> dict:
    values = np.asarray(values, dtype=float)
    return {"mean": float(values.mean()), "std": float(values.std(ddof=1)) if len(values) > 1 else 0.0,
            "range": float(values.max() - values.min())}


def evaluate(session: Path, runs: int, fraction: float, jobs: int, solver_args: list[str],
             sensors_path: str, seed: int, max_points: int, progress=print) -> dict:
    snapshots = _snapshots(session / "sweep.json", 100, 50)
    if len(snapshots) < 60:
        raise ValueError(f"only {len(snapshots)} usable sweep snapshots — record a longer sweep")
    rng = random.Random(seed)
    splits = []
    for _ in range(runs):
        chosen = set(rng.sample(range(len(snapshots)), int(round(fraction * len(snapshots)))))
        splits.append(chosen)

    sensors = cb.load_sensor_positions(sensors_path)
    temp = Path(tempfile.mkdtemp(prefix="lh2_eval_"))
    geometries: dict[int, Path] = {}
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
            futures = {pool.submit(_solve_subset, session, temp / f"run{i}",
                                   [snapshots[k] for k in sorted(chosen)], solver_args): i
                       for i, chosen in enumerate(splits)}
            for done, future in enumerate(concurrent.futures.as_completed(futures), 1):
                i = futures[future]
                geometry = future.result()
                if geometry is not None:
                    geometries[i] = geometry
                progress(f"run {done}/{runs} {'ok' if geometry else 'FAILED'}")

        if len(geometries) < 2:
            raise ValueError(f"only {len(geometries)} of {runs} resampled solves succeeded")

        stations: dict[int, dict[str, list]] = {}
        baselines, held_errors, outliers, held_total = [], [], 0, 0
        per_point: dict[int, list[float]] = {}
        positions: dict[int, np.ndarray] = {}
        for i, geometry in sorted(geometries.items()):
            poses = load_geometry(geometry)
            for bs, pose in poses.items():
                entry = stations.setdefault(bs, {"x": [], "y": [], "z": [], "tilt": []})
                for axis, value in zip("xyz", pose.translation):
                    entry[axis].append(float(value))
                entry["tilt"].append(_tilt_deg(pose))
            baselines.append(float(np.linalg.norm(poses[1].translation - poses[0].translation)))
            for k in range(len(snapshots)):
                if k in splits[i]:
                    continue
                sample = cb.LhCfPoseSample()
                for bs_id, record in snapshots[k].items():
                    sample.angles_calibrated[bs_id] = cb.LighthouseBsVectors(
                        [cb.LighthouseBsVector(float(h), float(v)) for h, v in record["angles"]])
                located = cb.triangulate_wand(sample, poses, sensors)
                held_total += 1
                if located is None or located[1] * 1000 > OUTLIER_MM:
                    outliers += 1
                    continue
                error = located[1] * 1000
                held_errors.append(error)
                per_point.setdefault(k, []).append(error)
                positions[k] = located[0].translation
    finally:
        shutil.rmtree(temp, ignore_errors=True)

    errors = np.asarray(held_errors)
    station_summary = {}
    for bs, entry in sorted(stations.items()):
        xyz = np.array([entry["x"], entry["y"], entry["z"]]).T
        centre = xyz.mean(axis=0)
        station_summary[str(bs)] = {
            "position_mean": centre.tolist(),
            "position_std_mm": (xyz.std(axis=0, ddof=1) * 1000).tolist(),
            "position_spread_mm": float(np.max(np.linalg.norm(xyz - centre, axis=1)) * 1000),
            "tilt_deg": _stats(entry["tilt"]),
        }
    keys = sorted(per_point)
    if len(keys) > max_points:
        keys = keys[::math.ceil(len(keys) / max_points)]
    return {
        "session": str(session),
        "runs": runs, "solved": len(geometries), "fraction": fraction, "snapshots": len(snapshots),
        "stations": station_summary,
        "baseline_m": _stats(baselines),
        "holdout": {
            "median_mm": float(np.median(errors)), "p90_mm": float(np.percentile(errors, 90)),
            "max_mm": float(errors.max()), "count": int(errors.size),
            "outlier_share": outliers / held_total if held_total else 0.0,
        },
        "points": [[round(float(c), 4) for c in positions[k]] + [round(float(np.mean(per_point[k])), 2)] for k in keys],
    }


def print_report(result: dict) -> None:
    print(f"\nResampling: {result['solved']}/{result['runs']} solves on {result['fraction']:.0%} of "
          f"{result['snapshots']} sweep snapshots")
    for bs, s in result["stations"].items():
        std = s["position_std_mm"]
        print(f"BS{bs}: position spread {s['position_spread_mm']:.1f} mm (1 sigma x/y/z {std[0]:.1f} / {std[1]:.1f} / "
              f"{std[2]:.1f} mm), tilt {s['tilt_deg']['mean']:.2f} +- {s['tilt_deg']['std']:.2f} deg")
    b = result["baseline_m"]
    print(f"Baseline: {b['mean']:.4f} m +- {b['std'] * 1000:.1f} mm (range {b['range'] * 1000:.1f} mm)")
    h = result["holdout"]
    print(f"Hold-out board-fit error: median {h['median_mm']:.2f} mm, 90% {h['p90_mm']:.2f} mm, "
          f"max {h['max_mm']:.1f} mm over {h['count']} snapshots ({h['outlier_share']:.1%} mis-decoded, ignored)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="solved session directory (session.json + captures)")
    parser.add_argument("--runs", type=int, default=12, help="resampled solves (default 12)")
    parser.add_argument("--fraction", type=float, default=0.7, help="share of sweep snapshots per solve (default 0.7)")
    parser.add_argument("--jobs", type=int, default=max(1, min(4, (os.cpu_count() or 2) // 2)),
                        help="solves in parallel (default: half the CPUs, at most 4)")
    parser.add_argument("--baseline", type=float, help="tape BS0-BS1 distance, passed to the solver")
    parser.add_argument("--board-height", type=float, default=0.0, help="passed to the solver")
    parser.add_argument("--sensor-positions", default=str(Path(__file__).with_name("wand_sensors.json")))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=600, help="points in the error map (default 600)")
    parser.add_argument("--json", action="store_true", help="print the result as JSON on the last line")
    args = parser.parse_args()
    if not (args.session / "session.json").exists():
        print(f"Error: {args.session} has no session.json", file=sys.stderr)
        return 1
    solver_args = []
    if args.baseline:
        solver_args += ["--baseline", str(args.baseline)]
    if args.board_height:
        solver_args += ["--board-height", str(args.board_height)]
    try:
        with contextlib.redirect_stdout(sys.stderr if args.json else sys.stdout):
            result = evaluate(args.session, args.runs, args.fraction, args.jobs, solver_args,
                              args.sensor_positions, args.seed, args.max_points,
                              progress=lambda text: print(text, flush=True))
            print_report(result)
    except ValueError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
