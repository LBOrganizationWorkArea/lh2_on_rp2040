#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_bitcraze.py — lighthouse auto-calibration with the unmodified Bitcraze pipeline.

Unlike `calibrate_lighthouse.py --solve`, nothing about the base stations is
assumed up front (no nominal rotation, height or baseline seed). Everything is
recovered from the wand capture, exactly as the Crazyflie client does it:

  1. match      — pair BS0/BS1 records that share a timestamp       (LighthouseSampleMatcher)
  2. IPPE       — per-sample planar pose, mirror ambiguity resolved
                  by clustering across samples                        (LighthouseInitialEstimator)
  3. solve      — sparse least squares over all BS + wand poses       (LighthouseGeometrySolver)
  4. align      — Bitcraze wizard frame from static captures at origin / +X / floor
                                                                      (LighthouseSystemAligner)
                  (--auto-frame instead: BS0 at (0, 0, --height), +X toward BS1)
  5. scale      — rescale so the x-axis capture is --x-axis-dist (1 m) away
                                                                      (LighthouseSystemScaler)

SENSOR GEOMETRY — the only metric reference in the whole solve is the wand
layout in --sensor-positions. Scale error in that file maps 1:1 onto every
lighthouse position (a 5 cm square entered as 4 cm shrinks the room by 20 %).
Rows must be in firmware sensor order S0..S3. See README "Sensor geometry".

Usage (default — Bitcraze wizard session recorded by capture_lh2.py):
    python calibrate_bitcraze.py calib_20261009_101500/ --baseline 2.26

    The world frame comes from the static captures: origin -> (0, 0, 0), the
    x-axis capture -> +X (rescaled to its --x-axis-dist, 1 m, as Bitcraze does),
    floor captures -> Z = 0. Z = 0 is the sensor plane while the drone sits on
    the floor; pass --board-height to make Z = 0 the floor itself.

Same thing from loose files:
    python calibrate_bitcraze.py sweep.json --origin o.json --x-axis x.json --xy-plane p1.json p2.json p3.json

Without static captures (old free-motion recordings):
    python calibrate_bitcraze.py measurements.json --auto-frame --height 3.45
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

# Repository root on sys.path so the relative imports inside calibration_lib
# (`from ...angle_lib import ...`) resolve to utils/angle_lib.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from utils.angle_lib.lighthouse_types import LhBsCfPoses, LhCfPoseSample, LhMeasurement, Pose  # noqa: E402
from utils.calibration.calibration_lib.lighthouse_bs_vector import (  # noqa: E402
    LighthouseBsVector, LighthouseBsVectors)
from utils.calibration.calibration_lib.lighthouse_geometry_solver import LighthouseGeometrySolver  # noqa: E402
from utils.calibration.calibration_lib.lighthouse_initial_estimator import LighthouseInitialEstimator  # noqa: E402
from utils.calibration.calibration_lib.lighthouse_sample_matcher import LighthouseSampleMatcher  # noqa: E402
from utils.calibration.calibration_lib.lighthouse_system_aligner import LighthouseSystemAligner  # noqa: E402
from utils.calibration.calibration_lib.lighthouse_system_scaler import LighthouseSystemScaler  # noqa: E402

# Planarity tolerance for the wand: IPPE is a *planar* pose estimator.
PLANARITY_TOL_M = 0.002


# --------------------------------------------------------------------------- #
# Sensor geometry
# --------------------------------------------------------------------------- #

def load_sensor_positions(path: str) -> np.ndarray:
    """Load and validate the wand layout (4 x [x, y, z] metres, S0..S3 order)."""
    with open(path, encoding="utf-8") as source:
        positions = np.asarray(json.load(source), dtype=float)
    if positions.shape != (4, 3) or not np.isfinite(positions).all():
        raise ValueError(f"{path}: expected a finite 4x3 array of [x, y, z] metres in S0..S3 order")

    centred = positions - positions.mean(axis=0)
    singular_values = np.linalg.svd(centred, compute_uv=False)
    if singular_values[1] < 1e-3:
        raise ValueError(f"{path}: sensors are collinear — the wand pose is unobservable")
    # Smallest singular value ~ RMS out-of-plane distance * 2.
    if singular_values[2] / 2.0 > PLANARITY_TOL_M:
        raise ValueError(f"{path}: sensors are not coplanar (out-of-plane ~{singular_values[2] / 2 * 1000:.1f} mm); "
                         "IPPE needs a planar wand")

    distances = {f"S{a}-S{b}": float(np.linalg.norm(positions[a] - positions[b]))
                 for a, b in itertools.combinations(range(4), 2)}
    if min(distances.values()) < 0.005 or max(distances.values()) > 0.5:
        raise ValueError(f"{path}: sensor spacing {distances} is implausible — are the units metres?")
    # Centre the board on its sensor centroid, like the Crazyflie deck. The
    # drone's pose (and so the origin / x-axis / floor marks) then refers to
    # the middle of the board, not to whichever sensor the file put at 0,0,0.
    return positions - positions.mean(axis=0)


def describe_sensor_positions(positions: np.ndarray) -> str:
    lines = []
    for a, b in itertools.combinations(range(4), 2):
        lines.append(f"S{a}-S{b} {np.linalg.norm(positions[a] - positions[b]) * 1000:6.1f} mm")
    return ", ".join(lines)


# --------------------------------------------------------------------------- #
# Measurements
# --------------------------------------------------------------------------- #

def load_records(path: str) -> list[dict]:
    with open(path, encoding="utf-8") as source:
        records = json.load(source)
    if not isinstance(records, list) or not records:
        raise ValueError(f"{path}: expected a non-empty JSON array of measurements")
    return records


def filter_by_age(records: list[dict], max_age_ms: int, max_skew_ms: int) -> tuple[list[dict], int]:
    """Keep timestamps whose 8 angles (both stations) are fresh and mutually coherent.

    Same rule as calibrate_lighthouse.filter_coherent_measurements. Records
    without angle_ages_ms (older firmware) are passed through untouched.
    """
    if any("angle_ages_ms" not in record for record in records):
        return records, 0
    grouped: dict[float, dict[int, dict]] = {}
    for record in records:
        grouped.setdefault(float(record["timestamp"]), {})[int(record["base_station_id"])] = record
    kept, rejected = [], 0
    for sample in grouped.values():
        ages = [int(age) for record in sample.values() for age in record["angle_ages_ms"]]
        if len(sample) < 2 or max(ages) > max_age_ms or max(ages) - min(ages) > max_skew_ms:
            rejected += 1
            continue
        kept.extend(sample.values())
    return kept, rejected


def filter_by_wand_span(records: list[dict], sensors: np.ndarray,
                        min_distance: float) -> tuple[list[dict], int]:
    """Drop records whose four sensors span more angle than the wand physically can.

    The wand cannot subtend more than atan(diagonal / min_distance) from a
    station. Larger spans mean a sensor's angle is corrupt (e.g. a mis-paired
    sweep shifts the vertical angle by several degrees) and would poison the
    solve. Both stations' records at that timestamp are dropped together.
    """
    diagonal = max(np.linalg.norm(a - b) for a, b in itertools.combinations(sensors, 2))
    limit = math.atan2(diagonal, min_distance)
    bad_timestamps = set()
    for record in records:
        angles = np.asarray(record["angles"], dtype=float)
        if np.max(np.ptp(angles, axis=0)) > limit:
            bad_timestamps.add(float(record["timestamp"]))
    kept = [record for record in records if float(record["timestamp"]) not in bad_timestamps]
    return kept, len(bad_timestamps)


def to_measurements(records: list[dict]) -> list[LhMeasurement]:
    measurements = []
    for index, record in enumerate(records):
        try:
            angles = np.asarray(record["angles"], dtype=float)
            if angles.shape != (4, 2) or not np.isfinite(angles).all():
                raise ValueError("expected four finite [horizontal, vertical] pairs")
            vectors = LighthouseBsVectors([LighthouseBsVector(float(h), float(v)) for h, v in angles])
            measurements.append(LhMeasurement(float(record["timestamp"]), int(record["base_station_id"]), vectors))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid measurement at JSON index {index}: {error}") from error
    measurements.sort(key=lambda item: item.timestamp)
    return measurements


def average_static_capture(path: str) -> LhCfPoseSample:
    """Average a static capture into one pose sample (what the Bitcraze wizard records)."""
    per_bs: dict[int, list[np.ndarray]] = {}
    for record in load_records(path):
        per_bs.setdefault(int(record["base_station_id"]), []).append(np.asarray(record["angles"], dtype=float))
    if len(per_bs) < 2:
        raise ValueError(f"{path}: a reference capture must see both base stations")
    sample = LhCfPoseSample()
    for bs_id, captures in per_bs.items():
        stack = np.stack(captures)
        spread_deg = math.degrees(float(np.max(np.std(stack, axis=0))))
        if spread_deg > 0.5:
            print(f"Warning: {path} BS{bs_id} angle spread {spread_deg:.2f} deg — was the wand held still?")
        mean = np.median(stack, axis=0)
        sample.angles_calibrated[bs_id] = LighthouseBsVectors(
            [LighthouseBsVector(float(h), float(v)) for h, v in mean])
    return sample


# --------------------------------------------------------------------------- #
# Solve
# --------------------------------------------------------------------------- #

def _rereference(bs_poses: dict[int, Pose], cf_poses: list[Pose]) -> LhBsCfPoses:
    """Express all poses in the frame of cf_poses[0] (the solver's fixed gauge)."""
    reference = cf_poses[0]
    bs = {bs_id: reference.inv_rotate_translate_pose(pose) for bs_id, pose in bs_poses.items()}
    cfs = [reference.inv_rotate_translate_pose(pose) for pose in cf_poses]
    cfs[0] = Pose()
    return LhBsCfPoses(bs, cfs)


def triangulate_wand(sample: LhCfPoseSample, bs_poses: dict[int, Pose],
                     sensors: np.ndarray) -> Optional[tuple[Pose, float]]:
    """Wand pose for one sample from known station poses.

    Each sensor is the midpoint of closest approach of its BS0 and BS1 rays;
    the wand layout is then rigidly fitted (Kabsch) to those four points.
    Returns (pose, RMS fit error [m]) or None if the rays are degenerate.
    """
    points = []
    for sensor in range(len(sensors)):
        origins, directions = [], []
        for bs_id in (0, 1):
            pose = bs_poses[bs_id]
            origins.append(pose.translation)
            directions.append(pose.rot_matrix @ sample.angles_calibrated[bs_id][sensor].cart)
        d0, d1 = directions
        cross = float(np.dot(d0, d1))
        denominator = 1.0 - cross * cross
        if denominator < 1e-6:
            return None
        delta = origins[0] - origins[1]
        s0 = (cross * np.dot(d1, delta) - np.dot(d0, delta)) / denominator
        s1 = (np.dot(d1, delta) - cross * np.dot(d0, delta)) / denominator
        if s0 <= 0.0 or s1 <= 0.0:
            return None
        points.append(0.5 * (origins[0] + s0 * d0 + origins[1] + s1 * d1))
    points = np.asarray(points)

    sensor_centre, point_centre = sensors.mean(axis=0), points.mean(axis=0)
    u, _, vt = np.linalg.svd((sensors - sensor_centre).T @ (points - point_centre))
    correction = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))])
    rotation = vt.T @ correction @ u.T
    translation = point_centre - rotation @ sensor_centre
    fitted = (rotation @ sensors.T).T + translation
    error = float(np.sqrt(np.mean(np.sum((fitted - points) ** 2, axis=1))))
    return Pose(R_matrix=rotation, t_vec=translation), error


def solve(samples: list[LhCfPoseSample], sensors: np.ndarray, n_protected: int,
          max_sample_error: float, max_iter: int):
    """IPPE initial guess + least squares, with iterative outlier rejection.

    The first `n_protected` samples (static reference captures) are never dropped.

    Extension over the Bitcraze flow: IPPE on a small wand seen face-on (stations
    pointing straight down) is ambiguous and its outlier gate discards most
    samples. After the first solve, every sample is re-admitted with a wand pose
    triangulated from the solved stations, and the full set is solved again.
    """
    # IPPE may reject the static captures too (a level board seen face-on is
    # its worst case); they come back in via triangulation below.
    initial_guess, cleaned = LighthouseInitialEstimator.estimate(samples, sensors)
    ippe_kept = len(cleaned)
    seed = LighthouseGeometrySolver.solve(initial_guess, cleaned, sensors, max_nr_iter=max_iter)

    readmitted, wand_poses, rigid_errors = [], [], []
    for sample in samples:
        result = triangulate_wand(sample, seed.bs_poses, sensors)
        if result is not None:
            readmitted.append(sample)
            wand_poses.append(result[0])
            rigid_errors.append(result[1])
    if readmitted[:n_protected] != samples[:n_protected]:
        raise RuntimeError("a static reference capture could not be triangulated — recapture it")
    cleaned = readmitted
    solution = LighthouseGeometrySolver.solve(
        _rereference(seed.bs_poses, wand_poses), cleaned, sensors, max_nr_iter=max_iter)

    rejected = 0
    for _ in range(5):
        worst = np.array([max(errors.values()) for errors in solution.estimated_errors])
        keep = [i for i in range(len(cleaned)) if i < n_protected or worst[i] <= max_sample_error]
        if len(keep) == len(cleaned):
            break
        rejected += len(cleaned) - len(keep)
        cleaned = [cleaned[i] for i in keep]
        guess = _rereference(solution.bs_poses, [solution.cf_poses[i] for i in keep])
        solution = LighthouseGeometrySolver.solve(guess, cleaned, sensors, max_nr_iter=max_iter)

    # Sensor order / shape check: with the final stations, the triangulated
    # sensors must form the wand. A wrong S0..S3 order or layout cannot be
    # absorbed by a rigid fit, so the median fit error grows to centimetres.
    # (A wrong *scale* is absorbed — only --baseline can catch that.)
    final_errors = [result[1] for result in (triangulate_wand(sample, solution.bs_poses, sensors)
                                             for sample in cleaned) if result is not None]
    stats = {
        "ippe_kept": ippe_kept,
        "rejected": rejected,
        "median_rigid_error": float(np.median(final_errors)) if final_errors else math.inf,
    }
    return solution, cleaned, stats


# --------------------------------------------------------------------------- #
# World frame
# --------------------------------------------------------------------------- #

def auto_frame(bs_poses: dict[int, Pose], height: float) -> dict[int, Pose]:
    """BS0 at (0, 0, height), +X along the BS0->BS1 baseline, up = -mean boresight.

    Same convention as calibrate_lighthouse.py --solve, so outputs are comparable.
    """
    bs0, bs1 = bs_poses[0], bs_poses[1]
    up = -np.mean([pose.rot_matrix[:, 0] for pose in bs_poses.values()], axis=0)
    if np.linalg.norm(up) < 1e-3:
        raise RuntimeError("boresights cancel out — cannot infer world up; use --origin/--x-axis/--xy-plane")
    up /= np.linalg.norm(up)
    baseline = bs1.translation - bs0.translation
    x_axis = baseline - np.dot(baseline, up) * up
    if np.linalg.norm(x_axis) < 1e-3:
        raise RuntimeError("baseline is vertical — cannot define +X; use --origin/--x-axis/--xy-plane")
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(up, x_axis)
    to_world = np.vstack((x_axis, y_axis, np.cross(x_axis, y_axis)))  # rows = world axes in solver frame
    target = np.array([0.0, 0.0, height])
    return {bs_id: Pose(R_matrix=to_world @ pose.rot_matrix,
                        t_vec=target + to_world @ (pose.translation - bs0.translation))
            for bs_id, pose in bs_poses.items()}


def bitcraze_frame(solution, n_x: int, n_xy: int, x_axis_dist: Optional[float]):
    """Bitcraze wizard alignment: sample 0 = origin, then x-axis, then floor samples."""
    cf_poses = solution.cf_poses
    origin = cf_poses[0].translation
    x_axis = [pose.translation for pose in cf_poses[1:1 + n_x]]
    xy_plane = [pose.translation for pose in cf_poses[1 + n_x:1 + n_x + n_xy]]
    bs_aligned, transform = LighthouseSystemAligner.align(origin, x_axis, xy_plane, solution.bs_poses)
    cf_aligned = [transform.rotate_translate_pose(pose) for pose in cf_poses]
    measured_x = float(np.linalg.norm(cf_aligned[1].translation))
    if x_axis_dist is None:
        return bs_aligned, measured_x, 1.0
    bs_scaled, _, scale = LighthouseSystemScaler.scale_fixed_point(
        bs_aligned, cf_aligned, [x_axis_dist, 0, 0], cf_aligned[1])
    return bs_scaled, measured_x, scale


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #

def print_pose(bs_id: int, pose: Pose) -> None:
    R = pose.rot_matrix
    boresight = R[:, 0]
    tilt_down = math.degrees(math.asin(max(-1.0, min(1.0, -boresight[2]))))
    heading = math.degrees(math.atan2(boresight[1], boresight[0]))
    print(f"BS{bs_id}: origin [{pose.translation[0]:+.4f}, {pose.translation[1]:+.4f}, {pose.translation[2]:+.4f}] m"
          f"   boresight tilt-down {tilt_down:5.1f} deg, heading {heading:+6.1f} deg")
    for row in range(3):
        print(f"       R[{row}] = [{R[row, 0]:+.6f}, {R[row, 1]:+.6f}, {R[row, 2]:+.6f}]")


def compare_truth(path: str, poses: dict[int, Pose]) -> None:
    with open(path, encoding="utf-8") as source:
        truth = json.load(source)
    for bs_id, pose in sorted(poses.items()):
        expected = truth[str(bs_id)]
        position_error = np.linalg.norm(pose.translation - np.asarray(expected["origin"]))
        relative = Rotation.from_matrix(pose.rot_matrix.T @ np.asarray(expected["R"]))
        print(f"Truth check BS{bs_id}: position error {position_error * 1000:.1f} mm, "
              f"rotation error {math.degrees(relative.magnitude()):.2f} deg")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="session directory from capture_lh2.py, or a sweep JSON file")
    parser.add_argument("--sensor-positions", default=str(Path(__file__).with_name("wand_sensors.json")),
                        metavar="SENSORS.JSON",
                        help="drone sensor board: 4 x [x, y, z] metres in firmware order S0..S3 — sets the "
                             "metric scale (default: wand_sensors.json, the measured 40 mm board)")
    parser.add_argument("-o", "--output", default="lighthouse_geometry_candidate.yaml")
    parser.add_argument("--auto-frame", action="store_true",
                        help="no static captures: BS0 at (0, 0, --height), +X toward BS1, up = -mean boresight")
    parser.add_argument("--height", type=float, default=3.45,
                        help="--auto-frame: BS0 height above the floor [m] (default 3.45)")
    parser.add_argument("--board-height", type=float, default=0.0,
                        help="height of the sensor plane above the floor while the drone sits on it [m]; "
                             "added to Z so that Z = 0 is the floor (default 0)")
    parser.add_argument("--baseline", type=float,
                        help="tape-measured BS0-BS1 distance [m]; comparison only, never used in the fit")
    parser.add_argument("--origin", metavar="JSON", help="static capture at the world origin")
    parser.add_argument("--x-axis", metavar="JSON", nargs="+", default=[],
                        help="static capture(s) on the +X axis")
    parser.add_argument("--xy-plane", metavar="JSON", nargs="+", default=[],
                        help="static captures on the floor (Z = 0)")
    parser.add_argument("--x-axis-dist", type=float,
                        help="distance [m] of the x-axis capture from the origin (default: from the session, "
                             "else 1.0 as in Bitcraze)")
    parser.add_argument("--keep-sensor-scale", action="store_true",
                        help="do not rescale to --x-axis-dist; keep the scale from the sensor spacing")
    parser.add_argument("--max-angle-age-ms", type=int, default=100)
    parser.add_argument("--max-angle-skew-ms", type=int, default=50)
    parser.add_argument("--max-sample-error", type=float, default=0.02,
                        help="drop sweep samples whose RMS residual exceeds this [m] (default 0.02)")
    parser.add_argument("--min-wand-distance", type=float, default=0.5,
                        help="closest the wand ever gets to a station [m]; sets the wand-span filter (default 0.5)")
    parser.add_argument("--max-iter", type=int, default=2000)
    parser.add_argument("--minimum-samples", type=int, default=30)
    parser.add_argument("--truth", help="ground-truth JSON from make_synthetic_measurements.py")
    args = parser.parse_args()

    sweep_path = args.input
    if Path(args.input).is_dir():
        session_dir = Path(args.input)
        manifest_path = session_dir / "session.json"
        if not manifest_path.is_file():
            parser.error(f"{session_dir} has no session.json — is it a capture_lh2.py wizard session?")
        with open(manifest_path, encoding="utf-8") as stream:
            manifest = json.load(stream)
        sweep_path = str(session_dir / manifest["sweep"])
        args.origin = str(session_dir / manifest["origin"])
        args.x_axis = [str(session_dir / name) for name in manifest["x_axis"]]
        args.xy_plane = [str(session_dir / name) for name in manifest["xy_plane"]]
        if args.x_axis_dist is None:
            args.x_axis_dist = float(manifest.get("x_axis_dist", 1.0))
    if args.x_axis_dist is None:
        args.x_axis_dist = 1.0

    reference_mode = bool(args.origin or args.x_axis or args.xy_plane)
    if reference_mode and args.auto_frame:
        parser.error("--auto-frame cannot be combined with static captures")
    if reference_mode and not (args.origin and args.x_axis and args.xy_plane):
        parser.error("the wizard frame needs all of --origin, --x-axis and --xy-plane")
    if not reference_mode and not args.auto_frame:
        parser.error("no static captures: record a wizard session with capture_lh2.py, pass "
                     "--origin/--x-axis/--xy-plane, or use --auto-frame --height H")

    sensors = load_sensor_positions(args.sensor_positions)
    print(f"Wand layout ({args.sensor_positions}): {describe_sensor_positions(sensors)}")

    records, age_rejected = filter_by_age(load_records(sweep_path),
                                          args.max_angle_age_ms, args.max_angle_skew_ms)
    timestamps = len({float(record["timestamp"]) for record in records})
    records, span_rejected = filter_by_wand_span(records, sensors, args.min_wand_distance)
    print(f"Wand-span filter dropped {span_rejected}/{timestamps} snapshots whose sensors spread wider "
          f"than the wand can at {args.min_wand_distance} m")
    if span_rejected > 0.2 * timestamps:
        print("Warning: many snapshots have physically impossible sensor spreads — the per-sensor angles are "
              "inconsistent (decoder problem), not just noisy. Fix that before trusting any calibration.")
    sweep = LighthouseSampleMatcher.match(to_measurements(records), max_time_diff=0.020, min_nr_of_bs_in_match=2)
    print(f"Sweep samples with both stations: {len(sweep)} (age filter dropped {age_rejected})")
    if len(sweep) < args.minimum_samples:
        print(f"Error: need at least {args.minimum_samples} samples", file=sys.stderr)
        return 1

    references: list[LhCfPoseSample] = []
    if reference_mode:
        references = ([average_static_capture(args.origin)]
                      + [average_static_capture(path) for path in args.x_axis]
                      + [average_static_capture(path) for path in args.xy_plane])

    total = len(references) + len(sweep)
    solution, used, stats = solve(
        references + sweep, sensors, len(references), args.max_sample_error, args.max_iter)

    if set(solution.bs_poses) != {0, 1}:
        print(f"Error: solver returned stations {sorted(solution.bs_poses)}, expected 0 and 1", file=sys.stderr)
        return 1
    errors = solution.error_info
    print(f"IPPE seed kept {stats['ippe_kept']}/{total}; final solve used {len(used)} "
          f"({stats['rejected']} residual outliers dropped)")
    print(f"Residual: mean {errors['mean_error'] * 1000:.2f} mm, max {errors['max_error'] * 1000:.2f} mm")
    print(f"Wand shape check: median rigid-fit error {stats['median_rigid_error'] * 1000:.2f} mm")
    if not solution.success:
        print("Warning: optimizer hit its iteration limit — inspect residuals before trusting the result")
    if stats["median_rigid_error"] > 0.005 or len(used) < 0.5 * total:
        print(f"Error: the measured sensors do not form the wand in {args.sensor_positions}. "
              "Check that rows are in firmware order S0..S3 and the coordinates match the board.",
              file=sys.stderr)
        return 1

    estimated_baseline = float(np.linalg.norm(solution.bs_poses[1].translation - solution.bs_poses[0].translation))

    if reference_mode:
        world, measured_x, scale = bitcraze_frame(
            solution, len(args.x_axis), len(args.xy_plane), None if args.keep_sensor_scale else args.x_axis_dist)
        print(f"Wizard frame: x-axis capture is {measured_x:.4f} m from the origin by the sensor-spacing scale "
              f"(marked at {args.x_axis_dist:.4f} m)")
        sensor_scale_ratio = args.x_axis_dist / measured_x
        if abs(sensor_scale_ratio - 1.0) > 0.05:
            print(f"Warning: sensor-spacing scale and the x-axis mark disagree by {sensor_scale_ratio:.3f}x. "
                  "Either the x-axis mark is not at that distance from the origin mark, or the sensor "
                  f"spacing in {args.sensor_positions} is off (40 mm would really be {40 * sensor_scale_ratio:.1f} mm).")
        if not args.keep_sensor_scale:
            print(f"Rescaled by {scale:.4f} to put the x-axis capture at {args.x_axis_dist:.4f} m (Bitcraze)")
            estimated_baseline *= scale
        if args.board_height:
            world = {bs_id: Pose(R_matrix=pose.rot_matrix, t_vec=pose.translation + [0.0, 0.0, args.board_height])
                     for bs_id, pose in world.items()}
    else:
        world = auto_frame(solution.bs_poses, args.height)

    print(f"Estimated baseline: {estimated_baseline:.4f} m")
    if args.baseline is None:
        print("Note: no --baseline given — a wrong wand scale would go undetected (it does not raise the residual)")
    else:
        ratio = args.baseline / estimated_baseline
        print(f"Tape baseline {args.baseline:.4f} m -> ratio tape/estimate = {ratio:.4f}")
        if abs(ratio - 1.0) > 0.03:
            source = ("the x-axis mark distance" if reference_mode and not args.keep_sensor_scale
                      else f"the sensor spacing in {args.sensor_positions}")
            print(f"Warning: scale is off by more than 3 % — check {source}, and the tape measurement.")

    for bs_id in sorted(world):
        print_pose(bs_id, world[bs_id])
    if args.truth:
        compare_truth(args.truth, world)

    import yaml
    output = {
        "type": "lighthouse_system_configuration",
        "version": "1",
        "systemType": 2,
        "geos": {bs_id: {"position": pose.translation.tolist(),
                         "rotation_quat": Rotation.from_matrix(pose.rot_matrix).as_quat().tolist()}
                 for bs_id, pose in sorted(world.items())},
        "calibs": {},
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, sort_keys=False)
    print(f"Wrote candidate geometry: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
