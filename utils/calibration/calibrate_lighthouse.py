#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
  _     _    _ __  __ _____ _   _  ____  _    _  _____   ____  ______ ______  _____ 
 | |   | |  | |  \/  |_   _| \ | |/ __ \| |  | |/ ____| |  _ \|  ____|  ____|/ ____|
 | |   | |  | | \  / | | | |  \| | |  | | |  | | (___   | |_) | |__  | |__  | (___  
 | |   | |  | | |\/| | | | | . ` | |  | | |  | |\___ \  |  _ <|  __| |  __|  \___ \ 
 | |___| |__| | |  | |_| |_| |\  | |__| | |__| |____) | | |_) | |____| |____ ____) |
 |______\____/|_|  |_|_____|_| \_|\____/ \____/|_____/  |____/|______|______|_____/ 
                                                                                    
Giorgio Rinolfi
Victor Bianchi
Antoine el Kahi
Eduardo Gonzalez

Lighthouse calibration CLI - reuses the calibration pipeline from the wizard.

This script collects lighthouse measurements and produces calibrated base station
geometry through the same pipeline as the Qt-based calibration wizard, but as a
standalone command-line tool.

Workflow:
1. Collect/load measurements
2. Match them into pose samples
3. Build initial geometry guess
4. Solve geometry (least squares optimization)
5. Align to world frame
6. Scale using reference measurement
7. Save base station geometry
"""



from __future__ import annotations

import argparse
from collections import deque
import json
import socket
import struct
import time
from typing import TYPE_CHECKING, Optional

from pymavlink.dialects.v20 import common

if TYPE_CHECKING:
    from utils.angle_lib.lighthouse_types import LhCfPoseSample, Pose

REFERENCE_DIST = 1.0
LH2_TUNNEL_TYPE = 0x4C48
LH2_TUNNEL_DATA_LEN = 94


def collect_udp_measurements(endpoint: str,
                            duration: Optional[float] = None,
                            heartbeat_endpoint: Optional[str] = None) -> list[dict]:
    """Collect raw LH2 angle snapshots from MAVLink TUNNEL over UDP."""
    host, port_text = endpoint.rsplit(":", 1)
    mavlink = common.MAVLink(None, srcSystem=255, srcComponent=190)
    parser = common.MAVLink(None)
    measurements = []
    seen_sequences = set()
    tunnel_packets = 0
    crc_errors = 0
    byte_window = deque(maxlen=145)
    deadline = None if duration is None else time.monotonic() + duration
    heartbeat_target = None
    if heartbeat_endpoint:
        heartbeat_host, heartbeat_port = heartbeat_endpoint.rsplit(":", 1)
        heartbeat_target = (heartbeat_host, int(heartbeat_port))

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, int(port_text)))
        sock.settimeout(0.5)
        print(f"Listening for LH2 TUNNEL packets on {host}:{port_text}; Ctrl-C to stop")
        if heartbeat_target:
            print(f"Sending GCS heartbeat to {heartbeat_target[0]}:{heartbeat_target[1]}")
        last_heartbeat = 0.0
        while deadline is None or time.monotonic() < deadline:
            now = time.monotonic()
            if heartbeat_target and now - last_heartbeat >= 1.0:
                heartbeat = common.MAVLink_heartbeat_message(
                    common.MAV_TYPE_GCS,
                    common.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    common.MAV_STATE_ACTIVE,
                    3,
                )
                sock.sendto(heartbeat.pack(mavlink), heartbeat_target)
                last_heartbeat = now
            try:
                packet, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                print("Capture stopped")
                break

            for byte in packet:
                byte_window.append(byte)
                try:
                    message = parser.parse_char(bytes((byte,)))
                except common.MAVError as error:
                    crc_errors += 1
                    if crc_errors == 1:
                        print(f"MAVLink parse error: {error}")
                        print(f"Frame bytes: {bytes(byte_window).hex()}")
                    continue
                if message is None or message.get_type() != "TUNNEL":
                    continue
                if message.payload_type != LH2_TUNNEL_TYPE or message.payload_length < LH2_TUNNEL_DATA_LEN:
                    continue

                data = bytes(message.payload[:message.payload_length])
                if len(data) < LH2_TUNNEL_DATA_LEN or data[:2] != b"LH" or data[2] != 2:
                    continue
                sequence, timestamp_us, valid_mask = struct.unpack_from("<HQB", data, 3)
                if sequence in seen_sequences:
                    continue
                seen_sequences.add(sequence)
                tunnel_packets += 1
                if tunnel_packets == 1 or tunnel_packets % 50 == 0:
                    print(f"Received {tunnel_packets} LH2 TUNNEL packets; valid_mask=0x{valid_mask:02x}")
                angles = struct.unpack_from("<16f", data, 14)
                ages_ms = struct.unpack_from("<8H", data, 78)

                timestamp = timestamp_us / 1_000_000.0
                for base_station_id in range(2):
                    required = sum(1 << (sensor * 2 + base_station_id) for sensor in range(4))
                    if valid_mask & required != required:
                        continue
                    measurements.append({
                        "timestamp": timestamp,
                        "base_station_id": base_station_id,
                        "angles": [
                            [angles[(sensor * 2 + base_station_id) * 2],
                             angles[(sensor * 2 + base_station_id) * 2 + 1]]
                            for sensor in range(4)
                        ],
                        "angle_ages_ms": [ages_ms[sensor * 2 + base_station_id]
                                          for sensor in range(4)],
                    })
                if len(measurements) % 20 == 0:
                    print(f"Collected {len(measurements)} base-station measurements")

    print(f"Received {tunnel_packets} LH2 TUNNEL packets, {crc_errors} MAVLink parse errors; collected {len(measurements)} complete base-station measurements")
    return measurements


def _capture_cli() -> None:
    parser = argparse.ArgumentParser(description="Collect LH2 calibration angles over MAVLink UDP")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--udp", metavar="HOST:PORT", help="UDP bind endpoint, e.g. 0.0.0.0:14550")
    mode.add_argument("--solve", metavar="MEASUREMENTS.JSON", help="solve lighthouse geometry from a capture")
    mode.add_argument("--baseline-only", metavar="MEASUREMENTS.JSON",
                      help="estimate only the relative base-station distance; do not fit/export geometry")
    parser.add_argument("--heartbeat-to", metavar="HOST:PORT",
                        help="send GCS heartbeat to MavESP, e.g. 192.168.4.1:14555")
    parser.add_argument("-o", "--output", default="measurements.json")
    parser.add_argument("--seconds", type=float, help="stop after this many seconds")
    parser.add_argument("--sensor-positions", metavar="WAND.JSON",
                        help="JSON array of the four sensor [x,y,z] positions in metres, in S0..S3 order")
    parser.add_argument("--repo-root", metavar="PATH",
                        help="repository root containing utils/angle_lib (defaults to auto-detection)")
    parser.add_argument("--baseline", type=float,
                        help="optional measured BS4-to-BS10 distance in metres, used only as a comparison")
    parser.add_argument("--height", type=float, default=3.45,
                        help="measured lighthouse height in metres (default: 3.45)")
    parser.add_argument("--geometry-output", default="lighthouse_geometry_candidate.yaml",
                        help="candidate geometry YAML output for --solve")
    parser.add_argument("--minimum-samples", type=int, default=30,
                        help="minimum synchronized two-base-station samples (default: 30)")
    parser.add_argument("--max-angle-age-ms", type=int, default=100,
                        help="maximum age allowed for each angle in a solve (default: 100 ms)")
    parser.add_argument("--max-angle-skew-ms", type=int, default=50,
                        help="maximum age difference across the 8 angles in a solve (default: 50 ms)")
    args = parser.parse_args()
    if args.udp:
        if args.sensor_positions:
            parser.error("--sensor-positions is only used with --solve")
        measurements = collect_udp_measurements(args.udp, args.seconds, args.heartbeat_to)
        with open(args.output, "w", encoding="utf-8") as output:
            json.dump(measurements, output, indent=2)
        print(f"Wrote {len(measurements)} measurements to {args.output}")
        return

    if not args.sensor_positions:
        parser.error("--solve/--baseline-only requires --sensor-positions WAND.JSON")
    measurement_path = args.solve or args.baseline_only
    with open(measurement_path, encoding="utf-8") as source:
        measurements = json.load(source)
    with open(args.sensor_positions, encoding="utf-8") as source:
        sensor_positions = json.load(source)
    measurements, age_report = filter_coherent_measurements(
        measurements, args.max_angle_age_ms, args.max_angle_skew_ms)
    print(f"Age filter: kept {age_report['kept_pairs']} paired snapshots, "
          f"discarded {age_report['rejected_pairs']} stale/incoherent snapshots")

    if args.baseline_only:
        estimated_baseline, relative_vector, sample_count = estimate_relative_baseline(
            measurements, sensor_positions, args.minimum_samples, args.repo_root)
        print(f"Synchronized samples: {sample_count}")
        print(f"Relative BS4-to-BS10 vector: [{relative_vector[0]:.4f}, "
              f"{relative_vector[1]:.4f}, {relative_vector[2]:.4f}] m")
        print(f"Estimated baseline: {estimated_baseline:.4f} m")
        if args.baseline is not None:
            print(f"Difference from reference {args.baseline:.4f} m: "
                  f"{estimated_baseline - args.baseline:+.4f} m")
        return

    geometry, diagnostics = solve_measurements(
        measurements, sensor_positions, args.baseline, args.height, args.minimum_samples,
        args.repo_root)

    import yaml

    output_data = {
        "type": "lighthouse_system_configuration",
        "version": "1",
        "systemType": 2,
        "geos": {bs_id: pose for bs_id, pose in geometry.items()},
        "calibs": {},
    }
    with open(args.geometry_output, "w", encoding="utf-8") as output:
        yaml.safe_dump(output_data, output, sort_keys=False)
    print(f"Matched synchronized samples: {diagnostics['matched_samples']}")
    print(f"Rejected outlier samples: {diagnostics['rejected_samples']}")
    print(f"Selected initial roll: {diagnostics['seed_roll_degrees']} degrees")
    print(f"Initial wand rigid-fit error: {diagnostics['seed_rigid_error']:.4f} m")
    if args.baseline is None:
        print("Measured baseline reference: not provided")
    else:
        print(f"Measured baseline reference: {args.baseline:.4f} m")
    print(f"Estimated baseline from angles: {diagnostics['estimated_baseline']:.4f} m")
    print(f"Mean fit residual: {diagnostics['mean_error']:.6f} m")
    print(f"Maximum fit residual: {diagnostics['max_error']:.6f} m")
    if not diagnostics["optimizer_converged"]:
        print("Warning: optimizer reached its evaluation limit; inspect residuals before using this geometry")
    print(f"Wrote candidate geometry: {args.geometry_output}")


def estimate_relative_baseline(measurements: list[dict], sensor_positions,
                               minimum_samples: int = 30,
                               repo_root: Optional[str] = None):
    """Estimate the metric relative BS0-to-BS1 pose directly from paired samples."""
    import sys
    from pathlib import Path

    import numpy as np

    candidates = []
    if repo_root:
        candidates.append(Path(repo_root).expanduser().resolve())
    candidates.extend(Path(__file__).resolve().parents)
    candidates.extend(Path.cwd().resolve().parents)
    candidates.append(Path.cwd().resolve())
    required = (Path("utils/angle_lib/lighthouse_types.py"),
                Path("utils/calibration/calibration_lib/lighthouse_bs_vector.py"))
    project_root = next((candidate for candidate in candidates
                         if all((candidate / item).is_file() for item in required)), None)
    if project_root is None:
        raise FileNotFoundError("Cannot locate repository modules; pass --repo-root")
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

    from utils.angle_lib.lighthouse_types import LhMeasurement
    from utils.calibration.calibration_lib.lighthouse_bs_vector import LighthouseBsVector, LighthouseBsVectors
    from utils.calibration.calibration_lib.lighthouse_sample_matcher import LighthouseSampleMatcher
    from utils.calibration.calibration_lib.lighthouse_initial_estimator import LighthouseInitialEstimator

    sensor_positions = np.asarray(sensor_positions, dtype=float)
    if sensor_positions.shape != (4, 3) or not np.isfinite(sensor_positions).all():
        raise ValueError("sensor positions must be a finite 4x3 array in S0..S3 order")

    parsed = []
    for index, record in enumerate(measurements):
        try:
            timestamp = float(record["timestamp"])
            bs_id = int(record["base_station_id"])
            angle_pairs = np.asarray(record["angles"], dtype=float)
            if bs_id not in (0, 1) or angle_pairs.shape != (4, 2):
                raise ValueError("expected base station id 0/1 and four angle pairs")
            if not np.isfinite(angle_pairs).all():
                raise ValueError("angles must be finite")
            vectors = LighthouseBsVectors([
                LighthouseBsVector(float(horizontal), float(vertical))
                for horizontal, vertical in angle_pairs
            ])
            parsed.append(LhMeasurement(timestamp, bs_id, vectors))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid measurement at JSON index {index}: {error}") from error

    paired = LighthouseSampleMatcher.match(
        sorted(parsed, key=lambda item: item.timestamp),
        max_time_diff=0.020, min_nr_of_bs_in_match=2)
    if len(paired) < minimum_samples:
        raise ValueError(f"only {len(paired)} synchronized samples; need {minimum_samples}")

    relative_positions = LighthouseInitialEstimator._find_solutions(paired, sensor_positions)
    pair = next((key for key in relative_positions if key.bs1 == 0 and key.bs2 == 1), None)
    if pair is None:
        raise RuntimeError("no relative pose found for BS0 to BS1")
    relative_vector = np.asarray(relative_positions[pair], dtype=float)
    return float(np.linalg.norm(relative_vector)), relative_vector, len(paired)


def filter_coherent_measurements(measurements: list[dict], max_age_ms: int = 100,
                                 max_skew_ms: int = 50) -> tuple[list[dict], dict[str, int]]:
    """Keep timestamp-paired BS records whose eight angle ages are coherent."""
    if max_age_ms < 0 or max_skew_ms < 0:
        raise ValueError("angle age/skew limits must be non-negative milliseconds")
    if not measurements or any("angle_ages_ms" not in item for item in measurements):
        raise ValueError(
            "measurement JSON has no per-angle age metadata; update and flash the TUNNEL-v2 firmware, "
            "then capture a new file before solving"
        )

    grouped: dict[float, dict[int, dict]] = {}
    for item in measurements:
        grouped.setdefault(float(item["timestamp"]), {})[int(item["base_station_id"])] = item

    kept = []
    rejected = 0
    for sample in grouped.values():
        if set(sample) != {0, 1}:
            rejected += 1
            continue
        ages = []
        valid = True
        for bs_id in (0, 1):
            bs_ages = sample[bs_id]["angle_ages_ms"]
            if len(bs_ages) != 4:
                valid = False
                break
            ages.extend(int(age) for age in bs_ages)
        if not valid or max(ages) > max_age_ms or max(ages) - min(ages) > max_skew_ms:
            rejected += 1
            continue
        kept.extend((sample[0], sample[1]))

    return kept, {"kept_pairs": len(kept) // 2, "rejected_pairs": rejected}


def solve_measurements(measurements: list[dict], sensor_positions,
                       baseline: Optional[float] = None, height: float = 3.45,
                       minimum_samples: int = 30,
                       repo_root: Optional[str] = None) -> tuple[dict[int, dict], dict[str, float]]:
    """Fit two LH2 base stations from synchronized four-sensor angle records."""
    import math
    import sys
    from pathlib import Path

    import numpy as np
    from scipy.spatial.transform import Rotation

    required_files = (
        Path("utils/angle_lib/lighthouse_types.py"),
        Path("utils/calibration/calibration_lib/lighthouse_bs_vector.py"),
    )
    candidates = []
    if repo_root:
        candidates.append(Path(repo_root).expanduser().resolve())
    candidates.extend(Path(__file__).resolve().parents)
    candidates.extend(Path.cwd().resolve().parents)
    candidates.append(Path.cwd().resolve())
    project_root = next(
        (candidate for candidate in candidates
         if all((candidate / required_file).is_file() for required_file in required_files)),
        None,
    )
    if project_root is None:
        expected = Path(repo_root).expanduser().resolve() if repo_root else Path(__file__).resolve().parents[2]
        raise FileNotFoundError(
            "Could not locate the repository calibration modules. Run the repository copy of "
            "calibrate_lighthouse.py or pass --repo-root PATH to the repository root "
            f"(expected to contain utils/angle_lib/lighthouse_types.py); checked {expected}."
        )
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

    from utils.angle_lib.lighthouse_types import LhBsCfPoses, LhCfPoseSample, LhMeasurement, Pose
    from utils.calibration.calibration_lib.lighthouse_bs_vector import LighthouseBsVector, LighthouseBsVectors
    from utils.calibration.calibration_lib.lighthouse_sample_matcher import LighthouseSampleMatcher
    from utils.calibration.calibration_lib.lighthouse_initial_estimator import LighthouseInitialEstimator
    from utils.calibration.calibration_lib.lighthouse_geometry_solver import LighthouseGeometrySolver

    sensor_positions = np.asarray(sensor_positions, dtype=float)
    if sensor_positions.shape != (4, 3) or not np.isfinite(sensor_positions).all():
        raise ValueError("sensor positions must be a finite 4x3 array in S0..S3 order")
    if np.linalg.matrix_rank(sensor_positions - sensor_positions.mean(axis=0)) < 2:
        raise ValueError("the four wand sensors must not be collinear")
    if baseline is not None and (not math.isfinite(baseline) or baseline <= 0):
        raise ValueError("baseline must be a positive finite distance in metres")
    if not math.isfinite(height) or height <= 0:
        raise ValueError("height must be a positive finite distance in metres")

    parsed_measurements = []
    for index, record in enumerate(measurements):
        try:
            timestamp = float(record["timestamp"])
            bs_id = int(record["base_station_id"])
            angle_pairs = record["angles"]
            if bs_id not in (0, 1) or len(angle_pairs) != 4 or any(len(pair) != 2 for pair in angle_pairs):
                raise ValueError("expected base_station_id 0/1 and four [horizontal, vertical] pairs")
            angle_values = np.asarray(angle_pairs, dtype=float)
            if not math.isfinite(timestamp) or not np.isfinite(angle_values).all():
                raise ValueError("timestamp and angles must be finite")
            vectors = LighthouseBsVectors([
                LighthouseBsVector(float(horizontal), float(vertical))
                for horizontal, vertical in angle_values
            ])
            parsed_measurements.append(LhMeasurement(timestamp, bs_id, vectors))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid measurement at JSON index {index}: {error}") from error

    parsed_measurements.sort(key=lambda item: item.timestamp)
    matched_samples = LighthouseSampleMatcher.match(
        parsed_measurements, max_time_diff=0.020, min_nr_of_bs_in_match=2)
    if len(matched_samples) < minimum_samples:
        raise ValueError(
            f"only {len(matched_samples)} synchronized samples contain both base stations; "
            f"need at least {minimum_samples}. Capture more packets with valid_mask=0xff."
        )

    if baseline is None:
        relative_positions = LighthouseInitialEstimator._find_solutions(matched_samples, sensor_positions)
        relative_pair = next((key for key in relative_positions if key.bs1 == 0 and key.bs2 == 1), None)
        if relative_pair is None:
            raise RuntimeError("cannot initialize baseline from paired angle samples")
        initial_baseline = float(np.linalg.norm(relative_positions[relative_pair]))
    else:
        initial_baseline = baseline
    if not math.isfinite(initial_baseline) or initial_baseline <= 0:
        raise RuntimeError("could not estimate a positive base-station baseline")

    nominal_rotation = np.array([
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
    ])
    roll_candidates = []
    for roll_degrees in (0, 90, 180, 270):
        roll_rotation = Rotation.from_rotvec((math.radians(roll_degrees), 0.0, 0.0)).as_matrix()
        initial_rotation = nominal_rotation @ roll_rotation
        initial_bs_world = {
            0: Pose(R_matrix=initial_rotation, t_vec=(0.0, 0.0, height)),
            1: Pose(R_matrix=initial_rotation, t_vec=(initial_baseline, 0.0, height)),
        }
        candidate_samples = []
        candidate_wand_poses = []
        rigid_errors = []

        for sample in matched_samples:
            measured_points = []
            sample_valid = True
            for sensor_index in range(4):
                ray_origins = []
                ray_directions = []
                for bs_id in (0, 1):
                    vector = sample.angles_calibrated[bs_id][sensor_index]
                    direction = initial_bs_world[bs_id].rot_matrix @ vector.cart
                    direction /= np.linalg.norm(direction)
                    ray_origins.append(initial_bs_world[bs_id].translation)
                    ray_directions.append(direction)

                first_direction, second_direction = ray_directions
                direction_dot = float(np.dot(first_direction, second_direction))
                denominator = 1.0 - direction_dot * direction_dot
                if denominator < 1e-6:
                    sample_valid = False
                    break

                origin_delta = ray_origins[0] - ray_origins[1]
                first_distance = (
                    direction_dot * np.dot(second_direction, origin_delta)
                    - np.dot(first_direction, origin_delta)
                ) / denominator
                second_distance = (
                    np.dot(second_direction, origin_delta)
                    - direction_dot * np.dot(first_direction, origin_delta)
                ) / denominator
                if first_distance <= 0.0 or second_distance <= 0.0:
                    sample_valid = False
                    break

                first_point = ray_origins[0] + first_distance * first_direction
                second_point = ray_origins[1] + second_distance * second_direction
                measured_points.append((first_point + second_point) * 0.5)

            if not sample_valid:
                continue

            measured_points = np.asarray(measured_points)
            sensor_center = sensor_positions.mean(axis=0)
            point_center = measured_points.mean(axis=0)
            left, _, right = np.linalg.svd(
                (sensor_positions - sensor_center).T @ (measured_points - point_center)
            )
            correction = np.eye(3)
            correction[2, 2] = np.linalg.det(right.T @ left.T)
            wand_rotation = right.T @ correction @ left.T
            wand_translation = point_center - wand_rotation @ sensor_center
            fitted_points = (wand_rotation @ sensor_positions.T).T + wand_translation
            rigid_errors.append(float(np.sqrt(np.mean(np.sum((fitted_points - measured_points) ** 2, axis=1)))) )
            candidate_wand_poses.append(Pose(R_matrix=wand_rotation, t_vec=wand_translation))
            candidate_samples.append(sample)

        median_rigid_error = float(np.median(rigid_errors)) if rigid_errors else math.inf
        roll_candidates.append((len(candidate_samples), -median_rigid_error, roll_degrees,
                                initial_rotation, initial_bs_world, candidate_samples,
                                candidate_wand_poses, median_rigid_error))

    _, _, selected_roll, initial_rotation, initial_bs_world, seeded_samples, wand_poses_world, seed_error = max(
        roll_candidates, key=lambda candidate: (candidate[0], candidate[1]))

    if len(seeded_samples) < minimum_samples:
        raise ValueError(
            f"only {len(seeded_samples)} samples could be triangulated with the initial lighthouse geometry; "
            "check the sensor order, sensor coordinates, and initial base-station pose"
        )

    first_wand = wand_poses_world[0]
    world_to_reference = first_wand.rot_matrix.T
    reference_bs_poses = {
        bs_id: Pose(
            R_matrix=world_to_reference @ pose.rot_matrix,
            t_vec=world_to_reference @ (pose.translation - first_wand.translation),
        )
        for bs_id, pose in initial_bs_world.items()
    }
    reference_wand_poses = [
        Pose(
            R_matrix=world_to_reference @ pose.rot_matrix,
            t_vec=world_to_reference @ (pose.translation - first_wand.translation),
        )
        for pose in wand_poses_world
    ]
    reference_wand_poses[0] = Pose()
    initial_guess = LhBsCfPoses(reference_bs_poses, reference_wand_poses)

    bootstrap_samples = list(seeded_samples)
    bootstrap_wand_poses = list(wand_poses_world)
    retained_indices = list(range(len(bootstrap_samples)))
    solution = LighthouseGeometrySolver.solve(
        initial_guess, seeded_samples, sensor_positions, max_nr_iter=500)
    rejected_samples = 0
    for _ in range(3):
        sample_errors = np.asarray([
            max(errors.values()) for errors in solution.estimated_errors
        ])
        inlier_indices = np.flatnonzero(sample_errors <= 0.02)
        if len(inlier_indices) == len(seeded_samples):
            break
        if len(inlier_indices) < minimum_samples:
            break

        rejected_samples += len(seeded_samples) - len(inlier_indices)
        retained_indices = [retained_indices[index] for index in inlier_indices]
        seeded_samples = [bootstrap_samples[index] for index in retained_indices]
        retained_poses = [bootstrap_wand_poses[index] for index in retained_indices]
        first_pose = retained_poses[0]
        reference_rotation = first_pose.rot_matrix.T
        refined_bs_poses = {
            bs_id: Pose(
                R_matrix=reference_rotation @ pose.rot_matrix,
                t_vec=reference_rotation @ (pose.translation - first_pose.translation),
            )
            for bs_id, pose in initial_bs_world.items()
        }
        refined_wand_poses = [
            Pose(
                R_matrix=reference_rotation @ pose.rot_matrix,
                t_vec=reference_rotation @ (pose.translation - first_pose.translation),
            )
            for pose in retained_poses
        ]
        refined_wand_poses[0] = Pose()
        initial_guess = LhBsCfPoses(refined_bs_poses, refined_wand_poses)
        solution = LighthouseGeometrySolver.solve(
            initial_guess, seeded_samples, sensor_positions, max_nr_iter=500)

    if len(solution.bs_poses) != 2:
        raise RuntimeError("geometry solver did not produce exactly two base-station poses")
    mean_error = float(solution.error_info["mean_error"])
    max_error = float(solution.error_info["max_error"])
    if not math.isfinite(max_error) or max_error > 0.02:
        raise RuntimeError(
            f"geometry fit residual is too high (mean {mean_error:.4f} m, "
            f"max {max_error:.4f} m after rejecting {rejected_samples} outliers; "
            f"seed roll {selected_roll} degrees. Verify S0..S3 coordinates/order and recapture with the wand "
            "held still at each pose before moving to the next one."
        )

    bs0 = solution.bs_poses[0]
    bs1 = solution.bs_poses[1]
    baseline_vector = bs1.translation - bs0.translation
    estimated_baseline = float(np.linalg.norm(baseline_vector))
    if estimated_baseline < 1e-6:
        raise RuntimeError("solver returned coincident base stations")
    baseline_relative_error = None if baseline is None else abs(estimated_baseline - baseline) / baseline
    if baseline_relative_error is not None and baseline_relative_error > 0.10:
        raise RuntimeError(
            f"estimated baseline {estimated_baseline:.3f} m differs from the measured "
            f"{baseline:.3f} m by {baseline_relative_error:.0%}; collect stationary wand samples "
            "and verify sensor positions/order before accepting the fit"
        )
    source_up = -np.mean([pose.rot_matrix[:, 0] for pose in solution.bs_poses.values()], axis=0)
    source_up_norm = float(np.linalg.norm(source_up))
    if source_up_norm < 1e-4:
        raise RuntimeError("could not define world up from the two lighthouse boresights")
    source_up /= source_up_norm
    source_x = baseline_vector - np.dot(baseline_vector, source_up) * source_up
    source_x_norm = float(np.linalg.norm(source_x))
    if source_x_norm < 1e-4:
        raise RuntimeError("lighthouse baseline is parallel to the estimated world up axis")
    source_x /= source_x_norm
    source_y = np.cross(source_up, source_x)
    source_y /= np.linalg.norm(source_y)
    source_up = np.cross(source_x, source_y)
    source_basis = np.column_stack((source_x, source_y, source_up))
    rotation_to_world = source_basis.T
    target_bs0 = np.array((0.0, 0.0, height))

    result = {}
    for bs_id, pose in solution.bs_poses.items():
        translation = target_bs0 + rotation_to_world @ (pose.translation - bs0.translation)
        rotation = rotation_to_world @ pose.rot_matrix
        result[bs_id] = {
            "position": translation.tolist(),
            "rotation_quat": Rotation.from_matrix(rotation).as_quat().tolist(),
        }

    diagnostics = {
        "matched_samples": len(seeded_samples),
        "mean_error": mean_error,
        "max_error": max_error,
        "estimated_baseline": estimated_baseline,
        "baseline_reference": baseline,
        "optimizer_converged": bool(solution.success),
        "seed_roll_degrees": selected_roll,
        "seed_rigid_error": seed_error,
        "rejected_samples": rejected_samples,
    }
    return result, diagnostics


class EstimateGeometryThread():
    def __init__(self, origin, x_axis, xy_plane, samples):
        super(EstimateGeometryThread, self).__init__()

        self.origin = origin
        self.x_axis = x_axis
        self.xy_plane = xy_plane
        self.samples = samples
        self.bs_poses = {}

    def run(self):
        try:
            self.bs_poses = self._estimate_geometry(self.origin, self.x_axis, self.xy_plane, self.samples)
            self.finished.emit()
        except Exception as ex:
            print(ex)
            self.failed.emit()

    def get_poses(self):
        return self.bs_poses

    def _estimate_geometry(self, origin: LhCfPoseSample,
                           x_axis: list[LhCfPoseSample],
                           xy_plane: list[LhCfPoseSample],
                           samples: list[LhCfPoseSample]) -> dict[int, Pose]:
        """Estimate the geometry of the system based on samples recorded by a Crazyflie"""
        from utils.angle_lib.lighthouse_types import LhDeck4SensorPositions
        from calibration_lib.lighthouse_sample_matcher import LighthouseSampleMatcher
        from calibration_lib.lighthouse_initial_estimator import LighthouseInitialEstimator
        from calibration_lib.lighthouse_geometry_solver import LighthouseGeometrySolver
        from calibration_lib.lighthouse_system_aligner import LighthouseSystemAligner
        from calibration_lib.lighthouse_system_scaler import LighthouseSystemScaler

        matched_samples = [origin] + x_axis + xy_plane + LighthouseSampleMatcher.match(samples, min_nr_of_bs_in_match=2)
        initial_guess, cleaned_matched_samples = LighthouseInitialEstimator.estimate(matched_samples,
                                                                                     LhDeck4SensorPositions.positions)

        solution = LighthouseGeometrySolver.solve(initial_guess,
                                                  cleaned_matched_samples,
                                                  LhDeck4SensorPositions.positions)
        if not solution.success:
            raise Exception("No lighthouse base station geometry solution could be found!")

        start_x_axis = 1
        start_xy_plane = 1 + len(x_axis)
        origin_pos = solution.cf_poses[0].translation
        x_axis_poses = solution.cf_poses[start_x_axis:start_x_axis + len(x_axis)]
        x_axis_pos = list(map(lambda x: x.translation, x_axis_poses))
        xy_plane_poses = solution.cf_poses[start_xy_plane:start_xy_plane + len(xy_plane)]
        xy_plane_pos = list(map(lambda x: x.translation, xy_plane_poses))

        # Align the solution
        bs_aligned_poses, transformation = LighthouseSystemAligner.align(
            origin_pos, x_axis_pos, xy_plane_pos, solution.bs_poses)

        cf_aligned_poses = list(map(transformation.rotate_translate_pose, solution.cf_poses))

        # Scale the solution
        bs_scaled_poses, cf_scaled_poses, scale = LighthouseSystemScaler.scale_fixed_point(bs_aligned_poses,
                                                                                           cf_aligned_poses,
                                                                                           [REFERENCE_DIST, 0, 0],
                                                                                           cf_aligned_poses[1])

        return bs_scaled_poses


if __name__ == "__main__":
    _capture_cli()

