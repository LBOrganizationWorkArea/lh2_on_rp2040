#!/usr/bin/env python3
"""Fit Lighthouse v2 sweep-angle coefficients from synchronized serial logs."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import defaultdict
import json
import math
from pathlib import Path

import numpy as np

TAN_30 = math.tan(math.pi / 6.0)


def expected_sweep_angles(point_world: np.ndarray, origin: np.ndarray,
                          rotation: np.ndarray) -> tuple[float, float] | None:
    """Return expected sweep 0/1 angles in degrees for a world point."""
    local = rotation.T @ (point_world - origin)
    if local[0] <= 0.0:
        return None

    horizontal = math.atan2(local[1], local[0])
    vertical = math.atan2(local[2], local[0])
    q = math.tan(vertical) / math.sqrt(1.0 + math.tan(horizontal) ** 2)
    arg = q * TAN_30
    if abs(arg) > 1.0:
        return None

    sweep_0 = horizontal + math.asin(-arg)
    sweep_1 = horizontal + math.asin(arg)
    return math.degrees(sweep_0), math.degrees(sweep_1)


def fit_linear_coefficients(samples: list[tuple[int, float]],
                            min_lfsr_span: int = 1000) -> dict[str, float | int]:
    if len(samples) < 10:
        raise ValueError(f"only {len(samples)} paired readings; need at least 10")

    counts = np.asarray([sample[0] for sample in samples], dtype=float)
    angles = np.asarray([sample[1] for sample in samples], dtype=float)
    span = int(np.ptp(counts))
    if span < min_lfsr_span:
        raise ValueError(f"LFSR span {span} is too small; need at least {min_lfsr_span}")

    design = np.column_stack((counts, np.ones_like(counts)))
    slope, intercept = np.linalg.lstsq(design, angles, rcond=None)[0]
    residuals = angles - (slope * counts + intercept)
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    total = float(np.sum((angles - np.mean(angles)) ** 2))
    r_squared = 1.0 - float(np.sum(residuals ** 2)) / total if total > 0 else 1.0
    return {
        "A": float(slope),
        "B": float(intercept),
        "rmse_deg": rmse,
        "r_squared": r_squared,
        "samples": len(samples),
        "lfsr_span": span,
    }


def load_capture(path: Path):
    lfsr_rows = []
    positions: dict[int, list[tuple[int, np.ndarray]]] = defaultdict(list)
    with path.open(encoding="utf-8", errors="replace") as source:
        for line_number, line in enumerate(source, 1):
            fields = [field.strip() for field in line.strip().split(",")]
            try:
                if fields[0] == "L" and len(fields) >= 7:
                    _, sensor, bs_id, sweep, _poly, lfsr, timestamp = fields[:7]
                    lfsr_rows.append((int(sensor), int(bs_id), int(sweep),
                                      int(lfsr), int(timestamp)))
                elif fields[0] == "Q" and len(fields) >= 6:
                    _, timestamp, sensor, x, y, z = fields[:6]
                    positions[int(sensor)].append(
                        (int(timestamp), np.asarray((float(x), float(y), float(z))))
                    )
            except (ValueError, IndexError) as error:
                raise ValueError(f"invalid L/Q record at line {line_number}: {error}") from error

    for samples in positions.values():
        samples.sort(key=lambda sample: sample[0])
    return lfsr_rows, positions


def pair_reference_position(sensor: int, timestamp_us: int,
                            positions: dict[int, list[tuple[int, np.ndarray]]],
                            max_skew_us: int):
    samples = positions.get(sensor, [])
    if not samples:
        return None
    times = [sample[0] for sample in samples]
    index = bisect_left(times, timestamp_us)
    candidates = samples[max(0, index - 1):min(len(samples), index + 1)]
    nearest = min(candidates, key=lambda sample: abs(sample[0] - timestamp_us))
    return nearest[1] if abs(nearest[0] - timestamp_us) <= max_skew_us else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="serial capture containing L and Q records")
    parser.add_argument("--bs-id", type=int, choices=(0, 1), required=True)
    parser.add_argument("--pose", type=Path, required=True,
                        help="JSON with origin [x,y,z] and rotation matrix R from the pose editor")
    parser.add_argument("--max-pair-skew-ms", type=float, default=125.0)
    parser.add_argument("--min-lfsr-span", type=int, default=1000)
    args = parser.parse_args()

    with args.pose.open(encoding="utf-8") as source:
        pose = json.load(source)
    origin = np.asarray(pose["origin"], dtype=float)
    rotation = np.asarray(pose["R"], dtype=float)
    if origin.shape != (3,) or rotation.shape != (3, 3):
        parser.error("pose JSON requires origin[3] and R[3][3]")
    if not np.isfinite(origin).all() or not np.isfinite(rotation).all():
        parser.error("pose values must be finite")

    lfsr_rows, positions = load_capture(args.capture)
    max_skew_us = int(args.max_pair_skew_ms * 1000.0)
    samples_by_sweep: dict[int, list[tuple[int, float]]] = {0: [], 1: []}
    unmatched = 0
    outside_fov = 0
    for sensor, bs_id, sweep, lfsr, timestamp in lfsr_rows:
        if bs_id != args.bs_id or sweep not in samples_by_sweep:
            continue
        point = pair_reference_position(sensor, timestamp, positions, max_skew_us)
        if point is None:
            unmatched += 1
            continue
        angles = expected_sweep_angles(point, origin, rotation)
        if angles is None:
            outside_fov += 1
            continue
        samples_by_sweep[sweep].append((lfsr, angles[sweep]))

    print(f"BS{args.bs_id}: unmatched L readings: {unmatched}; outside FOV/behind: {outside_fov}")
    for sweep in (0, 1):
        fit = fit_linear_coefficients(samples_by_sweep[sweep], args.min_lfsr_span)
        print(f"sweep {sweep}: n={fit['samples']}, LFSR span={fit['lfsr_span']}, "
              f"RMSE={fit['rmse_deg']:.4f} deg, R2={fit['r_squared']:.6f}")
        print(f"CAL_BS{args.bs_id}_A{sweep} {fit['A']:.10f}f")
        print(f"CAL_BS{args.bs_id}_B{sweep} {fit['B']:.6f}f")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())