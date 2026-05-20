#!/usr/bin/env python3
"""Helpers for parsing and converting raw Lighthouse 2.0 LFSR observations.

The Pico firmware is intentionally kept simple: it sends decoded polynomial and
LFSR location values. The PC side owns the interpretation and geometry.
"""

from __future__ import annotations

import csv
import math
import re
import time
from collections import defaultdict
from pathlib import Path


DEFAULT_BS_POLYS = {
    4: (8, 9),
    10: (20, 21),
}

TAN_30 = math.tan(math.radians(30.0))
SEN_LINE_RE = re.compile(r"^\s*sen_(?P<sensor>\d+)\s*\((?P<body>.*)\)\s*$")
PAIR_RE = re.compile(r"(?P<poly>-?\d+)\s*-\s*(?P<lfsr>-?\d+)")


def polynomial_to_lighthouse_sweep(polynomial: int) -> tuple[int, int]:
    """Map LH2 polynomial id to basestation id and stable sweep id."""
    return polynomial >> 1, polynomial & 1


def parse_lh2_serial_line(line: str, pc_time: float | None = None) -> list[dict]:
    """Parse supported firmware line formats into raw LFSR observations.

    Supported inputs:
      LH2,time_us,sensor,sweep,basestation,polynomial,lfsr
      LH2,sensor,sweep,basestation,polynomial,lfsr
      sen_0 (8-12906 9-53998 21-54125 21-18082)

    The firmware sweep column is ignored on purpose. The stable label is
    derived from the polynomial: even polynomial -> sweep 0, odd -> sweep 1.
    """
    if pc_time is None:
        pc_time = time.time()

    raw = line.strip()
    if not raw:
        return []

    compact = SEN_LINE_RE.match(raw)
    if compact:
        sensor_id = int(compact.group("sensor"))
        observations = []
        for match in PAIR_RE.finditer(compact.group("body")):
            polynomial = int(match.group("poly"))
            lfsr = int(match.group("lfsr"))
            if lfsr < 0:
                continue
            lighthouse_id, sweep = polynomial_to_lighthouse_sweep(polynomial)
            observations.append({
                "pc_time": pc_time,
                "firmware_time_us": "",
                "sensor_id": sensor_id,
                "lighthouse_id": lighthouse_id,
                "polynomial": polynomial,
                "sweep": sweep,
                "lfsr": lfsr,
                "raw_line": raw,
            })
        return observations

    parts = [part.strip() for part in raw.split(",")]
    if not parts or parts[0] != "LH2":
        return []

    try:
        if len(parts) == 7:
            firmware_time_us = int(parts[1])
            sensor_id = int(parts[2])
            polynomial = int(parts[5])
            lfsr = int(parts[6])
        elif len(parts) == 6:
            firmware_time_us = ""
            sensor_id = int(parts[1])
            polynomial = int(parts[4])
            lfsr = int(parts[5])
        else:
            return []
    except ValueError:
        return []

    if lfsr < 0:
        return []

    lighthouse_id, sweep = polynomial_to_lighthouse_sweep(polynomial)
    return [{
        "pc_time": pc_time,
        "firmware_time_us": firmware_time_us,
        "sensor_id": sensor_id,
        "lighthouse_id": lighthouse_id,
        "polynomial": polynomial,
        "sweep": sweep,
        "lfsr": lfsr,
        "raw_line": raw,
    }]


def load_lfsr_coefficients(path: str | Path) -> dict[int, dict[str, float]]:
    """Load v7 history_calibration coefficients keyed by basestation id."""
    coeffs: dict[int, dict[str, float]] = {}
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if "DATE_TIME" in line or line.startswith("-") or not line.strip():
                continue
            parts = [part.strip() for part in line.split("|")]
            if len(parts) < 7:
                continue
            try:
                coeffs[int(parts[1])] = {
                    "A0": float(parts[3]),
                    "B0": float(parts[4]),
                    "A1": float(parts[5]),
                    "B1": float(parts[6]),
                }
            except ValueError:
                continue
    return coeffs


def unwrap_near(value_deg: float, reference_deg: float) -> float:
    while value_deg - reference_deg > 180.0:
        value_deg -= 360.0
    while value_deg - reference_deg < -180.0:
        value_deg += 360.0
    return value_deg


def sweeps_to_az_el_deg(sweep0_deg: float, sweep1_deg: float) -> tuple[float, float]:
    """Approximate the two LH2 tilted sweeps as azimuth/elevation.

    This is the same provisional approximation used by the v7 experiments. It
    is useful for debugging and for the angular-camera test solver, but the
    final model should predict the real LH2 sweep planes directly.
    """
    sweep1_deg = unwrap_near(sweep1_deg, sweep0_deg)
    azimuth = (sweep0_deg + sweep1_deg) / 2.0
    elevation = (sweep0_deg - sweep1_deg) / (2.0 * TAN_30)
    return azimuth, elevation


def score_az_el(azimuth_deg: float, elevation_deg: float) -> float:
    return abs(elevation_deg) + max(0.0, abs(azimuth_deg) - 90.0)


def lfsr_pair_to_measurement(
    lfsr0: float,
    lfsr1: float,
    coeffs: dict[str, float],
    mode: str = "auto",
) -> dict:
    """Convert two sweep LFSR medians into provisional sweep/angle values."""
    normal_sweep0 = coeffs["A0"] * lfsr0 + coeffs["B0"]
    normal_sweep1 = coeffs["A1"] * lfsr1 + coeffs["B1"]
    normal_az, normal_el = sweeps_to_az_el_deg(normal_sweep0, normal_sweep1)

    swapped_sweep0 = coeffs["A0"] * lfsr1 + coeffs["B0"]
    swapped_sweep1 = coeffs["A1"] * lfsr0 + coeffs["B1"]
    swapped_az, swapped_el = sweeps_to_az_el_deg(swapped_sweep0, swapped_sweep1)

    if mode == "normal":
        chosen = ("normal", normal_sweep0, normal_sweep1, normal_az, normal_el)
    elif mode == "swapped":
        chosen = ("swapped", swapped_sweep0, swapped_sweep1, swapped_az, swapped_el)
    elif score_az_el(swapped_az, swapped_el) < score_az_el(normal_az, normal_el):
        chosen = ("swapped", swapped_sweep0, swapped_sweep1, swapped_az, swapped_el)
    else:
        chosen = ("normal", normal_sweep0, normal_sweep1, normal_az, normal_el)

    chosen_mode, sweep0_deg, sweep1_deg, azimuth_deg, elevation_deg = chosen
    return {
        "mode": chosen_mode,
        "sweep0_deg": sweep0_deg,
        "sweep1_deg": sweep1_deg,
        "azimuth_deg": azimuth_deg,
        "elevation_deg": elevation_deg,
    }


def lfsr_pair_to_ordered_sweeps(lfsr0: float, lfsr1: float, coeffs: dict[str, float]) -> dict:
    """Convert LFSR values without ever swapping sweep identity."""
    sweep0_deg = coeffs["A0"] * lfsr0 + coeffs["B0"]
    sweep1_deg = coeffs["A1"] * lfsr1 + coeffs["B1"]
    return {
        "sweep0_deg": sweep0_deg,
        "sweep1_deg": sweep1_deg,
    }


def read_raw_lfsr_csv(path: str | Path) -> list[dict]:
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["pc_time"] = float(row["pc_time"])
        row["sensor_id"] = int(row["sensor_id"])
        row["lighthouse_id"] = int(row["lighthouse_id"])
        row["polynomial"] = int(row["polynomial"])
        row["sweep"] = int(row["sweep"])
        row["lfsr"] = int(row["lfsr"])
    return rows


def group_raw_rows_by_window(rows: list[dict], window_s: float) -> dict[int, list[dict]]:
    grouped: dict[int, list[dict]] = defaultdict(list)
    if not rows:
        return grouped
    t0 = min(row["pc_time"] for row in rows)
    for row in rows:
        frame_index = int((row["pc_time"] - t0) / window_s)
        grouped[frame_index].append(row)
    return grouped
