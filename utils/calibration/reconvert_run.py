#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reconvert_run.py — re-derive the angles of a recorded capture_lh2.py session with another angle model.

The firmware (main.c CAL_BS*) converts each plane's LFSR count with a fitted
line, angle = A * count + B. That map is linear, so the exact counts can be
recovered from the recorded (horizontal, vertical) angles and converted again:

  period   DotBots / Alvarado / Bitcraze model: angle = count * 8 / period * 360 deg,
           same slope for both planes, planes 120 deg apart (-120 / -240 deg offsets).
  factory  period model, then the base station's own factory calibration (OOTX:
           phase, tilt, gibbous per plane) removed as in Bitcraze's
           lighthouseCalibrationApplyV2. Values from lh2_ootx.py.

The converted session can be solved with calibrate_bitcraze.py as usual.

Usage:
    python reconvert_run.py calib_runs/run2 calib_runs/run2_factory --model factory
    python calibrate_bitcraze.py calib_runs/run2_factory --baseline 2.26
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

TAN_30 = math.tan(math.pi / 6)

# Fitted CAL_BS* constants of the firmware the recording was made with: A0, B0, A1, B1 [deg].
# These are the OLD constants (firmware before the period + factory conversion, i.e. run1 / run2).
# Sessions recorded with the current firmware already have correct angles: do not re-convert them.
FIRMWARE_CAL = {0: (0.00315641, -121.7511, 0.00307607, -234.6501),
                1: (0.00327992, -126.1425, 0.00317364, -236.6446)}
# Rotor period in 48 MHz ticks (DotBots LH_PERIODS): BS0 poly 8/9 = mode 5, BS1 poly 20/21 = mode 11.
PERIOD_48MHZ = {0: 947000, 1: 919000}


def hv_to_planes(h: float, v: float) -> tuple[float, float]:
    """Inverse of the firmware's (s0, s1) -> (h, v), angle_decoder.c _finalize_angles [rad]."""
    d = 2 * math.asin(max(-1.0, min(1.0, math.tan(v) * TAN_30 * math.cos(h))))
    return h - d / 2, h + d / 2


def planes_to_hv(a1: float, a2: float) -> tuple[float, float]:
    return (a1 + a2) / 2, math.atan2(math.sin(a2 - a1), TAN_30 * (math.cos(a1) + math.cos(a2)))


def recover_counts(bs: int, h: float, v: float) -> tuple[float, float]:
    a0, b0, a1, b1 = FIRMWARE_CAL[bs]
    s0, s1 = (math.degrees(x) for x in hv_to_planes(h, v))
    return (s0 - b0) / a0, (s1 - b1) / a1


def period_beams(bs: int, c0: float, c1: float) -> tuple[float, float]:
    """pulse_processor_v2.c: firstBeam / secondBeam [rad]."""
    k = 2 * math.pi * 8 / PERIOD_48MHZ[bs]
    return c0 * k - math.pi + math.pi / 3, c1 * k - math.pi - math.pi / 3


def _model_lh2(x: float, y: float, z: float, t: float, calib: dict) -> float:
    """lighthouseCalibrationMeasurementModelLh2 (curve / ogee unused, as in Bitcraze)."""
    ax = math.atan2(y, x)
    r = math.sqrt(x * x + y * y)
    base = ax + math.asin(max(-1.0, min(1.0, z * math.tan(t - calib["tilt"]) / r)))
    comp_gib = -calib["gibmag"] * math.cos(ax + calib["gibphase"])
    return base - (calib["phase"] + comp_gib)


def _ideal_to_distorted(sweeps: list[dict], a1: float, a2: float) -> tuple[float, float]:
    y = math.tan((a2 + a1) / 2)
    z = math.sin(a2 - a1) / (TAN_30 * (math.cos(a2) + math.cos(a1)))
    return _model_lh2(1.0, y, z, -math.pi / 6, sweeps[0]), _model_lh2(1.0, y, z, math.pi / 6, sweeps[1])


def apply_factory_calibration(sweeps: list[dict], raw: tuple[float, float]) -> tuple[float, float]:
    """lighthouseCalibrationApply: invert the distortion model by fixed-point iteration."""
    estimate = list(raw)
    for _ in range(10):
        d0, d1 = _ideal_to_distorted(sweeps, *estimate)
        estimate[0] += raw[0] - d0
        estimate[1] += raw[1] - d1
        if abs(raw[0] - d0) < 1e-7 and abs(raw[1] - d1) < 1e-7:
            break
    return estimate[0], estimate[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="capture_lh2.py session directory")
    parser.add_argument("output", type=Path, help="directory for the converted session")
    parser.add_argument("--model", choices=("period", "factory"), default="factory")
    parser.add_argument("--factory-cal", type=Path,
                        default=Path(__file__).with_name("lh2_factory_calibration.json"),
                        help="OOTX calibration from lh2_ootx.py (for --model factory)")
    args = parser.parse_args()

    factory = None
    if args.model == "factory":
        with open(args.factory_cal, encoding="utf-8") as source:
            factory = {int(bs): calib["sweeps"] for bs, calib in json.load(source).items()}

    args.output.mkdir(parents=True, exist_ok=True)
    converted = skipped = 0
    for path in sorted(args.session.glob("*.json")):
        with open(path, encoding="utf-8") as source:
            data = json.load(source)
        if isinstance(data, list):
            for record in data:
                # A mis-decoded sweep gives |angle| > 90 deg and cannot be inverted;
                # the solver's span filter drops such records anyway.
                if any(abs(h) > math.pi / 2 or abs(v) > math.pi / 2 for h, v in record["angles"]):
                    skipped += 1
                    continue
                bs = int(record["base_station_id"])
                angles = []
                for h, v in record["angles"]:
                    beams = period_beams(bs, *recover_counts(bs, h, v))
                    if factory is not None:
                        beams = apply_factory_calibration(factory[bs], beams)
                    angles.append(list(planes_to_hv(*beams)))
                record["angles"] = angles
                converted += 1
        with open(args.output / path.name, "w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=1)
    print(f"{args.session} -> {args.output} ({args.model}): {converted} records converted, "
          f"{skipped} mis-decoded records left as recorded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
