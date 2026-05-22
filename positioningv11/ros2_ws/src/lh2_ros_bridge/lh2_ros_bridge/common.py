import json
import math
import sys
import time
from pathlib import Path
from statistics import median


POSITIONING_ROOT = Path(__file__).resolve().parents[4]
TOOLS_DIR = POSITIONING_ROOT / "tools"
CONFIG_DIR = POSITIONING_ROOT / "config"

if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from lh2_factory_model import load_factory_calibration_map  # noqa: E402
from lh2v10 import (  # noqa: E402
    frame_axes,
    frame_to_observations,
    parse_lh2p_line,
    select_clean_lh2p_frames,
)


DEFAULT_CALIBRATION_POSES = {
    "P0_center": {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0},
    "P1_right_40cm": {"x_m": 0.4, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0},
    "P2_left_40cm": {"x_m": -0.4, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0},
    "P3_front_40cm": {"x_m": 0.0, "y_m": 0.4, "z_m": 0.0, "yaw_deg": 0.0},
    "P4_back_40cm": {"x_m": 0.0, "y_m": -0.4, "z_m": 0.0, "yaw_deg": 0.0},
    "P5_front_right": {"x_m": 0.4, "y_m": 0.4, "z_m": 0.0, "yaw_deg": 0.0},
    "P6_front_left": {"x_m": -0.4, "y_m": 0.4, "z_m": 0.0, "yaw_deg": 0.0},
    "P7_back_left": {"x_m": -0.4, "y_m": -0.4, "z_m": 0.0, "yaw_deg": 0.0},
    "P8_back_right": {"x_m": 0.4, "y_m": -0.4, "z_m": 0.0, "yaw_deg": 0.0},
}


def json_dumps(data):
    return json.dumps(data, separators=(",", ":"), sort_keys=True)


def parse_basestations(text):
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def load_factory_calibs_for_ros(spec):
    old_cwd = Path.cwd()
    try:
        # The existing loader resolves "auto" relative to positioningv10.
        import os

        os.chdir(POSITIONING_ROOT)
        return load_factory_calibration_map(spec)
    finally:
        import os

        os.chdir(old_cwd)


def frame_to_json_payload(line, factory_calibs=None):
    frame = parse_lh2p_line(line)
    if frame is None:
        return None

    axes = frame_axes(frame)
    observations = frame_to_observations(frame, factory_calibs)
    return {
        "stamp_unix_time_s": time.time(),
        "raw_line": frame.raw_line,
        "basestation": int(frame.basestation),
        "sweep0": int(frame.sweep0),
        "sweep1": int(frame.sweep1),
        "poly0": int(frame.poly0),
        "poly1": int(frame.poly1),
        "block0": int(frame.block0),
        "block1": int(frame.block1),
        "delta": int(frame.delta),
        "offsets": [[int(a), int(b)] for a, b in frame.offsets],
        "axes": list(axes) if axes is not None else None,
        "valid_axes": axes is not None,
        "observations": observations,
    }


def aggregate_observations(parsed_payloads):
    grouped = {}
    for payload in parsed_payloads:
        for obs in payload.get("observations", []):
            key = (int(obs["sensor"]), int(obs["basestation"]), int(obs["sweep"]))
            grouped.setdefault(key, []).append(obs)

    measurements = []
    for (sensor, basestation, sweep), values in sorted(grouped.items()):
        lfsr_values = [
            float(item.get("lfsr_location", item.get("offset_ticks")))
            for item in values
            if "lfsr_location" in item or "offset_ticks" in item
        ]
        raw_angles = [float(item["raw_angle_rad"]) for item in values if "raw_angle_rad" in item]
        calibrated_angles = [
            float(item["calibrated_angle_rad"])
            for item in values
            if "calibrated_angle_rad" in item
        ]
        if not lfsr_values:
            continue

        measurement = {
            "sensor": int(sensor),
            "basestation": int(basestation),
            "sweep": int(sweep),
            "median_lfsr_location": float(median(lfsr_values)),
            "sample_count": int(len(values)),
        }
        if raw_angles:
            measurement["raw_angle_rad"] = float(median(raw_angles))
        if calibrated_angles:
            measurement["calibrated_angle_rad"] = float(median(calibrated_angles))
        measurements.append(measurement)

    return measurements


def missing_channels(measurements, basestations):
    found = {
        (int(m["sensor"]), int(m["basestation"]), int(m["sweep"]))
        for m in measurements
    }
    missing = []
    for sensor in range(4):
        for basestation in basestations:
            for sweep in range(2):
                key = (sensor, basestation, sweep)
                if key not in found:
                    missing.append({"sensor": sensor, "basestation": basestation, "sweep": sweep})
    return missing


def clean_payload_window(parsed_payloads, factory_calibs=None):
    frames = []
    for payload in parsed_payloads:
        frame = parse_lh2p_line(payload.get("raw_line", ""))
        if frame is not None:
            frames.append(frame)

    clean_frames = select_clean_lh2p_frames(frames)
    clean_payloads = []
    for frame in clean_frames:
        clean_payloads.append({
            "stamp_unix_time_s": time.time(),
            "raw_line": frame.raw_line,
            "basestation": int(frame.basestation),
            "observations": frame_to_observations(frame, factory_calibs),
        })
    return clean_payloads


def finite_float(value, default=0.0):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default
