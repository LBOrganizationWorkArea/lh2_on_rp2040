#!/usr/bin/env python3

import json
from pathlib import Path


CALIBRATION_FIELDS = (
    "phase",
    "tilt",
    "gibmag",
    "gibphase",
    "curve",
    "ogeemag",
    "ogeephase",
)


def _require_number(value, field, axis_name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{axis_name}.{field} must be a number, got {value!r}")
    return float(value)


def normalize_lighthouse_factory_calibration(data):
    if not isinstance(data, dict):
        raise ValueError("Calibration JSON must contain an object at the top level")

    if "calibration" not in data or not isinstance(data["calibration"], dict):
        raise ValueError("Calibration JSON must contain a 'calibration' object")

    normalized = dict(data)
    normalized["calibration"] = {}

    for axis_name in ("axis0", "axis1"):
        axis = data["calibration"].get(axis_name)
        if not isinstance(axis, dict):
            raise ValueError(f"Calibration JSON is missing '{axis_name}'")

        normalized_axis = {}
        for field in CALIBRATION_FIELDS:
            if field not in axis:
                raise ValueError(f"Calibration JSON is missing '{axis_name}.{field}'")
            normalized_axis[field] = _require_number(axis[field], field, axis_name)

        normalized["calibration"][axis_name] = normalized_axis

    base_station = normalized.get("base_station")
    if base_station is None:
        normalized["base_station"] = {
            "serial": None,
            "channel": None,
            "model": "LH2",
        }
    elif not isinstance(base_station, dict):
        raise ValueError("'base_station' must be an object")
    else:
        normalized["base_station"] = dict(base_station)
        normalized["base_station"].setdefault("serial", None)
        normalized["base_station"].setdefault("channel", None)
        normalized["base_station"].setdefault("model", "LH2")

    normalized.setdefault("source", "json")
    normalized.setdefault("device", None)
    normalized.setdefault("timestamp", None)

    return normalized


def load_lighthouse_factory_calibration(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    return normalize_lighthouse_factory_calibration(data)
