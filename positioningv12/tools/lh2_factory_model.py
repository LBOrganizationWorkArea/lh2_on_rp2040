#!/usr/bin/env python3

import math
from pathlib import Path

from lighthouse_factory_calibration import load_lighthouse_factory_calibration


DEFAULT_FACTORY_CALIBRATIONS = {
    4: Path("config/lighthouse_factory_calibration_bs4.json"),
    10: Path("config/lighthouse_factory_calibration_bs10.json"),
}


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def lh2_nominal_angle(p_lh, nominal_tilt):
    x, y, z = p_lh
    r = math.sqrt(x * x + y * y)
    if r < 1e-9:
        r = 1e-9

    value = (z * math.tan(nominal_tilt)) / r
    value = max(-0.999999, min(0.999999, value))

    return math.atan2(y, x) + math.asin(value)


def lh2_factory_angle(p_lh, nominal_tilt, axis_calibration=None):
    if axis_calibration is None:
        return lh2_nominal_angle(p_lh, nominal_tilt)

    # Bitcraze's public notes describe this as a measurement model:
    # correct geometry -> distorted/measured sweep angle. They use phase,
    # tilt, gibbous magnitude and gibbous phase for LH2; curve/ogee are kept
    # in the JSON but not applied here.
    tilt = nominal_tilt + float(axis_calibration["tilt"])
    angle = lh2_nominal_angle(p_lh, tilt)
    angle += float(axis_calibration["phase"])
    angle += float(axis_calibration["gibmag"]) * math.cos(angle + float(axis_calibration["gibphase"]))
    return angle


def load_factory_calibration_map(spec="auto"):
    if spec is None:
        spec = "auto"

    spec = str(spec).strip()
    if spec.lower() in ("", "none", "off", "false", "0"):
        return {}

    paths_by_bs = {}
    if spec.lower() == "auto":
        for bs, path in DEFAULT_FACTORY_CALIBRATIONS.items():
            if path.is_file():
                paths_by_bs[bs] = path
    else:
        for item in spec.split(","):
            item = item.strip()
            if not item:
                continue

            if "=" in item:
                bs_text, path_text = item.split("=", 1)
            elif ":" in item:
                bs_text, path_text = item.split(":", 1)
            else:
                raise ValueError(
                    "Factory calibration entries must look like "
                    "4=config/file.json,10=config/file.json"
                )

            paths_by_bs[int(bs_text)] = Path(path_text)

    factory = {}
    for bs, path in paths_by_bs.items():
        data = load_lighthouse_factory_calibration(path)
        factory[int(bs)] = {
            "path": str(path),
            "base_station": data.get("base_station", {}),
            "axes": {
                0: data["calibration"]["axis0"],
                1: data["calibration"]["axis1"],
            },
        }

    return factory


def serialize_factory_for_geometry(factory_entry):
    if factory_entry is None:
        return None

    return {
        "source_path": factory_entry["path"],
        "base_station": factory_entry.get("base_station", {}),
        "axis0": factory_entry["axes"][0],
        "axis1": factory_entry["axes"][1],
        "model": "phase_tilt_gibmag_gibphase",
        "unused_fields_kept_in_json": ["curve", "ogeemag", "ogeephase"],
    }


def factory_axis_for_sweep(factory_entry, sweep):
    if factory_entry is None:
        return None
    return factory_entry["axes"].get(int(sweep))
