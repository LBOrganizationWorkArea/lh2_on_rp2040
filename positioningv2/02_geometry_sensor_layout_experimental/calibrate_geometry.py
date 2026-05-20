#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Calibrate Lighthouse geometry from captured data.")
    parser.add_argument("--capture", required=True)
    parser.add_argument("--sensors", default="config/sensors_layout.json")
    parser.add_argument("--output", default="config/geometry.json")
    args = parser.parse_args()

    sensors_layout = load_json(args.sensors)

    print("Calibration placeholder")
    print(f"Capture file: {args.capture}")
    print(f"Sensor layout: {args.sensors}")
    print(f"Number of sensors: {len(sensors_layout['sensors'])}")

    geometry = {
        "description": "Estimated Lighthouse base station geometry. Placeholder output.",
        "unit": "meter",
        "base_stations": [
            {
                "id": 0,
                "position_m": [0.0, 0.0, 2.0],
                "rotation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]
            },
            {
                "id": 1,
                "position_m": [2.0, 0.0, 2.0],
                "rotation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]
            }
        ]
    }

    save_json(args.output, geometry)
    print(f"Saved geometry to: {args.output}")


if __name__ == "__main__":
    main()
