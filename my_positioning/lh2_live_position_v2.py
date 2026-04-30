import json
import re
import time
from pathlib import Path

import numpy as np
import serial


SERIAL_PORT = "COM3"
BAUD_RATE = 115200

WORK_DIR = Path(__file__).resolve().parent
MODEL_FILE = WORK_DIR / "lh2_model_v2.json"

LINE_RE = re.compile(
    r"LH2,"
    r"(?P<sensor>\d+),"
    r"(?P<sweep>\d+),"
    r"(?P<basestation>\d+),"
    r"(?P<poly>-?\d+),"
    r"(?P<lfsr>-?\d+)"
)


def parse_line(line):
    match = LINE_RE.search(line.strip())
    if not match:
        return None

    return {
        "sensor": int(match.group("sensor")),
        "sweep": int(match.group("sweep")),
        "basestation": int(match.group("basestation")),
        "lfsr": int(match.group("lfsr")),
    }


def estimate_position(lh4_value, lh10_value, model):
    x_raw = np.array([lh4_value, lh10_value], dtype=float)

    mean = np.array(model["mean"], dtype=float)
    std = np.array(model["std"], dtype=float)
    coeff_x = np.array(model["coeff_x"], dtype=float)
    coeff_y = np.array(model["coeff_y"], dtype=float)

    x_norm = (x_raw - mean) / std

    features = np.array([1.0, x_norm[0], x_norm[1]], dtype=float)

    x = float(features @ coeff_x)
    y = float(features @ coeff_y)

    return x, y


def main():
    with open(MODEL_FILE, "r") as f:
        model = json.load(f)

    print("Live LH2 position V2")
    print("Using model:", MODEL_FILE)
    print("Press Ctrl+C to stop.")
    print()

    last = {
        (4, 0): None,
        (4, 1): None,
        (10, 0): None,
        (10, 1): None,
    }

    last_print_time = 0.0

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            data = parse_line(raw)
            if data is None:
                continue

            if data["sensor"] != 2:
                continue

            basestation = data["basestation"]
            sweep = data["sweep"]
            lfsr = data["lfsr"]

            key = (basestation, sweep)

            if key not in last:
                continue

            last[key] = lfsr

            if any(value is None for value in last.values()):
                continue

            now = time.time()
            if now - last_print_time < 0.2:
                continue

            lh4_value = min(last[(4, 0)], last[(4, 1)])
            lh10_value = min(last[(10, 0)], last[(10, 1)])

            x, y = estimate_position(lh4_value, lh10_value, model)

            print(
                f"LH4={lh4_value:7d} | "
                f"LH10={lh10_value:7d} | "
                f"x={x:.2f} m | "
                f"y={y:.2f} m"
            )

            last_print_time = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
        