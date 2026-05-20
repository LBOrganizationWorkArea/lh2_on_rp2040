import argparse
import math
import time
from pathlib import Path

import serial


TAN_30 = math.tan(math.radians(30.0))
DEFAULT_BS_POLYS = {
    4: (8, 9),
    10: (20, 21),
}


def parse_lh2_line(line):
    line = line.strip()
    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    try:
        if len(parts) == 7:
            return {
                "time_us": int(parts[1]),
                "sensor": int(parts[2]),
                "sweep": int(parts[3]),
                "basestation": int(parts[4]),
                "polynomial": int(parts[5]),
                "lfsr": int(parts[6]),
            }

        # Older colleague scripts used: LH2,sensor,sweep,basestation,polynomial,lfsr
        if len(parts) == 6:
            return {
                "time_us": None,
                "sensor": int(parts[1]),
                "sweep": int(parts[2]),
                "basestation": int(parts[3]),
                "polynomial": int(parts[4]),
                "lfsr": int(parts[5]),
            }
    except ValueError:
        return None

    return None


def load_latest_coefficients(path):
    coeffs = {}

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if "DATE_TIME" in line or line.startswith("-") or not line.strip():
                continue

            parts = [p.strip() for p in line.split("|")]
            if len(parts) < 7:
                continue

            try:
                bs = int(parts[1])
                sensor = int(parts[2])
                coeffs[bs] = {
                    "sensor_calibrated_with": sensor,
                    "A0": float(parts[3]),
                    "B0": float(parts[4]),
                    "A1": float(parts[5]),
                    "B1": float(parts[6]),
                }
            except ValueError:
                continue

    return coeffs


def unwrap_near(angle_deg, reference_deg):
    while angle_deg - reference_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg - reference_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def compute_az_el(sweep0_deg, sweep1_deg):
    sweep1_deg = unwrap_near(sweep1_deg, sweep0_deg)
    azimuth = (sweep0_deg + sweep1_deg) / 2.0
    elevation = (sweep0_deg - sweep1_deg) / (2.0 * TAN_30)
    return azimuth, elevation, sweep1_deg


def score_angles(azimuth, elevation):
    return abs(elevation) + max(0.0, abs(azimuth) - 90.0)


def compute_best_az_el(lfsr0, lfsr1, coeffs):
    normal_sweep0 = coeffs["A0"] * lfsr0 + coeffs["B0"]
    normal_sweep1 = coeffs["A1"] * lfsr1 + coeffs["B1"]
    normal_az, normal_el, _ = compute_az_el(normal_sweep0, normal_sweep1)

    swapped_sweep0 = coeffs["A0"] * lfsr1 + coeffs["B0"]
    swapped_sweep1 = coeffs["A1"] * lfsr0 + coeffs["B1"]
    swapped_az, swapped_el, _ = compute_az_el(swapped_sweep0, swapped_sweep1)

    if score_angles(swapped_az, swapped_el) < score_angles(normal_az, normal_el):
        return swapped_az, swapped_el, "swapped"

    return normal_az, normal_el, "normal"


def main():
    parser = argparse.ArgumentParser(description="Live calibrated LH2 azimuth/elevation from A0/B0/A1/B1 coefficients.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--print-period", type=float, default=0.2)
    args = parser.parse_args()

    coeffs = load_latest_coefficients(args.history)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensors = [int(x) for x in args.sensors.split(",")]

    missing = [bs for bs in basestations if bs not in coeffs]
    if missing:
        raise SystemExit(f"Missing coefficients for base station(s): {missing}")

    state = {
        sensor: {
            bs: {"lfsr_by_poly": {}, "az": None, "el": None, "age": None, "mode": None}
            for bs in basestations
        }
        for sensor in sensors
    }

    print("=" * 70)
    print("Live calibrated LH2 azimuth/elevation")
    print(f"Port: {args.port}")
    for bs in basestations:
        c = coeffs[bs]
        print(
            f"BS{bs}: A0={c['A0']:.8f} B0={c['B0']:.4f} "
            f"A1={c['A1']:.8f} B1={c['B1']:.4f} "
            f"(calibrated using sensor {c['sensor_calibrated_with']})"
        )
    print("=" * 70)

    last_print = 0.0

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_lh2_line(raw)
            if data is None:
                continue

            sensor = data["sensor"]
            bs = data["basestation"]
            sweep = data["sweep"]
            poly = data["polynomial"]
            lfsr = data["lfsr"]

            if sensor not in state or bs not in coeffs:
                continue

            if bs in DEFAULT_BS_POLYS and poly not in DEFAULT_BS_POLYS[bs]:
                continue

            c = coeffs[bs]
            poly_state = state[sensor][bs]["lfsr_by_poly"].setdefault(poly, {})
            poly_state[sweep] = lfsr

            if 0 in poly_state and 1 in poly_state:
                az, el, mode = compute_best_az_el(poly_state[0], poly_state[1], c)
                state[sensor][bs].update({
                    "az": az,
                    "el": el,
                    "age": time.time(),
                    "mode": mode,
                })
                del state[sensor][bs]["lfsr_by_poly"][poly]

            now = time.time()
            if now - last_print < args.print_period:
                continue

            print("\033[H\033[J", end="")
            print("Calibrated angles")
            print("-" * 70)
            for sensor_id in sensors:
                parts = []
                for bs_id in basestations:
                    item = state[sensor_id][bs_id]
                    if item["az"] is None:
                        parts.append(f"BS{bs_id}: Az=... El=...")
                    else:
                        age = now - item["age"]
                        stale = " stale" if age > 0.6 else ""
                        parts.append(f"BS{bs_id}: Az={item['az']:+7.2f} deg El={item['el']:+7.2f} deg {item['mode']}{stale}")
                print(f"sensor {sensor_id}: " + " | ".join(parts))
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
