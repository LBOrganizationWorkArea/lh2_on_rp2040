import argparse
import json
import math
import time

import cv2
import numpy as np
import serial


TAN_30 = math.tan(math.radians(30.0))
DEFAULT_BS_POLYS = {
    4: (8, 9),
    10: (20, 21),
}


def parse_lh2_line(line):
    parts = line.strip().split(",")
    if not parts or parts[0] != "LH2":
        return None

    try:
        if len(parts) == 7:
            return {
                "sensor": int(parts[2]),
                "sweep": int(parts[3]),
                "basestation": int(parts[4]),
                "polynomial": int(parts[5]),
                "lfsr": int(parts[6]),
            }

        if len(parts) == 6:
            return {
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
                coeffs[int(parts[1])] = {
                    "A0": float(parts[3]),
                    "B0": float(parts[4]),
                    "A1": float(parts[5]),
                    "B1": float(parts[6]),
                }
            except ValueError:
                continue

    return coeffs


def load_object_points(path, sensor_order):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    by_sensor = {
        int(item["sensor"]): [
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ]
        for item in data["sensors"]
    }

    return np.array([by_sensor[sensor] for sensor in sensor_order], dtype=np.float32).reshape((-1, 1, 3))


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
    return azimuth, elevation


def score_angles(azimuth, elevation):
    return abs(elevation) + max(0.0, abs(azimuth) - 90.0)


def compute_best_az_el(lfsr0, lfsr1, coeffs):
    normal_az, normal_el = compute_az_el(
        coeffs["A0"] * lfsr0 + coeffs["B0"],
        coeffs["A1"] * lfsr1 + coeffs["B1"],
    )
    swapped_az, swapped_el = compute_az_el(
        coeffs["A0"] * lfsr1 + coeffs["B0"],
        coeffs["A1"] * lfsr0 + coeffs["B1"],
    )

    if score_angles(swapped_az, swapped_el) < score_angles(normal_az, normal_el):
        return swapped_az, swapped_el, "swapped"
    return normal_az, normal_el, "normal"


def solve_one_bs(state, bs, sensor_order, object_points, y_sign):
    image = []
    labels = []
    now = time.time()

    for sensor in sensor_order:
        item = state[sensor][bs]
        if item["az"] is None or item["el"] is None:
            return None
        if now - item["age"] > 0.5:
            return None

        az_rad = math.radians(item["az"])
        el_rad = math.radians(item["el"])
        image.append([math.tan(az_rad), y_sign * math.tan(el_rad)])
        labels.append(item["mode"])

    image_points = np.array(image, dtype=np.float32).reshape((-1, 1, 2))
    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not success:
        return None

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    reproj = image_points.reshape((-1, 2)) - projected.reshape((-1, 2))
    reproj_rmse = float(np.sqrt(np.mean(np.sum(reproj * reproj, axis=1))))

    return {
        "rvec": rvec.reshape(3),
        "tvec": tvec.reshape(3),
        "reproj_rmse": reproj_rmse,
        "modes": labels,
    }


def main():
    parser = argparse.ArgumentParser(description="Live PnP pose from calibrated LH2 azimuth/elevation.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--history", default="config/history_calibration.txt")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--sensors", default="0,1,2,3")
    parser.add_argument("--print-period", type=float, default=0.1)
    parser.add_argument("--y-sign", type=float, default=1.0, help="Use -1 if vertical PnP axis is inverted.")
    args = parser.parse_args()

    coeffs = load_latest_coefficients(args.history)
    basestations = [int(x) for x in args.basestations.split(",")]
    sensor_order = [int(x) for x in args.sensors.split(",")]
    object_points = load_object_points(args.layout, sensor_order)

    missing = [bs for bs in basestations if bs not in coeffs]
    if missing:
        raise SystemExit(f"Missing coefficients for base station(s): {missing}")

    state = {
        sensor: {
            bs: {"lfsr_by_poly": {}, "az": None, "el": None, "age": None, "mode": None}
            for bs in basestations
        }
        for sensor in sensor_order
    }

    print("=" * 70)
    print("Live calibrated LH2 PnP")
    print("Output tvec is drone center in each Lighthouse virtual camera frame.")
    print(f"Sensors: {sensor_order}")
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

            poly_state = state[sensor][bs]["lfsr_by_poly"].setdefault(poly, {})
            poly_state[sweep] = lfsr

            if 0 in poly_state and 1 in poly_state:
                az, el, mode = compute_best_az_el(poly_state[0], poly_state[1], coeffs[bs])
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
            print("Live PnP")
            print("-" * 70)

            for bs_id in basestations:
                result = solve_one_bs(state, bs_id, sensor_order, object_points, args.y_sign)
                if result is None:
                    print(f"BS{bs_id}: waiting for fresh 4-sensor angles")
                    continue

                x, y, z = result["tvec"]
                print(
                    f"BS{bs_id}: X={x:+.3f} m | Y={y:+.3f} m | Z={z:+.3f} m | "
                    f"reproj={result['reproj_rmse']:.4f} | modes={','.join(result['modes'])}"
                )

            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
