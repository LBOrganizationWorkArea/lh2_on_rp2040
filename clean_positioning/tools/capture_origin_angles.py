import argparse
import json
import math
import re
import time
from pathlib import Path
from statistics import median

import serial


TICKS_PER_REV = 833333

LINE_RE = re.compile(
    r"^LH2,"
    r"(?P<time_us>\d+),"
    r"(?P<sensor>\d+),"
    r"(?P<sweep>\d+),"
    r"(?P<basestation>\d+),"
    r"(?P<polynomial>-?\d+),"
    r"(?P<lfsr_location>-?\d+)"
    r"$"
)


def parse_line(line):
    m = LINE_RE.match(line.strip())
    if not m:
        return None

    return {
        "time_us": int(m.group("time_us")),
        "sensor": int(m.group("sensor")),
        "sweep": int(m.group("sweep")),
        "basestation": int(m.group("basestation")),
        "polynomial": int(m.group("polynomial")),
        "lfsr_location": int(m.group("lfsr_location")),
    }


def lfsr_to_alpha_rad(lfsr_location):
    deg = (((lfsr_location % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)


def alphas_to_theta_phi(alpha0, alpha1):
    theta = (alpha0 + alpha1) / 2.0

    numerator = math.sin(((alpha1 - alpha0) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha0 + alpha1) / 2.0)

    phi = math.atan2(numerator, denominator)
    return theta, phi


def load_layout(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(description="Capture origin LH2 angles for all visible sensors.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    layout = load_layout(args.layout)
    expected_sensors = set(int(s) for s in layout["sensors"].keys())

    samples = {}

    print("Capture origin angles")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Layout: {args.layout}")
    print(f"Duration: {args.duration} s")
    print(f"Expected sensors: {sorted(expected_sensors)}")
    print("=" * 60)
    print("Place the drone at the origin, flat and still.")
    input("Press ENTER to start capture...")

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)

        t0 = time.time()

        while time.time() - t0 < args.duration:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_line(line)

            if data is None:
                continue

            sensor = data["sensor"]
            bs = data["basestation"]
            sweep = data["sweep"]

            if sensor not in expected_sensors:
                continue

            if sweep not in (0, 1):
                continue

            key = (bs, sensor, sweep)
            samples.setdefault(key, []).append(data["lfsr_location"])

    basestations = sorted(set(bs for bs, _, _ in samples.keys()))

    output = {
        "unit": "radians",
        "type": "origin_angles",
        "origin": "drone_start_pose_is_local_origin",
        "duration_s": args.duration,
        "layout_file": args.layout,
        "basestations": basestations,
        "measurements": {}
    }

    print()
    print("=" * 60)
    print("Capture summary")
    print("=" * 60)

    for bs in basestations:
        output["measurements"][str(bs)] = {}

        print(f"\nBasestation {bs}")

        for sensor in sorted(expected_sensors):
            k0 = (bs, sensor, 0)
            k1 = (bs, sensor, 1)

            if k0 not in samples or k1 not in samples:
                print(f"  sensor {sensor}: missing sweep")
                continue

            if len(samples[k0]) < 3 or len(samples[k1]) < 3:
                print(f"  sensor {sensor}: not enough samples")
                continue

            lfsr0 = int(median(samples[k0]))
            lfsr1 = int(median(samples[k1]))

            alpha0 = lfsr_to_alpha_rad(lfsr0)
            alpha1 = lfsr_to_alpha_rad(lfsr1)
            theta, phi = alphas_to_theta_phi(alpha0, alpha1)

            output["measurements"][str(bs)][str(sensor)] = {
                "theta": theta,
                "phi": phi,
                "lfsr0_median": lfsr0,
                "lfsr1_median": lfsr1,
                "samples0": len(samples[k0]),
                "samples1": len(samples[k1])
            }

            print(
                f"  sensor {sensor}: "
                f"theta={math.degrees(theta):+.3f} deg | "
                f"phi={math.degrees(phi):+.3f} deg | "
                f"samples=({len(samples[k0])},{len(samples[k1])})"
            )

    save_json(args.output, output)

    print()
    print("=" * 60)
    print(f"Saved: {args.output}")
    print("=" * 60)


if __name__ == "__main__":
    main()
