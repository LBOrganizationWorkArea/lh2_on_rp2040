import argparse
import json
import time
from pathlib import Path

import serial

from lbees.indoor.lh_positioning_pnp.lh2_angle_processor import AngleAverager, POINT_BY_SENSOR, TICKS_PER_REV

"""
  _     _    _ __  __ _____ _   _  ____  _    _  _____   ____  ______ ______  _____ 
 | |   | |  | |  \/  |_   _| \ | |/ __ \| |  | |/ ____| |  _ \|  ____|  ____|/ ____|
 | |   | |  | | \  / | | | |  \| | |  | | |  | | (___   | |_) | |__  | |__  | (___  
 | |   | |  | | |\/| | | | | . ` | |  | | |  | |\___ \  |  _ <|  __| |  __|  \___ \ 
 | |___| |__| | |  | |_| |_| |\  | |__| | |__| |____) | | |_) | |____| |____ ____) |
 |______\____/|_|  |_|_____|_| \_|\____/ \____/|_____/  |____/|______|______|_____/ 
                                                                                    
Giorgio Rinolfi
Victor Bianchi
Antoine el Kahi
Eduardo Gonzalez
"""

WORK_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = WORK_DIR / "angles.json"


def parse_sensor_map(value):
    point_by_sensor = {}

    for item in value.split(","):
        sensor_id, point_name = item.split(":", maxsplit=1)
        point_by_sensor[int(sensor_id)] = point_name

    return point_by_sensor


def write_json(path, data):
    with open(path, "w") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


def print_progress(averager, samples_per_angle):
    parts = []
    for sensor, point_name in averager.point_by_sensor.items():
        theta_count = averager.count(sensor, 0)
        phi_count = averager.count(sensor, 1)
        parts.append(f"{point_name}=theta {theta_count}/{samples_per_angle}, phi {phi_count}/{samples_per_angle}")

    print(" | ".join(parts))


def main():
    parser = argparse.ArgumentParser(
        description="Read LH2 CSV serial output, average sweep angles, and write angles.json."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--basestation", type=int, required=True)
    parser.add_argument("--samples", type=int, default=50, help="Samples per sensor/sweep angle.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--ticks-per-rev", type=int, default=TICKS_PER_REV)
    parser.add_argument(
        "--sensor-map",
        default=",".join(f"{sensor}:{point}" for sensor, point in POINT_BY_SENSOR.items()),
        help="Mapping like 0:top_left,1:top_right,2:bottom_right,3:bottom_left.",
    )

    args = parser.parse_args()

    point_by_sensor = parse_sensor_map(args.sensor_map)
    averager = AngleAverager(
        basestation=args.basestation,
        point_by_sensor=point_by_sensor,
        ticks_per_rev=args.ticks_per_rev,
    )

    print(f"Reading LH2 serial from {args.port} at {args.baud} baud")
    print(f"Averaging basestation {args.basestation}")
    print(f"Writing output to {args.output}")
    print("Press Ctrl+C to stop without writing.\n")

    last_progress_time = 0.0

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        while not averager.ready(args.samples):
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            accepted = averager.add_line(line)
            now = time.time()

            if accepted and now - last_progress_time >= 1.0:
                print_progress(averager, args.samples)
                last_progress_time = now

    output = averager.to_angles_json()
    write_json(args.output, output)

    print()
    print("Averaged angles written:")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped. No JSON was written.")
