#!/usr/bin/env python3

import argparse
import csv
import time
from pathlib import Path

import serial


def parse_lh2_line(line):
    """
    Expected format:
    LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location

    Example:
    LH2,12345678,0,0,4,8,51232
    """
    line = line.strip()

    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    if len(parts) != 7:
        return None

    try:
        return {
            "time_us": int(parts[1]),
            "sensor": int(parts[2]),
            "sweep": int(parts[3]),
            "basestation": int(parts[4]),
            "polynomial": int(parts[5]),
            "lfsr_location": int(parts[6]),
        }
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(description="Log positioningv2 LH2 serial data to CSV.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", required=True, help="Output CSV file")
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Opening serial port: {args.port}")
    print(f"Baudrate: {args.baudrate}")
    print(f"Saving to: {output_path}")
    print("Press Ctrl+C to stop.")

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser, open(output_path, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "pc_time_s",
            "time_us",
            "sensor",
            "sweep",
            "basestation",
            "polynomial",
            "lfsr_location",
        ])

        try:
            while True:
                raw = ser.readline().decode(errors="ignore").strip()
                data = parse_lh2_line(raw)

                if data is None:
                    continue

                writer.writerow([
                    time.time(),
                    data["time_us"],
                    data["sensor"],
                    data["sweep"],
                    data["basestation"],
                    data["polynomial"],
                    data["lfsr_location"],
                ])
                f.flush()

                print(
                    f"t={data['time_us']} | "
                    f"sensor={data['sensor']} | "
                    f"sweep={data['sweep']} | "
                    f"bs={data['basestation']} | "
                    f"poly={data['polynomial']} | "
                    f"lfsr={data['lfsr_location']}"
                )

        except KeyboardInterrupt:
            print()
            print("Stopped.")
            print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
