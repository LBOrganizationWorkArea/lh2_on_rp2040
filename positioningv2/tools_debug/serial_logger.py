#!/usr/bin/env python3

import argparse
import csv
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with: pip install pyserial")
    raise


def main():
    parser = argparse.ArgumentParser(description="Log serial Lighthouse data to CSV.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM5 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Opening serial port: {args.port} at {args.baudrate} baud")
    print(f"Saving data to: {output_path}")

    with serial.Serial(args.port, args.baudrate, timeout=1) as ser, open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["pc_time_s", "raw_line"])

        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            pc_time = time.time()
            writer.writerow([pc_time, line])
            f.flush()
            print(line)


if __name__ == "__main__":
    main()
