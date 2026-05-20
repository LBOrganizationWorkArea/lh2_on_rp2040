#!/usr/bin/env python3

import argparse
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with: pip install pyserial")
    raise


def main():
    parser = argparse.ArgumentParser(description="Live serial viewer for Lighthouse data.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM5 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()

    print(f"Opening serial port: {args.port} at {args.baudrate} baud")

    with serial.Serial(args.port, args.baudrate, timeout=1) as ser:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"{time.time():.3f} | {line}")


if __name__ == "__main__":
    main()
