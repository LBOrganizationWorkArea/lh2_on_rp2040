#!/usr/bin/env python3

import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description="Raw Lighthouse serial viewer.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()

    print("=" * 70)
    print("Raw LH2 serial viewer")
    print(f"Port: {args.port}")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as exc:
                print()
                print(f"Serial disconnected or reset: {exc}")
                print("Reset/replug the Pico, then rerun the command.")
                break
            if line:
                print(f"{time.time():.3f} | {line}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
