#!/usr/bin/env python3
"""Capture the firmware's raw Lighthouse LFSR and solved-position records.

Windows usage:
    py capture_lh2_coeff_log.py COM5 -o bs01_calibration.csv

Install pyserial first with: py -m pip install pyserial
"""

import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Pico USB serial port, e.g. COM5")
    parser.add_argument("-o", "--output", default="bs01_calibration.csv")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, help="stop automatically after this many seconds")
    args = parser.parse_args()

    counts = {"L": 0, "Q": 0}
    started = time.monotonic()
    deadline = None if args.seconds is None else started + args.seconds

    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as device, \
                open(args.output, "w", encoding="utf-8", newline="") as output:
            print(f"Capturing L/Q records from {args.port} at {args.baud} baud to {args.output}")
            print("Move the wand through the volume; press Ctrl-C to stop.")
            while deadline is None or time.monotonic() < deadline:
                raw_line = device.readline()
                if not raw_line:
                    continue
                line = raw_line.decode("ascii", errors="ignore").strip()
                if not (line.startswith("L,") or line.startswith("Q,")):
                    continue
                output.write(line + "\n")
                output.flush()
                counts[line[0]] += 1
    except KeyboardInterrupt:
        pass

    print(f"Saved {counts['L']} raw LFSR records and {counts['Q']} solved positions to {args.output}")


if __name__ == "__main__":
    main()
