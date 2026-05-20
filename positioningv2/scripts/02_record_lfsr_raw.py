#!/usr/bin/env python3
"""Record raw decoded LH2 LFSR observations from the serial port."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from lh2_lfsr_common import parse_lh2_serial_line


FIELDS = [
    "pc_time",
    "firmware_time_us",
    "sensor_id",
    "lighthouse_id",
    "polynomial",
    "sweep",
    "lfsr",
    "raw_line",
]


def parse_ids(text: str) -> set[int]:
    return {int(part.strip()) for part in text.split(",") if part.strip()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, for example COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--output", default="data/captures/calibration_001_lfsr_raw.csv")
    parser.add_argument("--lighthouses", default="4,10")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is required: py -m pip install pyserial", file=sys.stderr)
        return 2

    expected_lighthouses = parse_ids(args.lighthouses) if args.lighthouses else set()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Record raw LH2 LFSR observations")
    print(f"Port: {args.port} @ {args.baud}")
    print(f"Output: {output}")
    print(f"Duration: {args.seconds:.1f} s")
    print("=" * 70)

    count = 0
    ignored = 0
    deadline = time.time() + args.seconds
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        with output.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()

            while time.time() < deadline:
                raw = ser.readline().decode(errors="ignore").strip()
                if not raw:
                    continue
                observations = parse_lh2_serial_line(raw, pc_time=time.time())
                if not observations:
                    ignored += 1
                    continue
                for obs in observations:
                    if expected_lighthouses and obs["lighthouse_id"] not in expected_lighthouses:
                        ignored += 1
                        continue
                    writer.writerow({field: obs.get(field, "") for field in FIELDS})
                    count += 1

    print("=" * 70)
    print(f"Saved: {output}")
    print(f"raw observations: {count}")
    print(f"ignored lines/entries: {ignored}")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
