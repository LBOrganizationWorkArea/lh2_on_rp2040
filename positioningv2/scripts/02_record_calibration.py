import argparse
import csv
import time
from pathlib import Path

import serial


FIELDNAMES = ["timestamp", "sensor_id", "lighthouse_id", "angle_1_deg", "angle_2_deg"]


def parse_angle_line(line):
    """Parse angle CSV lines.

    Accepted formats:
    timestamp,sensor_id,lighthouse_id,angle_1_deg,angle_2_deg
    ANGLE,timestamp,sensor_id,lighthouse_id,angle_1_deg,angle_2_deg

    Current LH2 firmware often emits LFSR lines, not angles. Those lines are
    intentionally ignored here; add an LFSR->angle conversion stage upstream.
    """
    parts = [part.strip() for part in line.split(",")]
    if not parts:
        return None
    if parts[0] == "ANGLE":
        parts = parts[1:]
    if len(parts) != 5:
        return None
    try:
        return {
            "timestamp": float(parts[0]),
            "sensor_id": int(parts[1]),
            "lighthouse_id": int(parts[2]),
            "angle_1_deg": float(parts[3]),
            "angle_2_deg": float(parts[4]),
        }
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(description="Record angular LH2 observations to calibration CSV.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--output", default="data/captures/calibration_001.csv")
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    written = 0

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        start = time.time()
        while time.time() - start < args.seconds:
            raw = ser.readline().decode(errors="ignore").strip()
            row = parse_angle_line(raw)
            if row is None:
                continue
            writer.writerow(row)
            written += 1

    print(f"Saved: {output} ({written} observations)")


if __name__ == "__main__":
    main()
