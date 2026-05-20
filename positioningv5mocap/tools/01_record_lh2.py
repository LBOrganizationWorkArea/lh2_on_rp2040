import argparse
import csv
import time
from pathlib import Path

import serial

from mocap_lh2 import parse_lh2_line


def main():
    parser = argparse.ArgumentParser(description="Record LH2 serial lines with PC timestamps for mocap calibration.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", default="data/lh2_record.csv")
    parser.add_argument("--duration", type=float, help="Optional duration in seconds. Default: record until Ctrl+C.")
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Record LH2 with PC timestamps")
    print(f"Port:   {args.port}")
    print(f"Output: {output}")
    print("Start mocap recording at the same time if possible.")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    count = 0
    start = time.time()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)

        with open(output, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "pc_time_s",
                "time_us",
                "sensor",
                "sweep",
                "basestation",
                "polynomial",
                "lfsr_location",
            ])
            writer.writeheader()

            while True:
                if args.duration is not None and time.time() - start >= args.duration:
                    break

                pc_time_s = time.time()
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                data = parse_lh2_line(raw)
                if data is None:
                    continue

                data["pc_time_s"] = pc_time_s
                writer.writerow(data)
                count += 1

                if count % 100 == 0:
                    print(f"recorded {count} LH2 rows")

    print("=" * 70)
    print(f"Saved {count} rows to {output}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
