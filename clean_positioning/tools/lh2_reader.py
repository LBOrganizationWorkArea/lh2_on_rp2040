#!/usr/bin/env python3

import argparse
import csv
import time
from pathlib import Path

import serial


def parse_lh2_csv_line(line: str):
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
    parser = argparse.ArgumentParser(description="Read LH2 CSV serial stream.")
    parser.add_argument("--port", required=True, help="Example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", default="data/captures/lh2_capture.csv")
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("LH2 reader")
    print("=" * 60)
    print(f"Port:     {args.port}")
    print(f"Baudrate: {args.baudrate}")
    print(f"Duration: {args.duration:.1f} s")
    print(f"Output:   {output_path}")
    print("=" * 60)

    counts_by_sensor = {}
    counts_by_bs = {}
    counts_by_channel = {}

    start = time.time()
    rows = 0

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)

        with open(output_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "pc_time_s",
                    "time_us",
                    "sensor",
                    "sweep",
                    "basestation",
                    "polynomial",
                    "lfsr_location",
                ],
            )
            writer.writeheader()

            while time.time() - start < args.duration:
                raw = ser.readline().decode(errors="ignore").strip()
                data = parse_lh2_csv_line(raw)

                if data is None:
                    continue

                data["pc_time_s"] = time.time()
                writer.writerow(data)

                rows += 1

                sensor = data["sensor"]
                bs = data["basestation"]
                sweep = data["sweep"]
                channel = (sensor, bs, sweep)

                counts_by_sensor[sensor] = counts_by_sensor.get(sensor, 0) + 1
                counts_by_bs[bs] = counts_by_bs.get(bs, 0) + 1
                counts_by_channel[channel] = counts_by_channel.get(channel, 0) + 1

                print(
                    f"sensor={sensor} | "
                    f"bs={bs} | "
                    f"sweep={sweep} | "
                    f"poly={data['polynomial']} | "
                    f"lfsr={data['lfsr_location']}"
                )

    print()
    print("=" * 60)
    print(f"Saved: {output_path}")
    print(f"Rows: {rows}")
    print("Sensors:", dict(sorted(counts_by_sensor.items())))
    print("Basestations:", dict(sorted(counts_by_bs.items())))
    print(f"Channels seen: {len(counts_by_channel)}")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
