import serial
import time
import csv
from pathlib import Path

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

TARGET_SENSOR = 2
TARGET_BASESTATIONS = [4, 10]

WORK_DIR = Path(__file__).resolve().parent
DATA_DIR = WORK_DIR / "data_clean"
DATA_DIR.mkdir(exist_ok=True)


def parse_line(line):
    if not line.startswith("LH2,"):
        return None

    parts = line.split(",")

    if len(parts) != 6:
        return None

    try:
        return {
            "sensor": int(parts[1]),
            "sweep": int(parts[2]),
            "basestation": int(parts[3]),
            "polynomial": int(parts[4]),
            "lfsr_location": int(parts[5]),
        }
    except ValueError:
        return None


def is_useful(data):
    return (
        data["sensor"] == TARGET_SENSOR
        and data["basestation"] in TARGET_BASESTATIONS
        and data["sweep"] in [0, 1]
    )


def read_filtered():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

    print("Filtered LH2 reader")
    print("Showing only sensor=2, basestation=4 and 10")
    print("Press Ctrl+C to stop.\n")

    latest = {}

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            data = parse_line(line)

            if data is None:
                continue

            if not is_useful(data):
                continue

            key = (data["basestation"], data["sweep"])
            latest[key] = data["lfsr_location"]

            bs4_s0 = latest.get((4, 0), None)
            bs4_s1 = latest.get((4, 1), None)
            bs10_s0 = latest.get((10, 0), None)
            bs10_s1 = latest.get((10, 1), None)

            print(
                f"BS4_s0={bs4_s0} | "
                f"BS4_s1={bs4_s1} | "
                f"BS10_s0={bs10_s0} | "
                f"BS10_s1={bs10_s1}"
            )

    except KeyboardInterrupt:
        print("\nStopped.")


def record_filtered(name):
    out_file = DATA_DIR / f"{name}.csv"
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

    print(f"Recording filtered data to: {out_file}")
    print("Press Ctrl+C to stop.\n")

    with open(out_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s",
            "sensor",
            "sweep",
            "basestation",
            "polynomial",
            "lfsr_location",
        ])

        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                data = parse_line(line)

                if data is None:
                    continue

                if not is_useful(data):
                    continue

                writer.writerow([
                    time.time(),
                    data["sensor"],
                    data["sweep"],
                    data["basestation"],
                    data["polynomial"],
                    data["lfsr_location"],
                ])

                f.flush()

                print(
                    f"sensor={data['sensor']} | "
                    f"sweep={data['sweep']} | "
                    f"bs={data['basestation']} | "
                    f"poly={data['polynomial']} | "
                    f"lfsr={data['lfsr_location']}"
                )

        except KeyboardInterrupt:
            print("\nStopped recording.")
            print(f"Saved: {out_file}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["read", "record"])
    parser.add_argument("--name", default="test")

    args = parser.parse_args()

    if args.mode == "read":
        read_filtered()
    elif args.mode == "record":
        record_filtered(args.name)