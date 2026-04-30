import serial
import csv
import time
from pathlib import Path

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

WORK_DIR = Path(__file__).resolve().parent
DATA_DIR = WORK_DIR / "data_clean"
DATA_DIR.mkdir(exist_ok=True)

def parse_lh2_csv_line(line):
    """
    Expected format:
    LH2,sensor,sweep,basestation,polynomial,lfsr_location
    LH2,2,0,4,8,51232
    """
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


def read_live():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    print("Reading clean LH2 data. Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            data = parse_lh2_csv_line(line)

            if data is None:
                continue

            print(
                f"sensor={data['sensor']} | "
                f"sweep={data['sweep']} | "
                f"bs={data['basestation']} | "
                f"poly={data['polynomial']} | "
                f"lfsr={data['lfsr_location']}"
            )

    except KeyboardInterrupt:
        print("\nStopped.")


def record_csv(name):
    out_file = DATA_DIR / f"{name}.csv"
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

    print(f"Recording to {out_file}")
    print("Press Ctrl+C to stop.")

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
                data = parse_lh2_csv_line(line)

                if data is None:
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
        read_live()
    elif args.mode == "record":
        record_csv(args.name)