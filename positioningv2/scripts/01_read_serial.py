import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description="Print raw serial lines from the LH2 receiver.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
        start = time.time()
        while time.time() - start < args.seconds:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(line)


if __name__ == "__main__":
    main()
