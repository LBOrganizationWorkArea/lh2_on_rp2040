#!/usr/bin/env python3
"""
Capture raw serial output from RP2040/Pico for one dynamic Lighthouse calibration.
This script does not decode angles. It stores every serial line with a PC timestamp.
"""
import argparse
import json
import time
from pathlib import Path

try:
    import serial
except ImportError:
    raise SystemExit("Missing dependency: py -m pip install pyserial")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM3 on Windows or /dev/ttyACM0 on Linux")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    print(f"Opening serial {args.port} @ {args.baud}...")
    print(f"Capturing for {args.seconds:.1f} seconds -> {out}")
    print("Move the drone freely now. Try yaw, pitch, roll, left/right, forward/back, up/down.")

    n = 0
    t0 = time.time()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, out.open("w", encoding="utf-8") as f:
        while True:
            now = time.time()
            if now - t0 >= args.seconds:
                break
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            rec = {"timestamp_pc": now - t0, "raw": line}
            f.write(json.dumps(rec) + "\n")
            n += 1
            if n % 100 == 0:
                print(f"captured {n} lines, t={now-t0:.1f}s")

    print(f"Done. Captured {n} lines.")


if __name__ == "__main__":
    main()
