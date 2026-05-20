import argparse
import time
from pathlib import Path
import serial

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    print("Raw Pico serial logger")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Duration: {args.duration} s")
    print(f"Output: {out}")
    print("=" * 60)

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser, open(out, "w", encoding="utf-8") as f:
        ser.reset_input_buffer()
        t0 = time.time()

        while time.time() - t0 < args.duration:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print(line)
            f.write(line + "\n")

    print("=" * 60)
    print(f"Saved raw data to: {out}")

if __name__ == "__main__":
    main()
