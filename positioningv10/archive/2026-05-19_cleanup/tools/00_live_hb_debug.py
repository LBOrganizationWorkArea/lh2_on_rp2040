#!/usr/bin/env python3

import argparse
import time

import serial


HB_FIELDS = [
    "time_ms",
    "frames",
    "pairs",
    "age_frame_ms",
    "age_pair_ms",
    "seq0",
    "seq1",
    "seq2",
    "seq3",
    "blocks",
    "block_attempts",
    "block_rejects",
    "builder_timeouts",
    "pair_candidates",
    "pair_offset_rejects",
    "pair_age_rejects",
    "pair_timestamp_rejects",
    "poly_rejects",
]


def parse_hb(line):
    parts = line.strip().split(";")
    if len(parts) != 19 or parts[0] != "HB":
        return None
    try:
        values = [int(item) for item in parts[1:]]
    except ValueError:
        return None
    return dict(zip(HB_FIELDS, values))


def delta(now, previous, key):
    if previous is None:
        return 0
    return int(now[key]) - int(previous[key])


def main():
    parser = argparse.ArgumentParser(description="Decode firmware HB diagnostics.")
    parser.add_argument("--port", required=True, help="Serial port, example: COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()

    print("=" * 100)
    print("LH2 heartbeat diagnostics")
    print(f"Port: {args.port}")
    print("Press Ctrl+C to stop.")
    print("=" * 100)
    print(
        "dt | frames pairs lh2p | seq0 seq1 seq2 seq3 | "
        "blocks attempts rejects timeouts | candidates off_rej age_rej ts_rej poly_rej | ages"
    )

    previous = None
    lh2p_count = 0
    last_print = time.time()

    with serial.Serial(args.port, args.baudrate, timeout=0.5) as ser:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue
            if raw.startswith("LH2P;"):
                lh2p_count += 1
                continue

            hb = parse_hb(raw)
            if hb is None:
                continue

            now = time.time()
            if now - last_print < 0.2:
                continue

            if previous is None:
                print("waiting for next HB...")
            else:
                print(
                    f"+{delta(hb, previous, 'time_ms'):4d}ms | "
                    f"{delta(hb, previous, 'frames'):5d} "
                    f"{delta(hb, previous, 'pairs'):5d} "
                    f"{lh2p_count:4d} | "
                    f"{delta(hb, previous, 'seq0'):4d} "
                    f"{delta(hb, previous, 'seq1'):4d} "
                    f"{delta(hb, previous, 'seq2'):4d} "
                    f"{delta(hb, previous, 'seq3'):4d} | "
                    f"{delta(hb, previous, 'blocks'):5d} "
                    f"{delta(hb, previous, 'block_attempts'):8d} "
                    f"{delta(hb, previous, 'block_rejects'):7d} "
                    f"{delta(hb, previous, 'builder_timeouts'):8d} | "
                    f"{delta(hb, previous, 'pair_candidates'):10d} "
                    f"{delta(hb, previous, 'pair_offset_rejects'):7d} "
                    f"{delta(hb, previous, 'pair_age_rejects'):7d} "
                    f"{delta(hb, previous, 'pair_timestamp_rejects'):6d} "
                    f"{delta(hb, previous, 'poly_rejects'):8d} | "
                    f"frame_age={hb['age_frame_ms']}ms pair_age={hb['age_pair_ms']}ms"
                )

            previous = hb
            lh2p_count = 0
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
