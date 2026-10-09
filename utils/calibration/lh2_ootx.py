#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lh2_ootx.py — read the factory calibration that each Lighthouse v2 base station broadcasts.

Every LH2 base station is measured at the factory; the result (per light plane:
phase, tilt, curve, gibbous magnitude/phase, ogee magnitude/phase) is stored in
the station and broadcast in the sweeps as the OOTX stream, one bit per rotor
turn. The bit is which polynomial of the station's pair carried the sweep
(e.g. 8 vs 9 for mode 5). This is the data the Crazyflie uses to correct its
lighthouse angles.

Pipeline (port of bitcraze/crazyflie-firmware):
  1. Firmware streams every decoded sweep over the Pico USB serial:
       O,<sensor>,<bs index>,<polynomial>,<lfsr>,<capture time us>
     (enabled by sending 'o'; see angle_decoder_set_ootx_stream()).
  2. Rotor turn start = capture time - lfsr / 6 MHz. All hits (any sensor,
     either plane) with the same turn start carry the same bit; a majority vote
     per turn gives one bit (pulse_processor_v2.c: ootxTimestamps / slowBit).
  3. OOTX framing: 17 zeros + 1 preamble, 16-bit little-endian length, 16-bit
     words each followed by a stuffing 1, CRC32 (ootx_decoder.c).
  4. Payload -> struct ootxDataFrame_s (ootx_decoder.h); values are float16,
     angles in radians, used as in lighthouseCalibrationInitFromFrame().

The stream must not skip a rotor turn for a whole frame (~6-8 s), so keep at
least one sensor in view of each station and still. Frames repeat continuously;
a frame is accepted only if its CRC matches.

Usage:
    python lh2_ootx.py                         # /dev/ttyACM0, until both stations decode
    python lh2_ootx.py --port /dev/ttyACM0 --seconds 120 --save-log ootx_log.txt
    python lh2_ootx.py --input ootx_log.txt    # decode a saved log offline
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
import zlib
from collections import Counter, defaultdict
from pathlib import Path

LFSR_CLOCK_HZ = 6_000_000
# Rotor periods in 48 MHz ticks per mode (DotBots LH_PERIODS, Bitcraze CYCLE_PERIODS * 2).
LH_PERIODS_48MHZ = [959000, 957000, 953000, 949000, 947000, 943000, 941000, 939000,
                    937000, 929000, 919000, 911000, 907000, 901000, 893000, 887000]
TURN_TOLERANCE = 0.3          # hits within 30 % of a turn of each other belong to the same turn
OOTX_MAX_FRAME_LENGTH = 43    # bytes (ootx_decoder.h)

# struct ootxDataFrame_s, packed little-endian. LH2 frames add the four ogee terms.
FRAME_FIELDS = [
    ("version", "H"), ("id", "I"),
    ("phase0", "e"), ("phase1", "e"), ("tilt0", "e"), ("tilt1", "e"),
    ("unlock_count", "B"), ("hw_version", "B"),
    ("curve0", "e"), ("curve1", "e"),
    ("accel_x", "b"), ("accel_y", "b"), ("accel_z", "b"),
    ("gibphase0", "e"), ("gibphase1", "e"), ("gibmag0", "e"), ("gibmag1", "e"),
    ("mode", "B"), ("faults", "B"),
    ("ogeephase0", "e"), ("ogeephase1", "e"), ("ogeemag0", "e"), ("ogeemag1", "e"),
]
LH1_FIELD_COUNT = 19  # fields up to and including "faults"


# --------------------------------------------------------------------------- #
# Bits per rotor turn
# --------------------------------------------------------------------------- #

def parse_line(line: str):
    """O,<sensor>,<bs>,<poly>,<lfsr>,<capture_us> -> (bs, poly, turn start [us]) or None."""
    fields = line.strip().split(",")
    if len(fields) != 6 or fields[0] != "O":
        return None
    try:
        _, _sensor, bs, poly, lfsr, capture_us = fields
        return int(bs), int(poly), int(capture_us) - int(lfsr) * 1e6 / LFSR_CLOCK_HZ
    except ValueError:
        return None


def turn_period_us(poly: int) -> float:
    """Rotor period of the station that uses this polynomial (pair 2k/2k+1 -> mode k+1)."""
    return LH_PERIODS_48MHZ[poly >> 1] / 48.0


def bit_segments(hits: list[tuple[int, float]]) -> list[list[int]]:
    """Turn (poly, turn start) hits of one station into runs of consecutive OOTX bits.

    Hits are grouped per rotor turn and majority-voted. A run ends where a turn
    is missing (no hit) or the vote is tied, since one lost bit breaks a frame.
    """
    if not hits:
        return []
    period = turn_period_us(Counter(poly for poly, _ in hits).most_common(1)[0][0])
    hits = sorted(hits, key=lambda hit: hit[1])

    turns: list[tuple[float, int | None]] = []
    group: list[tuple[int, float]] = [hits[0]]
    for hit in hits[1:]:
        if hit[1] - group[0][1] < TURN_TOLERANCE * period:
            group.append(hit)
            continue
        turns.append(_vote(group))
        group = [hit]
    turns.append(_vote(group))

    segments: list[list[int]] = []
    current: list[int] = []
    previous_start = None
    for start, bit in turns:
        consecutive = previous_start is not None and round((start - previous_start) / period) == 1
        if bit is None or (previous_start is not None and not consecutive):
            if current:
                segments.append(current)
            current = []
        if bit is not None:
            current.append(bit)
        previous_start = start
    if current:
        segments.append(current)
    return segments


def _vote(group: list[tuple[int, float]]) -> tuple[float, int | None]:
    start = sum(hit[1] for hit in group) / len(group)
    counts = Counter(poly & 1 for poly, _ in group)
    if len(counts) == 2 and counts[0] == counts[1]:
        return start, None
    return start, counts.most_common(1)[0][0]


# --------------------------------------------------------------------------- #
# OOTX framing
# --------------------------------------------------------------------------- #

def decode_frames(bits: list[int]) -> list[bytes]:
    """All CRC-valid OOTX payloads in a run of consecutive bits (ootx_decoder.c)."""
    payloads = []
    zeros = 0
    index = 0
    while index < len(bits):
        bit = bits[index]
        index += 1
        if zeros >= 17 and bit == 1:
            payload, end = _read_frame(bits, index)
            if payload is not None:
                payloads.append(payload)
                index = end  # a failed frame resumes the preamble search right after this preamble
            zeros = 0
            continue
        zeros = zeros + 1 if bit == 0 else 0
    return payloads


def _read_frame(bits: list[int], index: int) -> tuple[bytes | None, int]:
    """Read length, data and CRC words after a preamble. Returns (payload or None, next index)."""
    words: list[int] = []

    def read_word() -> int | None:
        nonlocal index
        if index + 17 > len(bits):
            return None
        word = 0
        for bit in bits[index:index + 16]:
            word = (word << 1) | bit
        stuffing = bits[index + 16]
        index += 17
        return word if stuffing == 1 else None

    length_word = read_word()
    if length_word is None:
        return None, index
    length = ((length_word & 0xFF) << 8) | (length_word >> 8)  # byte-swapped (betole)
    if length > OOTX_MAX_FRAME_LENGTH:
        return None, index
    for _ in range((length + 1) // 2 + 2):  # data words, then CRC0, CRC1
        word = read_word()
        if word is None:
            return None, index
        words.append(word)

    stream = b"".join(word.to_bytes(2, "big") for word in words)  # received byte order
    data, crc_bytes = stream[:-4], stream[-4:]
    payload = data[:length]
    if zlib.crc32(payload) != int.from_bytes(crc_bytes, "little"):
        return None, index
    return payload, index


def parse_payload(payload: bytes) -> dict:
    fields = FRAME_FIELDS if len(payload) >= struct.calcsize("<" + "".join(f for _, f in FRAME_FIELDS)) \
        else FRAME_FIELDS[:LH1_FIELD_COUNT]
    fmt = "<" + "".join(code for _, code in fields)
    values = dict(zip((name for name, _ in fields), struct.unpack_from(fmt, payload)))
    values["protocol_version"] = values["version"] & 0x3F
    values["firmware_version"] = values.pop("version") >> 6
    return values


def to_calibration(frame: dict) -> dict:
    """Same layout as lighthouseCalibrationInitFromFrame(): sweep[0]/[1] per light plane."""
    def sweep(i: int) -> dict:
        return {key: float(frame.get(f"{key}{i}", 0.0))
                for key in ("phase", "tilt", "curve", "gibmag", "gibphase", "ogeemag", "ogeephase")}
    return {"uid": f"{frame['id']:08X}", "mode": frame.get("mode"), "sweeps": [sweep(0), sweep(1)],
            "raw_frame": {key: (float(value) if isinstance(value, float) else value)
                          for key, value in frame.items()}}


# --------------------------------------------------------------------------- #
# Driver
# --------------------------------------------------------------------------- #

class Collector:
    def __init__(self) -> None:
        self.hits: dict[int, list[tuple[int, float]]] = defaultdict(list)
        self.lines = 0

    def add(self, line: str) -> None:
        parsed = parse_line(line)
        if parsed is not None:
            bs, poly, start = parsed
            self.hits[bs].append((poly, start))
            self.lines += 1

    def decode(self) -> dict[int, dict]:
        """Calibration per station index, from the payload decoded most often (both bit polarities)."""
        results = {}
        for bs, hits in sorted(self.hits.items()):
            payloads: Counter = Counter()
            for segment in bit_segments(hits):
                for bits in (segment, [1 - bit for bit in segment]):
                    payloads.update(decode_frames(bits))
            if payloads:
                payload, count = payloads.most_common(1)[0]
                calibration = to_calibration(parse_payload(payload))
                calibration["frames_decoded"] = count
                calibration["polynomials"] = sorted({poly for poly, _ in hits})
                results[bs] = calibration
        return results

    def status(self) -> str:
        parts = []
        for bs, hits in sorted(self.hits.items()):
            longest = max((len(s) for s in bit_segments(hits)), default=0)
            parts.append(f"BS{bs}: {len(hits)} hits, longest unbroken run {longest} bits")
        return "; ".join(parts) or "no O lines yet"


def print_calibration(results: dict[int, dict]) -> None:
    for bs, calib in sorted(results.items()):
        print(f"\nBS{bs}  uid {calib['uid']}  mode {calib['mode']}  polys {calib['polynomials']}  "
              f"({calib['frames_decoded']} matching frames)")
        print("        phase[deg]   tilt[deg]   curve      gibmag     gibphase[deg]  ogeemag    ogeephase")
        for i, sweep in enumerate(calib["sweeps"]):
            print(f"  plane{i} {math.degrees(sweep['phase']):+9.4f}  {math.degrees(sweep['tilt']):+9.4f}  "
                  f"{sweep['curve']:+.5f}  {sweep['gibmag']:+.5f}  {math.degrees(sweep['gibphase']):+10.3f}     "
                  f"{sweep['ogeemag']:+.5f}  {sweep['ogeephase']:+.5f}")


def run_serial(args, collector: Collector, log) -> None:
    import serial  # pyserial

    with serial.Serial(args.port, 115200, timeout=0.2) as port:
        port.write(b"o")
        print(f"Streaming OOTX bits from {args.port} — keep the drone still with its sensors "
              "in view of both stations (a frame takes ~6-8 s of unbroken turns) ...")
        started = last_report = time.monotonic()
        try:
            while time.monotonic() - started < args.seconds:
                line = port.readline().decode(errors="replace")
                if line:
                    collector.add(line)
                    if log:
                        log.write(line)
                now = time.monotonic()
                if now - last_report >= 3.0:
                    last_report = now
                    found = collector.decode()
                    print(f"  [{now - started:4.0f}s] {collector.status()}; decoded: {sorted(found) or 'none'}")
                    if len(found) >= args.stations:
                        break
        except KeyboardInterrupt:
            print()
        finally:
            port.write(b"x")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0", help="Pico USB serial port (default /dev/ttyACM0)")
    parser.add_argument("--input", type=Path, help="decode a saved O-line log instead of the serial port")
    parser.add_argument("--save-log", type=Path, help="also write the raw O lines to this file")
    parser.add_argument("--seconds", type=float, default=120.0, help="give up after this long (default 120)")
    parser.add_argument("--stations", type=int, default=2, help="stop once this many stations decode")
    parser.add_argument("-o", "--output", type=Path,
                        default=Path(__file__).with_name("lh2_factory_calibration.json"))
    args = parser.parse_args()

    collector = Collector()
    if args.input:
        with open(args.input, encoding="utf-8", errors="replace") as source:
            for line in source:
                collector.add(line)
    else:
        log = open(args.save_log, "w", encoding="utf-8") if args.save_log else None
        try:
            run_serial(args, collector, log)
        finally:
            if log:
                log.close()

    results = collector.decode()
    print(f"\n{collector.lines} O lines; {collector.status()}")
    if not results:
        print("No CRC-valid OOTX frame. Need unbroken runs of several hundred bits per station: keep the "
              "drone still, all sensors in view, and run longer.", file=sys.stderr)
        return 1
    print_calibration(results)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump({str(bs): calib for bs, calib in sorted(results.items())}, stream, indent=2)
    print(f"\nWrote {args.output}")
    return 0 if len(results) >= args.stations else 1


if __name__ == "__main__":
    sys.exit(main())
