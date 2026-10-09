#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_lh2.py — record raw LH2 angles from the drone for calibrate_bitcraze.py.

The Pico sends one raw-angle snapshot per 100 ms as a MAVLink TUNNEL message
(payload_type 0x4C48 "LH", addressed to GCS sysid 255 / compid 190). This tool
receives it over ANY MAVLink link pymavlink understands, so it works however
the drone is connected:

  Wi-Fi (MavESP8266 / DroneBridge), what AUTO-CALIB-EXP uses (default):
      python capture_lh2.py --connect udpin:0.0.0.0:14550 --heartbeat-to 192.168.4.1:14555
  Telemetry radio (SiK) or FC USB, routed through the flight controller:
      python capture_lh2.py --connect /dev/ttyUSB0 --baud 57600        (Windows: COM5)
      python capture_lh2.py --connect /dev/ttyACM0 --baud 115200
  Pico UART0 straight into a USB-UART adapter (no FC):
      python capture_lh2.py --connect /dev/ttyUSB0 --baud 115200
  TCP (Mission Planner / MAVProxy forwarding, ESP in TCP mode):
      python capture_lh2.py --connect tcp:192.168.4.1:5760

Through a flight controller, ArduPilot only forwards the TUNNEL to a link on
which it has seen sysid 255 / compid 190 — this tool sends that heartbeat every
second. The Pico's port on the FC must be MAVLink 2 (e.g. SERIAL2_PROTOCOL = 2).

Output: the JSON format read by calibrate_bitcraze.py / calibrate_lighthouse.py.
Ctrl-C (or --seconds) stops and saves.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import struct
import sys
import time

os.environ.setdefault("MAVLINK20", "1")  # TUNNEL is a MAVLink 2 message

import numpy as np  # noqa: E402
from pymavlink import mavutil  # noqa: E402

LH2_TUNNEL_TYPE = 0x4C48
LH2_TUNNEL_DATA_LEN = 94
GCS_SYSID = 255
GCS_COMPID = 190  # the Pico addresses its TUNNEL to this component
NUM_SENSORS = 4
NUM_BS = 2

DEFAULT_SENSORS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wand_sensors.json")


def parse_lh2_tunnel(payload: bytes) -> dict | None:
    """Decode one LH2 TUNNEL v2 payload (see mavlink_send_lh2_angles in mavlink.c)."""
    if len(payload) < LH2_TUNNEL_DATA_LEN or payload[:2] != b"LH" or payload[2] != 2:
        return None
    sequence, timestamp_us, valid_mask = struct.unpack_from("<HQB", payload, 3)
    angles = struct.unpack_from("<16f", payload, 14)
    ages_ms = struct.unpack_from("<8H", payload, 78)
    return {"sequence": sequence, "timestamp_us": timestamp_us, "valid_mask": valid_mask,
            "angles": angles, "ages_ms": ages_ms}


def snapshot_to_records(snapshot: dict) -> list[dict]:
    """One record per base station that has all four sensors valid."""
    records = []
    for bs in range(NUM_BS):
        indices = [sensor * NUM_BS + bs for sensor in range(NUM_SENSORS)]
        if not all(snapshot["valid_mask"] & (1 << index) for index in indices):
            continue
        records.append({
            "timestamp": snapshot["timestamp_us"] / 1e6,
            "base_station_id": bs,
            "angles": [[snapshot["angles"][index * 2], snapshot["angles"][index * 2 + 1]] for index in indices],
            "angle_ages_ms": [snapshot["ages_ms"][index] for index in indices],
        })
    return records


def wand_span_limit(sensor_file: str, min_distance: float) -> float:
    """Largest angle [rad] the sensor board can subtend at min_distance (see calibrate_bitcraze)."""
    with open(sensor_file, encoding="utf-8") as source:
        sensors = np.asarray(json.load(source), dtype=float)
    diagonal = max(np.linalg.norm(a - b) for a, b in itertools.combinations(sensors, 2))
    return math.atan2(diagonal, min_distance)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="udpin:0.0.0.0:14550",
                        help="pymavlink connection string or serial port (default udpin:0.0.0.0:14550)")
    parser.add_argument("--baud", type=int, default=57600, help="serial baud rate (default 57600)")
    parser.add_argument("--heartbeat-to", metavar="HOST:PORT",
                        help="also send the GCS heartbeat to this UDP address (MavESP8266 learns the "
                             "GCS from it, e.g. 192.168.4.1:14555)")
    parser.add_argument("-o", "--output", default="measurements.json")
    parser.add_argument("--seconds", type=float, help="stop after this many seconds")
    parser.add_argument("--sensor-positions", default=DEFAULT_SENSORS,
                        help="drone sensor layout, only for the live quality readout")
    parser.add_argument("--min-wand-distance", type=float, default=0.5)
    args = parser.parse_args()

    span_limit = wand_span_limit(args.sensor_positions, args.min_wand_distance)

    print(f"Connecting to {args.connect} ...")
    link = mavutil.mavlink_connection(args.connect, baud=args.baud,
                                      source_system=GCS_SYSID, source_component=GCS_COMPID)
    side_link = None
    if args.heartbeat_to:
        side_link = mavutil.mavlink_connection(f"udpout:{args.heartbeat_to}",
                                               source_system=GCS_SYSID, source_component=GCS_COMPID)

    records: list[dict] = []
    seen: set[tuple[int, int]] = set()
    packets = complete = plausible = 0
    window_total = window_plausible = 0
    other_types: set[str] = set()
    started = last_heartbeat = last_report = time.monotonic()
    deadline = None if args.seconds is None else started + args.seconds

    print("Move the drone slowly through the whole volume, at different heights and tilts, "
          "keeping all four sensors visible to both stations. Ctrl-C to stop.")
    try:
        while deadline is None or time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_heartbeat >= 1.0:
                for target in (link, side_link):
                    if target is not None:
                        target.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                  mavutil.mavlink.MAV_STATE_ACTIVE)
                last_heartbeat = now

            message = link.recv_match(blocking=True, timeout=0.2)
            if message is not None:
                kind = message.get_type()
                if kind != "TUNNEL":
                    other_types.add(kind)
                elif message.payload_type == LH2_TUNNEL_TYPE:
                    snapshot = parse_lh2_tunnel(bytes(message.payload[:message.payload_length]))
                    if snapshot is not None:
                        # Sequence is per boot; pair it with the time so a Pico reboot is not "duplicate".
                        key = (snapshot["sequence"], snapshot["timestamp_us"] // 10_000_000)
                        if key not in seen:
                            seen.add(key)
                            packets += 1
                            new = snapshot_to_records(snapshot)
                            records.extend(new)
                            if len(new) == NUM_BS:
                                complete += 1
                                window_total += 1
                                spans = [np.max(np.ptp(np.asarray(r["angles"]), axis=0)) for r in new]
                                if max(spans) <= span_limit:
                                    plausible += 1
                                    window_plausible += 1

            if now - last_report >= 2.0:
                last_report = now
                if packets == 0:
                    heard = ", ".join(sorted(other_types)) or "nothing"
                    print(f"[{now - started:5.0f}s] no LH2 TUNNEL yet (heard: {heard})")
                else:
                    quality = 100.0 * window_plausible / window_total if window_total else 0.0
                    print(f"[{now - started:5.0f}s] snapshots {packets}, both stations {complete}, "
                          f"physically plausible {plausible} (last 2 s: {quality:.0f} %)")
                window_total = window_plausible = 0
    except KeyboardInterrupt:
        print("\nStopped.")

    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(records, stream, indent=1)
    print(f"Wrote {len(records)} records ({complete} snapshots with both stations, "
          f"{plausible} physically plausible) to {args.output}")
    if packets == 0:
        print("No LH2 TUNNEL received. Check: Pico firmware from AUTO-CALIB-EXP or later is flashed; "
              "the link is right (wrong baud shows nothing at all); through an FC, its port to the "
              "Pico is MAVLink 2.", file=sys.stderr)
        return 1
    if complete and plausible < 0.5 * complete:
        print("Warning: most snapshots are physically impossible for the sensor board — the per-sensor "
              "angles are inconsistent. The calibration will reject them.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
