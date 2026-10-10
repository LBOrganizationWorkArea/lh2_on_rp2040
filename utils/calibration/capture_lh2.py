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

Default: the Bitcraze calibration wizard. It prompts for each step and writes
a session directory that calibrate_bitcraze.py solves directly:

  1. origin    — drone flat on the floor at the point that becomes (0, 0, 0)
  2. x-axis    — drone flat on the floor on the +X axis, --x-axis-dist (1 m) from the origin
  3. xy-plane  — drone flat on the floor at --xy-points other spots (defines Z = 0)
  4. sweep     — walk the drone through the whole volume, Ctrl-C to finish

The drone's reference point is the CENTRE of its four sensors, so place that
centre on the marks. Then:  python calibrate_bitcraze.py <session-dir>

--raw records a single free-motion file instead (old behaviour).
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


class Receiver:
    """Receives LH2 TUNNEL snapshots and keeps the GCS heartbeat going."""

    def __init__(self, connect: str, baud: int, heartbeat_to: str | None, span_limit: float):
        print(f"Connecting to {connect} ...")
        self.link = mavutil.mavlink_connection(connect, baud=baud,
                                               source_system=GCS_SYSID, source_component=GCS_COMPID)
        self.side_link = None
        if heartbeat_to:
            self.side_link = mavutil.mavlink_connection(f"udpout:{heartbeat_to}",
                                                        source_system=GCS_SYSID, source_component=GCS_COMPID)
        self.span_limit = span_limit
        self.seen: set[tuple[int, int]] = set()
        self.other_types: set[str] = set()
        self.last_heartbeat = 0.0

    def _heartbeat(self) -> None:
        now = time.monotonic()
        if now - self.last_heartbeat < 1.0:
            return
        for target in (self.link, self.side_link):
            if target is not None:
                target.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                          mavutil.mavlink.MAV_STATE_ACTIVE)
        self.last_heartbeat = now

    def _next_snapshot(self, timeout: float) -> dict | None:
        self._heartbeat()
        message = self.link.recv_match(blocking=True, timeout=timeout)
        if message is None:
            return None
        if message.get_type() != "TUNNEL":
            self.other_types.add(message.get_type())
            return None
        if message.payload_type != LH2_TUNNEL_TYPE:
            return None
        snapshot = parse_lh2_tunnel(bytes(message.payload[:message.payload_length]))
        if snapshot is None:
            return None
        # Sequence is per boot; pair it with the time so a Pico reboot is not "duplicate".
        key = (snapshot["sequence"], snapshot["timestamp_us"] // 10_000_000)
        if key in self.seen:
            return None
        self.seen.add(key)
        return snapshot

    def drain(self, max_seconds: float = 5.0) -> None:
        """Discard everything buffered while the user was reading a prompt.

        Backlog arrives back-to-back; live data arrives every ~100 ms. Stop once
        a read actually had to wait, i.e. we have caught up with the live stream.
        """
        started = time.monotonic()
        while time.monotonic() - started < max_seconds:
            before = time.monotonic()
            self._heartbeat()
            message = self.link.recv_match(blocking=True, timeout=0.2)
            if time.monotonic() - before > 0.005 or message is None:
                if time.monotonic() - started > 0.2:
                    return
            if message is not None and message.get_type() == "TUNNEL" \
                    and message.payload_type == LH2_TUNNEL_TYPE:
                snapshot = parse_lh2_tunnel(bytes(message.payload[:message.payload_length]))
                if snapshot is not None:
                    self.seen.add((snapshot["sequence"], snapshot["timestamp_us"] // 10_000_000))

    def record(self, seconds: float | None, label: str) -> tuple[list[dict], int, int]:
        """Record until `seconds` elapse (None: until Ctrl-C). Returns (records, complete, plausible)."""
        records: list[dict] = []
        complete = plausible = window_total = window_plausible = packets = 0
        started = last_report = time.monotonic()
        try:
            while seconds is None or time.monotonic() - started < seconds:
                snapshot = self._next_snapshot(0.2)
                if snapshot is not None:
                    packets += 1
                    new = snapshot_to_records(snapshot)
                    records.extend(new)
                    if len(new) == NUM_BS:
                        complete += 1
                        window_total += 1
                        spans = [np.max(np.ptp(np.asarray(r["angles"]), axis=0)) for r in new]
                        if max(spans) <= self.span_limit:
                            plausible += 1
                            window_plausible += 1
                now = time.monotonic()
                if now - last_report >= 2.0:
                    last_report = now
                    if packets == 0:
                        heard = ", ".join(sorted(self.other_types)) or "nothing"
                        print(f"  [{label} {now - started:4.0f}s] no LH2 TUNNEL yet (heard: {heard})")
                    else:
                        quality = 100.0 * window_plausible / window_total if window_total else 0.0
                        print(f"  [{label} {now - started:4.0f}s] snapshots {packets}, both stations {complete}, "
                              f"physically plausible {plausible} (last 2 s: {quality:.0f} %)")
                    window_total = window_plausible = 0
        except KeyboardInterrupt:
            print()
        return records, complete, plausible


def _save(path: str, records: list[dict]) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(records, stream, indent=1)


def _static_step(receiver: Receiver, prompt: str, seconds: float, path: str, label: str) -> None:
    """Prompt, record a static capture, and repeat until it is usable."""
    while True:
        input(f"\n{prompt}\n  Press Enter when the drone is in place and still ...")
        receiver.drain()
        records, complete, plausible = receiver.record(seconds, label)
        if complete >= 10 and plausible >= 0.5 * complete:
            _save(path, records)
            print(f"  OK: {complete} snapshots ({plausible} plausible) -> {os.path.basename(path)}")
            return
        print(f"  Not usable: {complete} snapshots with both stations, {plausible} plausible "
              "(need >= 10, mostly plausible). Check that both stations see all four sensors and retry.")


def run_wizard(receiver: Receiver, args) -> int:
    os.makedirs(args.session, exist_ok=True)
    print(f"Bitcraze calibration wizard -> {args.session}/")
    print("Place the CENTRE of the four sensors on each mark, drone flat on the floor, sensors facing up.")

    _static_step(receiver, "Step 1/4  ORIGIN: put the drone on the point that will become (0, 0, 0).",
                 args.static_seconds, os.path.join(args.session, "origin.json"), "origin")
    _static_step(receiver, f"Step 2/4  X-AXIS: put the drone on the floor {args.x_axis_dist:g} m from the origin "
                           "along the direction that will become +X.",
                 args.static_seconds, os.path.join(args.session, "x_axis.json"), "x-axis")
    xy_files = []
    for index in range(1, args.xy_points + 1):
        name = f"xy_plane_{index}.json"
        _static_step(receiver, f"Step 3/4  FLOOR {index}/{args.xy_points}: put the drone somewhere else on the "
                               "floor, spread out from the other points (not on the X axis).",
                     args.static_seconds, os.path.join(args.session, name), f"floor {index}")
        xy_files.append(name)

    input("\nStep 4/4  SWEEP: pick up the drone. Press Enter, then walk it slowly through the whole flight "
          "volume — low/middle/high, tilted up to ~30 deg, many headings, all four sensors visible to both "
          "stations. Press Ctrl-C when done (2-3 minutes) ...")
    receiver.drain()
    records, complete, plausible = receiver.record(args.seconds, "sweep")
    _save(os.path.join(args.session, "sweep.json"), records)
    print(f"  Sweep: {complete} snapshots ({plausible} plausible) -> sweep.json")

    manifest = {
        "type": "lh2_bitcraze_calibration_session",
        "origin": "origin.json",
        "x_axis": ["x_axis.json"],
        "x_axis_dist": args.x_axis_dist,
        "xy_plane": xy_files,
        "sweep": "sweep.json",
    }
    with open(os.path.join(args.session, "session.json"), "w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2)
    print(f"\nSession saved. Solve with:\n  python calibrate_bitcraze.py {args.session} --baseline <tape BS0-BS1 m>")
    if complete and plausible < 0.5 * complete:
        print("Warning: most sweep snapshots are physically impossible for the sensor board — the per-sensor "
              "angles are inconsistent. The calibration will reject them.", file=sys.stderr)
        return 1
    return 0


def run_raw(receiver: Receiver, args) -> int:
    print("Move the drone slowly through the whole volume, at different heights and tilts, "
          "keeping all four sensors visible to both stations. Ctrl-C to stop.")
    records, complete, plausible = receiver.record(args.seconds, "raw")
    _save(args.output, records)
    print(f"Wrote {len(records)} records ({complete} snapshots with both stations, "
          f"{plausible} physically plausible) to {args.output}")
    if not records:
        print("No LH2 TUNNEL received. Check: Pico firmware from AUTO-CALIB-EXP or later is flashed; "
              "the link is right (wrong baud shows nothing at all); through an FC, its port to the "
              "Pico is MAVLink 2.", file=sys.stderr)
        return 1
    if complete and plausible < 0.5 * complete:
        print("Warning: most snapshots are physically impossible for the sensor board — the per-sensor "
              "angles are inconsistent. The calibration will reject them.", file=sys.stderr)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="udpin:0.0.0.0:14550",
                        help="pymavlink connection string or serial port (default udpin:0.0.0.0:14550)")
    parser.add_argument("--baud", type=int, default=57600, help="serial baud rate (default 57600)")
    parser.add_argument("--heartbeat-to", metavar="HOST:PORT",
                        help="also send the GCS heartbeat to this UDP address (MavESP8266 learns the "
                             "GCS from it, e.g. 192.168.4.1:14555)")
    parser.add_argument("--session", default=time.strftime("calib_%Y%m%d_%H%M%S"),
                        help="wizard output directory (default calib_<date>_<time>)")
    parser.add_argument("--x-axis-dist", type=float, default=1.0,
                        help="distance of the x-axis mark from the origin [m] (Bitcraze: 1.0)")
    parser.add_argument("--xy-points", type=int, default=3, help="number of floor captures (default 3)")
    parser.add_argument("--static-seconds", type=float, default=5.0, help="length of each static capture")
    parser.add_argument("--seconds", type=float, help="stop the sweep (or --raw capture) after this many seconds")
    parser.add_argument("--raw", action="store_true", help="record one free-motion file instead of the wizard")
    parser.add_argument("-o", "--output", default="measurements.json", help="--raw output file")
    parser.add_argument("--sensor-positions", default=DEFAULT_SENSORS,
                        help="drone sensor layout, only for the live quality readout")
    parser.add_argument("--min-wand-distance", type=float, default=0.5)
    args = parser.parse_args()
    if args.xy_points < 1:
        parser.error("--xy-points must be at least 1")

    receiver = Receiver(args.connect, args.baud, args.heartbeat_to,
                        wand_span_limit(args.sensor_positions, args.min_wand_distance))
    try:
        return run_raw(receiver, args) if args.raw else run_wizard(receiver, args)
    except (KeyboardInterrupt, EOFError):
        print("\nAborted.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
