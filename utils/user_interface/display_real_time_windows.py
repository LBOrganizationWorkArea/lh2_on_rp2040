"""Standalone Windows UDP-to-HTTP bridge for MAVLink ODOMETRY.

Requires only pymavlink: py -m pip install pymavlink
"""

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from pymavlink.dialects.v20 import common as mavlink2

_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (os.path.join(_HERE, "..", "calibration"), os.path.join(_HERE, "..", "..")):
    if os.path.exists(os.path.join(_candidate, "capture_lh2.py")):
        sys.path.insert(0, os.path.abspath(_candidate))
        break
try:
    import capture_lh2 as lh2cap
except ImportError as _error:  # numpy missing or capture_lh2.py not found
    lh2cap = None
    print(f"LH2 calibration capture disabled: {_error}")

STEP_NAMES = ("origin", "x_axis", "sweep")


class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.sequence = 0
        self.odometry = None
        self.target_host = ""
        self.target_port = 14550

    def set_target(self, host, port):
        with self.lock:
            self.target_host = host
            self.target_port = port

    def get_target(self):
        with self.lock:
            return self.target_host, self.target_port

    def publish(self, message):
        with self.lock:
            self.sequence += 1
            self.odometry = message
            return self.sequence

    def snapshot(self):
        with self.lock:
            return {"sequence": self.sequence, "odometry": self.odometry}


class CaptureManager:
    """Records LH2 TUNNEL snapshots for the calibration wizard (steps driven from the web UI)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.session_dir = ""
        self.config = {"x_axis_dist": 1.0, "xy_points": 3, "static_seconds": 5.0}
        self.target = None  # (host, port) where the extra GCS heartbeat goes
        self.span_limit = None
        self.seen = set()
        self.packets = 0
        self.last_packet = 0.0
        self.recent = []  # (time, complete, plausible) for the live quality figure
        self.active = None  # running capture
        self.last_result = None
        self.done = {}
        self.solve = {"state": "idle", "log": "", "poses": None, "ok": None}
        if lh2cap is not None:
            sensors = os.path.join(os.path.dirname(os.path.abspath(lh2cap.__file__)), "wand_sensors.json")
            try:
                self.span_limit = lh2cap.wand_span_limit(sensors, 0.5)
            except OSError:
                print("wand_sensors.json not found: plausibility check disabled")

    def set_target(self, host, port):
        with self.lock:
            self.target = (host, port) if host else None

    def get_target(self):
        with self.lock:
            return self.target

    def configure(self, session_dir, config):
        with self.lock:
            os.makedirs(session_dir, exist_ok=True)
            self.session_dir = os.path.abspath(session_dir)
            self.config.update(config)
            self.done = {name[:-5]: True for name in os.listdir(self.session_dir) if name.endswith(".json")}
            self.last_result = None

    def _plausible(self, records):
        if self.span_limit is None:
            return True
        spans = [max(max(a[i] for a in r["angles"]) - min(a[i] for a in r["angles"]) for i in (0, 1)) for r in records]
        return max(spans) <= self.span_limit

    def feed(self, payload):
        if lh2cap is None:
            return
        snapshot = lh2cap.parse_lh2_tunnel(payload)
        if snapshot is None:
            return
        key = (snapshot["sequence"], snapshot["timestamp_us"] // 10_000_000)
        with self.lock:
            if key in self.seen:
                return
            self.seen.add(key)
            now = time.monotonic()
            self.packets += 1
            self.last_packet = now
            records = lh2cap.snapshot_to_records(snapshot)
            complete = len(records) == lh2cap.NUM_BS
            plausible = complete and self._plausible(records)
            self.recent = [r for r in self.recent if now - r[0] < 2.0] + [(now, complete, plausible)]
            cap = self.active
            if cap is not None:
                cap["records"].extend(records)
                cap["snapshots"] += 1
                cap["complete"] += int(complete)
                cap["plausible"] += int(plausible)
            self._maybe_finish(now)

    def start(self, step, seconds):
        with self.lock:
            if lh2cap is None:
                raise ValueError("capture_lh2 module unavailable (install numpy)")
            if not self.session_dir:
                raise ValueError("session not configured")
            if self.active is not None:
                raise ValueError("a capture is already running")
            if step != "sweep" and not (step == "origin" or step == "x_axis" or step.startswith("xy_plane_")):
                raise ValueError("unknown step")
            self.active = {"step": step, "seconds": None if step == "sweep" else seconds,
                           "started": time.monotonic(), "records": [], "snapshots": 0,
                           "complete": 0, "plausible": 0}
            self.last_result = None

    def stop(self):
        with self.lock:
            self._finish(force=True)

    def _maybe_finish(self, now):
        cap = self.active
        if cap and cap["seconds"] is not None and now - cap["started"] >= cap["seconds"]:
            self._finish()

    @staticmethod
    def _clean_static(records, limit_deg=3.0):
        # Wand is still during static steps: drop snapshots with a glitched sensor (far from the per-BS median).
        kept = []
        for bs in sorted({r["base_station_id"] for r in records}):
            rows = [r for r in records if r["base_station_id"] == bs]
            med = {}
            for sensor in range(4):
                for axis in (0, 1):
                    values = sorted(math.degrees(r["angles"][sensor][axis]) for r in rows)
                    med[sensor, axis] = values[len(values) // 2]
            kept += [r for r in rows
                     if all(abs(math.degrees(r["angles"][sn][ax]) - m) <= limit_deg for (sn, ax), m in med.items())]
        kept.sort(key=lambda r: (r["timestamp"], r["base_station_id"]))
        return kept

    def _clean_session_files(self, session):
        for name in os.listdir(session):
            if name in ("origin.json", "x_axis.json") or name.startswith("xy_plane_"):
                path = os.path.join(session, name)
                with open(path, encoding="utf-8") as stream:
                    records = json.load(stream)
                cleaned = self._clean_static(records)
                if len(cleaned) != len(records):
                    with open(path, "w", encoding="utf-8") as stream:
                        json.dump(cleaned, stream, indent=1)

    def _finish(self, force=False):
        cap = self.active
        if cap is None:
            return
        self.active = None
        step, complete, plausible = cap["step"], cap["complete"], cap["plausible"]
        if cap["step"] == "sweep":
            ok = complete > 0
        else:
            ok = complete >= 10 and plausible >= 0.5 * complete
        message = f"{complete} snapshots with both stations, {plausible} plausible"
        if ok and step != "sweep":
            cleaned = self._clean_static(cap["records"])
            if len(cleaned) < 0.7 * len(cap["records"]):
                ok = False
                message += "; wand not still"
            cap["records"] = cleaned
        if ok:
            with open(os.path.join(self.session_dir, step + ".json"), "w", encoding="utf-8") as stream:
                json.dump(cap["records"], stream, indent=1)
            self.done[step] = True
            message = f"OK: {message} -> {step}.json"
        else:
            message = f"Not usable: {message} (need >= 10, mostly plausible, wand still). Check that both stations see all four sensors; retry."
        self.last_result = {"step": step, "ok": ok, "message": message}

    def finish_session(self):
        with self.lock:
            if not self.session_dir:
                raise ValueError("session not configured")
            n = int(self.config["xy_points"])
            manifest = {
                "type": "lh2_bitcraze_calibration_session",
                "origin": "origin.json",
                "x_axis": ["x_axis.json"],
                "x_axis_dist": float(self.config["x_axis_dist"]),
                "xy_plane": [f"xy_plane_{i}.json" for i in range(1, n + 1)],
                "sweep": "sweep.json",
            }
            missing = [f for f in [manifest["origin"], *manifest["x_axis"], *manifest["xy_plane"], manifest["sweep"]]
                       if not os.path.exists(os.path.join(self.session_dir, f))]
            if missing:
                raise ValueError("missing steps: " + ", ".join(missing))
            with open(os.path.join(self.session_dir, "session.json"), "w", encoding="utf-8") as stream:
                json.dump(manifest, stream, indent=2)
            return self.session_dir

    def start_solve(self, baseline, board_height):
        with self.lock:
            if lh2cap is None:
                raise ValueError("capture_lh2 module unavailable")
            if self.solve["state"] == "running":
                raise ValueError("solver already running")
            if not os.path.exists(os.path.join(self.session_dir or "", "session.json")):
                raise ValueError("finish the session first (session.json missing)")
            self.solve = {"state": "running", "log": "", "poses": None, "ok": None}
            session = self.session_dir
            self._clean_session_files(session)
        threading.Thread(target=self._run_solve, args=(session, baseline, board_height), daemon=True).start()

    def _run_solve(self, session, baseline, board_height):
        calib_dir = os.path.dirname(os.path.abspath(lh2cap.__file__))
        script = os.path.join(calib_dir, "calibrate_bitcraze.py")
        output = os.path.join(session, "lighthouse_geometry_candidate.yaml")
        command = [sys.executable, "-u", script, session, "-o", output]
        if baseline:
            command += ["--baseline", str(baseline)]
        if board_height:
            command += ["--board-height", str(board_height)]
        poses, ok = None, False
        try:
            process = subprocess.Popen(command, cwd=os.path.abspath(os.path.join(calib_dir, "..", "..")),
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, errors="replace")
            for line in process.stdout:
                with self.lock:
                    self.solve["log"] += line
            ok = process.wait() == 0
            if ok:
                import yaml
                with open(output, encoding="utf-8") as stream:
                    geos = yaml.safe_load(stream)["geos"]
                poses = [{"id": int(k), "position": v["position"], "quat": v["rotation_quat"]}
                         for k, v in sorted(geos.items(), key=lambda kv: int(kv[0]))]
        except Exception as error:  # report any solver/launch failure to the UI
            with self.lock:
                self.solve["log"] += f"\nSolver launch failed: {error}\n"
        with self.lock:
            self.solve.update(state="done", ok=ok, poses=poses)

    def status(self):
        with self.lock:
            now = time.monotonic()
            self._maybe_finish(now)
            recent = [r for r in self.recent if now - r[0] < 2.0]
            total = sum(1 for r in recent if r[1])
            good = sum(1 for r in recent if r[2])
            cap = self.active
            return {
                "available": lh2cap is not None,
                "packets": self.packets,
                "last_packet_age": (now - self.last_packet) if self.packets else None,
                "quality": (100.0 * good / total) if total else None,
                "session_dir": self.session_dir,
                "config": self.config,
                "done": sorted(self.done),
                "capture": None if cap is None else {
                    "step": cap["step"], "elapsed": now - cap["started"], "seconds": cap["seconds"],
                    "snapshots": cap["snapshots"], "complete": cap["complete"], "plausible": cap["plausible"],
                },
                "last_result": self.last_result,
                "solve": dict(self.solve),
            }


class GcsEncoder:
    """Builds GCS-originated MAVLink v2 frames with an incrementing sequence number."""

    def __init__(self):
        self.mav = mavlink2.MAVLink(None)
        self.mav.srcSystem = 255
        self.mav.srcComponent = 190

    def heartbeat(self):
        return mavlink2.MAVLink_heartbeat_message(
            mavlink2.MAV_TYPE_GCS, mavlink2.MAV_AUTOPILOT_INVALID, 0, 0,
            mavlink2.MAV_STATE_ACTIVE, 3,
        ).pack(self.mav)

    def request_streams(self, sysid, compid):
        frames = []
        for stream_id in (mavlink2.MAV_DATA_STREAM_POSITION, mavlink2.MAV_DATA_STREAM_EXTRA3):
            frames.append(
                mavlink2.MAVLink_request_data_stream_message(
                    sysid, compid, stream_id, 4, 1
                ).pack(self.mav)
            )
        return frames


def udp_receiver(state, capture, listen_host, listen_port):
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    encoder = GcsEncoder()
    next_heartbeat = 0.0
    peers = {}  # address -> last time a stream request was sent
    peer_addresses = set()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind((listen_host, listen_port))
        udp.settimeout(0.1)
        print(f"MAVLink UDP in ascolto su {listen_host}:{listen_port}")

        while True:
            now = time.monotonic()
            if now >= next_heartbeat:
                # Heartbeat to the configured target AND to every address that has sent us data,
                # so the FC/bridge learns our address even without Mission Planner.
                destinations = set(peer_addresses)
                target_host, target_port = state.get_target()
                if target_host:
                    destinations.add((target_host, target_port))
                if capture.get_target():
                    destinations.add(capture.get_target())
                for destination in destinations:
                    try:
                        udp.sendto(encoder.heartbeat(), destination)
                    except OSError as error:
                        print(f"Heartbeat UDP: {error}")
                next_heartbeat = now + 1.0

            try:
                packet, address = udp.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as error:
                print(f"Ricezione UDP: {error}")
                continue

            if address not in peer_addresses:
                peer_addresses.add(address)
                print(f"Peer MAVLink: {address[0]}:{address[1]}")

            for value in packet:
                message = parser.parse_char(bytes((value,)))
                if message is None:
                    continue
                kind = message.get_type()

                if kind == "HEARTBEAT" and message.type != mavlink2.MAV_TYPE_GCS:
                    sysid = message.get_srcSystem()
                    if now - peers.get(address, 0.0) > 10.0:
                        peers[address] = now
                        for frame in encoder.request_streams(sysid, message.get_srcComponent()):
                            try:
                                udp.sendto(frame, address)
                            except OSError as error:
                                print(f"Stream request: {error}")
                    continue

                if kind == "TUNNEL":
                    if message.payload_type == 0x4C48:
                        capture.feed(bytes(message.payload[:message.payload_length]))
                    continue

                if kind != "ODOMETRY":
                    continue
                if message.frame_id != 20 or message.child_frame_id != 12:
                    continue

                position = {
                    "x": float(message.x),
                    "y": float(message.y),
                    "z": -float(message.z),
                }
                sequence = state.publish(position)
                if sequence == 1 or sequence % 10 == 0:
                    print(
                        "ODOMETRY "
                        f"x={position['x']:.3f} y={position['y']:.3f} "
                        f"z={position['z']:.3f} m"
                    )


def make_handler(state, capture):
    class Handler(BaseHTTPRequestHandler):
        def _cors_headers(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Cache-Control", "no-store")

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors_headers()
            self.end_headers()

        def _json(self, payload, status=200):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self._cors_headers()
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _lh2(self, request):
            query = parse_qs(request.query)
            arg = lambda name, default="": query.get(name, [default])[0].strip()
            action = request.path[len("/api/lh2/"):]
            try:
                host = arg("host")
                if host:
                    capture.set_target(host, int(arg("port", "14555")))
                if action == "status":
                    pass
                elif action == "session":
                    capture.configure(arg("dir") or time.strftime("calib_runs/run_%Y%m%d_%H%M%S"), {
                        "x_axis_dist": float(arg("x_axis_dist", "1.0")),
                        "xy_points": max(1, int(arg("xy_points", "3"))),
                        "static_seconds": float(arg("static_seconds", "5.0")),
                    })
                elif action == "start":
                    capture.start(arg("step"), float(arg("seconds", "5.0")))
                elif action == "stop":
                    capture.stop()
                elif action == "finish":
                    capture.finish_session()
                elif action == "solve":
                    capture.start_solve(float(arg("baseline", "0") or 0), float(arg("board_height", "0") or 0))
                else:
                    self.send_error(404)
                    return
            except (ValueError, OSError) as error:
                self._json({"error": str(error)}, 400)
                return
            self._json(capture.status())

        def do_POST(self):
            self.do_GET()

        def do_GET(self):
            request = urlsplit(self.path)
            if request.path.startswith("/api/lh2/"):
                self._lh2(request)
                return
            if request.path != "/api/odometry":
                self.send_error(404)
                return

            query = parse_qs(request.query)
            host = query.get("host", [""])[0].strip()
            try:
                port = int(query.get("port", ["14550"])[0])
                if not 1 <= port <= 65535:
                    raise ValueError
            except ValueError:
                self.send_error(400, "Invalid UDP port")
                return

            state.set_target(host, port)
            self._json(state.snapshot())

        def log_message(self, _format, *_args):
            pass

    return Handler


def main():
    cli = argparse.ArgumentParser(description="Standalone MAVLink ODOMETRY UDP relay")
    cli.add_argument("--udp-host", default="0.0.0.0", help="UDP bind address")
    cli.add_argument("--udp-port", type=int, default=14550, help="Local UDP listen port")
    cli.add_argument("--http-port", type=int, default=8051, help="Local HTTP API port")
    args = cli.parse_args()

    state = BridgeState()
    capture = CaptureManager()
    receiver = threading.Thread(
        target=udp_receiver,
        args=(state, capture, args.udp_host, args.udp_port),
        daemon=True,
    )
    receiver.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(state, capture))
    print(f"HTTP API: http://127.0.0.1:{args.http_port}/api/odometry")
    print("Lascia aperta questa finestra mentre usi il Pose Editor.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nRelay arrestato.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
