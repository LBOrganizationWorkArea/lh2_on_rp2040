"""Standalone Windows UDP-to-HTTP bridge for MAVLink ODOMETRY.

Requires only pymavlink: py -m pip install pymavlink
"""

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from pymavlink.dialects.v20 import common as mavlink2


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


def make_heartbeat():
    encoder = mavlink2.MAVLink(None)
    encoder.srcSystem = 255
    encoder.srcComponent = 190
    message = mavlink2.MAVLink_heartbeat_message(
        mavlink2.MAV_TYPE_GCS,
        mavlink2.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavlink2.MAV_STATE_ACTIVE,
        3,
    )
    return message.pack(encoder)


def udp_receiver(state, listen_host, listen_port):
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    heartbeat = make_heartbeat()
    next_heartbeat = 0.0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind((listen_host, listen_port))
        udp.settimeout(0.1)
        print(f"MAVLink UDP in ascolto su {listen_host}:{listen_port}")

        while True:
            target_host, target_port = state.get_target()
            now = time.monotonic()
            if target_host and now >= next_heartbeat:
                try:
                    udp.sendto(heartbeat, (target_host, target_port))
                except OSError as error:
                    print(f"Heartbeat UDP: {error}")
                next_heartbeat = now + 1.0

            try:
                packet, _address = udp.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as error:
                print(f"Ricezione UDP: {error}")
                continue

            for value in packet:
                message = parser.parse_char(bytes((value,)))
                if message is None or message.get_type() != "ODOMETRY":
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


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def _cors_headers(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Cache-Control", "no-store")

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors_headers()
            self.end_headers()

        def do_GET(self):
            request = urlsplit(self.path)
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
            body = json.dumps(state.snapshot()).encode("utf-8")
            self.send_response(200)
            self._cors_headers()
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

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
    receiver = threading.Thread(
        target=udp_receiver,
        args=(state, args.udp_host, args.udp_port),
        daemon=True,
    )
    receiver.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(state))
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
