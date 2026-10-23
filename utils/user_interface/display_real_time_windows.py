"""Standalone Windows UDP-to-HTTP bridge for MAVLink ODOMETRY.

Requires only pymavlink: py -m pip install pymavlink
"""

import argparse
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from pymavlink.dialects.v20 import ardupilotmega as mavlink2  # common + ArduPilot messages (EKF_STATUS_REPORT)

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
CALIB_DIR = os.path.dirname(os.path.abspath(lh2cap.__file__)) if lh2cap else os.path.join(_HERE, "..", "calibration")
# Sessions live next to the calibration scripts, unless LH2_RUNS_DIR says otherwise (the desktop
# app points it to a writable user folder, since its install folder is read-only).
RUNS_DIR = os.path.abspath(os.environ.get("LH2_RUNS_DIR") or os.path.join(CALIB_DIR, "calib_runs"))


def session_path(directory):
    """Absolute session folder: absolute paths as given, 'calib_runs/<name>' or '<name>' inside RUNS_DIR."""
    if os.path.isabs(directory):
        return os.path.abspath(directory)
    parts = directory.replace("\\", "/").split("/")
    if parts and parts[0] == "calib_runs":
        parts = parts[1:]
    return os.path.abspath(os.path.join(RUNS_DIR, *parts))
# Base stations' factory calibration (OOTX, utils/calibration/lh2_ootx.py). The desktop app sets
# LH2_DATA_DIR to a writable folder; until something is read there the shipped file is used.
DATA_DIR = os.path.abspath(os.environ.get("LH2_DATA_DIR") or CALIB_DIR)
SHIPPED_FACTORY_CAL = os.path.join(CALIB_DIR, "lh2_factory_calibration.json")
FACTORY_CAL = os.path.join(DATA_DIR, "lh2_factory_calibration.json")
# Station poses compiled into the firmware: repo layout, then the Windows bundle layout.
FIRMWARE_POSES = next((p for p in (os.path.join(CALIB_DIR, "..", "..", "rp2350_firmware", "src", "bs_poses_cal.h"),
                                    os.path.join(CALIB_DIR, "..", "..", "firmware", "bs_poses_cal.h"))
                       if os.path.exists(p)), None)


def _world_heading():
    """(cos, sin) of BS_WORLD_HEADING_DEG, the +X heading main.c turns ODOMETRY by (env
    LH2_WORLD_HEADING_DEG overrides the header, for a drone flashed with another build)."""
    degrees = os.environ.get("LH2_WORLD_HEADING_DEG")
    if degrees is None and FIRMWARE_POSES:
        import re
        with open(FIRMWARE_POSES, encoding="utf-8", errors="replace") as stream:
            found = re.search(r"#define\s+BS_WORLD_HEADING_DEG\s+([-+0-9.eE]+)", stream.read())
        degrees = found.group(1) if found else None
    angle = math.radians(float(degrees or 0.0))
    return math.cos(angle), math.sin(angle)


WORLD_HEADING = _world_heading()
DISPLAY_CLOUD_MAX = 2000  # room-recording points kept for the 3D view
# Calibration scripts print and write UTF-8 (e.g. headers) also on Windows, where the default is cp1252.
# One BLAS thread per solver: ~2x faster on these small problems, and parallel solves do not fight.
_CHILD_ENV = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8",
                  OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")

# Key lines of calibrate_bitcraze.py's report, shown as figures in the pose editor.
_LOG_METRICS = (
    ("residual_mean_mm", r"Residual: mean ([\d.]+) mm"),
    ("residual_max_mm", r"Residual: mean [\d.]+ mm, max ([\d.]+) mm"),
    ("board_fit_mm", r"Wand shape check: median rigid-fit error ([\d.]+) mm"),
    ("mark_dist_m", r"x-axis capture is ([\d.]+) m from the origin"),
    ("baseline_m", r"Estimated baseline: ([\d.]+) m"),
    ("tape_m", r"Tape baseline ([\d.]+) m"),
    ("samples", r"final solve used (\d+)"),
    # The solver stopped at the board-shape check: geometry not written (see board_fit_mm).
    ("board_fit_refused_mm", r"do not form the wand .*?\(median rigid-fit error ([\d.]+) mm >"),
)


def _read_text(path):
    if not os.path.exists(path):
        return ""
    with open(path, encoding="utf-8", errors="replace") as stream:
        return stream.read()


def load_solve_result(directory, name):
    """Poses, precision and report figures of a solved session directory (geometry.yaml + solve.log)."""
    import re
    import yaml
    result = {"name": name, "dir": directory, "ok": False, "poses": None, "precision": {}, "metrics": {}}
    log = _read_text(os.path.join(directory, "solve.log"))
    for key, pattern in _LOG_METRICS:
        match = re.search(pattern, log)
        if match:
            result["metrics"][key] = float(match.group(1))
    distances = re.search(r"Station distances: (.+)", log)  # printed with three or more stations
    if distances:
        result["distances"] = distances.group(1).strip()
    geometry = os.path.join(directory, "geometry.yaml")
    if not os.path.exists(geometry):
        return result
    with open(geometry, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    result["poses"] = [{"id": int(k), "position": v["position"], "quat": v["rotation_quat"]}
                       for k, v in sorted(data["geos"].items(), key=lambda kv: int(kv[0]))]
    result["precision"] = {k: {"value": v["value"], "std": v["std"]}
                           for k, v in (data.get("uncertainty_1sigma") or {}).items()}
    result["ok"] = True
    return result


def factory_summary(data):
    """{station index: uid, polynomials, accelerometer tilt, plane phase/tilt/gibmag [deg]} for the page."""
    stations = {}
    for bs, calib in data.items():
        frame = calib.get("raw_frame", {})
        accel = [frame.get("accel_x"), frame.get("accel_y"), frame.get("accel_z")]
        tilt = None
        if None not in accel and any(accel):
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, accel[2] / math.hypot(*accel)))))
        stations[bs] = {
            "uid": calib.get("uid"),
            "polynomials": calib.get("polynomials"),
            "accel_tilt_deg": tilt,
            "planes": [{k: math.degrees(s[k]) for k in ("phase", "tilt", "gibmag")} for s in calib.get("sweeps", [])],
            "frames_decoded": calib.get("frames_decoded"),
        }
    return stations


def load_factory_calibration():
    """OOTX factory data per station (lh2_ootx.py), with the tilt its accelerometer reports."""
    path = FACTORY_CAL if os.path.exists(FACTORY_CAL) else SHIPPED_FACTORY_CAL
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as stream:
        return factory_summary(json.load(stream))


def factory_header_path():
    """Where lh2_factory_cal.h goes: the firmware sources when running from the repository,
    else the data folder (the desktop app cannot rebuild the firmware)."""
    if FIRMWARE_POSES and os.path.basename(os.path.dirname(FIRMWARE_POSES)) == "src" \
            and os.access(os.path.dirname(FIRMWARE_POSES), os.W_OK):
        return os.path.normpath(os.path.join(os.path.dirname(FIRMWARE_POSES), "lh2_factory_cal.h"))
    return os.path.join(DATA_DIR, "lh2_factory_cal.h")


class OotxReader:
    """Reads the base stations' factory calibration (OOTX) from the Pico's USB serial port.

    The firmware streams one "O,..." line per decoded sweep after receiving 'o' (see
    angle_decoder_set_ootx_stream); lh2_ootx.Collector turns them into OOTX frames. A frame
    needs ~6-8 s of unbroken rotor turns per station: drone still, sensors in view."""

    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self._reset()

    def _reset(self, **extra):
        wireless = self.state.get("wireless", {}) if hasattr(self, "state") else {}
        self.state = {"state": "idle", "port": None, "elapsed": 0.0, "lines": 0, "stations": {},
                      "decoded": {}, "error": None, "saved": None, "wireless": wireless, **extra}
        self.results = {}
        if not hasattr(self, "wireless_results"):
            self.wireless_results = {}  # {station index: lh2_ootx calibration dict}, from the drone

    # Firmware station index -> polynomial pair (angle_decoder.h)
    POLYNOMIALS = {0: [8, 9], 1: [20, 21], 2: [0, 1], 3: [2, 3]}

    def feed_wireless(self, message):
        """An "LO" TUNNEL from the drone: what it has decoded by itself (capture_lh2.parse_lh2_ootx)."""
        bs = message["bs"]
        entry = {"bits": message["bits"], "frames_ok": message["frames_ok"], "valid": message["valid"],
                 "missed": message.get("missed"), "coverage": message.get("coverage"), "at": time.time()}
        if message["valid"]:
            try:
                import lh2_ootx
                calibration = lh2_ootx.to_calibration(lh2_ootx.parse_payload(message["frame"]))
            except Exception:  # malformed frame: keep the counters only
                calibration = None
            if calibration is not None:
                calibration["frames_decoded"] = message["frames_ok"]
                calibration["polynomials"] = self.POLYNOMIALS.get(bs, [])
                entry.update(factory_summary({str(bs): calibration})[str(bs)])
                with self.lock:
                    self.wireless_results[bs] = calibration
        with self.lock:
            self.state["wireless"][str(bs)] = entry

    def wireless_ids(self):
        with self.lock:
            return {str(bs) for bs in self.wireless_results}

    @staticmethod
    def ports():
        try:
            from serial.tools import list_ports
        except ImportError:
            return []
        found = []
        for port in list_ports.comports():
            pico = port.vid == 0x2E8A  # Raspberry Pi (RP2040 / RP2350)
            found.append({"device": port.device, "description": port.description or "", "pico": pico})
        return sorted(found, key=lambda p: (not p["pico"], p["device"]))

    def start(self, port, seconds, expected):
        with self.lock:
            if self.state["state"] == "reading":
                raise ValueError("already reading")
            if not port:
                raise ValueError("choose the drone's USB serial port")
            self._reset(state="reading", port=port)
            self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, args=(port, seconds, expected), daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def _run(self, port, seconds, expected):
        try:
            import serial  # pyserial
            import lh2_ootx
        except ImportError as error:
            with self.lock:
                self.state.update(state="error", error=f"missing module: {error}")
            return
        collector = lh2_ootx.Collector()
        started = last = time.monotonic()
        error = None
        try:
            with serial.Serial(port, 115200, timeout=0.2) as link:
                link.write(b"o")  # start the O-line stream
                try:
                    while not self.stop_event.is_set() and time.monotonic() - started < seconds:
                        line = link.readline().decode(errors="replace")
                        if line:
                            collector.add(line)
                        now = time.monotonic()
                        if now - last >= 2.0:
                            last = now
                            self._update(collector, lh2_ootx, now - started)
                            if len(self.results) >= expected:
                                break
                finally:
                    link.write(b"x")  # stop it again
        except Exception as exc:  # port busy / unplugged / not a serial device
            error = f"{type(exc).__name__}: {exc}"
        self._update(collector, lh2_ootx, time.monotonic() - started)
        with self.lock:
            if error:
                self.state.update(state="error", error=error)
            elif collector.lines == 0:
                self.state.update(state="error", error="no O lines from this port: is it the drone's Pico, "
                                                       "running the LH2 firmware?")
            else:
                self.state["state"] = "done"

    def _update(self, collector, lh2_ootx, elapsed):
        stations = {}
        for bs, hits in sorted(collector.hits.items()):
            longest = max((len(segment) for segment in lh2_ootx.bit_segments(hits)), default=0)
            stations[str(bs)] = {"hits": len(hits), "longest_bits": longest}
        results = collector.decode()
        with self.lock:
            self.results = results
            self.state.update(elapsed=elapsed, lines=collector.lines, stations=stations,
                              decoded=factory_summary({str(k): v for k, v in results.items()}))

    def save(self, source="usb"):
        """Merge the stations decoded now (over USB, or by the drone itself: source="wireless")
        into the factory-calibration JSON and write the header."""
        import shutil as _shutil
        import lh2_ootx
        from pathlib import Path
        with self.lock:
            results = dict(self.wireless_results if source == "wireless" else self.results)
        if not results:
            raise ValueError("no station decoded yet")
        if not os.path.exists(FACTORY_CAL) and os.path.exists(SHIPPED_FACTORY_CAL):
            os.makedirs(DATA_DIR, exist_ok=True)
            _shutil.copy2(SHIPPED_FACTORY_CAL, FACTORY_CAL)  # keep the stations decoded before
        header = factory_header_path()
        merged, kept = lh2_ootx.save_calibration(results, Path(FACTORY_CAL), Path(header))
        saved = {"json": FACTORY_CAL, "header": header, "stations": sorted(merged), "kept": kept,
                 "updated": sorted(results), "source": source}
        with self.lock:
            self.state["saved"] = saved
        return saved

    def status(self):
        now = time.time()
        with self.lock:
            state = dict(self.state)
            state["wireless"] = {bs: {**entry, "age_s": round(now - entry["at"], 1)}
                                 for bs, entry in self.state["wireless"].items()}
            return state


def list_runs():
    if not os.path.isdir(RUNS_DIR):
        return []
    runs = []
    for name in sorted(os.listdir(RUNS_DIR)):
        path = os.path.join(RUNS_DIR, name)
        if os.path.isdir(path) and os.path.exists(os.path.join(path, "session.json")):
            runs.append({"name": name, "dir": path, "solved": os.path.exists(os.path.join(path, "geometry.yaml"))})
    return runs


class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.sequence = 0
        self.odometry = None
        self.odometry_at = 0.0
        self.history = []           # (time, x, y, z) of the last 2 s, to compare with the late EKF
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
            self.odometry_at = time.monotonic()
            self.history.append((self.odometry_at, message["x"], message["y"], message["z"]))
            while self.history and self.odometry_at - self.history[0][0] > 2.0:
                self.history.pop(0)
            return self.sequence

    def recent(self, since, until):
        """World positions received between two monotonic times."""
        with self.lock:
            return [(x, y, z) for t, x, y, z in self.history if since <= t <= until]

    def latest(self):
        """(world position dict, age [s]) of the last ODOMETRY, or (None, inf)."""
        with self.lock:
            if self.odometry is None:
                return None, float("inf")
            return dict(self.odometry), time.monotonic() - self.odometry_at

    def snapshot(self):
        with self.lock:
            return {"sequence": self.sequence, "odometry": self.odometry}


class FlightController:
    """The flight controller seen over the same UDP link: telemetry for the editor's HUD and
    parameter read / write (only the ones in WRITABLE, with ArduPilot's ranges)."""

    WRITABLE = {  # name -> (min, max, integer)
        "VISO_DELAY_MS": (0.0, 250.0, True),
        "VISO_POS_M_NSE": (0.01, 10.0, False),  # ArduPilot documents 0.1..10 but accepts less
    }
    GPS_FIX = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}

    def __init__(self):
        self.cond = threading.Condition()
        self.hud = {}
        self.params = {}            # name -> {"value", "type", "at"}
        self.sysid = self.compid = None
        self.address = None         # where the FC's packets come from (the radio)
        self.udp = self.encoder = None
        self.last_at = 0.0
        self.last_seq = None
        self.link = []              # (time, received, lost) per FC message, last 5 s
        self.local_ned = None       # (north, east, down, time) from LOCAL_POSITION_NED

    def attach(self, udp, encoder):
        self.udp, self.encoder = udp, encoder

    def feed(self, message, address, kind):
        """Every message from the flight controller (component 1)."""
        if message.get_srcComponent() != 1:
            return
        now = time.monotonic()
        with self.cond:
            if self.sysid is None or message.get_srcSystem() == self.sysid:
                seq = message.get_seq()
                lost = 0 if self.last_seq is None else (seq - self.last_seq - 1) % 256
                self.last_seq = seq
                self.link.append((now, lost))
                while self.link and now - self.link[0][0] > 5.0:
                    self.link.pop(0)
            h = self.hud
            if kind == "HEARTBEAT":
                if message.autopilot != mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA:
                    return
                first = self.sysid is None
                self.sysid, self.compid, self.address = message.get_srcSystem(), message.get_srcComponent(), address
                if first:  # which channel is which stick (RCMAP_*), for the hold accuracy
                    threading.Thread(target=self._read_rcmap, daemon=True).start()
                h["armed"] = bool(message.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    from pymavlink import mavutil
                    h["mode"] = mavutil.mode_string_v10(message)
                except Exception:  # unknown vehicle type: the number
                    h["mode"] = f"Mode {message.custom_mode}"
            elif kind == "ATTITUDE":
                h.update(roll=math.degrees(message.roll), pitch=math.degrees(message.pitch),
                         yaw=math.degrees(message.yaw), yawspeed=math.degrees(message.yawspeed))
            elif kind == "VFR_HUD":
                h.update(airspeed=message.airspeed, groundspeed=message.groundspeed, heading=message.heading,
                         throttle=message.throttle, climb=message.climb)
                h.setdefault("alt", message.alt)
            elif kind == "GLOBAL_POSITION_INT":
                h["alt"] = message.relative_alt / 1000.0      # above home, like Mission Planner
            elif kind == "LOCAL_POSITION_NED":
                self.local_ned = (message.x, message.y, message.z, now)
            elif kind == "SYS_STATUS":
                h.update(voltage=message.voltage_battery / 1000.0,
                         current=message.current_battery / 100.0 if message.current_battery >= 0 else None,
                         battery=message.battery_remaining if message.battery_remaining >= 0 else None)
            elif kind == "GPS_RAW_INT":
                h.update(gps=self.GPS_FIX.get(message.fix_type, str(message.fix_type)), sats=message.satellites_visible)
            elif kind == "NAV_CONTROLLER_OUTPUT":
                h.update(wp_dist=message.wp_dist, xtrack=message.xtrack_error)
            elif kind == "MISSION_CURRENT":
                h["wp"] = message.seq
            elif kind == "SYSTEM_TIME":
                h.update(time_unix=message.time_unix_usec / 1e6, time_boot=message.time_boot_ms / 1000.0)
            elif kind == "EKF_STATUS_REPORT":
                h["ekf_flags"] = message.flags
            elif kind == "RC_CHANNELS":       # channels 1..8; which one is which stick: hud["rcmap"]
                h["rc"] = [getattr(message, f"chan{i}_raw") for i in range(1, 9)]
            elif kind == "STATUSTEXT":
                h["text"] = message.text.rstrip("\x00")
                h["text_at"] = now
            elif kind == "PARAM_VALUE":
                name = message.param_id.rstrip("\x00") if isinstance(message.param_id, str) else message.param_id
                self.params[name] = {"value": float(message.param_value), "type": message.param_type, "at": now}
                self.cond.notify_all()
            self.last_at = now

    def _read_rcmap(self):
        """RCMAP_ROLL / PITCH / THROTTLE / YAW (1-based channels; radios often put throttle on 1)."""
        rcmap = {}
        for attempt in range(5):
            for stick in ("ROLL", "PITCH", "THROTTLE", "YAW"):
                if stick.lower() not in rcmap:
                    try:
                        rcmap[stick.lower()] = int(self.param("RCMAP_" + stick)["value"])
                    except (ValueError, OSError):
                        pass
            if len(rcmap) == 4:
                break
            time.sleep(2.0)
        with self.cond:
            self.hud["rcmap"] = rcmap

    def snapshot(self):
        now = time.monotonic()
        with self.cond:
            if not self.last_at:
                return None
            received = len(self.link)
            lost = sum(lost for _, lost in self.link)
            hud = dict(self.hud, age=round(now - self.last_at, 2),
                       link=round(100.0 * received / (received + lost)) if received else None)
            if "text_at" in hud:
                hud["text_age"] = round(now - hud.pop("text_at"), 1)
            return hud

    def local(self):
        """((north, east, down), age [s]) of the EKF position, or (None, inf)."""
        with self.cond:
            if self.local_ned is None:
                return None, float("inf")
            n, e, d, at = self.local_ned
            return (n, e, d), time.monotonic() - at

    def local_at(self):
        """((north, east, down), monotonic time it arrived) or (None, None)."""
        with self.cond:
            if self.local_ned is None:
                return None, None
            n, e, d, at = self.local_ned
            return (n, e, d), at

    def status(self):
        with self.cond:
            return {"armed": self.hud.get("armed"), "mode": self.hud.get("mode"),
                    "yaw": self.hud.get("yaw"), "age": time.monotonic() - self.last_at if self.last_at else None}

    def goto_ned(self, north, east, down, yaw_deg):
        """GUIDED position target in the EKF frame (MAV_FRAME_LOCAL_NED), holding the given yaw."""
        with self.cond:
            sysid, compid = self.sysid, self.compid
        mask = 0x9F8  # use position + yaw; ignore velocity, acceleration, yaw rate
        self._send(self.encoder.mav.set_position_target_local_ned_encode(
            0, sysid, compid, mavlink2.MAV_FRAME_LOCAL_NED, mask, north, east, down,
            0, 0, 0, 0, 0, 0, math.radians(yaw_deg), 0).pack(self.encoder.mav))

    def set_speed(self, speed):
        """MAV_CMD_DO_CHANGE_SPEED: horizontal speed of GUIDED moves [m/s]."""
        with self.cond:
            sysid, compid = self.sysid, self.compid
        self._send(self.encoder.mav.command_long_encode(
            sysid, compid, mavlink2.MAV_CMD_DO_CHANGE_SPEED, 0, 1, speed, -1, 0, 0, 0, 0).pack(self.encoder.mav))

    def _send(self, frame):
        if self.udp is None or self.address is None:
            raise ValueError("no flight controller yet: connect in the Drone tab and wait for its heartbeat")
        self.udp.sendto(frame, self.address)

    def param(self, name, value=None, timeout=3.0):
        """Read (value None) or write one parameter; returns the value the FC reports back."""
        if not name or len(name) > 16 or not all(c.isupper() or c.isdigit() or c == "_" for c in name):
            raise ValueError(f"bad parameter name {name!r}")
        if value is not None:
            if name not in self.WRITABLE:
                raise ValueError(f"{name} cannot be changed from here")
            low, high, integer = self.WRITABLE[name]
            value = float(value)
            if not low <= value <= high:
                raise ValueError(f"{name} must be between {low:g} and {high:g}")
            if integer:
                value = float(round(value))
            if name not in self.params:
                self.param(name)                           # learn its type first
        for attempt in range(2):
            asked = time.monotonic()
            with self.cond:
                sysid, compid = self.sysid, self.compid
            if sysid is None:
                raise ValueError("no flight controller yet: connect in the Drone tab and wait for its heartbeat")
            encoded = name.encode("ascii")
            if value is None:
                frame = self.encoder.mav.param_request_read_encode(sysid, compid, encoded, -1).pack(self.encoder.mav)
            else:
                frame = self.encoder.mav.param_set_encode(sysid, compid, encoded, value,
                                                          self.params[name]["type"]).pack(self.encoder.mav)
            self._send(frame)
            with self.cond:
                answered = self.cond.wait_for(lambda: self.params.get(name, {}).get("at", 0) > asked,
                                              timeout / 2)
                if answered:
                    got = self.params[name]["value"]
                    if value is not None and abs(got - value) > 1e-4 * max(1.0, abs(value)):
                        raise ValueError(f"{name}: the flight controller kept {got:g}")
                    return {"name": name, "value": got}
        raise ValueError(f"{name}: no answer from the flight controller")


def world_to_ned(x, y, z):
    """Room coordinates (X to the 1 m mark, Y left, Z up) -> the EKF's NED: the same turn main.c
    gives ODOMETRY (FRD, then the +X heading); the EKF frame is the ODOMETRY frame."""
    c, s = WORLD_HEADING
    return x * c + y * s, x * s - y * c, -z


def ned_to_world(north, east, down):
    """Inverse of world_to_ned."""
    c, s = WORLD_HEADING
    return north * c + east * s, north * s - east * c, -down


TRACKS_DIR = os.path.join(DATA_DIR, "tracks")


class TrackRecorder:
    """Flight path log for hover / accuracy studies: one row per lighthouse ODOMETRY (10 Hz) with the
    EKF position (when a new LOCAL_POSITION_NED arrived), attitude, battery, mode and the mission
    target. Saved as JSON in TRACKS_DIR (rewritten every few seconds, so a crash loses little)."""

    COLUMNS = ["t", "x", "y", "z", "yaw", "ex", "ey", "ez", "roll", "pitch", "volt",
               "mode", "armed", "mission", "leg", "tx", "ty", "tz", "hold"]
    MAX_ROWS = 100000             # ~2.7 h at 10 Hz
    SAVE_EVERY_S = 5.0

    def __init__(self, fc, mission):
        self.fc, self.mission = fc, mission
        self.lock = threading.Lock()
        self.active = None        # {"name", "meta", "rows", "t0", "saved_at", "ekf_at"}

    def start(self, name=""):
        name = "".join(ch for ch in name if ch.isalnum() or ch in "-_")[:40]
        name = time.strftime("track_%Y%m%d_%H%M%S") + (f"_{name}" if name else "")
        os.makedirs(TRACKS_DIR, exist_ok=True)
        with self.lock:
            if self.active:
                raise ValueError(f"already recording {self.active['name']}")
            meta = {"name": name, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "heading_deg": math.degrees(math.atan2(WORLD_HEADING[1], WORLD_HEADING[0])),
                    "stations_area": station_area(0.0), "columns": self.COLUMNS}
            self.active = {"name": name, "meta": meta, "rows": [], "t0": time.monotonic(),
                           "saved_at": time.monotonic(), "ekf_at": None, "was_running": False}
        return self.status()

    def stop(self):
        with self.lock:
            active, self.active = self.active, None
        if active:
            self._save(active)
        return self.status()

    def add(self, position):
        """Called for every lighthouse ODOMETRY (receiver thread)."""
        with self.lock:
            active = self.active
            if not active:
                return
        now = time.monotonic()
        local, at = self.fc.local_at()
        ekf = [None, None, None]
        if local and at != active["ekf_at"] and now - at < 1.0:
            active["ekf_at"] = at
            ekf = [round(v, 4) for v in ned_to_world(*local)]
        with self.fc.cond:
            hud = dict(self.fc.hud)
        with self.mission.lock:
            ms = dict(self.mission.status_)
        running = ms["state"] == "running"
        target = ms.get("target") if running or ms["state"] == "done" else None
        if running and not active["was_running"]:          # a mission started: keep its points
            active["meta"].setdefault("missions", []).append(
                {"points": ms.get("points"), "started_t": round(now - active["t0"], 2)})
        active["was_running"] = running
        yaw = position.get("yaw")
        row = [round(now - active["t0"], 3), round(position["x"], 4), round(position["y"], 4), round(position["z"], 4),
               None if yaw is None else round(yaw, 1), *ekf,
               None if hud.get("roll") is None else round(hud["roll"], 2),
               None if hud.get("pitch") is None else round(hud["pitch"], 2),
               hud.get("voltage"), hud.get("mode"), 1 if hud.get("armed") else 0, ms["state"],
               ms.get("leg") if running else None,
               *(target if target else [None, None, None]),
               1 if running and ms.get("hold_left") is not None else 0]
        with self.lock:
            if self.active is not active:
                return
            active["rows"].append(row)
            full = len(active["rows"]) >= self.MAX_ROWS
            due = now - active["saved_at"] >= self.SAVE_EVERY_S
            if due:
                active["saved_at"] = now
        if full:
            self.stop()
        elif due:
            self._save(active)

    def _save(self, active):
        with self.lock:
            data = {"meta": dict(active["meta"], rows=len(active["rows"]),
                                 duration=active["rows"][-1][0] if active["rows"] else 0.0),
                    "rows": list(active["rows"])}
        path = os.path.join(TRACKS_DIR, active["name"] + ".json")
        with open(path + ".tmp", "w", encoding="utf-8") as f:
            json.dump(data, f, separators=(",", ":"))
        os.replace(path + ".tmp", path)

    def status(self):
        with self.lock:
            a = self.active
            if not a:
                return {"recording": False, "dir": TRACKS_DIR}
            return {"recording": True, "name": a["name"], "rows": len(a["rows"]),
                    "elapsed": round(time.monotonic() - a["t0"], 1), "dir": TRACKS_DIR}

    @staticmethod
    def list():
        if not os.path.isdir(TRACKS_DIR):
            return []
        out = []
        for name in sorted(os.listdir(TRACKS_DIR), reverse=True):
            if name.endswith(".json"):
                path = os.path.join(TRACKS_DIR, name)
                out.append({"name": name[:-5], "size": os.path.getsize(path),
                            "modified": time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(path)))})
        return out

    @staticmethod
    def _path(name):
        if not name or any(ch in name for ch in "/\\:") or name.startswith("."):
            raise ValueError("bad track name")
        path = os.path.join(TRACKS_DIR, name + ".json")
        if not os.path.exists(path):
            raise ValueError(f"no track {name}")
        return path

    def get(self, name):
        with open(self._path(name), encoding="utf-8") as f:
            return json.load(f)

    def delete(self, name):
        with self.lock:
            if self.active and self.active["name"] == name:
                raise ValueError("stop the recording first")
        os.remove(self._path(name))


def station_area(margin=0.3):
    """Rectangle of the room the stations cover [xmin, xmax, ymin, ymax]: between the calibrated
    stations (bs_poses_cal.h origins), shrunk by `margin`; None without a pose header."""
    if not FIRMWARE_POSES:
        return None
    import re
    with open(FIRMWARE_POSES, encoding="utf-8", errors="replace") as stream:
        origins = [tuple(float(v) for v in m.groups()) for m in
                   re.finditer(r"\.origin\s*=\s*\{\s*([-0-9.eE+]+)f?\s*,\s*([-0-9.eE+]+)f?\s*,\s*([-0-9.eE+]+)f?", stream.read())]
    origins = [o for o in origins if any(o)]
    if len(origins) < 2:
        return None
    xs, ys = [o[0] for o in origins], [o[1] for o in origins]
    area = [min(xs) + margin, max(xs) - margin, min(ys) + margin, max(ys) - margin]
    return area if area[0] < area[1] and area[2] < area[3] else None


class MissionRunner:
    """A list of room points flown one by one in GUIDED: the target goes to the flight controller
    in its NED frame, arrival is judged on the lighthouse position. The pilot stays in charge:
    targets are sent only while the FC is armed in GUIDED, and any other mode on the radio
    (Loiter, PosHold) ends the mission at once."""

    # Where targets may go: heights (Z of the room, 0 = calibration floor) and the rectangle between
    # the stations shrunk by a margin. Set from the editor (Mission → Limits) within these bounds.
    LIMITS_DEFAULT = {"z_min": 0.4, "z_max": 2.0, "margin": 0.3}
    LIMITS_BOUNDS = {"z_min": (0.15, 2.5), "z_max": (0.3, 3.0), "margin": (0.0, 1.0)}
    MAX_SPEED = 1.0               # [m/s]
    AGREE_M = 0.15                # EKF vs lighthouse position: larger = do not send targets
    EKF_LAG_S = 0.8               # the EKF position may be this much behind the lighthouse one
    PERSIST_S = 1.0               # a failed check must last this long to stop a running mission
    ODOMETRY_MAX_AGE = 0.5        # [s]

    def __init__(self, state, fc):
        self.state, self.fc = state, fc
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.status_ = {"state": "idle", "points": [], "leg": None, "message": "", "log": [], "distance": None,
                        "area": None}
        self.limits = dict(self.LIMITS_DEFAULT)
        # Auto-start: the editor arms a figure; it runs when the radio switches INTO GUIDED
        # (a transition seen here, never because the app connected while already in GUIDED).
        self.auto = None            # {"frame", "points", "speed", "tolerance", "settle"}
        threading.Thread(target=self._watch_mode, daemon=True).start()

    def set_auto(self, config):
        """Arm (config dict) or disarm (None) the auto-start on GUIDED."""
        if config is not None:
            points = [[float(v) for v in p] for p in config.get("points", [])]
            if not points or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
                raise ValueError("give at least one point as [x, y, z]")
            config = {"frame": "relative" if config.get("frame") == "relative" else "absolute", "points": points,
                      "pass_through": bool(config.get("pass_through")),
                      "speed": float(config.get("speed", 0.3)), "tolerance": float(config.get("tolerance", 0.10)),
                      "settle": float(config.get("settle", 1.0))}
        with self.lock:
            changed = (self.auto is None) != (config is None)
            self.auto = config
        if changed:
            self._log("auto-start armed: switch to GUIDED on the radio to fly the figure" if config
                      else "auto-start disarmed")

    def _watch_mode(self):
        previous = None
        while True:
            time.sleep(0.1)
            st = self.fc.status()
            mode = st["mode"] if st["age"] is not None and st["age"] < 3 else None
            entered = mode == "GUIDED" and previous not in (None, "GUIDED")
            previous = mode
            with self.lock:
                auto, running = self.auto, self.status_["state"] == "running"
            if not (entered and auto and st["armed"]) or running:
                continue
            time.sleep(0.5)                           # let GUIDED take over, then start from here
            try:
                self.start(auto["points"], auto["speed"], auto["tolerance"], auto["settle"], auto["frame"],
                           auto.get("pass_through", False))
                self._log("auto-start: GUIDED engaged")
            except ValueError as error:
                self._log(f"auto-start refused (the drone just hovers in GUIDED): {error}")

    def _log(self, text):
        with self.lock:
            self.status_["message"] = text
            self.status_["log"] = (self.status_["log"] + [f"{time.strftime('%H:%M:%S')} {text}"])[-30:]

    def checks(self):
        """Pre-flight checks the editor shows live: {name: (ok, detail)}."""
        st = self.fc.status()
        pos, odo_age = self.state.latest()
        local, local_age = self.fc.local()
        out = {
            "fc": (st["age"] is not None and st["age"] < 3, "flight controller telemetry"),
            "armed": (bool(st["armed"]), "armed"),
            "guided": (st["mode"] == "GUIDED", "mode GUIDED" if st["mode"] == "GUIDED"
                       else f"mode {st['mode'] or '?'}: switch to GUIDED on the radio"),
            "lighthouse": (odo_age < self.ODOMETRY_MAX_AGE, f"lighthouse position {'%.1f s old' % odo_age if odo_age < 99 else 'missing'}"),
        }
        if pos and local and local_age < 2.0:
            # The EKF position arrives a few times a second and late: compare it with the lighthouse
            # positions of the moments it can refer to, and keep the best match.
            _, at = self.fc.local_at()
            candidates = self.state.recent(at - self.EKF_LAG_S, at + 0.05) or [(pos["x"], pos["y"], pos["z"])]
            gap = min(math.dist(world_to_ned(*p), local) for p in candidates)
            out["agree"] = (gap < self.AGREE_M, f"EKF and lighthouse agree within {100 * gap:.0f} cm")
        else:
            out["agree"] = (False, "EKF position (LOCAL_POSITION_NED) missing")
        return {k: {"ok": ok, "detail": detail} for k, (ok, detail) in out.items()}

    def set_limits(self, limits):
        """Update the target limits (missing keys keep their value), clamped to LIMITS_BOUNDS."""
        new = dict(self.limits)
        for key, (low, high) in self.LIMITS_BOUNDS.items():
            if limits.get(key) not in (None, ""):
                new[key] = min(high, max(low, float(limits[key])))
        if new["z_max"] < new["z_min"] + 0.1:
            raise ValueError("the maximum height must be at least 0.1 m above the minimum")
        with self.lock:
            self.limits = new

    def validate(self, points):
        lim = self.limits
        area = station_area(lim["margin"])
        bad = []
        for i, p in enumerate(points):
            x, y, z = p
            if z < lim["z_min"]:
                bad.append(f"point {i + 1}: height {z:.2f} m is below the minimum height {lim['z_min']:.2f} m")
            elif z > lim["z_max"]:
                bad.append(f"point {i + 1}: height {z:.2f} m is above the maximum height {lim['z_max']:.2f} m")
            if area and not (area[0] <= x <= area[1] and area[2] <= y <= area[3]):
                bad.append(f"point {i + 1}: ({x:.2f}, {y:.2f}) outside the area between the stations "
                           f"X {area[0]:.2f}..{area[1]:.2f}, Y {area[2]:.2f}..{area[3]:.2f}")
        return bad

    PASS_RADIUS = 0.15            # pass-through: go on to the next point once this close [m]
    MAX_HOLD_S = 600.0            # longest hold at a point (hover studies) [s]

    def start(self, points, speed, tolerance, settle, frame="absolute", pass_through=False):
        """Fly `points`: room coordinates, or (frame "relative") offsets from where the drone is now."""
        with self.lock:
            if self.status_["state"] == "running":
                raise ValueError("a mission is already running")
        points = [[float(v) for v in p] for p in points]
        if not points or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError("give at least one point as [x, y, z]")
        if frame == "relative":
            pos, age = self.state.latest()
            if pos is None or age > self.ODOMETRY_MAX_AGE:
                raise ValueError("no fresh lighthouse position to start the figure from")
            base = (pos["x"], pos["y"], pos["z"])
            points = [[round(b + d, 3) for b, d in zip(base, p)] for p in points]
        if len(points) > 50:
            raise ValueError("at most 50 points")
        bad = self.validate(points)
        if bad:
            raise ValueError("; ".join(bad))
        failed = [c["detail"] for c in self.checks().values() if not c["ok"]]
        if failed:
            raise ValueError("not ready: " + "; ".join(failed))
        speed = min(self.MAX_SPEED, max(0.1, float(speed)))
        tolerance = min(0.5, max(0.03, float(tolerance)))
        settle = min(self.MAX_HOLD_S, max(0.0, float(settle)))
        with self.lock:
            self.status_.update(state="running", points=points, leg=0, distance=None, log=[],
                                area=station_area(self.limits["margin"]))
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, args=(points, speed, tolerance, settle, bool(pass_through)),
                                       daemon=True)
        self.thread.start()

    def stop(self, reason="stopped from the editor"):
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join(timeout=3.0)
        with self.lock:
            running = self.status_["state"] == "running"
        if running:
            self._hold_here()
            self._finish("aborted", reason)

    def _hold_here(self):
        """Target = where the drone is now (only while still in GUIDED): it stops and hovers."""
        local, age = self.fc.local()
        st = self.fc.status()
        if local and age < 2.0 and st["mode"] == "GUIDED":
            try:
                self.fc.goto_ned(*local, st["yaw"] or 0.0)
            except (ValueError, OSError):
                pass

    def _finish(self, state, text):
        self._log(text)
        with self.lock:
            self.status_.update(state=state, distance=None, hold_left=None)

    def _run(self, points, speed, tolerance, settle, pass_through=False):
        """Fly the points. pass_through: every point but the last only has to be passed within
        PASS_RADIUS (no hold), so curves made of many points are flown without stopping."""
        try:
            yaw = self.fc.status()["yaw"] or 0.0      # keep the heading the drone has now
            self.fc.set_speed(speed)
            self._log(f"start: {len(points)} points at {speed:.2f} m/s, yaw held at {yaw:.0f} deg"
                      + (", passing through the points" if pass_through else ""))
            for leg, (x, y, z) in enumerate(points):
                last = leg == len(points) - 1
                leg_tol = tolerance if last or not pass_through else max(tolerance, self.PASS_RADIUS)
                leg_settle = settle if last or not pass_through else 0.0
                with self.lock:
                    self.status_["leg"] = leg
                    self.status_["hold_left"] = None
                    self.status_["target"] = [x, y, z]
                pos, _ = self.state.latest()
                distance = math.dist((pos["x"], pos["y"], pos["z"]), (x, y, z))
                timeout = 8.0 + 2.0 * distance / speed
                self._log(f"point {leg + 1}/{len(points)}: ({x:.2f}, {y:.2f}, {z:.2f}), {distance:.2f} m away")
                started = sent = time.monotonic()
                self.fc.goto_ned(*world_to_ned(x, y, z), yaw)
                inside_since = failing_since = None
                while True:
                    if self.stop_event.wait(0.1):
                        return
                    now = time.monotonic()
                    st = self.fc.status()
                    if st["mode"] != "GUIDED" or not st["armed"]:
                        self._finish("aborted", f"pilot took over (mode {st['mode']}, {'armed' if st['armed'] else 'disarmed'})")
                        return
                    checks = self.checks()
                    failing = [checks[n]["detail"] for n in ("lighthouse", "agree", "fc") if not checks[n]["ok"]]
                    if failing:
                        failing_since = failing_since or now
                        if now - failing_since >= self.PERSIST_S:
                            self._hold_here()
                            self._finish("aborted", f"stopped, holding position: {'; '.join(failing)}")
                            return
                    else:
                        failing_since = None
                    pos, _ = self.state.latest()
                    d = math.dist((pos["x"], pos["y"], pos["z"]), (x, y, z))
                    with self.lock:
                        self.status_["distance"] = d
                    # Arrived = first time within the radius; the hold then runs for leg_settle seconds
                    # whatever small excursions the drone makes (a long hold is the hover being studied).
                    if inside_since is None and d <= leg_tol:
                        inside_since = now
                        if leg_settle >= 5:
                            self._log(f"point {leg + 1}: arrived, holding {leg_settle:.0f} s")
                    if inside_since is not None:
                        with self.lock:
                            self.status_["hold_left"] = max(0.0, leg_settle - (now - inside_since))
                        if now - inside_since >= leg_settle:
                            break
                    elif now - started > timeout:
                        self._hold_here()
                        self._finish("aborted", f"point {leg + 1} not reached in {timeout:.0f} s ({100 * d:.0f} cm away): holding position")
                        return
                    if now - sent > 1.0:              # repeat the target: a lost packet is harmless
                        self.fc.goto_ned(*world_to_ned(x, y, z), yaw)
                        sent = now
                self._log(f"point {leg + 1} reached")
            self._finish("done", "mission complete: hovering at the last point (GUIDED) — take over on the radio")
        except Exception as error:  # link or socket trouble: stop where we are
            self._hold_here()
            self._finish("error", f"{type(error).__name__}: {error}")

    def snapshot(self):
        with self.lock:
            out = dict(self.status_)
        out["checks"] = self.checks()
        out["area"] = station_area(self.limits["margin"])
        out["z_range"] = [self.limits["z_min"], self.limits["z_max"]]
        out["limits"] = dict(self.limits)
        out["stations_area"] = station_area(0.0)
        with self.lock:
            out["auto"] = self.auto
        return out


class CaptureManager:
    """Bitcraze-style sample collection for the pose editor's calibration wizard.

    Sample kinds, as in the Crazyflie client's lighthouse wizard:
      origin    one static capture at (0, 0, 0)                     -> origin.json
      x_axis    one static capture on the +X mark                   -> x_axis.json
      xy_plane  any number of static captures on the floor          -> xy_plane_<n>.json
      sweep     XYZ-space samples, appended on every recording      -> sweep.json
      verify    optional static captures the solver never sees      -> verify_<n>.json
    As soon as there are enough samples the session is solved in the background
    (calibrate_bitcraze.py), and re-solved whenever an estimation sample changes.
    """

    STATIC_KINDS = ("origin", "x_axis", "xy_plane", "verify")
    MIN_SWEEP = 100  # snapshots with both stations before a solve is attempted
    PREVIEW_EVERY_S = 12.0  # during a sweep, re-estimate the stations this often (live 3D preview)

    def __init__(self):
        self.lock = threading.Lock()
        self.session_dir = ""
        self.config = {"x_axis_dist": 1.0, "static_seconds": 5.0, "baseline": 0.0, "board_height": 0.0,
                       "reconvert": "", "auto_solve": True, "max_board_fit": 5.0}
        self.target = None  # (host, port) where the extra GCS heartbeat goes
        self.span_limit = None
        self.seen = set()
        self.packets = 0
        self.last_packet = 0.0
        self.recent = []  # (time, complete, plausible) for the live quality figure
        self.bs_last = {}  # base station id -> last time all four of its sensors were seen
        self.active = None  # running capture
        self.last_result = None
        self.sweep_count = 0
        self.solve = self._idle_solve()
        self.solve_version = 0
        self.pending = False  # samples changed while the solver was running
        self.sim = None  # simulation: a recorded run replayed as a virtual drone
        self._preview_proc = None  # running preview solver, if any
        self.udp_error = None  # set when the MAVLink UDP port could not be opened
        self.live_records, self.live_at = None, 0.0  # latest snapshot seen by >= 2 stations
        self._geo_cache = {}  # geometry path -> (mtime, poses)
        # Samples placed in real time for the 3D view, with the best geometry available: this
        # calibration, else its estimate during the recording, else the one in the firmware.
        self.display = self._empty_display()
        self._display_key, self._display_geo = None, None
        if lh2cap is not None:
            sensors = os.path.join(CALIB_DIR, "wand_sensors.json")
            try:
                self.span_limit = lh2cap.wand_span_limit(sensors, 0.5)
            except OSError:
                print("wand_sensors.json not found: plausibility check disabled")

    @staticmethod
    def _idle_solve():
        return {"state": "idle", "log": "", "ok": None, "variants": [], "primary": None, "samples": None,
                "quality": None, "preview": None, "preview_running": False}

    def set_target(self, host, port):
        with self.lock:
            self.target = (host, port) if host else None

    def get_target(self):
        with self.lock:
            return self.target

    # ----------------------------------------------------------------- session

    def configure(self, session_dir, config):
        # Relative directories go to RUNS_DIR, like capture_lh2.py's calib_runs/run_<date> sessions.
        session_dir = session_path(session_dir)
        with self.lock:
            if self.active is not None or self.solve["state"] == "running":
                raise ValueError("a capture or solve is running")
            os.makedirs(session_dir, exist_ok=True)
            self.session_dir = session_dir
            manifest = os.path.join(session_dir, "session.json")
            if os.path.exists(manifest):  # reopening a recorded run: keep its own mark distance
                with open(manifest, encoding="utf-8") as stream:
                    config["x_axis_dist"] = float(json.load(stream).get("x_axis_dist", config["x_axis_dist"]))
            self.config.update(config)
            self.last_result = None
            self.pending = False
            if self.sim is not None:  # a new session replays the recording from the start
                self.sim.update(used=set(), sweep_cursor=0)
            self.sweep_count = self._count_sweep()
            self.solve = self._idle_solve()
            self.solve_version += 1
            self._invalidate_display()
            variants = [(session_dir, "firmware angles")] + [
                (session_dir + "_" + model, model + " re-conversion") for model in ("period", "factory")]
            found = [load_solve_result(d, n) for d, n in variants if os.path.exists(os.path.join(d, "geometry.yaml"))]
        if found:  # a solved run: show its result and place its samples, without re-solving
            with self.lock:
                self.solve.update(state="running", variants=found,
                                  log="".join(_read_text(os.path.join(v["dir"], "solve.log")) for v in found))
            threading.Thread(target=self._finish_solve, args=(session_dir, found), daemon=True).start()
        elif config.get("auto_solve") and self._ready()[0]:
            self.start_solve()

    def _files(self, kind):
        if not self.session_dir or not os.path.isdir(self.session_dir):  # e.g. deleted from Explorer
            return []
        if kind in ("origin", "x_axis", "sweep"):
            return [kind] if os.path.exists(os.path.join(self.session_dir, kind + ".json")) else []
        names = [n[:-5] for n in os.listdir(self.session_dir) if n.startswith(kind + "_") and n.endswith(".json")]
        return sorted(names, key=lambda n: int(n.rsplit("_", 1)[1]) if n.rsplit("_", 1)[1].isdigit() else 0)

    def _count_sweep(self):
        path = os.path.join(self.session_dir, "sweep.json")
        if not os.path.exists(path):
            return 0
        with open(path, encoding="utf-8") as stream:
            records = json.load(stream)
        per_timestamp = {}
        for record in records:
            per_timestamp.setdefault(record["timestamp"], set()).add(record["base_station_id"])
        return sum(1 for ids in per_timestamp.values() if len(ids) >= 2)

    def _ready(self):
        missing = []
        if not self._files("origin"):
            missing.append("origin")
        if not self._files("x_axis"):
            missing.append("x-axis")
        if not self._files("xy_plane"):
            missing.append("xy-plane")
        if self.sweep_count < self.MIN_SWEEP:
            missing.append(f"XYZ-space ({self.sweep_count}/{self.MIN_SWEEP})")
        return not missing, missing

    def _write_manifest(self):
        manifest = {
            "type": "lh2_bitcraze_calibration_session",
            "origin": "origin.json",
            "x_axis": ["x_axis.json"],
            "x_axis_dist": float(self.config["x_axis_dist"]),
            "xy_plane": [name + ".json" for name in self._files("xy_plane")],
            "sweep": "sweep.json",
        }
        with open(os.path.join(self.session_dir, "session.json"), "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2)

    def _set_aside(self, names, folder):
        """Move sample files into <session>/<folder>/ instead of deleting them."""
        target = os.path.join(self.session_dir, folder)
        os.makedirs(target, exist_ok=True)
        for name in names:
            source = os.path.join(self.session_dir, name + ".json")
            if os.path.exists(source):
                os.replace(source, os.path.join(target, name + ".json"))

    def delete_sample(self, name):
        with self.lock:
            if self.active is not None:
                raise ValueError("a capture is running")
            kinds = ("origin", "x_axis", "sweep") + tuple(k + "_" for k in ("xy_plane", "verify"))
            if not self.session_dir or not name.startswith(kinds) or os.sep in name or "/" in name:
                raise ValueError("unknown sample")
            self._set_aside([name], "_deleted")
            self._invalidate_display()
            if name == "sweep":
                self.sweep_count = 0
            self.last_result = {"step": name, "ok": True, "message": f"{name} moved to _deleted/"}
            kind = name.rsplit("_", 1)[0] if name.startswith(("xy_plane_", "verify_")) else name
        self._samples_changed(kind)

    def clear_samples(self):
        with self.lock:
            if self.active is not None or self.solve["state"] == "running":
                raise ValueError("a capture or solve is running")
            if not self.session_dir:
                raise ValueError("session not configured")
            names = [n for kind in ("origin", "x_axis", "sweep", "xy_plane", "verify") for n in self._files(kind)]
            folder = time.strftime("_cleared_%Y%m%d_%H%M%S")
            self._set_aside(names + ["session"], folder)
            for derived in ("geometry.yaml", "solve.log"):
                path = os.path.join(self.session_dir, derived)
                if os.path.exists(path):
                    os.replace(path, os.path.join(self.session_dir, folder, derived))
            self.sweep_count = 0
            self.solve = self._idle_solve()
            self.solve_version += 1
            self._invalidate_display()
            self.last_result = {"step": "clear", "ok": True, "message": f"All samples moved to {folder}/"}

    def export_samples(self):
        with self.lock:
            if not self.session_dir:
                raise ValueError("session not configured")
            files = {}
            for kind in ("origin", "x_axis", "xy_plane", "sweep", "verify"):
                for name in self._files(kind):
                    with open(os.path.join(self.session_dir, name + ".json"), encoding="utf-8") as stream:
                        files[name] = json.load(stream)
            return {"type": "lh2_calibration_samples", "version": 1, "session": os.path.basename(self.session_dir),
                    "x_axis_dist": float(self.config["x_axis_dist"]), "files": files}

    def import_samples(self, bundle, config):
        if not isinstance(bundle, dict) or bundle.get("type") != "lh2_calibration_samples":
            raise ValueError("not an lh2_calibration_samples file")
        files = bundle.get("files") or {}
        allowed = ("origin", "x_axis", "sweep", "xy_plane_", "verify_")
        if not files or any(not n.startswith(allowed) or not n.replace("_", "").isalnum() for n in files):
            raise ValueError("unexpected sample names in the file")
        session = os.path.join(RUNS_DIR, time.strftime("import_%Y%m%d_%H%M%S"))
        os.makedirs(session)
        for name, records in files.items():
            with open(os.path.join(session, name + ".json"), "w", encoding="utf-8") as stream:
                json.dump(records, stream, indent=1)
        config["x_axis_dist"] = float(bundle.get("x_axis_dist", config["x_axis_dist"]))
        self.configure(session, config)

    # ----------------------------------------------------------------- capture

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
            if self.sim is not None or key in self.seen:  # the real drone is ignored while simulating
                return
            self.seen.add(key)
            finished = self._ingest(lh2cap.snapshot_to_records(snapshot))
        if finished:
            self._samples_changed(finished)

    def _ingest(self, records):
        """Account one snapshot (lock held). Returns the kind of a capture it completed, if any."""
        now = time.monotonic()
        self.packets += 1
        self.last_packet = now
        for record in records:
            self.bs_last[record["base_station_id"]] = now
        if len(records) >= 2:
            self.live_records, self.live_at = records, now
        complete = len(records) >= lh2cap.MIN_BS  # at least two stations see all four sensors
        plausible = complete and self._plausible(records)
        self.recent = [r for r in self.recent if now - r[0] < 2.0] + [(now, complete, plausible)]
        cap = self.active
        if cap is not None:
            cap["records"].extend(records)
            cap["snapshots"] += 1
            cap["complete"] += int(complete)
            cap["plausible"] += int(plausible)
            if cap["step"] == "sweep" and complete and plausible and self._display_geo and cap["snapshots"] % 2 == 0:
                self._display_add_cloud(records)
        return self._maybe_finish(now)

    def start(self, kind, seconds):
        with self.lock:
            if lh2cap is None:
                raise ValueError("capture_lh2 module unavailable (install numpy)")
            if not self.session_dir:
                raise ValueError("session not configured")
            if self.active is not None:
                raise ValueError("a capture is already running")
            if kind not in self.STATIC_KINDS + ("sweep",):
                raise ValueError("unknown sample kind")
            replay = self._sim_source(kind) if self.sim is not None else None
            self.active = {"step": kind, "seconds": None if kind == "sweep" else seconds,
                           "started": time.monotonic(), "records": [], "snapshots": 0,
                           "complete": 0, "plausible": 0}
            self.last_result = None
            if replay is not None:
                threading.Thread(target=self._replay, args=(self.active, *replay), daemon=True).start()
            if kind == "sweep":
                threading.Thread(target=self._preview_loop, args=(self.active,), daemon=True).start()

    # ------------------------------------------------------- live 3D preview

    def _preview_loop(self, cap):
        """While a sweep is recorded, re-estimate the stations every PREVIEW_EVERY_S seconds."""
        last = time.monotonic()
        while True:
            time.sleep(0.5)
            with self.lock:
                if self.active is not cap:
                    return
                due = time.monotonic() - last >= self.PREVIEW_EVERY_S
                statics_ready = bool(self._files("origin") and self._files("x_axis") and self._files("xy_plane"))
                enough = self.sweep_count + cap["complete"] >= self.MIN_SWEEP
                if not (due and statics_ready and enough) or self.solve.get("preview_running"):
                    continue
                self.solve["preview_running"] = True
                self.solve_version += 1
                session, records = self.session_dir, list(cap["records"])
                config = dict(self.config)
            last = time.monotonic()
            self._run_preview(session, records, config)

    def _run_preview(self, session, records, config):
        """Solve sweep-so-far + static captures in a fresh <session>/_preview_<n> and publish it as
        solve['preview']; the previous preview stays readable (live position) until this one is ready."""
        work = os.path.join(session, f"_preview_{int(time.time() * 1000)}")
        preview = None
        try:
            os.makedirs(work)
            for name in os.listdir(session):
                if name.endswith(".json") and name.startswith(("origin", "x_axis", "xy_plane_")):
                    shutil.copy2(os.path.join(session, name), work)
            sweep = os.path.join(session, "sweep.json")
            previous = []
            if os.path.exists(sweep):
                with open(sweep, encoding="utf-8") as stream:
                    previous = json.load(stream)
            with open(os.path.join(work, "sweep.json"), "w", encoding="utf-8") as stream:
                json.dump(previous + records, stream)
            xy = sorted(n for n in os.listdir(work) if n.startswith("xy_plane_"))
            with open(os.path.join(work, "session.json"), "w", encoding="utf-8") as stream:
                json.dump({"type": "lh2_bitcraze_calibration_session", "origin": "origin.json",
                           "x_axis": ["x_axis.json"], "x_axis_dist": float(config["x_axis_dist"]),
                           "xy_plane": xy, "sweep": "sweep.json"}, stream)
            command = [sys.executable, os.path.join(CALIB_DIR, "calibrate_bitcraze.py"), work,
                       "-o", os.path.join(work, "geometry.yaml")]
            if config.get("baseline"):
                command += ["--baseline", str(config["baseline"])]
            if config.get("board_height"):
                command += ["--board-height", str(config["board_height"])]
            # A preview only places things in 3D while recording: never stop it at the board check.
            command += ["--max-board-fit-mm", str(max(10.0, float(config.get("max_board_fit") or 5.0)))]
            process = subprocess.Popen(command, cwd=CALIB_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding="utf-8", errors="replace", env=_CHILD_ENV)
            with self.lock:
                self._preview_proc = process  # start_solve() stops it: the final solve supersedes it
            output, _ = process.communicate()
            with open(os.path.join(work, "solve.log"), "w", encoding="utf-8") as log:
                log.write(output)
            if process.returncode == 0:
                preview = load_solve_result(work, "preview")
                located = subprocess.run([sys.executable, os.path.join(CALIB_DIR, "locate_samples.py"), work,
                                          "--max-cloud", "250", "--json"], cwd=CALIB_DIR, stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, text=True, encoding="utf-8", errors="replace",
                                         env=_CHILD_ENV)
                if located.returncode == 0:
                    preview["samples"] = json.loads(located.stdout)
                preview["snapshots"] = len({r["timestamp"] for r in previous + records})
        except (OSError, ValueError) as error:
            print(f"Preview solve failed: {error}")
        with self.lock:
            current = session == self.session_dir
            old = self.solve.get("preview") if current else None
            if current:
                self.solve["preview_running"] = False
                if preview is not None:
                    self.solve["preview"] = preview
                self.solve_version += 1
        # Drop whichever preview is no longer shown: the replaced one, or this one if it is not used.
        stale = (old or {}).get("dir") if current and preview is not None else work
        if stale:
            shutil.rmtree(stale, ignore_errors=True)

    # ------------------------------------------------- samples in the 3D view

    def _empty_display(self, version=0):
        return {"version": version, "source": None, "label": "", "statics": {}, "cloud": []}

    def _display_geometry(self):
        """(key, source, label, poses) of the best geometry to place samples with (lock held), or None."""
        # Never import here: this runs under the capture lock inside every status request, and the
        # first import of scipy can take a minute (no .pyc yet, antivirus). main() imports it in the
        # background; until it is there, samples simply wait.
        ls = sys.modules.get("locate_samples")
        if ls is None or not hasattr(ls, "main"):  # main() is defined last: the import has finished
            return None
        candidates = []
        variant = self._primary_variant()
        if variant:
            candidates.append(("final", "this calibration", os.path.join(variant["dir"], "geometry.yaml")))
        preview = self.solve.get("preview")
        if preview and preview.get("ok"):
            candidates.append(("preview", "the estimate from the recording so far",
                               os.path.join(preview["dir"], "geometry.yaml")))
        if FIRMWARE_POSES:
            candidates.append(("firmware", "the calibration in the firmware (bs_poses_cal.h)", FIRMWARE_POSES))
        for source, label, path in candidates:
            try:
                mtime = os.path.getmtime(path)
                cached = self._geo_cache.get(path)
                if cached is None or cached[0] != mtime:
                    cached = (mtime, ls.load_any_geometry(ls.Path(path)))
                    self._geo_cache[path] = cached
                return f"{path}@{mtime}", source, label, cached[1]
            except Exception:  # being rewritten, or unreadable: try the next one
                continue
        return None

    def _layout(self):
        """Sensor board layout, loaded once."""
        import locate_samples as ls
        if not hasattr(self, "_sensors"):
            self._sensors = ls.cb.load_sensor_positions(os.path.join(CALIB_DIR, "wand_sensors.json"))
        return self._sensors

    def _locate(self, per_bs, poses):
        """Board centre and fit error for one snapshot {bs: angles}, or None."""
        import locate_samples as ls
        return ls._locate(ls._sample(per_bs), poses, self._layout())

    def _display_add_cloud(self, records):
        """Place one room-recording snapshot in the 3D view (lock held)."""
        try:
            located = self._locate({r["base_station_id"]: r["angles"] for r in records}, self._display_geo[3])
        except Exception:
            return
        if located and located["error_mm"] < 20.0:
            cloud = self.display["cloud"]
            cloud.append([round(c, 3) for c in located["position"]])
            if len(cloud) > DISPLAY_CLOUD_MAX:
                del cloud[::2]
            self.display["version"] += 1

    def _locate_static(self, session, name, poses):
        import contextlib
        import locate_samples as ls
        kind = next(k for k in ("origin", "x_axis", "xy_plane", "verify") if name.startswith(k))
        try:
            with contextlib.redirect_stdout(open(os.devnull, "w")):  # the solver warns on stdout
                sample = ls.cb.average_static_capture(os.path.join(session, name + ".json"))
            located = ls._locate(sample, poses, self._layout())
        except Exception as error:
            return {"name": name, "kind": kind, "error": str(error)}
        return {"name": name, "kind": kind, **(located or {"error": "rays do not intersect"})}

    def _refresh_display(self):
        """Re-place every sample when the best geometry changes (lock held)."""
        try:
            geo = self._display_geometry()
        except Exception:
            geo = None
        key = geo[0] if geo else None
        if key == self._display_key:
            return
        self._display_key, self._display_geo = key, geo
        self.display = self._empty_display(self.display["version"] + 1)
        if geo:
            self.display.update(source=geo[1], label=geo[2])
            threading.Thread(target=self._rebuild_display, args=(key, self.session_dir), daemon=True).start()

    def _rebuild_display(self, key, session):
        with self.lock:
            if key != self._display_key or session != self.session_dir or not session:
                return
            poses = self._display_geo[3]
            names = [n for kind in ("origin", "x_axis", "xy_plane", "verify") for n in self._files(kind)]
            running = list(self.active["records"]) if self.active and self.active["step"] == "sweep" else []
        statics = {name: self._locate_static(session, name, poses) for name in names}
        records = running
        sweep = os.path.join(session, "sweep.json")
        try:
            if os.path.exists(sweep):
                with open(sweep, encoding="utf-8") as stream:
                    records = json.load(stream) + running
        except (OSError, ValueError):
            pass
        grouped = {}
        for record in records:
            grouped.setdefault(record["timestamp"], {})[record["base_station_id"]] = record["angles"]
        snapshots = [g for g in grouped.values() if len(g) >= 2]
        cloud = []
        for per_bs in snapshots[::max(1, len(snapshots) // (DISPLAY_CLOUD_MAX // 2))]:
            try:
                located = self._locate(per_bs, poses)
            except Exception:
                continue
            if located and located["error_mm"] < 20.0:
                cloud.append([round(c, 3) for c in located["position"]])
        with self.lock:
            if key != self._display_key or session != self.session_dir:
                return
            statics.update(self.display["statics"])  # captured while this ran
            self.display["statics"] = statics
            self.display["cloud"] = cloud + self.display["cloud"]  # + points added live meanwhile
            self.display["version"] += 1

    def _display_static(self, session, name):
        """Place a just-captured static sample (runs in a thread)."""
        with self.lock:
            geo = self._display_geo
        if not geo:
            return
        located = self._locate_static(session, name, geo[3])
        with self.lock:
            if session == self.session_dir and geo[0] == self._display_key:
                self.display["statics"][name] = located
                self.display["version"] += 1

    def _invalidate_display(self):
        """Samples changed on disk (lock held): rebuild the 3D placement on the next status."""
        self._display_key = None

    def _live_position(self):
        """Drone position from the latest snapshot (lock held), for the 3D view."""
        if self.live_records is None or time.monotonic() - self.live_at > 1.0 or not self._display_geo:
            return None
        try:
            return self._locate({r["base_station_id"]: r["angles"] for r in self.live_records}, self._display_geo[3])
        except Exception:  # a bad snapshot must never break the status reply
            return None

    # -------------------------------------------------------------- simulation

    def start_simulation(self, source, config):
        """Replay a recorded run as a virtual drone into a new sim_<date>_<run> session."""
        source = session_path(source)
        if not os.path.exists(os.path.join(source, "sweep.json")):
            raise ValueError("the source run has no sweep.json")
        manifest = os.path.join(source, "session.json")
        if os.path.exists(manifest):
            with open(manifest, encoding="utf-8") as stream:
                config["x_axis_dist"] = float(json.load(stream).get("x_axis_dist", config["x_axis_dist"]))
        with self.lock:
            if self.active is not None:
                raise ValueError("a capture is running")
            self.sim = {"source": source, "name": os.path.basename(source), "used": set(), "sweep_cursor": 0}
        name = time.strftime("sim_%Y%m%d_%H%M%S_") + os.path.basename(source)
        self.configure(os.path.join(RUNS_DIR, name), config)

    def stop_simulation(self):
        with self.lock:
            if self.active is not None:
                raise ValueError("a capture is running")
            self.sim = None
            self.packets, self.recent, self.bs_last = 0, [], {}
            self.live_records = None
            # Back to the real drone with no session: its first measurement opens a new one.
            self.session_dir, self.sweep_count, self.last_result = "", 0, None
            self.solve = self._idle_solve()
            self.solve_version += 1
            self._invalidate_display()

    def _sim_files(self, prefix):
        return sorted((n[:-5] for n in os.listdir(self.sim["source"]) if n.startswith(prefix) and n.endswith(".json")),
                      key=lambda n: int(n.rsplit("_", 1)[1]) if n.rsplit("_", 1)[1].isdigit() else 0)

    def _sim_source(self, kind):
        """Pick the recorded capture a step replays (lock held): (snapshots, mode)."""
        sim = self.sim
        if kind in ("origin", "x_axis"):
            name = kind
        elif kind == "xy_plane":
            free = [n for n in self._sim_files("xy_plane_") if n not in sim["used"]]
            if not free:
                raise ValueError("all recorded floor points of this run are used")
            name = free[0]
        elif kind == "verify":
            # A recorded floor point that calibration did not use is an independent check.
            free = [n for n in self._sim_files("xy_plane_") + self._sim_files("verify_") if n not in sim["used"]]
            if not free:
                raise ValueError("no unused recorded floor point left for a check — in simulation, check points "
                                 "replay the floor points you did not use (add fewer floor points)")
            name = free[0]
        else:
            name = "sweep"
        path = os.path.join(sim["source"], name + ".json")
        if not os.path.exists(path):
            raise ValueError(f"the source run has no {name}.json")
        with open(path, encoding="utf-8") as stream:
            records = json.load(stream)
        grouped = {}
        for record in records:
            grouped.setdefault(record["timestamp"], []).append(record)
        snapshots = [grouped[t] for t in sorted(grouped)]
        if kind == "sweep":
            snapshots = snapshots[sim["sweep_cursor"]:]
            if not snapshots:
                raise ValueError("the recorded sweep is used up — the result already has all of it")
        else:
            sim["used"].add(name)
        sim["playing"] = name
        return snapshots, kind == "sweep"

    def _replay(self, cap, snapshots, is_sweep, speed=4.0):
        """Feed recorded snapshots as if the drone sent them, until the capture ends."""
        spacing = None if is_sweep else max(0.02, 0.85 * cap["seconds"] / max(1, len(snapshots)))
        previous = None
        for index, records in enumerate(snapshots):
            stamp = records[0]["timestamp"]
            delay = spacing if spacing else min(0.2, max(0.0, (stamp - previous) / speed)) if previous else 0.0
            previous = stamp
            time.sleep(delay)
            with self.lock:
                if self.active is not cap:
                    return
                if is_sweep:
                    self.sim["sweep_cursor"] += 1
                finished = self._ingest([dict(r) for r in records])
            if finished:
                self._samples_changed(finished)
                return
        if is_sweep:  # recording exhausted: end the capture like the user pressing Stop
            with self.lock:
                if self.active is not cap:
                    return
            self.stop()

    def stop(self):
        with self.lock:
            finished = self._finish()
        if finished:
            self._samples_changed(finished)

    def _maybe_finish(self, now):
        cap = self.active
        if cap and cap["seconds"] is not None and now - cap["started"] >= cap["seconds"]:
            return self._finish()
        return None

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
            if name in ("origin.json", "x_axis.json") or name.startswith(("xy_plane_", "verify_")):
                path = os.path.join(session, name)
                with open(path, encoding="utf-8") as stream:
                    records = json.load(stream)
                cleaned = self._clean_static(records)
                if len(cleaned) != len(records):
                    with open(path, "w", encoding="utf-8") as stream:
                        json.dump(cleaned, stream, indent=1)

    def _finish(self):
        """End the running capture and save it. Returns the sample kind if something was saved."""
        cap = self.active
        if cap is None:
            return None
        self.active = None
        kind, complete, plausible = cap["step"], cap["complete"], cap["plausible"]
        ok = complete > 0 if kind == "sweep" else complete >= 10 and plausible >= 0.5 * complete
        message = f"{complete} snapshots with both stations, {plausible} plausible"
        reason = None if ok else "no_data" if complete == 0 else "few" if complete < 10 else "implausible"
        if ok and kind != "sweep":
            cleaned = self._clean_static(cap["records"])
            if len(cleaned) < 0.7 * len(cap["records"]):
                ok, reason = False, "moving"
                message += "; drone not still"
            cap["records"] = cleaned
        # Structured fields let the page word the outcome; message stays for logs and older pages.
        result = {"step": kind, "ok": ok, "reason": reason, "complete": complete, "plausible": plausible}
        if not ok:
            self.last_result = {**result, "message": f"Not usable: {message} (need >= 10, mostly "
                                "plausible, drone still). Check that both stations see all four sensors; retry."}
            return None
        if kind == "sweep":
            name, path = "sweep", os.path.join(self.session_dir, "sweep.json")
            if os.path.exists(path):  # XYZ-space samples accumulate over recordings
                with open(path, encoding="utf-8") as stream:
                    cap["records"] = json.load(stream) + cap["records"]
        elif kind in ("xy_plane", "verify"):
            numbers = [int(n.rsplit("_", 1)[1]) for n in self._files(kind)]
            name = f"{kind}_{max(numbers, default=0) + 1}"
        else:
            name = kind
        with open(os.path.join(self.session_dir, name + ".json"), "w", encoding="utf-8") as stream:
            json.dump(cap["records"], stream, indent=1)
        if kind == "sweep":
            self.sweep_count = self._count_sweep()
        else:  # show it in 3D right away, with the best geometry available
            threading.Thread(target=self._display_static, args=(self.session_dir, name), daemon=True).start()
        message = f"OK: {message} -> {name}.json"
        if kind == "sweep" and plausible < 0.5 * complete:
            message += (" — WARNING: most snapshots are physically impossible for the sensor board;"
                        " the solver will reject them")
        self.last_result = {**result, "name": name, "total": self.sweep_count if kind == "sweep" else None,
                            "message": message}
        return kind

    def _samples_changed(self, kind):
        """Re-solve after an estimation sample changed; only re-place samples after a verification one."""
        with self.lock:
            if kind == "verify":
                variant = self._primary_variant()
                if variant is None or self.solve["state"] == "running":
                    return
                self.solve["state"] = "running"
                args = (self.session_dir, self.solve["variants"])
            else:
                ready = self._ready()[0]
                if not (self.config.get("auto_solve") and ready):
                    if not ready and self.solve["state"] != "running":
                        self.solve = self._idle_solve()  # samples changed: the old geometry no longer applies
                        self.solve_version += 1
                    return
                if self.solve["state"] == "running":
                    self.pending = True
                    return
                args = None
        if args is not None:
            threading.Thread(target=self._finish_solve, args=args, daemon=True).start()
        else:
            self.start_solve()

    # ------------------------------------------------------------------- solve

    def _primary_variant(self):
        ok = [v for v in self.solve["variants"] if v["ok"]]
        return ok[-1] if ok else None  # a re-converted variant, when requested, comes last

    def start_solve(self):
        with self.lock:
            if lh2cap is None:
                raise ValueError("capture_lh2 module unavailable")
            if self.solve["state"] == "running":
                raise ValueError("solver already running")
            ready, missing = self._ready()
            if not ready:
                raise ValueError("not enough samples: " + ", ".join(missing))
            reconvert = self.config.get("reconvert") or ""
            if reconvert not in ("", "period", "factory"):
                raise ValueError("reconvert must be period, factory or empty")
            if self._preview_proc is not None and self._preview_proc.poll() is None:
                self._preview_proc.terminate()  # an estimate from part of the data is no longer needed
            self._write_manifest()
            preview = self.solve.get("preview")  # keep showing it until the final solve lands
            self.solve = self._idle_solve()
            self.solve["preview"] = preview
            self.solve["state"] = "running"
            self.solve_version += 1
            self.pending = False
            session = self.session_dir
            self._clean_session_files(session)
            args = (session, float(self.config.get("baseline") or 0), float(self.config.get("board_height") or 0),
                    reconvert, float(self.config.get("max_board_fit") or 5.0))
        threading.Thread(target=self._run_solve, args=args, daemon=True).start()

    def _stream(self, command, log_path):
        """Run a calibration script, appending its output to the live log and to log_path."""
        process = subprocess.Popen(command, cwd=CALIB_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, encoding="utf-8", errors="replace", env=_CHILD_ENV)
        with open(log_path, "w", encoding="utf-8") as log:
            for line in process.stdout:
                log.write(line)
                with self.lock:
                    self.solve["log"] += line
        return process.wait() == 0

    def _log(self, text):
        with self.lock:
            self.solve["log"] += text

    def _run_solve(self, session, baseline, board_height, reconvert, max_board_fit=5.0):
        # Same flow as capture_lh2.solve_session(): the firmware angles as recorded and, for sessions
        # recorded with the old fitted CAL_BS* firmware, a re-converted copy <session>_<model>.
        targets = [(session, "firmware angles")]
        variants = []
        try:
            if reconvert:
                converted = f"{session.rstrip(os.sep)}_{reconvert}"
                self._log(f"===== Re-converting angles ({reconvert} model) -> {converted} =====\n")
                if self._stream([sys.executable, "-u", os.path.join(CALIB_DIR, "reconvert_run.py"), session,
                                 converted, "--model", reconvert], os.path.join(session, "reconvert.log")):
                    targets.append((converted, reconvert + " re-conversion"))
            for target, name in targets:
                self._log(f"\n===== Solving {target} =====\n")
                command = [sys.executable, "-u", os.path.join(CALIB_DIR, "calibrate_bitcraze.py"), target,
                           "-o", os.path.join(target, "geometry.yaml")]
                if baseline:
                    command += ["--baseline", str(baseline)]
                if board_height:
                    command += ["--board-height", str(board_height)]
                command += ["--max-board-fit-mm", str(max_board_fit)]
                if not self._stream(command, os.path.join(target, "solve.log")):
                    geometry = os.path.join(target, "geometry.yaml")
                    if os.path.exists(geometry):  # do not show a previous solve's poses as this one's
                        os.remove(geometry)
                variants.append(load_solve_result(target, name))
        except Exception as error:  # report any solver/launch failure to the UI
            self._log(f"\nSolver launch failed: {error}\n")
        self._finish_solve(session, variants)

    def _finish_solve(self, session, variants):
        """Place the samples with the solved geometry (locate_samples.py), then publish the result."""
        ok = [v for v in variants if v["ok"]]
        samples = None
        if ok:
            command = [sys.executable, os.path.join(CALIB_DIR, "locate_samples.py"), session,
                       "--geometry", os.path.join(ok[-1]["dir"], "geometry.yaml"), "--json"]
            try:
                result = subprocess.run(command, cwd=CALIB_DIR, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        text=True, encoding="utf-8", errors="replace", env=_CHILD_ENV)
                if result.returncode == 0:
                    samples = json.loads(result.stdout)
                else:
                    self._log("\nlocate_samples.py failed:\n" + result.stderr[-2000:])
            except (OSError, ValueError) as error:
                self._log(f"\nlocate_samples.py failed: {error}\n")
        rerun = False
        with self.lock:
            if session != self.session_dir:
                return  # the user switched session meanwhile
            if ok:
                self.solve["preview"] = None
                for name in os.listdir(session):
                    if name.startswith("_preview"):
                        shutil.rmtree(os.path.join(session, name), ignore_errors=True)
            self.solve.update(state="done", ok=bool(ok), variants=variants, samples=samples,
                              primary=variants.index(ok[-1]) if ok else None)
            self.solve_version += 1
            rerun, self.pending = self.pending, False
        if rerun and self._ready()[0]:
            self.start_solve()

    def start_evaluation(self, runs=12):
        """Resample the solved session (evaluate_calibration.py) to measure how good the geometry is."""
        with self.lock:
            variant = self._primary_variant()
            if variant is None or self.solve["state"] == "running":
                raise ValueError("calculate the calibration first")
            if (self.solve.get("quality") or {}).get("state") == "running":
                raise ValueError("an evaluation is already running")
            self.solve["quality"] = {"state": "running", "progress": f"0/{runs}", "result": None, "error": None}
            self.solve_version += 1
            command = [sys.executable, "-u", os.path.join(CALIB_DIR, "evaluate_calibration.py"), variant["dir"],
                       "--runs", str(runs), "--json"]
            if self.config.get("baseline"):
                command += ["--baseline", str(self.config["baseline"])]
            if self.config.get("board_height"):
                command += ["--board-height", str(self.config["board_height"])]
            session = self.session_dir
        threading.Thread(target=self._run_evaluation, args=(session, command, runs), daemon=True).start()

    def _run_evaluation(self, session, command, runs):
        result, error, tail = None, None, []
        try:
            process = subprocess.Popen(command, cwd=CALIB_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding="utf-8", errors="replace", env=_CHILD_ENV)
            for line in process.stdout:
                tail = (tail + [line])[-20:]
                if line.startswith("run ") and "/" in line:
                    with self.lock:
                        if session == self.session_dir and self.solve.get("quality"):
                            self.solve["quality"]["progress"] = line.split()[1]
                            self.solve_version += 1
                elif line.startswith("{"):
                    result = json.loads(line)
            if process.wait() != 0 or result is None:
                error = "".join(tail).strip()[-600:] or "evaluation failed"
        except (OSError, ValueError) as exc:
            error = str(exc)
        with self.lock:
            if session != self.session_dir or not self.solve.get("quality"):
                return
            self.solve["quality"] = {"state": "done", "progress": f"{runs}/{runs}", "result": result, "error": error}
            self.solve_version += 1

    def export_header(self, directory):
        """bs_poses_cal.h candidate from a solved variant via calibrate_export.py (never hand-edited)."""
        with self.lock:
            known = {v["dir"] for v in self.solve["variants"] if v["ok"]}
        if directory not in known:
            raise ValueError("no solved geometry for that variant")
        output = os.path.join(directory, "bs_poses_cal_candidate.h")
        result = subprocess.run([sys.executable, os.path.join(CALIB_DIR, "calibrate_export.py"),
                                 "--yaml", os.path.relpath(os.path.join(directory, "geometry.yaml"),
                                                           os.path.join(CALIB_DIR, "..", "..")),
                                 "-o", output],
                                cwd=os.path.abspath(os.path.join(CALIB_DIR, "..", "..")),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                errors="replace", env=_CHILD_ENV)
        if result.returncode != 0:
            raise ValueError("calibrate_export.py failed:\n" + result.stdout)
        return {"path": output, "header": _read_text(output), "log": result.stdout}

    # ------------------------------------------------------------------ status

    def status(self, factory_ids, since_version=None, display_since=None):
        finished = None
        with self.lock:
            now = time.monotonic()
            finished = self._maybe_finish(now)
        if finished:
            self._samples_changed(finished)
        with self.lock:
            recent = [r for r in self.recent if now - r[0] < 2.0]
            total = sum(1 for r in recent if r[1])
            good = sum(1 for r in recent if r[2])
            cap = self.active
            ready, missing = self._ready()
            primary = self._primary_variant()
            geometry_ids = {p["id"] for p in primary["poses"]} if primary else set()
            self._refresh_display()
            simulating = self.sim is not None  # the virtual drone always "sees" both stations
            stations = [{"id": bs, "receiving": simulating or now - self.bs_last.get(bs, -1e9) < 1.0,
                         "calibration": str(bs) in factory_ids, "geometry": bs in geometry_ids}
                        for bs in range(lh2cap.NUM_BS if lh2cap else 2)]
            solve = dict(self.solve)
            if since_version is not None and since_version == self.solve_version and solve["state"] != "running":
                solve = {"state": solve["state"], "unchanged": True}  # the page already has the big part
            return {
                "available": lh2cap is not None,
                "udp_error": self.udp_error,
                "packets": self.packets,
                "last_packet_age": 0.0 if simulating else (now - self.last_packet) if self.packets else None,
                "simulation": None if not simulating else {
                    "source": self.sim["name"],
                    "floor_left": len([n for n in self._sim_files("xy_plane_") if n not in self.sim["used"]]),
                    "playing": self.sim.get("playing") if cap else None,
                },
                "quality": (100.0 * good / total) if total else None,
                "stations": stations,
                "session_dir": self.session_dir,
                "config": self.config,
                "samples": {"origin": bool(self._files("origin")), "x_axis": bool(self._files("x_axis")),
                            "xy_plane": self._files("xy_plane"), "verify": self._files("verify"),
                            "sweep": self.sweep_count, "sweep_min": self.MIN_SWEEP},
                "ready": ready,
                "missing": missing,
                "capture": None if cap is None else {
                    "step": cap["step"], "elapsed": now - cap["started"], "seconds": cap["seconds"],
                    "snapshots": cap["snapshots"], "complete": cap["complete"], "plausible": cap["plausible"],
                },
                "last_result": self.last_result,
                "solve_version": self.solve_version,
                "live": self._live_position(),
                # Placed samples, only when changed since the page's copy (it can be thousands of points).
                "display": None if display_since == self.display["version"] else {
                    "version": self.display["version"], "source": self.display["source"],
                    "label": self.display["label"], "statics": list(self.display["statics"].values()),
                    "cloud": list(self.display["cloud"])},
                "solve": solve,
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
        # POSITION + EXTRA3 (EKF status) for tracking; EXTENDED_STATUS (battery, GPS, waypoint),
        # EXTRA1 (attitude) and EXTRA2 (VFR_HUD) for the editor's HUD; RC_CHANNELS (sticks) for
        # the position-hold accuracy.
        for stream_id, rate in ((mavlink2.MAV_DATA_STREAM_POSITION, 4), (mavlink2.MAV_DATA_STREAM_EXTRA3, 4),
                                (mavlink2.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
                                (mavlink2.MAV_DATA_STREAM_EXTRA1, 10), (mavlink2.MAV_DATA_STREAM_EXTRA2, 4),
                                (mavlink2.MAV_DATA_STREAM_RC_CHANNELS, 5)):
            frames.append(
                mavlink2.MAVLink_request_data_stream_message(
                    sysid, compid, stream_id, rate, 1
                ).pack(self.mav)
            )
        return frames


def udp_receiver(state, capture, ootx, fc, listen_host, listen_port, track=None):
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    encoder = GcsEncoder()
    next_heartbeat = 0.0
    peers = {}  # address -> last time a stream request was sent
    peer_addresses = set()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        if os.name == "nt":
            # On Windows SO_REUSEADDR lets a second program bind the same port and silently take
            # part of the drone's packets (e.g. avvia.bat and LightHouse Studio together): refuse instead.
            udp.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            udp.bind((listen_host, listen_port))
        except OSError as error:
            capture.udp_error = f"UDP port {listen_port} is in use by another program (close avvia.bat or the other GCS): {error}"
            print(capture.udp_error, flush=True)
            return
        udp.settimeout(0.1)
        fc.attach(udp, encoder)
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
                fc.feed(message, address, kind)

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
                        payload = bytes(message.payload[:message.payload_length])
                        if payload[:2] == b"LO":  # the drone's own OOTX decode
                            decoded = lh2cap.parse_lh2_ootx(payload) if lh2cap else None
                            if decoded:
                                ootx.feed_wireless(decoded)
                        else:
                            capture.feed(payload)
                    continue

                if kind != "ODOMETRY":
                    continue
                if message.frame_id != 20 or message.child_frame_id != 12:
                    continue

                # Undo main.c: world (X, Y left, Z up) -> FRD -> turned by the +X heading.
                north, east = float(message.x), float(message.y)
                position = {
                    "x": north * WORLD_HEADING[0] + east * WORLD_HEADING[1],
                    "y": north * WORLD_HEADING[1] - east * WORLD_HEADING[0],
                    "z": -float(message.z),
                }
                # Yaw (main.c: clockwise about down, + the +X heading) -> world, CCW from +X.
                qw, qz = float(message.q[0]), float(message.q[3])
                if math.isfinite(qw) and math.isfinite(qz) and (qw or qz):
                    yaw_ned = 2.0 * math.atan2(qz, qw)
                    heading = math.atan2(WORLD_HEADING[1], WORLD_HEADING[0])
                    world = math.degrees(heading - yaw_ned)
                    position["yaw"] = (world + 180.0) % 360.0 - 180.0
                sequence = state.publish(position)
                if track:
                    track.add(position)
                if sequence == 1 or sequence % 10 == 0:
                    print(
                        "ODOMETRY "
                        f"x={position['x']:.3f} y={position['y']:.3f} "
                        f"z={position['z']:.3f} m"
                    )


def make_handler(state, capture, ootx, fc, mission, track):
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

        def _config(self, arg):
            """Wizard settings sent with session / config / solve requests."""
            number = lambda name, default: float(arg(name, str(default)) or default)
            return {
                "x_axis_dist": number("x_axis_dist", 1.0),
                "static_seconds": number("static_seconds", 5.0),
                "baseline": number("baseline", 0.0),
                "board_height": number("board_height", 0.0),
                "reconvert": arg("reconvert"),
                "auto_solve": arg("auto_solve", "1") not in ("0", "false"),
                "max_board_fit": min(20.0, max(1.0, number("max_board_fit", 5.0))),
            }

        def _lh2(self, request, body=None):
            query = parse_qs(request.query)
            arg = lambda name, default="": query.get(name, [default])[0].strip()
            action = request.path[len("/api/lh2/"):]
            try:
                host = arg("host")
                if host:
                    capture.set_target(host, int(arg("port", "14555")))
                if action == "status":
                    pass
                elif action.startswith("ootx/"):
                    command = action[len("ootx/"):]
                    if command == "ports":
                        self._json({"ports": OotxReader.ports()})
                        return
                    if command == "start":
                        ootx.start(arg("port"), float(arg("seconds", "120") or 120), int(arg("expected", "4") or 4))
                    elif command == "stop":
                        ootx.stop()
                    elif command == "save":
                        ootx.save(arg("source", "usb"))
                    elif command != "status":
                        self.send_error(404)
                        return
                    self._json(ootx.status())
                    return
                elif action == "runs":
                    self._json({"runs": list_runs(), "factory": load_factory_calibration()})
                    return
                elif action == "export":
                    self._json(capture.export_header(arg("variant")))
                    return
                elif action == "export_samples":
                    self._json(capture.export_samples())
                    return
                elif action == "import_samples":
                    capture.import_samples(json.loads(body or b"null"), self._config(arg))
                elif action == "session":
                    capture.configure(arg("dir") or time.strftime("calib_runs/run_%Y%m%d_%H%M%S"), self._config(arg))
                elif action == "config":
                    with capture.lock:
                        capture.config.update(self._config(arg))
                elif action == "start":
                    capture.start(arg("step"), float(arg("seconds", "5.0")))
                elif action == "stop":
                    capture.stop()
                elif action == "delete":
                    capture.delete_sample(arg("name"))
                elif action == "clear":
                    capture.clear_samples()
                elif action == "simulate":
                    if arg("source"):
                        capture.start_simulation(arg("source"), self._config(arg))
                    else:
                        capture.stop_simulation()
                elif action == "evaluate":
                    capture.start_evaluation(max(3, min(30, int(arg("runs", "12") or 12))))
                elif action == "solve":
                    with capture.lock:
                        capture.config.update(self._config(arg))
                    capture.start_solve()
                else:
                    self.send_error(404)
                    return
            except (ValueError, OSError) as error:
                self._json({"error": str(error)}, 400)
                return
            since, dsince = arg("since"), arg("dsince")
            self._json(capture.status(set(load_factory_calibration()) | ootx.wireless_ids(),
                                      int(since) if since.isdigit() else None,
                                      int(dsince) if dsince.isdigit() else None))

        def do_POST(self):
            request = urlsplit(self.path)
            if request.path.startswith("/api/lh2/"):
                length = int(self.headers.get("Content-Length") or 0)
                if length > 64 * 1024 * 1024:
                    self.send_error(413)
                    return
                self._lh2(request, self.rfile.read(length) if length else None)
                return
            self.do_GET()

        def do_GET(self):
            request = urlsplit(self.path)
            if request.path.startswith("/api/lh2/"):
                self._lh2(request)
                return
            if request.path.startswith("/api/fc/mission/"):
                action = request.path[len("/api/fc/mission/"):]
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                    body = json.loads(self.rfile.read(length) or b"{}") if 0 < length < 65536 else {}
                    if body.get("limits"):
                        mission.set_limits(body["limits"])
                    if action == "limits":
                        pass
                    elif action == "start":
                        mission.start(body.get("points", []), body.get("speed", 0.3),
                                      body.get("tolerance", 0.10), body.get("settle", 1.0), body.get("frame", "absolute"),
                                      bool(body.get("pass_through")))
                    elif action == "auto":
                        mission.set_auto(body if body.get("enabled") else None)
                    elif action == "stop":
                        mission.stop()
                    elif action != "status":
                        self.send_error(404)
                        return
                    self._json(mission.snapshot())
                except (ValueError, OSError, TypeError) as error:
                    self._json(dict(mission.snapshot(), error=str(error)), 400)
                return
            if request.path.startswith("/api/track/"):
                action = request.path[len("/api/track/"):]
                query = parse_qs(request.query)
                arg = lambda name: query.get(name, [""])[0].strip()
                try:
                    if action == "start":
                        self._json(track.start(arg("name")))
                    elif action == "stop":
                        self._json(track.stop())
                    elif action == "status":
                        self._json(track.status())
                    elif action == "list":
                        self._json({"tracks": track.list(), "status": track.status()})
                    elif action == "get":
                        self._json(track.get(arg("name")))
                    elif action == "delete":
                        track.delete(arg("name"))
                        self._json({"tracks": track.list(), "status": track.status()})
                    else:
                        self.send_error(404)
                except (ValueError, OSError) as error:
                    self._json({"error": str(error)}, 400)
                return
            if request.path == "/api/fc/param":
                query = parse_qs(request.query)
                value = query.get("value", [""])[0].strip()
                try:
                    self._json(fc.param(query.get("name", [""])[0].strip(), value if value else None))
                except (ValueError, OSError) as error:
                    self._json({"error": str(error)}, 400)
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
            self._json(dict(state.snapshot(), fc=fc.snapshot(), mission=mission.snapshot(), track=track.status()))

        def log_message(self, _format, *_args):
            pass

    return Handler


def _exit_with_parent(pid):
    """Quit when the parent process ends, so no orphan bridge keeps UDP 14550 busy.

    Windows: wait on the process handle (reading a stdin pipe in a thread instead can stall
    other handle operations of this process). Elsewhere: poll that the process still exists."""
    if os.name == "nt":
        import ctypes
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.OpenProcess(0x00100000, False, pid)  # SYNCHRONIZE
        if handle:
            kernel32.WaitForSingleObject(handle, 0xFFFFFFFF)  # INFINITE
        os._exit(0)
    while True:
        try:
            os.kill(pid, 0)  # POSIX: signal 0 only checks that the process still exists
        except ProcessLookupError:
            os._exit(0)
        except PermissionError:
            pass  # exists, owned by someone else
        time.sleep(1.0)


def main():
    cli = argparse.ArgumentParser(description="Standalone MAVLink ODOMETRY UDP relay")
    cli.add_argument("--udp-host", default="0.0.0.0", help="UDP bind address")
    cli.add_argument("--udp-port", type=int, default=14550, help="Local UDP listen port")
    cli.add_argument("--http-port", type=int, default=8051, help="Local HTTP API port")
    cli.add_argument("--parent-pid", type=int,
                     help="quit when this process ends (the desktop app that started the bridge)")
    args = cli.parse_args()
    if args.parent_pid:
        threading.Thread(target=_exit_with_parent, args=(args.parent_pid,), daemon=True).start()

    state = BridgeState()
    capture = CaptureManager()
    # Import the solver modules (scipy) now, in the background, so the first live-position
    # request does not stall the HTTP replies for seconds while holding the capture lock.
    threading.Thread(target=lambda: __import__("locate_samples"), daemon=True).start()
    ootx = OotxReader()
    fc = FlightController()
    mission = MissionRunner(state, fc)
    track = TrackRecorder(fc, mission)
    receiver = threading.Thread(
        target=udp_receiver,
        args=(state, capture, ootx, fc, args.udp_host, args.udp_port, track),
        daemon=True,
    )
    receiver.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(state, capture, ootx, fc, mission, track))
    print(f"HTTP API: http://127.0.0.1:{args.http_port}/api/odometry")
    print(f"Calibration sessions: {RUNS_DIR}")
    print(f"Flight tracks: {TRACKS_DIR}")
    print(f"READY http://127.0.0.1:{args.http_port}", flush=True)  # the desktop app waits for this line
    print("Lascia aperta questa finestra mentre usi il Pose Editor.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nRelay arrestato.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
