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
CALIB_DIR = os.path.dirname(os.path.abspath(lh2cap.__file__)) if lh2cap else os.path.join(_HERE, "..", "calibration")
RUNS_DIR = os.path.join(CALIB_DIR, "calib_runs")
FACTORY_CAL = os.path.join(CALIB_DIR, "lh2_factory_calibration.json")
# Calibration scripts print and write UTF-8 (e.g. headers) also on Windows, where the default is cp1252.
_CHILD_ENV = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8")

# Key lines of calibrate_bitcraze.py's report, shown as figures in the pose editor.
_LOG_METRICS = (
    ("residual_mean_mm", r"Residual: mean ([\d.]+) mm"),
    ("residual_max_mm", r"Residual: mean [\d.]+ mm, max ([\d.]+) mm"),
    ("board_fit_mm", r"Wand shape check: median rigid-fit error ([\d.]+) mm"),
    ("mark_dist_m", r"x-axis capture is ([\d.]+) m from the origin"),
    ("baseline_m", r"Estimated baseline: ([\d.]+) m"),
    ("tape_m", r"Tape baseline ([\d.]+) m"),
    ("samples", r"final solve used (\d+)"),
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


def load_factory_calibration():
    """OOTX factory data per station (lh2_ootx.py), with the tilt its accelerometer reports."""
    if not os.path.exists(FACTORY_CAL):
        return {}
    with open(FACTORY_CAL, encoding="utf-8") as stream:
        data = json.load(stream)
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
        }
    return stations


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

    def __init__(self):
        self.lock = threading.Lock()
        self.session_dir = ""
        self.config = {"x_axis_dist": 1.0, "static_seconds": 5.0, "baseline": 0.0, "board_height": 0.0,
                       "reconvert": "", "auto_solve": True}
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
        if lh2cap is not None:
            sensors = os.path.join(CALIB_DIR, "wand_sensors.json")
            try:
                self.span_limit = lh2cap.wand_span_limit(sensors, 0.5)
            except OSError:
                print("wand_sensors.json not found: plausibility check disabled")

    @staticmethod
    def _idle_solve():
        return {"state": "idle", "log": "", "ok": None, "variants": [], "primary": None, "samples": None}

    def set_target(self, host, port):
        with self.lock:
            self.target = (host, port) if host else None

    def get_target(self):
        with self.lock:
            return self.target

    # ----------------------------------------------------------------- session

    def configure(self, session_dir, config):
        # Relative directories live next to capture_lh2.py, like its calib_runs/run_<date> sessions.
        session_dir = os.path.abspath(os.path.join(CALIB_DIR, session_dir))
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
            self.sweep_count = self._count_sweep()
            self.solve = self._idle_solve()
            self.solve_version += 1
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
        if not self.session_dir:
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
        finished = None
        with self.lock:
            if key in self.seen:
                return
            self.seen.add(key)
            now = time.monotonic()
            self.packets += 1
            self.last_packet = now
            records = lh2cap.snapshot_to_records(snapshot)
            for record in records:
                self.bs_last[record["base_station_id"]] = now
            complete = len(records) == lh2cap.NUM_BS
            plausible = complete and self._plausible(records)
            self.recent = [r for r in self.recent if now - r[0] < 2.0] + [(now, complete, plausible)]
            cap = self.active
            if cap is not None:
                cap["records"].extend(records)
                cap["snapshots"] += 1
                cap["complete"] += int(complete)
                cap["plausible"] += int(plausible)
            finished = self._maybe_finish(now)
        if finished:
            self._samples_changed(finished)

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
            self.active = {"step": kind, "seconds": None if kind == "sweep" else seconds,
                           "started": time.monotonic(), "records": [], "snapshots": 0,
                           "complete": 0, "plausible": 0}
            self.last_result = None

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
        if ok and kind != "sweep":
            cleaned = self._clean_static(cap["records"])
            if len(cleaned) < 0.7 * len(cap["records"]):
                ok = False
                message += "; drone not still"
            cap["records"] = cleaned
        if not ok:
            self.last_result = {"step": kind, "ok": False, "message": f"Not usable: {message} (need >= 10, mostly "
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
        message = f"OK: {message} -> {name}.json"
        if kind == "sweep" and plausible < 0.5 * complete:
            message += (" — WARNING: most snapshots are physically impossible for the sensor board;"
                        " the solver will reject them")
        self.last_result = {"step": kind, "ok": True, "message": message}
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
            self._write_manifest()
            self.solve = self._idle_solve()
            self.solve["state"] = "running"
            self.solve_version += 1
            self.pending = False
            session = self.session_dir
            self._clean_session_files(session)
            args = (session, float(self.config.get("baseline") or 0), float(self.config.get("board_height") or 0),
                    reconvert)
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

    def _run_solve(self, session, baseline, board_height, reconvert):
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
            self.solve.update(state="done", ok=bool(ok), variants=variants, samples=samples,
                              primary=variants.index(ok[-1]) if ok else None)
            self.solve_version += 1
            rerun, self.pending = self.pending, False
        if rerun and self._ready()[0]:
            self.start_solve()

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

    def status(self, factory_ids, since_version=None):
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
            stations = [{"id": bs, "receiving": now - self.bs_last.get(bs, -1e9) < 1.0,
                         "calibration": str(bs) in factory_ids, "geometry": bs in geometry_ids}
                        for bs in range(lh2cap.NUM_BS if lh2cap else 2)]
            solve = dict(self.solve)
            if since_version is not None and since_version == self.solve_version and solve["state"] != "running":
                solve = {"state": solve["state"], "unchanged": True}  # the page already has the big part
            return {
                "available": lh2cap is not None,
                "packets": self.packets,
                "last_packet_age": (now - self.last_packet) if self.packets else None,
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
            since = arg("since")
            self._json(capture.status(set(load_factory_calibration()), int(since) if since.isdigit() else None))

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
