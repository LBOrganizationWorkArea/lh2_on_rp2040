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
FACTORY_CAL = os.path.join(CALIB_DIR, "lh2_factory_calibration.json")
# Station poses compiled into the firmware: repo layout, then the Windows bundle layout.
FIRMWARE_POSES = next((p for p in (os.path.join(CALIB_DIR, "..", "..", "rp2350_firmware", "src", "bs_poses_cal.h"),
                                    os.path.join(CALIB_DIR, "..", "..", "firmware", "bs_poses_cal.h"))
                       if os.path.exists(p)), None)
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
    PREVIEW_EVERY_S = 12.0  # during a sweep, re-estimate the stations this often (live 3D preview)

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
        complete = len(records) == lh2cap.NUM_BS
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
        if os.name == "nt":
            # On Windows SO_REUSEADDR lets a second program bind the same port and silently take
            # part of the drone's packets (e.g. avvia.bat and LH2 Studio together): refuse instead.
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
            self._json(capture.status(set(load_factory_calibration()), int(since) if since.isdigit() else None,
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
    receiver = threading.Thread(
        target=udp_receiver,
        args=(state, capture, args.udp_host, args.udp_port),
        daemon=True,
    )
    receiver.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(state, capture))
    print(f"HTTP API: http://127.0.0.1:{args.http_port}/api/odometry")
    print(f"Calibration sessions: {RUNS_DIR}")
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
