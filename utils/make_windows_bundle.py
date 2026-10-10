#!/usr/bin/env python3
"""
make_windows_bundle.py — build lh2_windows_bundle/, a self-contained folder for Windows:
BS Pose Editor (3D models, toolbar, auto-calibration) + the UDP/HTTP bridge + the
Bitcraze calibration tools + the current firmware UF2s.

    python utils/make_windows_bundle.py              # -> lh2_windows_bundle/ and lh2_windows_bundle.zip
    python utils/make_windows_bundle.py --no-zip

Re-run after every change to the files listed in FILES. Calibration runs already in the
bundle's utils/calibration/calib_runs/ are kept; everything else is rebuilt.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# (source in the repo, destination in the bundle); directories are copied whole.
FILES = [
    ("bs_pose_editor.html", "bs_pose_editor.html"),
    ("docs/index.html", "docs/index.html"),
    ("docs/logview.html", "docs/logview.html"),
    ("docs/valve-index-lighthousebasestation-gen2/source/lh_basestation_valve_gen2.obj",
     "docs/valve-index-lighthousebasestation-gen2/source/lh_basestation_valve_gen2.obj"),
    ("docs/valve-index-lighthousebasestation-gen2/textures/lh_basestation_valve_gen2.png",
     "docs/valve-index-lighthousebasestation-gen2/textures/lh_basestation_valve_gen2.png"),
    ("utils/user_interface/display_real_time_windows.py", "utils/user_interface/display_real_time_windows.py"),
    ("utils/angle_lib/lighthouse_types.py", "utils/angle_lib/lighthouse_types.py"),
    ("utils/calibration/calibration_lib", "utils/calibration/calibration_lib"),
    ("utils/calibration/capture_lh2.py", "utils/calibration/capture_lh2.py"),
    ("utils/calibration/calibrate_bitcraze.py", "utils/calibration/calibrate_bitcraze.py"),
    ("utils/calibration/reconvert_run.py", "utils/calibration/reconvert_run.py"),
    ("utils/calibration/locate_samples.py", "utils/calibration/locate_samples.py"),
    ("utils/calibration/calibrate_export.py", "utils/calibration/calibrate_export.py"),
    ("utils/calibration/lh2_ootx.py", "utils/calibration/lh2_ootx.py"),
    ("utils/calibration/make_synthetic_measurements.py", "utils/calibration/make_synthetic_measurements.py"),
    ("utils/calibration/wand_sensors.json", "utils/calibration/wand_sensors.json"),
    ("utils/calibration/lh2_factory_calibration.json", "utils/calibration/lh2_factory_calibration.json"),
    ("utils/calibration/README.md", "utils/calibration/README.md"),
    ("utils/calibration/calib_runs/RESULTS.md", "utils/calibration/calib_runs/RESULTS.md"),
    # Reference run whose poses are in bs_poses_cal.h: open it from the editor's "Open run".
    ("utils/calibration/calib_runs/run_20261009_180300", "utils/calibration/calib_runs/run_20261009_180300"),
    ("rp2350_firmware/src/build/crossing_beams.uf2", "firmware/crossing_beams.uf2"),
    ("rp2350_firmware/src/build/crossing_beams_synthetic.uf2", "firmware/crossing_beams_synthetic.uf2"),
    ("rp2350_firmware/src/bs_poses_cal.h", "firmware/bs_poses_cal.h"),
    ("rp2350_firmware/src/lh2_factory_cal.h", "firmware/lh2_factory_cal.h"),
]
PACKAGES = ["utils", "utils/angle_lib", "utils/calibration", "utils/user_interface"]
KEEP = "utils/calibration/calib_runs"  # user data, survives a rebuild
LEGACY_RUNS = "calib_runs"  # where bundles before the merge kept them (bridge cwd-relative)

REQUIREMENTS = """pymavlink
numpy
scipy
pyyaml
pyserial
"""

# CRLF: cmd.exe misparses some LF-only batch files.
AVVIA_BAT = r"""@echo off
setlocal
cd /d "%~dp0"
title LH2 bridge

rem --- Python environment (created once, in .venv next to this file) ---
if not exist ".venv\Scripts\python.exe" (
    echo Creazione ambiente Python in .venv ...
    py -3 -m venv .venv 2>nul || python -m venv .venv
    if not exist ".venv\Scripts\python.exe" (
        echo ERRORE: Python 3.10+ non trovato. Installalo da https://www.python.org e riprova.
        pause
        exit /b 1
    )
)
".venv\Scripts\python.exe" -c "import pymavlink, numpy, scipy, yaml, serial" 2>nul || (
    echo Installazione dipendenze ...
    ".venv\Scripts\python.exe" -m pip install --upgrade pip
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt || (pause & exit /b 1)
)

rem --- Web server for the editor (port 8765) + browser ---
start "LH2 editor HTTP" /min ".venv\Scripts\python.exe" -m http.server 8765 --bind 127.0.0.1 --directory "%~dp0."
timeout /t 2 >nul
start "" http://127.0.0.1:8765/bs_pose_editor.html

rem --- MAVLink UDP 14550 -> HTTP 8051 bridge (keep this window open) ---
".venv\Scripts\python.exe" utils\user_interface\display_real_time_windows.py %*
pause
"""

LEGGIMI = """LH2 — bundle Windows ({commit})
=====================================

Avvio
-----
Doppio clic su avvia.bat.
 - La prima volta crea .venv e installa le dipendenze (serve Python 3.10+ e internet).
 - Apre il BS Pose Editor su http://127.0.0.1:8765/bs_pose_editor.html
 - Avvia il bridge MAVLink: UDP 14550 -> HTTP 127.0.0.1:8051. Tieni aperta la finestra.
I modelli 3D e la toolbar sono nell'editor; three.js arriva da cdn.jsdelivr.net,
quindi il PC deve avere internet (o averlo in cache) quando apri la pagina.

Collegamento al drone
---------------------
PC sulla Wi-Fi del MavESP8266 / DroneBridge (192.168.4.1).
Editor -> Drone: Link "UDP relay", Relay http://127.0.0.1:8051/api/odometry, Connect.
Windows Firewall deve permettere a Python di ricevere su UDP 14550.

Auto-calibrazione (Editor -> Calib), come il wizard del Crazyflie client
-----------------------------------------------------------------------
1. Firmware: flasha firmware\\crossing_beams.uf2 (angoli con modello period + factory).
2. SETTINGS: IP 192.168.4.1, Port 14555, Tape BS0-BS1 = distanza misurata col metro.
   BASE STATION STATUS: "Receiving" deve essere verde per entrambe le stazioni.
3. SAMPLE COLLECTION, un riquadro alla volta (< > per spostarsi):
   Origin -> X-axis (1 m) -> XY-plane (3 o piu' punti a terra) -> XYZ-space (sweep, Stop
   quando hai finito; si puo' aggiungere altro dopo). La prima misura crea
   utils\\calibration\\calib_runs\\run_<data>.
4. Appena i campioni bastano il solver parte da solo. Barra verde = geometria risolta.
   Nel 3D: campioni (O, X, punti blu, nuvola grigia) e stazioni risolte (magenta).
5. Verification (opzionale): misure in punti noti, non usate dal solver; errore in
   "Max verification sample error".
6. GEOMETRY RESULT: controlla le spunte, "Apply to editor", "Export header" ->
   utils\\calibration\\calib_runs\\<run>\\bs_poses_cal_candidate.h da copiare in
   rp2350_firmware\\src\\bs_poses_cal.h nel repo, ricompilare e committare.
SAMPLE MANAGEMENT: dettagli e cancellazione dei singoli campioni, Clear (sposta i file in
_cleared_<data>\\, non li cancella), Import / Export dei campioni in un file .json.
"Open run" (SETTINGS) riapre una run gia' fatta (c'e' run_20261009_180300, quella nel firmware).

Da riga di comando (dalla cartella del bundle)
----------------------------------------------
.venv\\Scripts\\python utils\\calibration\\capture_lh2.py                    wizard da terminale
.venv\\Scripts\\python utils\\calibration\\calibrate_bitcraze.py <run> --baseline 2.26
.venv\\Scripts\\python utils\\calibration\\lh2_ootx.py --port COM5           calibrazione di fabbrica (USB Pico)

Contenuto
---------
bs_pose_editor.html   editor 3D + auto-calibrazione
docs\\                 GCS (index.html, richiede display_real_time.py dal repo), logview.html, modello 3D
utils\\                bridge e strumenti di calibrazione
firmware\\             UF2 attuali + bs_poses_cal.h / lh2_factory_cal.h compilati dentro
"""


def _commit() -> str:
    try:
        out = subprocess.run(["git", "-C", str(REPO), "describe", "--always", "--dirty"],
                             capture_output=True, text=True, check=True)
        branch = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--abbrev-ref", "HEAD"],
                                capture_output=True, text=True, check=True)
        return f"{branch.stdout.strip()} @ {out.stdout.strip()}"
    except (OSError, subprocess.CalledProcessError):
        return "unknown commit"


def build(out: Path) -> None:
    kept = out.parent / (out.name + ".runs_tmp")
    if kept.exists():
        raise SystemExit(f"{kept} exists (left by an interrupted build): move its runs back first")
    if out.exists():
        for runs in (out / KEEP, out / LEGACY_RUNS):
            if runs.is_dir():
                kept.mkdir(exist_ok=True)
                for run in runs.iterdir():
                    shutil.move(str(run), kept / run.name)
        venv = out / ".venv"
        for item in out.iterdir():  # keep the Windows .venv so dependencies are not reinstalled
            if item != venv:
                shutil.rmtree(item) if item.is_dir() else item.unlink()
    out.mkdir(parents=True, exist_ok=True)
    if kept.exists():
        (out / KEEP).parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(kept), out / KEEP)

    ignore = shutil.ignore_patterns("__pycache__", "*.pyc")
    for src, dst in FILES:
        source, target = REPO / src, out / dst
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.is_dir():
            shutil.copytree(source, target, ignore=ignore, dirs_exist_ok=True)
        else:
            shutil.copy2(source, target)
    for package in PACKAGES:
        (out / package / "__init__.py").touch()

    commit = _commit()
    (out / "requirements.txt").write_text(REQUIREMENTS, encoding="utf-8")
    (out / "avvia.bat").write_bytes(AVVIA_BAT.replace("\n", "\r\n").encode("ascii"))
    (out / "LEGGIMI.txt").write_bytes(LEGGIMI.format(commit=commit).replace("\n", "\r\n").encode("utf-8"))
    print(f"Built {out} ({commit})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-o", "--output", type=Path, default=REPO / "lh2_windows_bundle")
    parser.add_argument("--no-zip", action="store_true", help="do not write <output>.zip")
    args = parser.parse_args()
    out = args.output.resolve()
    build(out)
    if not args.no_zip:
        print(f"Wrote {_zip_without_venv(out)}")
    return 0


def _zip_without_venv(out: Path) -> str:
    import zipfile
    archive = out.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(out.rglob("*")):
            rel = path.relative_to(out)
            if rel.parts[0] == ".venv" or "__pycache__" in rel.parts or path.is_dir():
                continue
            zf.write(path, Path(out.name) / rel)
    return str(archive)


if __name__ == "__main__":
    raise SystemExit(main())
