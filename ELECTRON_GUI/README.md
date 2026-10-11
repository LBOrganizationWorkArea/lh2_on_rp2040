# LH2 Studio — desktop app (Electron)

One Windows program for the base-station editor, the auto-calibration wizard, the simulation and
the drone tracking. No browser, no `avvia.bat`, no Python install on the PC.

```
LH2 Studio.exe  (Electron)
 ├─ window: bs_pose_editor.html, served from the app (app://), three.js included — works offline
 └─ backend: utils/user_interface/display_real_time_windows.py (the same bridge as the browser
             bundle), run by an embedded Python 3.11 with numpy / scipy / pymavlink; started on a
             free port when the app opens, stopped when it closes
```

The UI has a single source: the repository's `bs_pose_editor.html`. `scripts/prepare.mjs` copies it
into `app/` and points its three.js imports to local files; nothing in this folder is a fork of it.

Where things go in the installed app:

| What | Where |
|---|---|
| Calibration sessions (`calib_runs`) | `Documents\LH2\calib_runs` (the reference run is copied there on first start) |
| Backend log | `%APPDATA%\LH2 Studio\logs\backend.log` — Help → Backend log |
| Firmware UF2s / headers | File → Open firmware folder |

The app refuses a second copy of itself, and on Windows the bridge opens UDP 14550 exclusively: if
`avvia.bat` (or another GCS) is still running, the Calib tab says the port is in use instead of
silently losing half of the drone's packets.

## Run it while developing

Needs Node 20+ and the repository's Python environment (`.venv` at the repo root, or set `LH2_PYTHON`).

```bash
cd ELECTRON_GUI
npm install
npm start            # copies the editor into app/, starts Electron + the bridge from the repository
```

In development the sessions stay in `utils/calibration/calib_runs`, as with the browser bundle.

## Build the Windows app

```bash
npm run dist:win       # on Windows: installer (NSIS) + portable zip in dist/
npm run dist:win-zip   # on Linux / WSL: portable zip only (the installer needs Windows or Wine)
```

Each build:

1. `prepare.mjs --backend` copies the editor into `app/`, and builds `build/backend/` from the same
   file list as the Windows bundle (`utils/make_windows_bundle.py`): bridge, Bitcraze calibration
   tools, reference run, firmware UF2s.
2. `fetch-python-win.mjs` builds `build/python-win/`: the official embeddable CPython 3.11.9 plus the
   `requirements-win.txt` wheels (pinned to the versions the tools were tested with). It only
   downloads and unpacks, so it also works from Linux. Its `python311._pth` puts the backend's script
   folders on `sys.path`, because the embeddable Python ignores `PYTHONPATH` and the script folder.
3. `electron-builder` packages everything (`dist/win-unpacked`, zip, installer).

The executable is not code-signed: Windows SmartScreen warns on the first start ("More info → Run
anyway"). The icon is Electron's default until an `.ico` is added.

## Checking a build on Windows

```bat
"dist\win-unpacked\resources\python\python.exe" -c "import numpy, scipy, yaml, serial, pymavlink; print('ok')"
"dist\win-unpacked\LH2 Studio.exe"
```

If the window stays on the error page, Help → Backend log shows the Python output.
