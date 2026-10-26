# LH2 indoor positioning on RP2350

Indoor drone positioning with Valve **Lighthouse v2** base stations. A Raspberry Pi Pico 2 (RP2350) on
the drone reads 4 TS4231 light sensors. It decodes the sweeps of up to **4 base stations** and
triangulates its own position, then sends it to the Pixhawk as MAVLink `ODOMETRY` at 10 Hz. ArduPilot
flies indoors on it as if it were GPS.

The capture layer comes from [DotBots/lh2_on_rp2040](https://github.com/DotBots/lh2_on_rp2040). The
geometry calibration follows the Bitcraze Crazyflie lighthouse system, and so does the factory
calibration (OOTX). Everything is driven from one desktop app, **LightHouse Studio**.

```
 Base stations BS4 · BS10 · BS0 · BS1  (LH2, 50 Hz rotors)
        │  light sweeps
        ▼
 4× TS4231 ──► RP2350 (Pico 2) on the drone
               core 1: PIO + DMA capture, LFSR decoding
               core 0: angles → factory correction → triangulation (≥ 2 stations)
                       on-board OOTX decoding (factory calibration of each station)
        │ UART (MAVLink v2)
        ▼
 Pixhawk / ArduPilot ── ODOMETRY @ 10 Hz ──► EKF (position)
        │ Wi-Fi telemetry (MavESP8266 / DroneBridge, UDP 14550)
        ▼
 LightHouse Studio (Windows) ── base-station editor · calibration wizard · live tracking
```

---

## Contents

1. [Repository layout](#repository-layout)
2. [Quick start](#quick-start)
3. [Base stations](#base-stations)
4. [LightHouse Studio — the desktop app](#lighthouse-studio--the-desktop-app)
5. [Calibration: what to do and when](#calibration-what-to-do-and-when)
6. [Firmware](#firmware)
7. [Backend (bridge) and other tools](#backend-bridge-and-other-tools)
8. [Hardware](#hardware)
9. [Credits and references](#credits-and-references)

---

## Repository layout

| Path | What |
|---|---|
| `rp2350_firmware/src/` | Pico 2 firmware (C, Pico SDK). The built UF2s are committed in `build/`. |
| `bs_pose_editor.html` | The UI: 3D base-station editor, calibration wizard, drone view. It is the single source for the desktop app and the browser bundle. |
| `ELECTRON_GUI/` | **LightHouse Studio**: the Electron desktop app with an embedded Python backend ([its README](ELECTRON_GUI/README.md)). |
| `utils/user_interface/display_real_time_windows.py` | The **bridge**: it receives the drone's MAVLink over UDP and serves `/api/*` to the UI. It also runs captures, solves and OOTX. |
| `utils/calibration/` | Calibration tools: capture, Bitcraze solver, evaluation, export to firmware headers, OOTX ([its README](utils/calibration/README.md)). |
| `utils/make_windows_bundle.py` → `lh2_windows_bundle/` | The older browser bundle for Windows (`avvia.bat` + editor in the browser). |
| `docs/index.html`, `docs/logview.html` | Browser GCS and flight-log viewer. |
| `.claude/rules/` | Detailed engineering notes per area (firmware, utils, website, known bugs). |

---

## Quick start

1. **Flash the drone.** Hold BOOTSEL on the Pico 2, plug in USB and copy
   `rp2350_firmware/src/build/crossing_beams.uf2` to the `RP2350` drive.
   (`crossing_beams_synthetic.uf2` needs no sensors: it flies a virtual 1 × 1 m square, for bench tests.)
2. **Start LightHouse Studio.** Unzip `LightHouse Studio-<version>-win.zip` and run
   `LightHouse Studio.exe`. On the first start, Windows SmartScreen asks: *More info → Run anyway*.
   Nothing else needs to be installed, because Python and its libraries are inside the app.
3. **Connect.** Join the drone's Wi-Fi radio. In the **Drone** tab, enter the radio's IP (AP mode:
   `192.168.4.1`) and press **Connect**. The base-station grid in **Calib** shows which stations the
   drone receives.
4. **Calibrate.** First time only: get the factory calibration of each station, then calibrate the
   geometry. See [Calibration](#calibration-what-to-do-and-when).

---

## Base stations

The firmware handles up to four stations. Each station is identified by its polynomial pair, which is set by its **channel** (mode) on the
back of the station.

| Firmware index | Station | Channel | Polynomials | Rotor period (48 MHz ticks) |
|---|---|---|---|---|
| 0 | **BS4** | 5 | 8 / 9 | 947 000 |
| 1 | **BS10** | 11 | 20 / 21 | 919 000 |
| 2 | **BS0** | 1 | 0 / 1 | 959 000 |
| 3 | **BS1** | 2 | 2 / 3 | 957 000 |

- A position needs **at least 2 calibrated stations** that see the sensor. With more stations the
  solver uses all of them: it takes the least-squares closest point to every ray and drops an
  outlier ray when there are 3 or more.
- A station that has not been calibrated yet (all-zero pose in `bs_poses_cal.h`) is ignored, so 2-station setups and
  older calibrations keep working.
- BS4/BS10 keep indices 0/1, so sessions recorded with two stations stay valid.

---

## LightHouse Studio — the desktop app

The window has two modes, switched at the top left:

- **Sandbox**: plan an installation. Place base stations by hand, check field of view, range and
  overlap, and generate the C header for the poses.
- **Calibration**: measure the real installation with the drone, using the wizard, the 3D view of the
  samples and stations, and the live drone position.

### Toolbar tabs

| Tab | Mode | What it does |
|---|---|---|
| **Drone** | both | **The only connection** to the drone, shared by every other tab: the radio's IP, Connect/Disconnect, link status and live `ODOMETRY` position. Also a Mission Planner-style **HUD** of the flight controller (attitude, heading, speed and altitude tapes, climb rate, battery, link quality, mode, armed state, GPS/EKF, waypoint, status text), and read/write of the ExtNav parameters `VISO_DELAY_MS` and `VISO_POS_M_NSE`. A **delay-test log timer** sets `LOG_DISARMED = 1` after a chosen delay and back to 0 after a chosen time (run by the bridge, with beeps on the PC), so the back-and-forth for the delay measurement can be logged without touching the PC. Under X / Y / Z: battery, flight mode, armed state and, in PosHold / Loiter, the **hold accuracy** — horizontal distance from the point where the drone settled (1.5 s after the mode is engaged or the sticks are centred again), with rms, max, Z rms and time. |
| **Base stations** | Sandbox | Position (m, Z-up) and orientation of each station. *Inc* is the tilt from vertical: 0° = pointing straight down. |
| **Room** | both | Room size and colours. |
| **FOV** | Sandbox | Field-of-view cones and range. |
| **Overlap** | Sandbox | Where 1, 2, 3 or 4 stations cover the floor and the flight volume. |
| **Measure** | both | Distance tool in the 3D view. |
| **Code** | Sandbox | The `bs_poses_cal.h` content for the poses on screen. |
| **Calib** | Calibration | The calibration wizard (below). |
| **Mission** | both | A list of room points (X / Y / Z, the same as the ODOMETRY panel) flown one by one in GUIDED — see [Automatic mission](#automatic-mission-guided). |
| **Track** | both | Flight log: records the path and builds an **accuracy report** of how still the drone holds a point — see [Track: flight log and hover accuracy](#track-flight-log-and-hover-accuracy). |
| **?** | both | Help. |

### The Calib panel

**Base station status.** Like the Crazyflie client, each station has three rows, which turn green when ready:

- **Receiving**: the station currently sees all four sensors.
- **Calibration**: its factory calibration (OOTX) is known, either saved or just decoded by the drone.
- **Geometry**: its position has been solved in this session.

**Sample collection** (the wizard, in order):

| Step | What you do | File in the session |
|---|---|---|
| **Origin** | Drone flat on the origin mark, static capture | `origin.json` |
| **X axis** | Drone on the mark 1 m along +X | `x_axis.json` |
| **Floor** | Any number of points on the floor (sets Z = 0) | `xy_plane_<n>.json` |
| **Volume** | Walk the drone through the whole flight volume; every recording is appended | `sweep.json` |
| **Check** | Optional verification points, never given to the solver | `verify_<n>.json` |

Samples appear in the 3D view in real time. Each session is a folder `calib_runs/run_<date>/`. The first
measurement creates it.

**Solving.** Once there are an origin, an X axis, at least one floor point and at least 100 volume
snapshots, the session is solved in the background. It is solved again whenever a sample changes.
**Geometry result** grades the solution:

- residual
- board fit: how well the 4 triangulated sensors match the real board
- 1 m mark
- baseline vs the tape measurement
- station tilt vs the station's own accelerometer

**Apply** moves the solved poses into the editor. **Export header** writes
`<run>/bs_poses_cal_candidate.h`.

**Simulation.** This replays a recorded real run as a virtual drone, so you can rehearse the
whole procedure without hardware. It also shows the effect of a shorter sweep or fewer floor
points. Unused recorded floor points replay as independent check points, and their height must read 0.

**Evaluate quality.** This solves the session again 12 times, each time on a random 70 % of the
sweep. The spread of the station poses is the real repeatability. The error on the held-out 30 %
is drawn as a coloured map in 3D.

**Sample management.** Details and delete per sample (deleted files go to `_deleted/`), Clear all,
and Import / Export of a whole session as one JSON file.

**Settings & previous runs.** This holds:
- X-mark distance, tape BS4–BS10, board height, board-check limit and static capture time
- new session, or reopen a previous run
- the **Factory calibration (OOTX)** card

### Factory calibration (OOTX) card

- **Top list**: the saved factory data of every station, with uid, polynomials, accelerometer tilt
  and per-plane phase / tilt / gibbous.
- **From the drone (wireless)**: what the drone is decoding right now, per station. It shows either
  `decoding… x% of the frame collected, y% turns without sweeps` or `✔ uid … — same as saved`.
  Stations already saved show as *saved* while the drone checks them again.
- **Save decoded stations** is enabled only when a station is new or its uid changed, which means a
  replaced station.
- **Over USB instead**: the older path, which reads the `O,` lines from the Pico's USB port. It is
  only needed for firmware that predates on-board decoding.

### Automatic mission (GUIDED)

The **Mission** tab flies a list of points, either **relative to where the drone is** (dX, dY, dZ:
the figure runs from wherever the drone is when it starts) or in **room coordinates**. Type them,
add the drone's position, or create a figure of a chosen size that starts and ends where the drone
is. Straight-line figures stop at every corner: forward and back, square, triangle, cross, 5-point
star, zig-zag over a square, vertical square, cube. Curves are flown without stopping: circle,
figure of eight, flat spiral, rising helix (with turns and rise per turn).
**Fly through the points without stopping** (ticked automatically for curves) makes every point but
the last a pass-through point (go on once within 15 cm, no hold).
The path is drawn in 3D (a relative figure around the drone's current position) with the covered
area on the floor.

**Start automatically when the radio switches to GUIDED**: tick it and the bridge holds the figure.
Every time the radio switches *into* GUIDED (armed) it flies it from that point. No need to touch
the PC: fly anywhere, flip to GUIDED, flip back to Loiter / PosHold to stop. It never fires because
the app connected while the drone was already in GUIDED, only on a switch.

1. Take off by hand and hover; switch to **GUIDED** on the radio.
2. When every check is green, press **Start**. The checks are: telemetry, armed, GUIDED, a fresh
   lighthouse position, the EKF agreeing with the lighthouse within 15 cm, and every point
   inside the covered area at 0.4–2 m height.
3. The bridge sends one `SET_POSITION_TARGET_LOCAL_NED` at a time. It converts room coordinates
   to the EKF frame with the same turn the firmware applies to ODOMETRY, and holds the current yaw
   with `DO_CHANGE_SPEED` set as configured. A point counts as reached the first time the lighthouse
   position comes within the arrival radius. The drone then stays there for the **hold time**
   (0–600 s, counted from the arrival, whatever small excursions it makes), and the next point is sent.
4. At the end the drone hovers at the last point (still GUIDED): take over on the radio.

**The pilot stays in charge.** Targets are sent only while the drone is armed in GUIDED. Any other
mode on the radio (Loiter, PosHold) ends the mission at once and the app sends nothing more.
**Stop** sends "hover here". A point not reached in time, or a lighthouse / EKF problem lasting
over a second, also stops the mission with "hover here".

**Limits** (Mission tab): points must lie inside the rectangle between the calibrated stations,
shrunk by a margin (default 30 cm), at a height between the minimum and maximum height (default
0.4–2.0 m). The height is the Z of the ODOMETRY panel, where 0 is the calibration floor. With
relative points, a point's height is the drone's Z when it starts plus dZ. All three values can be
changed in the tab: the bridge allows 0.15–2.5 m for the minimum, 0.3–3 m for the maximum and
0–1 m for the margin. A refused point says which limit it breaks.

### Track: flight log and hover accuracy

The **Track** tab records the flight and measures how well the drone holds a point.
**Record** saves one row per lighthouse position (10 Hz) with:

- the EKF position (`LOCAL_POSITION_NED` turned into room coordinates);
- roll / pitch, battery voltage, flight mode and armed state;
- the mission state, the current point and its target;
- whether the drone is in the hold at that point.

The file is rewritten every 5 s while recording, so a crash loses little.

Typical hover study:

1. Mission tab: one point (room coordinates or relative), **Hold at each point** e.g. 120 s.
2. Track tab: **Record**.
3. Take off and switch to GUIDED (or press Start). The drone flies to the point and holds it.
4. When the mission says *complete*, **Stop** the recording. The newest track is loaded.

**Study** chooses the samples and what they are compared with:

- *Samples*: the mission hold (default), the hold plus the hovering after it, or the whole
  recording; all of these can be cut to a time window.
- *Target*: the mission point(s), a fixed point, or the median position. The median shows the
  spread only, with no target.

The 3D view shows the path (grey) and the studied samples, coloured by horizontal distance:
green ≤ 5 cm, yellow ≤ 10, orange ≤ 20, red beyond. Rings of 5 / 10 / 20 cm surround the target.

**📊 Accuracy report** opens the charts:

- position over time for the whole recording, with the studied part shaded;
- the samples seen from above around the target, with CEP50 / R95 circles and the mean offset;
- a histogram of the horizontal distance with its cumulative curve;
- distance over time;
- error per axis over time, plus |EKF − lighthouse|;
- roll / pitch and battery over time.

It also lists the numbers:

- **CEP50 / R95**: radius holding 50 % / 95 % of the samples.
- Horizontal RMS, mean and max.
- Share of samples within 5 / 10 / 20 cm.
- **Bias**: the mean offset, i.e. where the drone sat on average.
- **Spread**: σ about that mean, i.e. how much it moved around it.
- Vertical RMS / p95 / max and 3D RMS.
- **Drift**: the trend of the offset, in cm/min.
- EKF − lighthouse RMS.
- Mean roll / pitch: a steady tilt explains a slow drift.
- Battery at the start and at the end.

Point at a line of the numbers to see what it means. Two more numbers help to find the cause of an error:

- **Spread forward-back / sideways**: the spread split along the drone's own axes. Forward much larger
  than sideways means a pitch-axis problem (balance, tuning), not a lighthouse one.
- **Pitch / roll movement**: how much the drone tilts while it works.

**Study → a path** measures how well the drone follows a figure flown as a mission (square, circle,
helix…). Each sample is compared with the straight line of its leg, from the previous point (the start
position for the first leg) to the point being flown to. By default the holds at the points are left
out; tick *include the holds* to count them. The report shows:

- the planned lines and the flown samples seen from above, coloured by distance;
- a histogram of the distance from the line;
- the path error over time, leg by leg;
- the height along the path, planned against flown;
- a table per leg: length, time, speed, error RMS and max;
- the numbers: path error RMS / median / p95 / max, share within 5 / 10 / 20 cm, height error,
  planned against flown length, mean speed, EKF − lighthouse, attitude and battery.

A track with a figure opens on the path study, a track with a one-point hold on the point study.

**Compare sessions**: tick 2–8 tracks and press **Compare**. The same study (point or path, with the
settings above) runs on each track. For a point, each track uses its whole hold; for a path, its last
figure. The oldest track is the baseline. The comparison shows:

- one curve per session of the share of samples within a distance (higher and further left = better);
- CEP50 / R95 (or path RMS / p95) bars;
- a table of every number with its change against the baseline: green better, red worse, plain within 5 %;
- a small top view of each session at the same scale.

**Save PNG** writes the whole report (or comparison) as one image, for sharing. **CSV** exports the raw
rows, or the comparison table.
The positions are the lighthouse's own: they measure how well the drone holds the point *as the
lighthouse sees it*, not errors of the lighthouse system itself.

### Where the app keeps its files

| What | Where |
|---|---|
| Calibration sessions | `Documents\LH2\calib_runs` (the reference run is copied there on the first start) |
| Flight tracks (Track tab) | `Documents\LH2\tracks\track_<date>_<time>[_name].json` |
| Saved factory calibration | `Documents\LH2\lh2_factory_calibration.json` + `lh2_factory_cal.h` |
| Backend log | `%APPDATA%\LightHouse Studio\logs\backend.log` (Help → Backend log) |
| Firmware UF2s / headers shipped with the app | File → Open firmware folder |

The app starts its backend on a free port. On Windows it also opens UDP 14550 exclusively: if
another GCS or `avvia.bat` is already listening, the Calib tab says so. Without this, half of the
packets would be lost silently.

---

## Calibration: what to do and when

Two different calibrations are involved:

| | **Factory calibration (OOTX)** | **Geometry** |
|---|---|---|
| What | Each station's optical imperfections (phase, tilt, gibbous per light plane), measured by Valve at the factory | Where each station is and how it is oriented in the room |
| Source | Broadcast by the station itself, 1 bit per rotor turn | Measured with the drone (wizard) |
| How often | **Once per station.** Redo only when a station is replaced | **Every time a station is moved** |
| Where it ends up | `rp2350_firmware/src/lh2_factory_cal.h` | `rp2350_firmware/src/bs_poses_cal.h` |

### 1. Factory calibration — once per station

1. Turn on the stations and the drone and connect in the **Drone** tab.
2. Open **Calib → Settings & previous runs → Factory calibration (OOTX)**. The drone decodes every
   station it sees, in about a minute per station. With many lost sweeps it takes longer, but it
   still completes.
3. Press **Save decoded stations**.
4. Copy the saved `lh2_factory_cal.h` into `rp2350_firmware/src/`, rebuild the firmware and flash it.
   From then on the correction is active from power-up.

The drone decodes the OOTX again at every power-up and applies what it receives within about a minute.
So a forgotten rebuild only costs that first minute of correction, and a replaced station is
detected and flagged in the app.

### 2. Geometry — whenever stations move

1. Mark the origin and a point 1 m along +X on the floor. Tape-measure the BS4–BS10 distance and
   enter it in Settings.
2. In **Calib**, run Origin → X axis → Floor (2–4 points spread around the room) → Volume (walk slowly
   through the whole flight volume, at several heights) → optionally Check points.
3. Read **Geometry result**: residual and board fit should be a few mm, the 1 m mark about 1.00 m
   and the baseline within about 3 % of the tape.
4. Run **Evaluate quality** if in doubt.
5. **Export header**, copy `bs_poses_cal_candidate.h` over `rp2350_firmware/src/bs_poses_cal.h`,
   rebuild both UF2s and flash.

> Never flash poses scaled to an arbitrary unit (the old `REFERENCE_DIST = 1.0` flow): see
> `.claude/rules/scale_calibration_bug.md`. A sensor under BS10 must read the real separation, not 1.0 m.

The same steps are available from the command line (`capture_lh2.py`, `calibrate_bitcraze.py`,
`evaluate_calibration.py`, `calibrate_export.py`, `lh2_ootx.py`). See
[utils/calibration/README.md](utils/calibration/README.md).

---

## Firmware

### Pipeline

| Core | Stage | Source |
|---|---|---|
| 1 | PIO + DMA capture of the 4 TS4231, LFSR decoding of each sweep | `lh2/` |
| 0 | LFSR position → sweep angles with the period model (`A = 360·8/period`, `B0 = −120°`, `B1 = −240°`) | `angle_decoder/`, `main.c` |
| 0 | Factory correction per station and plane (phase, tilt, gibbous) | `lh2_factory_cal.h`, updated at run time by OOTX |
| 0 | On-board OOTX decoding (below) | `ootx/` |
| 0 | Triangulation of each sensor from ≥ 2 stations, then the drone centre | `solve3d/` |
| 0 | MAVLink: `ODOMETRY`, `TUNNEL` streams, EKF status, timesync | `mavlink/` |

**On-board OOTX.** Each sweep votes one bit for its station's current rotor turn: the parity of the
polynomial. Sweeps reach core 0 out of order, so turns stay open for a few turns before they are
voted. A station repeats its 43-byte frame back to back every 443 turns. Each turn is therefore voted
into slot *turn mod 443* of a ring, and turns without any sweep fill in on later repetitions. The
frame is read once every bit is known and the CRC32 matches. In simulation it decodes even with
85 % of turns missing. A newly decoded frame is applied immediately; on USB the drone prints
`OOTX,<station>,uid=…,applied`.

### Outputs

| Channel | Message | Content |
|---|---|---|
| UART → Pixhawk | `ODOMETRY` (#331), 10 Hz | Position, `frame_id = MAV_FRAME_LOCAL_FRD` (20), `child_frame_id = MAV_FRAME_BODY_FRD` (12); ArduPilot discards any other frame. Z is sent down-positive. Timestamp in the FC's timebase. |
| UART → Pixhawk | `TUNNEL` type `0x4C48`, payload `"LH"` v3 | Raw angle snapshot for calibration: int16 angles (1/16384 rad), ages, 16-bit valid mask, index = sensor × 4 + station |
| UART → Pixhawk | `TUNNEL` type `0x4C48`, payload `"LO"` v2 | OOTX state of one station every 0.5 s (round robin): valid flag, frames OK, bits, missed turns, coverage %, the 43-byte frame |
| USB serial (115200) | `A,` / `C,` / `STAT,` / `OOTX,` lines | Angles, position, decoded pairs per second per station, OOTX applied. Send `o` to stream the raw `O,` lines and `x` to stop them. |

### Build

The build directory `rp2350_firmware/src/build` is already configured, so there is no need to re-run CMake:

```bash
make -C rp2350_firmware/src/build crossing_beams crossing_beams_synthetic -j$(nproc)
```

- `crossing_beams.uf2`: real hardware.
- `crossing_beams_synthetic.uf2`: no sensors. It produces LFSR counts for every calibrated
  station along a 1 × 1 m square at 2 m height.

Both UF2s are committed: rebuild and commit them with every source change. Generated headers
(`bs_poses_cal.h`, `lh2_factory_cal.h`) are never edited by hand. For a build from zero (SDK,
CMake, VS Code extension), see [rp2350_firmware/src/utils/docs/BUILD.md](rp2350_firmware/src/utils/docs/BUILD.md).

---

## Backend (bridge) and other tools

**The bridge** `utils/user_interface/display_real_time_windows.py` is what LightHouse Studio runs
inside:

- It listens for the drone's MAVLink on UDP 14550 and sends GCS heartbeats to the radio on port 14555,
  which is how the radio learns where to send.
- It serves the UI on `http://127.0.0.1:8051`, with `/api/odometry` (position + the flight controller's HUD
  telemetry), `/api/fc/param` (read / write the whitelisted ArduPilot parameters), `/api/fc/mission/*`
  (GUIDED missions), `/api/track/*` (flight log: start, stop, list, get, delete) and `/api/lh2/*` for
  capture, solve, simulation, evaluation, export and OOTX. It asks the flight controller for the
  POSITION, EXTRA3, EXTENDED_STATUS, EXTRA1 and EXTRA2 streams.
- Data folders can be moved with `LH2_RUNS_DIR` (sessions) and `LH2_DATA_DIR` (factory calibration, flight tracks).

Without the desktop app (development, Linux):

```bash
source .venv/bin/activate
python utils/user_interface/display_real_time_windows.py      # then open bs_pose_editor.html
```

**Desktop app development.** `cd ELECTRON_GUI && npm install && npm start`. To build the Windows
zip, run `npm run dist:win-zip`. Details are in [ELECTRON_GUI/README.md](ELECTRON_GUI/README.md).

**Browser bundle.** `python utils/make_windows_bundle.py` regenerates `lh2_windows_bundle/`
(`avvia.bat` starts the bridge and opens the editor in the browser). It is the older path and
LightHouse Studio replaces it.

**Delay plot.** `utils/analysis/plot_ext_nav_delay.py` (Windows: `plot_delay.bat`, double-click or drop a
`.bin` on it) measures how late the lighthouse position reaches the flight controller from a
back-and-forth test log: IMU velocity vs lighthouse velocity, the shift that matches them best, and the
part `VISO_DELAY_MS` must cover. It writes a 4-panel PNG next to the log. `utils/analysis/spiega_delay.py`
(`spiega_delay.bat`) shows the same calculation step by step for a non-specialist: raw data, conversion
to velocities, the reversals compared, the curve shifted by several amounts, the similarity peak and the
result.

**GCS and logs.** `docs/index.html` is a browser GCS (EKF flags, attitude, 2D map). It needs
`utils/user_interface/display_real_time.py`. `docs/logview.html` replays ArduPilot DataFlash
`.bin` logs and MAVLink logs in 3D.

---

## Hardware

- 1× Raspberry Pi Pico 2 (RP2350).
- 4× TS4231 Lighthouse v2 sensors on a rigid board, 4 × 4 cm square
  ([TS4231 breakout](https://github.com/DotBots/TS4231-breakout-board)). The layout used by the
  solver is `utils/calibration/wand_sensors.json` (S0 upper-left, S1 upper-right, S2 lower-right,
  S3 lower-left, component side up).
- Pixhawk running ArduPilot: Pico UART to **TELEM2**.
  ExtNav parameters measured on this system: `VISO_TYPE = 1`, `EK3_SRC1_POSXY = 6`, `EK3_SRC1_POSZ = 6`,
  `EK3_SRC1_YAW = 6` (yaw from the four sensors, no compass), `VISO_DELAY_MS = 60`, `VISO_POS_M_NSE = 0.1`,
  `VISO_YAW_M_NSE = 0.35`.
- Wi-Fi telemetry radio (MavESP8266 or DroneBridge ESP32) for the ground station.

Sensor wiring, from the original DotBots design:

<p align="center">
  <img src="static/schematic.png" alt="schematic of how to connect the RP2040 to the 4 TS4231 sensors" width="65%" />
</p>

How the capture works: four PIO state machines sample each TS4231 data line at 32 MS/s. DMA moves
each capture to RAM, and a PIO interrupt tells the CPU to decode it. A full sweep's 8 pulses decode
in under 3 ms ([benchmark](https://github.com/DotBots/lh2_on_rp2040/issues/2)).

<p align="center">
  <img src="static/algorithm_explanation.png" alt="diagram of how the capture works" width="800" />
</p>

---

## Credits and references

Made by **Davide Lentini** and **Eduardo Gonzalez**.

- LH2 capture and LFSR decoding: [DotBots/lh2_on_rp2040](https://github.com/DotBots/lh2_on_rp2040).
- Geometry calibration (sample matching, IPPE, least-squares solve) and the OOTX frame format:
  Bitcraze [crazyflie-lib-python](https://github.com/bitcraze/crazyflie-lib-python) and
  [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware).
- ArduPilot external navigation via MAVLink `ODOMETRY`.
