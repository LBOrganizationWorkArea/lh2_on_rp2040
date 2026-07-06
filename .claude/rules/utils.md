# Utils Rules (`utils/`)

## Python environment

```bash
source .venv/bin/activate   # from repo root
```

Dependencies: `fastapi`, `uvicorn`, `pymavlink`, `pyserial`, `pyyaml`, `numpy`, `scipy`, `cflib`.

---

## Calibration (`utils/calibration/`)

### When to re-calibrate

Re-calibrate whenever base stations move. The output feeds the firmware directly.

### Workflow

1. Collect angle measurements (wand wave or Crazyflie sweep)
2. Run calibration:
   ```bash
   python calibrate_lighthouse.py measurements.json -o lab.yaml
   ```
3. Export to firmware header:
   ```bash
   python calibrate_export.py lab.yaml > ../../../rp2350_firmware/src/bs_poses_cal.h
   ```
4. Update `CAL_BS*` constants in `rp2350_firmware/src/main.c` from `tools/history_calibration.txt`
5. Rebuild both firmware targets and commit `bs_poses_cal.h` + UF2s

`lab.yaml` is the live calibration file for the current lab setup. `test_output.yaml` is a scratch file.

### Measurement format

JSON array of `{timestamp, base_station_id, angles: [[h,v]×4]}` where angles are in **radians**.

---

## GCS / real-time display (`utils/user_interface/`)

### Running

```bash
cd utils/user_interface
python display_real_time.py
# then open docs/index.html in a browser (or serve it)
```

`display_real_time.py` is a **FastAPI + WebSocket** server that:
- Reads MAVLink from a serial port (Pico USB) or a debug stub
- Streams position/attitude state to the browser frontend via WebSocket
- Exposes `/api/config`, `/api/ports`, `/api/connect` REST endpoints

The browser frontend (`docs/index.html`) connects to this server — it is **not** a standalone page.

### Debug mode

Pass `port=debug` via the API to run without hardware — the server generates synthetic motion internally.

### DroneBridge ESP32 (WiFi telemetry, no USB)

See [`utils/user_interface/README.md`](../../utils/user_interface/README.md) for how to connect
when the FC's MAVLink is relayed over WiFi via a DroneBridge ESP32 radio instead of USB serial —
covers both DroneBridge Access Point mode (fixed IP, direct WebSocket or backend UDP dial-out) and
Station/Client mode (DHCP IP you discover and enter, backend still dials out via `/ws/udp`); the
backend's passive-listen mode is only for the case where DroneBridge itself pushes UDP to the GCS.

---

## Lua scripts (`utils/lua_scripts/`)

`set_home_on_ekf.lua` — ArduPilot scripting: sets home automatically when EKF becomes healthy. Load via Mission Planner scripting interface.

---

## Post-flight analysis (`utils/user_interface/`)

`display_post_flight.py` — replays a recorded flight log.  
`compute_3d_coordinates.py` — offline triangulation from logged angle data.
