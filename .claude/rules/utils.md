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
3. Update `utils/lua_scripts/lh2_bs_params.lua` with the new origin and R values from `lab.yaml`. This is now the single source of truth — no firmware header to export.
4. Update `CAL_BS*` constants in `rp2350_firmware/src/main.c` from `tools/history_calibration.txt`
5. Deploy the updated Lua script to `APM/scripts/` on the FC SD card and reboot the FC.
6. Rebuild both firmware targets if `main.c` changed, and commit UF2s.

`lab.yaml` is the live calibration file for the current lab setup. `test_output.yaml` is a scratch file.

> `calibrate_export.py` (generates `bs_poses_cal.h`) is **no longer used** — the header has been removed from the firmware.

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
over WiFi via a DroneBridge ESP32 radio instead of USB serial.

---

## Lua scripts (`utils/lua_scripts/`)

`lh2_bs_params.lua` — **Single source of truth for room geometry.** Registers 25 `LH2_BS*` MAVLink parameters on the FC (visible in Mission Planner) and pushes them to the Pico every second via `gcs:send_named_float()`. Update parameter values via the GCS UI or Mission Planner, then deploy this script to `APM/scripts/` on the FC SD card.

**Critical:** the `update()` loop reads parameters with `param:get('LH2_BS0_X')` etc., not hardcoded literals — so whatever the FC has stored in flash is what gets sent to the Pico. Changes made via the UI take effect within ~1–2 seconds without rebooting the Pico.

Parameter naming: `LH2_BS{i}_X/Y/Z` for origin, `LH2_BS{i}_R{r}{c}` for rotation matrix (local→world, row-major). R convention matches `lh2_bs_pose_t` in `solve3d.h`.

Deploy:
```bash
# Copy to FC SD card
cp utils/lua_scripts/lh2_bs_params.lua /path/to/SD/APM/scripts/
# Reboot FC — script starts automatically
```

`set_home_on_ekf.lua` — ArduPilot scripting: sets home automatically when EKF becomes healthy. Load via Mission Planner scripting interface.

---

## Post-flight analysis (`utils/user_interface/`)

`display_post_flight.py` — replays a recorded flight log.  
`compute_3d_coordinates.py` — offline triangulation from logged angle data.
