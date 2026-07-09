# Firmware Rules (`rp2350_firmware/src/`)

## Build

```bash
# Both targets
make -C rp2350_firmware/src/build crossing_beams crossing_beams_synthetic -j$(nproc)
```

The build directory is pre-configured — never re-run `cmake`. Uses `make`, not `ninja`.

After any source change, rebuild and commit the affected UF2(s) alongside the source. Both UF2s are tracked in git.

## Source layout

```
rp2350_firmware/src/
  main.c               — dual-core entry point (real + synthetic via #ifdef)
  lh2/                 — PIO+DMA TS4231 capture + LFSR decoder (core 1 only)
  angle_decoder/       — LFSR counts → EMA-filtered (horiz, vert) angles
  solve3d/             — triangulation (DLT)
  cv/                  — projection matrix helpers used by solve3d
  mavlink/             — hand-rolled MAVLink v1+v2: ODOMETRY TX, EKF/TIMESYNC/NAMED_FLOAT RX
  build/               — artifacts (UF2s committed here)
```

`bs_poses_cal.h` has been **removed**. BS geometry is now fetched at runtime from the FC via MAVLink NAMED_VALUE_FLOAT (see "BS pose boot sequence" below).

## Dual-core constraint

All `db_lh2_init()` calls must run on **core 1** — they arm PIO IRQs on the executing core. Never call them from core 0 / `main()`.

`g_capture_ready` gates core 0's loop; `g_home_set` signals EKF health to core 1 (synthetic restarts square walk on rising edge).

## MAVLink ODOMETRY (msg #331)

- **frame_id = 20** (`MAV_FRAME_LOCAL_FRD`), **child_frame_id = 12** (`MAV_FRAME_BODY_FRD`). ArduPilot's `handle_odometry()` silently discards anything else — LOCAL_NED (1/8) does **not** work.
- World frame is z-up; MAVLink z is down → negate z on send: `mavlink_send_odometry(..., -last_cz)`.
- Odometry is sent at **10 Hz** gated by `PRINT_INTERVAL_US = 100000ULL`.
- Timestamp is corrected to FC timebase via `mavlink_timesync_corrected_us()`.

## TIMESYNC

We do **not** send outgoing TIMESYNC requests. The offset EMA is seeded from the FC's own broadcasts (incoming `tc1=0` frames). See `_dispatch_timesync()` in `mavlink/mavlink.c`.

## EKF health

`EKF_NEED_FLAGS = 0x0001 | 0x0008` (ATTITUDE | POS_HORIZ_REL). `g_home_set` latches true once healthy; `DO_SET_HOME` is sent exactly once.

## BS pose boot sequence (both real and synthetic)

On boot, core 0 blocks until all 25 BS pose values arrive via MAVLink **NAMED_VALUE_FLOAT** (msg #251) pushed by `utils/lua_scripts/lh2_bs_params.lua` running on the FC. The FC must have the Lua script deployed to `APM/scripts/` on its SD card.

Values received (25 total):
- `NUMBS` — number of active base stations
- `BS{i}X/Y/Z` — world-frame origin of BS i [m]
- `BS{i}R{r}{c}` — rotation matrix entries (local→world), row r, col c

After boot, `BS_POSES` is refreshed from the incoming stream **every 100 ms** (10 Hz). Changes written to FC params via the GCS UI take effect within ~1–2 seconds — no Pico reboot needed.

### NAMED_VALUE_FLOAT protocol details

- Lua calls `gcs:send_named_float(name, value)` 25 times/second; the FC reads current param values with `param:get()` so UI-changed poses are immediately reflected.
- ArduPilot sends these as **MAVLink v2** (STX=0xFD). The firmware parser handles both v1 (STX=0xFE) and v2.
- **MAVLink v2 truncates trailing zero bytes** from the payload. A name like `"NUMBS"` (5 chars in a char[10] field) results in `LEN=13` instead of 18. The parser handles this: it accepts any frame with `LEN ≥ 9` and zero-fills missing name bytes.
- CRC_EXTRA for NAMED_VALUE_FLOAT = **170** (0xAA).

### Debug diagnostics (boot wait loop)

```
lh2=<n>/25  named_seen=<m>  rx_bytes=<b>
```

| Counter | Meaning |
|---------|---------|
| `lh2` | How many of the 25 pose values received so far |
| `named_seen` | NAMED_VALUE_FLOAT frames that passed CRC (if 0 with bytes growing → v2 truncation bug) |
| `rx_bytes` | Raw bytes received; ~700 bytes/s = 25 frames × 28 bytes expected |

### Heartbeat requirement

The Pico sends a MAVLink v2 HEARTBEAT (`MAV_TYPE_ONBOARD_CONTROLLER`=18) every 1 s during the boot wait. ArduPilot requires a heartbeat from a connected device before broadcasting to it; without it `gcs:send_named_float()` may not reach the TELEM2 port.

## Calibration constants in main.c

`CAL_BS*_A0/B0/A1/B1` are linear calibration constants from `utils/user_interface/tools/history_calibration.txt`. Update them only after a new calibration run.

## Synthetic mode

`crossing_beams_synthetic` builds `main.c` with `-DSYNTHETIC_CAPTURE`. Core 1 fabricates LFSR counts for a 1×1 m square at z=2 m. Sensor offsets are a 5×5 cm square (`SYN_SENSOR_OFF`). No lh2/ layer is compiled in.

Synthetic mode uses the **same BS pose boot sequence** as real hardware — it blocks at startup until the Lua script delivers poses from the FC. This ensures the geometry used for fabricating angles and for solving is always consistent with the FC's stored params.
