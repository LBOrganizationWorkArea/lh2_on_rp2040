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
  bs_poses_cal.h       — GENERATED: base-station poses, do not hand-edit
  lh2/                 — PIO+DMA TS4231 capture + LFSR decoder (core 1 only)
  angle_decoder/       — LFSR counts → EMA-filtered (horiz, vert) angles
  solve3d/             — triangulation (fundamental matrix)
  mavlink/             — hand-rolled MAVLink v2: ODOMETRY TX, EKF/TIMESYNC RX
  build/               — artifacts (UF2s committed here)
```

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

## Calibration constants in main.c

`CAL_BS*_A0/B0/A1/B1` are linear calibration constants from `utils/user_interface/tools/history_calibration.txt`. Update them only after a new calibration run.

## Synthetic mode

`crossing_beams_synthetic` builds `main.c` with `-DSYNTHETIC_CAPTURE`. Core 1 fabricates LFSR counts for a 1×1 m square at z=2 m. Sensor offsets are a 5×5 cm square (`SYN_SENSOR_OFF`). No lh2/ layer is compiled in.
