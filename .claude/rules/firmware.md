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
- World frame is right-handed z-up (X → 1 m mark, Y left); `LOCAL_FRD` is X forward, Y right, Z down → negate **y and z** on send: `mavlink_send_odometry(..., last_cx, -last_cy, -last_cz)`. Negating z alone mirrors the lateral axis (position hold drifts away sideways). The bridge negates both back for the UI. With compass yaw ArduPilot treats these positions as north/east, so `main.c` then turns them by `BS_WORLD_HEADING_DEG` (compass heading of world +X, written into `bs_poses_cal.h` by `calibrate_export.py --heading-deg`; missing = 0). A wrong heading makes position hold circle the spot (toilet bowl). The bridge reads the same define and undoes it.
- ODOMETRY is sent at **30 Hz** (`ODOM_INTERVAL_US`), outside the 10 Hz block (`PRINT_INTERVAL_US`: TUNNEL, OOTX, USB prints), and only with a position newer than the last one sent whose stamp is ≥ `ODOM_MIN_GAP_US` (25 ms) later — EKF3 drops ExtNav samples stamped < 20 ms apart (`extNavIntervalMin_ms`, so ≤ 50 Hz; yaw from the quaternion is limited to 20 Hz by `sensorIntervalMin_ms`).
- UART to the FC at **921600 baud** (`MAV_BAUD`; set `SERIALx_BAUD = 921` on the Pico port). RX is interrupt-driven into a 1 KB ring (`mavlink_rx_update()` parses it), so bytes are not lost while the loop is busy; `STAT` reports `rx_lost` (should stay 0). At 115200 one 10 Hz burst (ODOMETRY 245 B + TUNNEL 145 B) took ~34 ms and blocked core 0; at 921600 ~4 ms.
- **Yaw from the four sensors** (branch `feat/extnav-yaw`): `_board_yaw()` in `main.c` fits the board layout `BOARD_XY` (wand_sensors.json, Y flipped: seen from above, component side up) to the triangulated sensors (2D least squares, ≥ 3 sensors) and adds `BOARD_NOSE_DEG` (where the nose points on the board; 180 = from S1 toward S0, from the 2026-10-21 flight log). ODOMETRY carries it as a yaw-only quaternion (clockwise about down, + the +X heading, like the position) with its variance in `pose_covariance[15/18/20]` (0.12 rad² ≈ 20° with 4 sensors — measured flight noise on the 4 cm board; 2.0 when older than 0.5 s). ArduPilot uses it with `EK3_SRC1_YAW = 6` (ExtNav) — no compass indoors. USB prints `YAW,<deg world CCW from +X>,<fresh>` at 10 Hz; the bridge/UI show it (0 = nose toward the 1 m mark). ODOMETRY is not sent before the first yaw.
- **Angle filter**: the solver's angles (`ema_horiz/ema_vert`) come from an alpha-beta filter (angle + rate, `AB_ALPHA` 0.30 / `AB_BETA` 0.05) run on the sweeps' capture times; a pair farther than `ANGLE_GATE_RAD` (1.5°) from the *prediction* is left out (`ANGLE_GATE_N` = 3 in a row restart it), late pairs (out of order) are skipped. The solver (`solve3d_calib_run`) uses only rays captured ≤ `SOLVE3D_RAY_MAX_AGE_US` (100 ms) ago and carries each one to the solve time with the filter's rate, so all rays describe the same moment and ODOMETRY `time_usec` is that solve time (log 2026-10-26: with rays up to `FRESHNESS_US` = 500 ms old, 23 % of the rays used were > 100 ms old, the old stamp — the mean capture time of all fresh rays — jumped, ODOMETRY came out at ~15 Hz instead of 30 and the measured Lc rose to 86 ms). ODOMETRY goes out only when new sweeps were decoded since the last one (`angle_decoder_pairs`). Simulated vs the old EMA 0.2 + 3° gate: lag 120–140 → ~30 ms. Synthetic capture now stamps its sweeps. Measured on the drone (back-and-forth test, IMU vs lighthouse velocity, 2026-10-22): total lag motion → arrival at the FC 357 → ~135 ms, frozen positions 16–28 % → < 1 %.
- **ArduPilot ExtNav parameters (measured 2026-10-22)**: `VISO_DELAY_MS = 60` (the EKF then tracks the real drone within ~5 ms; 80 overshoots by ~30 ms and doubles the innovations — ArduPilot does use the capture-time stamp), `VISO_POS_M_NSE = 0.1` (0.5 let the EKF drift 16 cm rms from the lighthouse position in PosHold; 0.1 → 3.7 cm), `VISO_YAW_M_NSE = 0.35`, `EK3_SRC1_YAW = 6`.
- **Outlier filtering** (flight log 2026-10-21: one wrong ray in ~13 % of snapshots, isolated ODOMETRY spikes up to metres): `angle_decoder.c` leaves a pair farther than `ANGLE_GATE_RAD` (3°) from its EMA out of the EMA (`ANGLE_GATE_N` = 3 in a row = real motion, restart) and restarts the EMA after `EMA_RESET_US` (100 ms) without data. `main.c` drops sensors farther than `BOARD_MAX_SPREAD_M` (8 cm) from the median of the others (`_board_check`) and believes a centre jump > `JUMP_GATE_M` (30 cm) only after `JUMP_PERSIST_US` (200 ms). Replayed on that flight: spikes > 10 cm 80 → 5, sensor yaw noise 25° → 13°.
- ODOMETRY `time_usec` is in the **Pico's own clock** (not TIMESYNC-corrected): ArduPilot's `JitterCorrection` learns the offset (max lag 500 ms, re-converges every 100 samples). A TIMESYNC lock mid-flight would step the stamps back ~6 s (Pico boots before the FC) and ArduPilot would treat ~100 samples as 500 ms old. TUNNEL still uses `mavlink_timesync_corrected_us()`. Before 2026-10-25 the UART RX was polled with a 32-byte FIFO and TIMESYNC never locked in flight (log 2026-10-23: steady +6 s offset, lag 55–70 ms) — RX is now interrupt-driven.

## TIMESYNC

We do **not** send outgoing TIMESYNC requests. The offset EMA is seeded from the FC's own broadcasts (incoming `tc1=0` frames). See `_dispatch_timesync()` in `mavlink/mavlink.c`.

## EKF health

`EKF_NEED_FLAGS = 0x0001 | 0x0008` (ATTITUDE | POS_HORIZ_REL). `g_home_set` latches true once healthy; `DO_SET_HOME` is sent exactly once.

## Angle conversion (main.c + angle_decoder.c)

`CAL_BS*_A0/B0/A1/B1` follow the period model: `A = 360 * 8 / period` (period from DotBots `LH_PERIODS`, mode 5 = 947000 for BS0, mode 11 = 919000 for BS1), `B0 = -120`, `B1 = -240`. They are physics, not fitted — do not replace them with fitted values (the old `history_calibration.txt` fits distorted large angles; see `utils/calibration/calib_runs/RESULTS.md`).

Each station's imperfections are removed afterwards with its factory calibration (phase, tilt, gibbous per plane) from `lh2_factory_cal.h`, which is GENERATED by `utils/calibration/lh2_ootx.py --header` — never hand-edit it; regenerate it when a base station is replaced. The synthetic build uses zero factory calibration.

The header is only the power-up default: like the Crazyflie, the drone also decodes each station's OOTX by itself (`ootx/ootx.c`, fed by `angle_decoder.c` with every sweep: bit = polynomial parity, turn start = capture time − lfsr/6 MHz, turns voted in a small window because sweeps reach core 0 out of order). Frames repeat back to back every 443 turns, so each turn is voted into slot (turn mod 443) of a ring: turns without sweeps (common with 4 stations) fill in on later repetitions — simulated with 85 % of turns missing it still decodes. `main.c` applies a newly decoded frame to `CAL[]` at run time (`OOTX,<name>,uid=...,applied` line) and sends every station's state every 0.5 s round-robin as a TUNNEL `"LO"` v2 message (`mavlink_send_lh2_ootx`: bs, flags bit0 valid, frames_ok, bits, missed, coverage %, payload_len, payload ≤ 43 B; the bridge also reads v1). The GCS (Calib → Factory calibration) shows it and saves it to `lh2_factory_calibration.json` + `lh2_factory_cal.h` without a USB cable.

## Base stations (up to 4)

`NUM_BS = 4` (angle_decoder.h). Index → physical station: 0 = BS4 (poly 8/9, period 947000), 1 = BS10 (poly 20/21, 919000), 2 = BS0 (poly 0/1, 959000), 3 = BS1 (poly 2/3, 957000). BS4/BS10 keep 0/1 so older calibrations and sessions stay valid.

- `bs_poses_cal.h` (generated) uses designated initializers; stations not calibrated yet stay all-zero and `bs_pose_valid()` makes the solver ignore them. `lh2_factory_cal.h` (generated) may lack a station: `main.c` then applies no factory correction for it.
- A sensor is triangulated when ≥ `MIN_BS_FOR_POINT` (2) calibrated stations see it: least-squares closest point to all their rays (`solve3d.c`); with 3+ rays one farther than `SOLVE3D_OUTLIER_M` is dropped.
- MAVLink TUNNEL is format **v3** (95 bytes): int16 angles in 1/16384 rad, u8 ages, 16-bit valid mask, index = sensor × NUM_BS + bs. `capture_lh2.py` parses v2 and v3.
- USB prints `STAT,pairs/s,BS4=..,BS10=..,BS0=..,BS1=..` every second: decoded angle pairs per station (≈ 50 per sensor per station at full rate).

## Synthetic mode

`crossing_beams_synthetic` builds `main.c` with `-DSYNTHETIC_CAPTURE`. Core 1 fabricates LFSR counts, for every calibrated station, for a 1×1 m square at z=2 m. Sensor offsets are a 5×5 cm square (`SYN_SENSOR_OFF`). No lh2/ layer is compiled in.
