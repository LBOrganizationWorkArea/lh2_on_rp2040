# crossing_beams — Firmware Architecture

Indoor 3D positioning on an RP2350 (Raspberry Pi Pico 2). Four TS4231
photodiodes watch two Lighthouse-2 (LH2) base stations sweeping laser fans
across the room. From the timing of those sweeps the firmware recovers each
sensor's **metric 3D position** and streams it to a Pixhawk flight controller as
a MAVLink `VISION_POSITION_ESTIMATE`, giving GPS-denied indoor navigation.

This document describes the firmware only (the `crossing_beams` project).
For the solver derivation see `ENHANCED_ALGORITHM.md`.

---

## 1. Pipeline at a glance

```
 LH2 lasers ─► [TS4231 ×4]
                   │   raw light pulses
   ┌───────────────┼────────────────────────────────────────────────┐
   │ lh2/          PIO + DMA capture → demodulate → LFSR position     │  core 1
   │ angle_decoder LFSR counts → (horiz, vert) angles, EMA-filtered   │  core 0
   │ solve3d/      project + DLT-triangulate against calibrated poses  │  core 0
   │ (centroid)    average the 4 sensor points → one body position    │  core 0
   │ mavlink/      position → VISION_POSITION_ESTIMATE over UART0      │  core 0
   └─────────────────────────────────────────────────────────────────┘
```

Each stage is a self-contained module under `crossing_beams/`. The orchestration
lives in `main_real.c`.

---

## 2. Module map

| Directory | Responsibility | Key entry points |
|---|---|---|
| `lh2/` | TS4231 capture via PIO+DMA; demodulate sweeps; LFSR index search → base-station-relative position counts | `db_lh2_init`, `db_lh2_process_location` |
| `angle_decoder/` | LFSR counts → calibrated sweep angles → Bitcraze `(horiz, vert)`; EMA filter; freshness tracking | `angle_decoder_init`, `angle_decoder_update`, `angle_decoder_is_fresh` |
| `solve3d/` | The solver. Projects each sensor's calibrated angles (`angles_to_pixels`) and DLT-triangulates them against projection matrices built from the **calibrated base-station poses** | `solve3d_calib_run`, `angles_to_pixels` |
| `cv/` | The DLT triangulation back-end (`triangulate_points`) + its Jacobi-SVD — the OpenCV-derived, paper-validated core | `triangulate_points` |
| `mavlink/` | Minimal MAVLink v2 encoder for `VISION_POSITION_ESTIMATE` (#102) | `mavlink_init`, `mavlink_send_vpe` |
| `main_real.c` | Dual-core orchestration; calibration constants; centroid + serial output | `main`, `core1_entry` |

The solver is an **enhancement of the original** epipolar pipeline
(`data_processing.py :: solve_3d_scene`), not a rewrite: it keeps that pipeline's
projection (`angles_to_pixels`) and DLT triangulation (`triangulate_points`), and
only replaces the scale-ambiguous pose *estimation* (fundamental matrix +
`recoverPose`) with projection matrices built from the known calibrated poses.
See `ENHANCED_ALGORITHM.md`.

---

## 3. Dual-core threading model

The work splits cleanly into a **producer** (sensor I/O) and a **consumer**
(math + comms):

```
┌── Core 1 — capture (producer) ──────────┐     ┌── Core 0 — compute (consumer) ──────────┐
│ db_lh2_init() ×4  → arms all PIO IRQs    │     │ angle_decoder_update(g_lh2 → g_angles)  │
│ loop: db_lh2_process_location() ×4       │ ──► │ solve3d_calib_run(BS_POSES, g_angles)   │
│   fills g_lh2[].locations + data_ready   │     │ centroid → mavlink_send_vpe()           │
│                                          │     │ USB diagnostics (A / P / C lines)       │
└──────────────────────────────────────────┘     └─────────────────────────────────────────┘
            writes g_lh2[]      ──── shared state ────►      reads g_lh2[]
```

### Why all four sensors init on core 1

On RP2350 each core has its own interrupt controller. `db_lh2_init()` calls
`irq_set_enabled()`, which arms the PIO IRQ **on whichever core executes it**.
Putting all four `db_lh2_init()` calls inside `core1_entry()` guarantees every
TS4231 capture interrupt fires on core 1 — so all sensor I/O (IRQ handlers *and*
the `db_lh2_process_location()` demodulation) is genuinely isolated there, and
core 0 is free for the math and MAVLink.

### Startup handshake

Core 0 sets up `angle_decoder` and launches core 1, then spins on a
`volatile bool g_capture_ready` flag that core 1 sets once all sensors are
initialised. This prevents core 0 from solving against an uninitialised buffer.

### Shared state & synchronisation

- `g_lh2[NUM_SENSORS]` — written by core 1, read by core 0.
- The per-slot `data_ready` field is the handshake: core 1 writes the decoded
  `locations` then sets `data_ready = DB_LH2_PROCESSED_DATA_AVAILABLE`; core 0
  only consumes a slot in that state, then clears it to `DB_LH2_NO_NEW_DATA`.
- `g_capture_ready` gates startup.

**Known limitation:** there is no explicit memory barrier (`__dmb()`) around the
`g_lh2` handshake. In practice the `data_ready` flag serialises access, but a
barrier after the `data_ready` write would close a theoretical torn-read window.

---

## 4. Core data structures

```c
// lh2/lh2.h — one per sensor, shared across cores
typedef struct {
    db_lh2_raw_data_t  raw_data  [SWEEP][BS];
    db_lh2_location_t  locations [SWEEP][BS];   // LFSR position + polynomial  ← consumed
    absolute_time_t    timestamps[SWEEP][BS];
    db_lh2_data_ready_state_t data_ready[SWEEP][BS];  // the cross-core handshake
    uint8_t            sensor;
} db_lh2_t;

// angle_decoder/angle_decoder.h — one per (sensor × base station)
typedef struct {
    float    raw_sweep[2];
    bool     has_sweep[2];
    float    ema_az, ema_el;        // legacy az/el [deg]   (kept; unused by solver)
    float    ema_horiz, ema_vert;   // Bitcraze angles [rad]  ← used by the solver
    bool     valid;
    uint64_t last_update_us;        // for the freshness check
} lh2_angles_t;

// solve3d/solve3d.h
typedef struct { float origin[3]; float R[3][3]; } lh2_bs_pose_t;  // BS pose, world frame
typedef struct { float xyz[3]; uint8_t sensor_id; } lh2_point3d_t; // solved point
```

Global instances in `main_real.c`:
```c
static db_lh2_t      g_lh2[NUM_SENSORS];              // NUM_SENSORS = 4
static lh2_angles_t  g_angles[NUM_SENSORS][NUM_BS];   // NUM_BS = 2
static volatile bool g_capture_ready;
// BS_POSES[NUM_BS] comes from the generated bs_poses_cal.h
```

---

## 5. The math (summary — full derivation in ENHANCED_ALGORITHM.md)

1. **Sweep angles → (horiz, vert).** `angle_decoder` turns each LFSR count into a
   calibrated sweep angle (`A·lfsr + B`) and combines the two sweeps with the
   Bitcraze model; EMA-filtered into `ema_horiz` / `ema_vert`.

2. **Project to image pixels.** `angles_to_pixels` gives the z=1 pinhole
   projection `px = (tan horiz, tan vert)` for each base station.

3. **Build calibrated projection matrices.** For each base station, a 3×4
   world→image matrix is built from its calibrated pose `(origin, R)`, with the
   boresight (`R` column 0, local +X) as the optical/depth axis.

4. **DLT-triangulate.** `triangulate_points(P0, P1, px0, px1)` returns the world
   point where the two base stations' rays meet — **metric** (the scale is baked
   into the calibrated poses) and **stable** (no per-frame pose estimation).

5. **World → NED.** The solver outputs the calibration world frame (z-up).
   MAVLink/ArduPilot expect NED (z-down), so `main_real.c` sends `(cx, cy, -cz)`.

---

## 6. Calibration inputs

Two independent calibrations feed the firmware as constants/headers:

| What | Where | Produced by |
|---|---|---|
| **Angle calibration** (`lfsr → degrees`, A/B per sweep per BS) | `CAL[]` in `main_real.c` | the angle-calibration workflow (`history_calibration.txt`) |
| **Base-station poses** (position + rotation, metric, world-aligned) | `bs_poses_cal.h` (`BS_POSES[]`) | `utils/calibration/calibrate_export.py` from the lighthouse geometry pipeline |

`calibrate_export.py` does the quaternion → rotation-matrix conversion in Python
(scipy), so the firmware never parses quaternions — it just `#include`s a header
of `origin[]` + `R[][]`. `--synthetic` mode emits a known geometry for testing;
the real mode consumes the calibration YAML.

---

## 7. Build targets

Both targets build from the **same** `main_real.c`; only core 1's data source
differs.

| Target | Define | Core 1 source | Needs hardware? |
|---|---|---|---|
| `crossing_beams` | — | real PIO/DMA capture from 4 TS4231 | yes (sensors + base stations) |
| `crossing_beams_synthetic` | `-DSYNTHETIC_CAPTURE` | fabricated LFSR counts for a 1×1 m square path | no |

### Synthetic mode

Core 1 runs the **forward model**: it places a virtual body on a 1×1 m square,
projects each sensor through the *same* `BS_POSES` the solver uses, inverts the
Bitcraze + calibration mapping to produce the exact LFSR counts a real sensor
would emit, and writes them into `g_lh2[]`. Because injector and solver share
the geometry and the round-trip is exact, the firmware recovers the known square
to the millimetre — validating the entire cross-core pipeline on-chip without
hardware. See `SYNTHETIC_CAPTURE.md`.

### Hardware / pin map

```
Sensor data/env pins:  S0 10/11   S1 12/13   S2 18/19   S3 20/21
MAVLink UART0:          TX 0 / RX 1  @ 115200  → Pixhawk TELEM2
System clock:          128 MHz
stdio:                 USB CDC (diagnostics);  UART stdio disabled (UART0 = MAVLink)
Compute cadence:       ~10 Hz (PRINT_INTERVAL_US = 100000); capture runs continuously
```

---

## 8. Serial output (USB CDC, 115200)

```
A,<sensor>,<bs>,<horiz_deg>,<vert_deg>                       machine-parseable angle
ANG S<sensor> | BS0 h=.. v=.. deg | BS1 h=.. v=.. deg        human-readable angle
P,<sensor>,<x>,<y>,<z>                                       per-sensor 3D point [m]
C,<n>,<cx>,<cy>,<cz>                                         centroid [m] (also sent as VPE)
```

VPE frames leave UART0 in parallel. The `utils/scripts/` tools
(`plot_cycle.py`, `plot_geometry.py`) parse these lines for visualization.

---

## 9. MAVLink output

`mavlink/` hand-encodes a MAVLink v2 `VISION_POSITION_ESTIMATE` (msg #102):
45-byte frame, CRC16/MCRF4XX (CRC_EXTRA = 158), sysid 1, compid 191
(VISUAL_INERTIAL_ODOMETRY). Position is sent in metres (NED); roll/pitch/yaw are
encoded as `NaN` so the flight controller fuses only position and derives
attitude from its own sensors. ArduPilot side: `VISO_TYPE=1`,
`EK3_SRC1_POSXY=6` (ExternalNav).

---

## 10. Open items / follow-ups

1. **World → NED mapping** — confirm the X/Y → North/East assignment for the
   physical room; only the Z sign is handled today.
2. **BS index ↔ geometry key** — verify `geos:0 ↔ poly 8/9 (BS4)` and
   `geos:1 ↔ poly 20/21 (BS10)` when loading real calibration.
3. **Cross-core memory barrier** — optional `__dmb()` on the `g_lh2` handshake.
4. **Real-hardware validation** — the synthetic path is verified end-to-end;
   the real capture (`lh2/`) still needs on-rig confirmation with calibrated
   poses.
