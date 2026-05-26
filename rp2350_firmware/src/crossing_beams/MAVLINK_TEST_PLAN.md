# MAVLink VPE Square Test — Implementation Plan

## 1. Goal

Add a second self-test firmware target (`crossing_beams_mavlink_test`) to the
`crossing_beams/` directory.  Like the existing `crossing_beams_test`, it
requires **no LH2 hardware** — no PIO, no DMA, no TS4231 sensors.

The target:

1. Continuously feeds synthetic LH2 angle measurements into the existing
   `solve3d` pipeline.
2. Every **8 seconds** jumps to the next corner of a **1 m × 1 m square** at
   z = 2 m, so the inferred 3D centroid traces a closed square path.
3. Packages the resulting centroid as a **MAVLink v2 `VISION_POSITION_ESTIMATE`
   (msg #102)** frame and sends it over **UART to a Pixhawk 6C** at **25 Hz**.
   Roll, pitch, and yaw are sent as **NaN** (MAVLink convention for "not
   provided"), so the FC fuses only the position.
4. Simultaneously prints human-readable `DBG` lines over USB serial for
   debugging — no PC↔RP2350 connection is required in normal use.

The end result: Mission Planner, connected only to the Pixhawk, shows the
virtual "drone" moving along a 1 m square at 2 m height.

---

## 2. System Topology

```
┌─────────────────┐   UART0 (MAVLink VPE, 25 Hz)   ┌──────────────────┐
│   RP2350        │ ──────────────────────────────► │  Pixhawk 6C      │
│ (fake angles    │   GPIO 0 TX → TELEM2 RX         │  (EKF3 fuses     │
│  + solve3d      │   GPIO 1 RX ← TELEM2 TX         │   position)      │
│  + mavlink enc) │                                 │                  │
└────────┬────────┘                                 └────────┬─────────┘
         │ USB (DBG text, optional)                          │ USB / radio
         │ ← PC terminal for debug only                      ▼
         │                                        ┌──────────────────────┐
         │                    NO direct link       │  PC — Mission Planner│
         └──────────────────────────────────────── │  (sees drone moving) │
                                                   └──────────────────────┘
```

**The PC connects only to the Pixhawk.** Mission Planner reads the Pixhawk's
EKF-estimated position, which is driven by the VPE messages from the RP2350.
The RP2350 USB port is purely optional — useful for debugging `DBG` lines, but
not required for Mission Planner to work.

---

## 3. Hardware Connection — RP2350 → Pixhawk 6C

```
RP2350 (Pico 2)           Pixhawk 6C TELEM2
──────────────────         ─────────────────────
GPIO 0  (UART0 TX) ──────► RX  (pin 3)
GPIO 1  (UART0 RX) ◄────── TX  (pin 2)
GND                ──────── GND (pin 6)
```

Both sides operate at **3.3 V logic** — no level shifter needed.
Baud rate: **115200**.

---

## 4. ArduPilot Configuration

> All changes are made in Mission Planner → **Config → Full Parameter List**.
> Search for the parameter name, change the value, click **Write Params**.
> Parameters marked 🔄 require a **reboot** to take effect.

---

### 4.1 — Tell TELEM2 to speak MAVLink v2

These two parameters tell the Pixhawk that something is sending MAVLink v2
frames on the TELEM2 serial port at 115200 baud.  Without this the Pixhawk
ignores everything the RP2350 sends.

| Parameter | Set to | Why |
|-----------|--------|-----|
| `SERIAL2_PROTOCOL` 🔄 | `2` | MAVLink 2 on TELEM2 |
| `SERIAL2_BAUD` 🔄 | `115` | 115200 baud (ArduPilot stores baud / 1000) |

---

### 4.2 — Enable the visual odometry frontend

`VISO_TYPE` activates ArduPilot's internal visual odometry library
(`AP_VisualOdom`).  When set to 1 it watches for incoming
`VISION_POSITION_ESTIMATE` MAVLink messages, validates them, and forwards
the position to the EKF3 as **ExternalNav** data.  Without this the EKF never
sees the RP2350 messages even if TELEM2 is configured correctly.

| Parameter | Set to | Why |
|-----------|--------|-----|
| `VISO_TYPE` 🔄 | `1` | MAVLink — consume VISION_POSITION_ESTIMATE |

---

### 4.3 — Point EKF3 at ExternalNav as its position source

This is the core of the setup.  EKF3 has a priority-based source system: for
each quantity it needs (XY position, Z position, yaw, …) you tell it where to
get the data.  Value `6` means **ExternalNav**, which is exactly what
`VISO_TYPE = 1` feeds it.

| Parameter | Set to | Why |
|-----------|--------|-----|
| `EK3_SRC1_POSXY` 🔄 | `6` | XY position comes from ExternalNav (our VPE) |
| `EK3_SRC1_POSZ` 🔄 | `6` | Z position also comes from ExternalNav |
| `EK3_SRC1_VELXY` | `0` | No velocity source (we don't send velocity) |
| `EK3_SRC1_VELZ` | `0` | No vertical velocity source |
| `EK3_SRC1_YAW` | `1` | Yaw from compass — RP2350 sends NaN so EKF3 must get yaw elsewhere |

> **Why not touch `EK3_SRC1_YAW = 6` (ExternalNav yaw)?**
> Our `VISION_POSITION_ESTIMATE` sends NaN for roll/pitch/yaw — ArduPilot
> will silently discard those fields.  If yaw source is set to ExternalNav
> but no yaw arrives, the EKF fails to initialise.  Compass (value 1) is the
> safe fallback for this demo.

---

### 4.4 — Disable GPS

With GPS active, ArduPilot defaults to using it for position and may ignore
the VPE entirely.  Setting `GPS_TYPE = 0` removes GPS from the picture and
forces the EKF to rely solely on ExternalNav.

| Parameter | Set to | Why |
|-----------|--------|-----|
| `GPS_TYPE` 🔄 | `0` | No GPS — EKF3 uses ExternalNav position only |

---

### 4.5 — Reboot and confirm

After writing all parameters, **reboot the Pixhawk**.

In Mission Planner open **MAVLink Inspector** (press `Ctrl+F`, search
"MAVLink Inspector") and verify:

- `VISION_POSITION_ESTIMATE` is appearing at ~25 Hz.
- The `x`, `y`, `z` fields step through the four corners every 8 s.

The HUD altitude and the map position marker should update in real time as
the RP2350 walks around its synthetic square.

---

### 4.6 — Quick reference (all params at a glance)

```
SERIAL2_PROTOCOL = 2     ← MAVLink 2 on TELEM2            🔄 reboot
SERIAL2_BAUD     = 115   ← 115200 baud                     🔄 reboot
VISO_TYPE        = 1     ← consume VISION_POSITION_ESTIMATE 🔄 reboot
EK3_SRC1_POSXY   = 6     ← XY position = ExternalNav        🔄 reboot
EK3_SRC1_POSZ    = 6     ← Z  position = ExternalNav        🔄 reboot
EK3_SRC1_VELXY   = 0     ← no velocity input
EK3_SRC1_VELZ    = 0     ← no vertical velocity input
EK3_SRC1_YAW     = 1     ← yaw from compass
GPS_TYPE         = 0     ← disable GPS                      🔄 reboot
```

---

## 5. `mavlink/` Library

### 5.1 Responsibility

Encode and send **one** MAVLink v2 message type: `VISION_POSITION_ESTIMATE`.
Nothing else — no parser, no heartbeat, no full protocol stack.

Everything is stack-allocated; no heap, no global buffers.

### 5.2 Public API — `mavlink.h`

```c
/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 encoder for VISION_POSITION_ESTIMATE (msg #102).
 *
 * Sends raw bytes to UART0 (GPIO 0/1, 115200 baud).
 * Attitude (roll/pitch/yaw) is always encoded as NaN — the flight controller
 * will fuse only the position and use its own sensors for attitude.
 *
 * Call mavlink_init() once before the first mavlink_send_vpe().
 */

/** Initialise UART0 at 115200 baud on GPIO 0 (TX) / GPIO 1 (RX). */
void mavlink_init(void);

/**
 * Encode and send VISION_POSITION_ESTIMATE over UART0.
 *
 * @param usec     timestamp [µs since boot]
 * @param x, y, z  position  [metres]
 */
void mavlink_send_vpe(uint64_t usec, float x, float y, float z);
```

Roll, pitch, and yaw are **not parameters** — they are encoded internally as
`NaN` (`0x7FC00000` in IEEE 754 single-precision), which is the MAVLink
convention for "field not populated / do not fuse".

### 5.3 MAVLink v2 Frame Format

`VISION_POSITION_ESTIMATE` wire layout
**(12 B header + 37 B payload + 2 B CRC = 51 bytes total):**

| Offset | Size | Field | Value |
|--------|------|-------|-------|
| 0 | 1 | STX | `0xFD` |
| 1 | 1 | len | `37` |
| 2 | 1 | incompat_flags | `0` |
| 3 | 1 | compat_flags | `0` |
| 4 | 1 | seq | auto-increment per call |
| 5 | 1 | sysid | `1` |
| 6 | 1 | compid | `191` (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY) |
| 7–9 | 3 | msgid | `102, 0, 0` (little-endian 24-bit) |
| 10–17 | 8 | payload: usec | `uint64_t` LE |
| 18–21 | 4 | payload: x | `float` LE |
| 22–25 | 4 | payload: y | `float` LE |
| 26–29 | 4 | payload: z | `float` LE |
| 30–33 | 4 | payload: roll | `NaN` (`0x7FC00000`) |
| 34–37 | 4 | payload: pitch | `NaN` (`0x7FC00000`) |
| 38–41 | 4 | payload: yaw | `NaN` (`0x7FC00000`) |
| 42 | 1 | payload: reset_counter | `0` |
| 43–44 | 2 | CRC | CRC16/MCRF4XX, `CRC_EXTRA = 158` |

### 5.4 CRC Algorithm

MAVLink uses **CRC16/MCRF4XX** (poly `0x1021`, seed `0xFFFF`, no bit
reflection):

```c
static uint16_t crc_accumulate(uint8_t b, uint16_t crc) {
    uint8_t tmp = b ^ (crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4);
}
```

Process bytes at offsets 1–42 (all header fields after STX + full payload),
then feed **`CRC_EXTRA = 158`** as one final byte before extracting the two
output bytes (low byte first).

### 5.5 Implementation Notes

- `seq` is a `static uint8_t` inside `mavlink.c`; wraps automatically.
- All multi-byte fields are **little-endian**.  On RP2350 (Cortex-M33, LE) a
  `memcpy` from a `float` / `uint64_t` directly into the byte buffer gives the
  correct layout.
- NaN is written as the 4-byte constant `{ 0x00, 0x00, 0xC0, 0x7F }` (LE IEEE
  754 quiet NaN).
- Output via `uart_write_blocking(uart0, buf, 51)`.

---

## 6. `main_mavlink_test.c` — Test Driver

### 6.1 Square Pattern

Four corners of a 1 m × 1 m square in the horizontal (XY) plane at z = 2 m:

```
corner 0 : body = (0.00, 0.00, 2.00)   ← starting position
corner 1 : body = (1.00, 0.00, 2.00)
corner 2 : body = (1.00, 1.00, 2.00)
corner 3 : body = (0.00, 1.00, 2.00)
```

Advances to next corner every **8 s**.  Full lap = 32 s.

Same 4-sensor body layout as `main_test.c` (5 cm × 5 cm flat square,
sensor Z = 0 in body frame).

### 6.2 Main Loop (25 Hz)

Loop period: **40 000 µs**.

```
initialise:
    stdio_init_all()        // USB for DBG text
    mavlink_init()          // UART0 → Pixhawk 6C
    solve3d_init(&ctx)
    last_corner_us = time_us_64()
    corner_idx     = 0

loop forever:
    now_us = time_us_64()

    ── corner advance ──────────────────────────────────────────────
    if (now_us - last_corner_us) >= 8_000_000:
        corner_idx     = (corner_idx + 1) % 4
        last_corner_us = now_us
        printf("CORNER → %d\n", corner_idx)

    ── push one measurement round (all 4 sensors) ──────────────────
    body = corners[corner_idx]

    for s in 0..3:
        Pw = body + (SENSOR_BODY[s].x, SENSOR_BODY[s].y, 0)

        // BS0 at (0, 0, 0) → +Z
        az_a = atan2f(Pw.x,      Pw.z)
        el_a = atan2f(Pw.y, hypotf(Pw.x, Pw.z))

        // BS1 at (1, 0, 0) → +Z
        bx   = Pw.x - 1.0f
        az_b = atan2f(bx,        Pw.z)
        el_b = atan2f(Pw.y, hypotf(bx,   Pw.z))

        angles_to_pixels(az_a, el_a, smp.px_a)
        angles_to_pixels(az_b, el_b, smp.px_b)
        smp.sensor_id = s
        solve3d_push_sample(&ctx, &smp)

    ── solve ────────────────────────────────────────────────────────
    if ctx.n_samples >= SOLVE3D_MIN_SAMPLES:
        n  = solve3d_run(&ctx, pts3d)
        cx = cy = cz = 0
        for i in 0..n:
            cx += pts3d[i].xyz[0]
            cy += pts3d[i].xyz[1]
            cz += pts3d[i].xyz[2]
        cx /= n;  cy /= n;  cz /= n

        ── MAVLink → Pixhawk 6C (UART0) ────────────────────────────
        mavlink_send_vpe(now_us, cx, cy, cz)      // roll/pitch/yaw = NaN

        ── DBG → PC terminal (USB, optional) ───────────────────────
        printf("DBG,%d,%.3f,%.3f,%.3f\n",
               corner_idx, (double)cx, (double)cy, (double)cz)

    sleep_us(40_000)
```

### 6.3 Rate Analysis

| Parameter | Value | Reasoning |
|-----------|-------|-----------|
| Loop rate | 25 Hz | Standard VPE rate for ArduPilot / PX4 |
| Dwell per corner | 8 s | 200 iterations × 4 sensors = 800 pushes per corner |
| Ring buffer refill | ~0.32 s | 32 slots ÷ (25 Hz × 4 sensors) |
| Full square lap | 32 s | 4 corners × 8 s |

---

## 7. CMakeLists.txt Additions

```cmake
# ============================================================================
# crossing_beams_mavlink_test
#
# Feeds fake solve3d angles tracing a 1 m square through the pipeline,
# sends VISION_POSITION_ESTIMATE (roll/pitch/yaw = NaN) at 25 Hz over
# UART0 (GPIO 0/1) to a Pixhawk 6C.
# USB serial outputs optional human-readable DBG lines.
# No LH2 hardware required.
# ============================================================================

add_executable(crossing_beams_mavlink_test
    main_mavlink_test.c
    cv/cv.c
    solve3d/solve3d.c
    mavlink/mavlink.c
)

target_include_directories(crossing_beams_mavlink_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(crossing_beams_mavlink_test
    pico_stdlib
    hardware_uart
)

pico_add_extra_outputs(crossing_beams_mavlink_test)

pico_enable_stdio_usb(crossing_beams_mavlink_test  1)   # DBG text
pico_enable_stdio_uart(crossing_beams_mavlink_test 0)   # UART used directly

target_compile_options(crossing_beams_mavlink_test PRIVATE
    -Wall -Wno-format -Wno-unused-function -O2
)
```

---

## 8. Memory Budget

| Item | Size | Notes |
|------|------|-------|
| `solve3d_ctx_t` history | 640 B | 32-slot ring buffer |
| `pts3d` output (stack) | 512 B | per solve call |
| MAVLink frame (stack) | 51 B | inside `mavlink_send_vpe` |
| `cv` working arrays (stack) | ~600 B | 9×9 AᵀA + SVD temps |
| **Total** | **< 2 KB** | Well within 520 KB SRAM |

---

## 9. Verification

### Step 1 — Flash the RP2350
Build and flash `crossing_beams_mavlink_test.uf2`.

### Step 2 — Wire to Pixhawk 6C
```
RP2350 GPIO 0  ──►  Pixhawk TELEM2 RX
RP2350 GPIO 1  ◄──  Pixhawk TELEM2 TX
RP2350 GND     ───  Pixhawk GND
```

### Step 3 — Configure Pixhawk (see §4 for full explanation)
```
SERIAL2_PROTOCOL = 2     EK3_SRC1_POSXY = 6
SERIAL2_BAUD     = 115   EK3_SRC1_POSZ  = 6
VISO_TYPE        = 1     EK3_SRC1_VELXY = 0
GPS_TYPE         = 0     EK3_SRC1_VELZ  = 0
                         EK3_SRC1_YAW   = 1
```
Reboot.

### Step 4 — Set a home position (needed for the map)

`VISION_POSITION_ESTIMATE` sends **local XYZ in metres**, not GPS
coordinates.  For Mission Planner's map to place the drone somewhere visible,
the EKF needs a geographic origin to convert XY → lat/lon.

1. Connect Mission Planner to the Pixhawk via USB.
2. Go to **Flight Data → Map**.
3. Right-click anywhere on the map → **"Set Home Here"**.

ArduPilot will use that point as the EKF origin.  The drone icon will then
appear at that spot and offset by the VPE XY values (max 1 m away from home —
zoom in on the map to see it move).

### Step 5 — Watch the drone move

There are three places in Mission Planner where you can see the position
updating, from most visual to most raw:

---

#### 5a — Map (most visual)

Go to **Flight Data → Map tab**.

The drone icon will step around a ~1 m square relative to the home you set.
**You need to zoom in very far** — at normal map zoom 1 m is invisible.
Use the scroll wheel until individual metres are visible on screen.

Expected sequence (every 8 s the icon jumps to the next corner):
```
corner 0 → corner 1 → corner 2 → corner 3 → corner 0 → ...
 (0, 0)      (1, 0)      (1, 1)      (0, 1)
```

---

#### 5b — HUD altitude

The **HUD** (the artificial horizon on the left of Flight Data) shows
altitude.  Since `EK3_SRC1_POSZ = 6`, the altitude displayed comes directly
from the VPE z field.  You should see it locked near **2.00 m** throughout
the test.  This is a quick sanity check that the Z channel is being fused.

---

#### 5c — Quick panel (numerical, most reliable for small movements)

On the right side of the Flight Data screen there is a **Quick** tab showing
live parameter values.  Right-click on any row → **"Add item"** and add:

| Field to add | What it shows |
|---|---|
| `localx` | EKF local X position [m] — steps 0 → 1 → 1 → 0 |
| `localy` | EKF local Y position [m] — steps 0 → 0 → 1 → 1 |
| `localz` | EKF local Z position [m] — stays near −2.0 (NED: down is positive, so z = −2 means 2 m up) |

These update at the EKF rate and are the clearest way to confirm the position
is being driven by the RP2350.

---

#### 5d — MAVLink Inspector (raw message verification)

Press `Ctrl+F` → search **"MAVLink Inspector"** → open it.

Find `VISION_POSITION_ESTIMATE` in the list.  You should see:
- **Rate** ≈ 25 Hz
- **x / y / z** stepping through the four corners
- **roll / pitch / yaw** = NaN (shown as a very large number or "nan")

This confirms the RP2350 is sending correctly before worrying about EKF
fusion.  If this message does not appear, the problem is in the wiring or the
`SERIAL2` parameters — not the EKF settings.

---

### Step 6 — Optional RP2350 debug

Open a serial terminal on the RP2350 USB port (115200 baud).  `DBG` lines
will show what the solve3d centroid is computing in real time:

```
DBG,0,0.004,0.003,1.999    ← corner 0, centroid ≈ (0, 0, 2)
DBG,0,0.005,0.001,2.001
CORNER → 1
DBG,1,0.997,0.002,2.001    ← corner 1, centroid ≈ (1, 0, 2)
```

Cross-check these numbers against `localx / localy / localz` in the Quick
panel — they should match (with sign flip on z due to NED convention).

---

## 10. Files Summary

| File | Status | Purpose |
|------|--------|---------|
| `mavlink/mavlink.h` | **NEW** | API: `mavlink_init()`, `mavlink_send_vpe(usec, x, y, z)` |
| `mavlink/mavlink.c` | **NEW** | MAVLink v2 encoder, NaN attitude, UART0 output |
| `main_mavlink_test.c` | **NEW** | Square-path driver, 25 Hz loop |
| `CMakeLists.txt` | **MODIFIED** | New `crossing_beams_mavlink_test` target |
| `cv/cv.c`, `solve3d/solve3d.c` | unchanged | Reused as-is |
