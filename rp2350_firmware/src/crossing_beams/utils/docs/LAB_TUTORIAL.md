# Lab Tutorial — Flashing and Testing crossing_beams on Real Hardware

---

## What this firmware does

Four TS4231 photodiode sensors watch two Lighthouse v2 base stations. The Pico
computes a metric 3D position (~10 Hz) and streams it to a Pixhawk as a MAVLink
`VISION_POSITION_ESTIMATE`. It also prints human-readable diagnostics over USB so
you can see what's happening without any extra equipment.

---

## Hardware setup

### Base stations

Mount the two base stations like this:

```
        Wall
   BS0 ──────── BS1
   (0,0,1.5)   (1,0,1.5)
   │←── 1 m ──►│
   │    1.5 m above floor
   │
   both facing forward into the room, parallel to each other
```

- 1 m apart, measured center to center
- 1.5 m off the floor
- Both pointing straight forward (not angled)
- Both must be powered on before the firmware does anything useful

### Sensor wiring

| Sensor | DATA → Pico GPIO | ENV → Pico GPIO |
|--------|-----------------|-----------------|
| S0 | 10 | 11 |
| S1 | 12 | 13 |
| S2 | 18 | 19 |
| S3 | 20 | 21 |

### STM32H7 wiring (UART)

Connect Pico UART0 to whichever UART you are using on the H7. **TX and RX must be crossed:**

```
Pico GPIO 0  (TX)  →  H7 UART RX
Pico GPIO 1  (RX)  →  H7 UART TX
Pico GND           →  H7 GND
```

Both sides at **115200 baud**. If TX/RX are not crossed, the H7 receives
nothing and gives no error — it just silently ignores the line.

---

## Step 1 — Flash the firmware

The built `.uf2` is at:
```
rp2350_firmware/src/crossing_beams/build/crossing_beams.uf2
```

1. **Hold BOOTSEL** on the Pico
2. **Plug in USB** while holding BOOTSEL
3. **Release BOOTSEL** — a drive called `RPI-RP2` appears on your computer
4. **Drag and drop** `crossing_beams.uf2` onto it
5. The drive disappears and the Pico reboots into the firmware automatically

---

## Step 2 — Open the serial monitor

The Pico prints diagnostics over USB at **115200 baud**.

On Linux:
```bash
screen /dev/ttyACM0 115200
```

On Windows: open PuTTY → Serial → `COMx` → 115200.

---

## Step 3 — Check the serial output

You should see lines like these scrolling at ~10 Hz:

```
=== LH2 Crossing-Beams 3D Solver (dual-core) ===
Core 1: capture (4 sensors)  Core 0: solve + MAVLink VPE
Capture core ready.
A,0,0,12.34,5.67        ← sensor 0, base station 0: azimuth / elevation [deg]
A,0,1,-9.81,5.40        ← sensor 0, base station 1
P,0,0.41,0.12,1.93      ← sensor 0 solved 3D position [m]
C,4,0.40,0.11,1.95      ← centroid of all sensors [m] — also sent as MAVLink VPE
```

| Line | Means |
|------|-------|
| `A,s,bs,h,v` | Sensor `s` sees base station `bs` at those angles |
| `P,s,x,y,z` | Sensor `s`'s computed 3D position in metres |
| `C,n,x,y,z` | Final position: average of `n` sensors — this is what goes to the Pixhawk |

**MAVLink is sent automatically** every time a `C` line is produced. There is no
separate switch to enable it.

---

## Step 4 — Sanity checks

**If you see no output at all:**
- Check USB connection and baud rate.

**If you see the header but no `A` lines:**
- Base stations are not powered on, not in view, or sensor wiring is wrong.

**If you see `A` lines but no `P` or `C` lines:**
- At least one sensor only sees one base station. Make sure both base stations are
  fully visible to all four sensors with no obstructions.

**If `C` lines appear but position looks wrong:**
- Hold the sensor board still — values should be stable.
- Move it forward (away from the wall) — the Y value should increase.
- Move it sideways — the X value should change.
- If axes are flipped or position is nonsense, the base station geometry or
  wiring may not match the configuration.

---

## Step 5 — Verify MAVLink with Mission Planner

You can use Mission Planner to see the position data live without needing the
H7 at all. You need a **USB-to-UART adapter** (e.g. FTDI, CP2102).

### Wiring for this test

```
Pico GPIO 0  (TX)  →  USB-UART adapter RX
Pico GPIO 1  (RX)  →  USB-UART adapter TX
Pico GND           →  USB-UART adapter GND
```

Plug the adapter into your PC. This is separate from the Pico's USB cable —
you need both plugged in at the same time (USB for serial diagnostics, adapter
for MAVLink).

### Connecting Mission Planner

1. Open Mission Planner
2. Top-right corner: select the COM port of your USB-UART adapter and set baud
   rate to **115200**
3. Click **Connect**
4. Mission Planner may show "no heartbeat" — that is expected, the Pico does
   not send heartbeats. The data still flows.

### Seeing the position data

Once connected:

1. Press **Ctrl + F** to open the temp menu
2. Click **MAVLink Inspector**
3. Look for message **`VISION_POSITION_ESTIMATE`** (ID 102) appearing at ~10 Hz
4. Click on it to expand — you will see `x`, `y`, `z` updating live in metres

If the message appears and the values move when you move the sensor board,
everything is working correctly.

---

## If you need to rebuild the firmware

Only needed if someone changed the source code.

```bash
cd ~/Repositories/lh2_on_rp2040/rp2350_firmware/src/crossing_beams
~/.pico-sdk/ninja/v1.12.1/ninja -C build crossing_beams
```

Then flash `build/crossing_beams.uf2` as described in Step 1.

If `CMakeLists.txt` was also changed, run this first:
```bash
~/.pico-sdk/cmake/v3.31.5/bin/cmake -B build -G Ninja -DPICO_BOARD=pico2 .
```
Then rebuild with ninja.
