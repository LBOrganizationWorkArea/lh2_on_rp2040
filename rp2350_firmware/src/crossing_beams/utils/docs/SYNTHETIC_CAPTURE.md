# Synthetic Capture — exercising the dual-core pipeline without sensors

> **Target:** `crossing_beams_synthetic`
> **Source:** `main_real.c` compiled with `-DSYNTHETIC_CAPTURE`
> **Hardware needed:** none (a bare Pico 2 — sensors and base stations optional)

The real firmware needs four TS4231 sensors and two LH2 base stations to
produce anything. This build lets you run **the exact same dual-core
architecture** with no hardware at all, so you can prove the plumbing works long
before the sensors arrive.

---

## The core idea: lie at the seam, not at the edges

The naive way to fake a position test is to skip the pipeline and just send a
made-up coordinate straight to MAVLink. That proves the MAVLink link — but it
tests *none* of the LH2 decode or solver code, and none of the cross-core
handoff.

This build does something sharper. It injects fake data at the **one seam where
core 1 hands off to core 0** — the shared `g_lh2[]` buffer — and lets every
downstream stage run for real:

```
        ┌──────────────── Core 1 (capture) ────────────────┐
 REAL:  │  PIO + DMA → demodulate → LFSR search → g_lh2[]   │
 SYNTH: │  geometry → INVERSE calibration → g_lh2[]         │   ← only this changes
        └───────────────────────┬───────────────────────────┘
                                 │  g_lh2[].locations + data_ready   (the seam)
        ┌───────────────────────▼───────────────────────────┐
 BOTH:  │  Core 0:  angle_decoder → solve3d → MAVLink VPE    │   ← runs unchanged
        └────────────────────────────────────────────────────┘
```

Everything below the seam — the angle decoder, the EMA filter, the calibrated-
pose triangulation, the MAVLink encoder, **and the cross-core
data sharing itself** — is the real code, byte-for-byte identical to the
hardware build. Only core 1's data *source* is swapped.

---

## What core 1 fakes, step by step

A virtual rigid body walks a 1 m × 1 m square at z = 2 m, 8 s per edge. Four
virtual sensors sit on it in a 5 × 5 cm pattern. Two virtual base stations watch
from BS0 = (0,0,0) and BS1 = (1,0,0), both facing +Z.

```
        z (up)
        │        body at (bx,by,2)  ── walks the square ──►
        │            ◇ S3   ◇ S2
        │            ◇ S0   ◇ S1
        │
   BS0 ─┴────────────── BS1
   (0,0,0)            (1,0,0)        x →
```

For each of the 4 sensors, every 20 ms (≈ the real LH2 sweep rate), core 1:

1. **Places the sensor in the world** — body position + body-frame offset.
2. **Computes the angles each base station would see** (plain trig):
   ```
   BS0:  az = atan2(wx, wz)            el = atan2(wy, √(wx²+wz²))
   BS1:  az = atan2(wx−1, wz)          el = atan2(wy, √((wx−1)²+wz²))
   ```
3. **Runs the angle decoder *backwards*** to find the two sweep angles, then
   runs the **calibration backwards** to find the LFSR counts that produce them
   (see next section).
4. **Writes those counts into `g_lh2[]`** exactly where
   `db_lh2_process_location()` would, and sets `data_ready =
   DB_LH2_PROCESSED_DATA_AVAILABLE`.

Core 0 never knows the difference.

---

## The clever bit: inverting the decoder

Core 0's angle decoder turns two raw *sweep angles* `(a0, a1)` into an
azimuth/elevation pair like this (`angle_decoder.c::_finalize_angles`):

```
az  = (a0 + a1) / 2
el  = atan( tan((a0−a1)/2) / TAN_30 / cos(az) )
```

To make the decoder output a position we *choose*, core 1 solves these for
`(a0, a1)` given the target `(az, el)`:

```
diff/2 = atan( tan(el) · TAN_30 · cos(az) )
a0     = az + diff/2
a1     = az − diff/2
```

Then it inverts the per-sweep linear calibration `angle = A·lfsr + B`:

```
lfsr_sweep = (angle_sweep − B_sweep) / A_sweep
```

using the **same calibration constants** the real firmware uses (`CAL_BS*`). The
result is the precise integer LFSR count a real photodiode would have produced.
Feed it back through the forward path and you recover the original `(az, el)` to
within integer-rounding error (~0.003°). It's a closed loop by construction.

> Polynomial/slot mapping used for the injection:
> BS0 → polynomial 8, array slot 4; BS1 → polynomial 20, array slot 10
> (`slot = polynomial >> 1`, matching `lh2.c`). Both sweep slots are written so
> the decoder's "need both sweeps before finalizing" rule is satisfied each tick.

---

## What this proves — and what it doesn't

**✅ Proven on real silicon by running this build:**

| Claim | Why this build tests it |
|---|---|
| PIO IRQ affinity isn't the issue | core 1 runs standalone; if core 0 sees data, the seam works |
| Cross-core `g_lh2[]` sharing is sound | core 1 writes it, core 0 reads it, live, at 50 Hz |
| `data_ready` handshake is correct | the enum-mismatch bug would show as zero output |
| angle_decoder math + EMA filter run | `A,…` USB lines appear with sane angles |
| solver executes end-to-end + is metric | `P,…`/`C,…` recover the 1×1 m square to the mm; z pinned at 2.000 |
| MAVLink encoder + UART path work | VPE frames leave UART0 every solve |

Because the synthetic injector and the calibrated-pose solver share the **same**
`BS_POSES` and the forward/inverse mapping is exact, the recovered points match
the known square to the millimetre — so this build validates solver *correctness*
as well as data flow.

**❌ NOT tested (still needs real hardware / calibration):**

- The PIO/DMA capture and LFSR *search* (`lh2.c`) — omitted from this build.
- Real sensor noise, dropouts, and sweep-slot timing.
- Whether the *real* calibrated poses + angle convention agree on hardware — in
  synthetic mode the geometry is shared, so it is exact by construction.

In short: if this build streams `A`/`P`/`C` lines tracing a clean metric square
and VPE frames, the **architecture and solver** are sound, and the day the
sensors arrive you're debugging optics and calibration, not plumbing.

---

## Build & run

```bash
cd ~/Repositories/lh2_on_rp2040/rp2350_firmware/src/crossing_beams
~/.pico-sdk/cmake/v3.31.5/bin/cmake -B build -G Ninja -DPICO_BOARD=pico2 .
~/.pico-sdk/ninja/v1.12.1/ninja -C build crossing_beams_synthetic
```

Flash `build/crossing_beams_synthetic.uf2` (BOOTSEL + drag, or `picotool load … -fx`).

Open the USB serial at 115200 baud. Expected output:

```
=== LH2 Crossing-Beams 3D Solver (dual-core) ===
Core 1: SYNTHETIC capture (no sensors, 1x1m square)
Capture core ready.
A,0,0,12.34,5.67        ← sensor 0, BS0, az/el degrees (these should move
A,0,1,-9.81,5.40           smoothly as the virtual body walks the square)
...
P,0,0.41,0.12,1.93      ← per-sensor 3D point from solve3d
C,4,0.40,0.11,1.95      ← centroid (also sent as MAVLink VPE)
```

The `A` angle lines are the cleanest signal of success: they should sweep
smoothly and repeat on a 32 s loop (4 edges × 8 s). VPE frames go out UART0 in
parallel, so a connected Pixhawk will show `VISION_POSITION_ESTIMATE` arriving
just as in the square-path test.

## Going back to hardware

Build the plain `crossing_beams` target (no `-DSYNTHETIC_CAPTURE`). Same
`main_real.c`, same core 0 — the only thing that swaps back is core 1's data
source, from fabricated counts to real PIO captures.
