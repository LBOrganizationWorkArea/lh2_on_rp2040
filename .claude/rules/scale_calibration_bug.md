# Scale / Calibration Bug — Root Cause and Fix

## The symptom

Moving a sensor from directly under BS0 to directly under BS1 (physical separation
2.26 m) causes the system to report only **1.0 m** of displacement.

---

## Root cause A — old firmware (`calibrate_export.py` with `REFERENCE_DIST = 1.0`)

`utils/calibration/calibrate_lighthouse.py` scales its output by `REFERENCE_DIST = 1.0`.
This means whatever the true physical LH separation is, `lab.yaml` (and everything
exported from it) always places BS1 at **x = 1.0 m** and rescales the height
proportionally:

```
BS1_x_scaled   = 1.0 m
height_scaled  = physical_height / physical_separation
               = 3.45 / 2.26 = 1.527 m
```

If `calibrate_export.py` is then run and the resulting UF2 is flashed, the DLT
solver uses these scaled poses.

### Why 1.0 m is reported — the math

The physical angle a sensor at x = 2.26 m subtends from BS0 at height 3.45 m is:

```
θ = atan(2.26 / 3.45) = 33.2°
```

The A/B LFSR calibration gives the **physically correct** angle (LFSR counts encode
beam-hit time, a hardware property independent of room geometry). BUT the DLT uses
the **scaled height** from the old firmware:

```
X_reported = tan(33.2°) × 1.527 m = 0.655 × 1.527 = 1.0 m
```

It gives exactly 1.0 m because `2.26/3.45 = 1.0/1.527` — `REFERENCE_DIST` scaling
preserves angles, so the old DLT consistently recovers 1.0 m regardless of the true
physical separation.

### Fix

`bs_poses_cal.h` must be set with physically measured values:

```c
/* BS0 */
.origin = {0.000000f, 0.000000f, 3.450000f},   // height physically measured
/* BS1 */
.origin = {2.260000f, 0.000000f, 3.450000f},   // separation physically measured
```

Both R matrices stay as `Ry(90°)` (boresight = world −Z, LH pointing straight down).
Rebuild and commit both UF2s. **Never run `calibrate_export.py` and flash the result
without also updating the origin values to physical measurements.**

---

## Root cause B — `compute_3d_coordinates.py` (`D_BS = 1.0`)

The standalone Python solver uses a closed-form formula:

```python
D_BS = 1.0   # ← was the bug, fixed to 2.26
Y = D_BS / denom
X = -Y * tan_a
```

The algebra makes the X displacement from BS4→BS10 equal to **exactly `D_BS`**,
regardless of the real physical separation. With `D_BS = 1.0` and 2.26 m real
separation the script always reports 1.0 m.

**Fixed:** `D_BS = 1.0 → 2.26` on line 14 of `compute_3d_coordinates.py`.

Note: this script reads `LH2,...` serial format (old firmware). The current
`crossing_beams.uf2` outputs `A,...` / `C,...` — the two are **incompatible**.

---

## Which UF2 to flash

Only two UF2s are kept in git (stale builds deleted 2026-07-02):

| File | Purpose |
|------|---------|
| `rp2350_firmware/src/build/crossing_beams.uf2` | Real hardware — flash this |
| `rp2350_firmware/src/build/crossing_beams_synthetic.uf2` | No sensors needed |

**Do not flash any other `.uf2` found on disk** — old builds have 1.0 m / 1.527 m
geometry and will reproduce the scale bug.

---

## Quick verification after flashing

Connect to Pico USB at 115200 baud. On startup:

```
BS poses: synthetic        ← label only, does not confirm geometry
```

Then watch `C,...` lines:
- Sensor under BS0 → `C,0.0000,...`
- Sensor under BS1 → `C,2.2600,...`

If you see `C,1.0000,...` under BS1, the wrong firmware is running.

Alternatively, open Mission Planner and watch the ODOMETRY message. The x field
should go from 0 to 2.26, not 0 to 1.
