# Plan: Calibration-Driven Ray-Crossing Solver

Replace the unstable fundamental-matrix solver (`solve3d.c`) with direct
**ray-crossing** that uses calibrated lighthouse poses — giving **metric,
stable** position instead of the scale-ambiguous output that swings 0→170.

> Status: PLAN ONLY — for review before coding.

---

## 0. Core idea

A **Python calibration script is the single source of truth.** It emits metric,
world-aligned base-station poses. You paste that output into the firmware —
exactly like the `CAL_BS*` angle constants you already maintain. The firmware
does pure ray-crossing with those poses: no quaternion math, no pose-solving on
the MCU.

The same script runs in two modes so the dev flow mirrors the real one:

| Mode | Input | Use |
|---|---|---|
| **`--synthetic`** (now) | a known, hardcoded BS geometry | develop + test firmware ray-crossing today, no hardware |
| **real** (later) | drone measurements via `calibrate_cli.py` | actual calibration when sensors arrive |

Both emit the **identical output format**, so the firmware can't tell them apart.

---

## 1. Why ray-crossing (vs. the current solver)

| | Current (`solve3d.c`) | New (ray-crossing) |
|---|---|---|
| Uses known BS positions? | No — re-derives them every frame | **Yes** (from calibration) |
| Inputs needed | history buffer of 8–32 samples | **one** sample per base station |
| Output scale | arbitrary, changes each frame | **real metres** |
| Stability | wobbles / explodes (the 0→170) | **rock solid** |
| Good for | cameras lost in space | **fixed base stations on a wall** |

The base stations don't move, so we *use* that fact instead of making the solver
rediscover it (badly) 10× a second.

---

## 2. The Python calibration script

### 2.1 What the existing pipeline already does

`utils/calibration/calibrate_cli.py` drives an interactive acquisition:

```
record_origin()        # place drone at the world origin
record_x_axis()        # place drone on the +X axis
record_xy_plane()      # place drone on the XY plane
record_samples()       # free movement around the volume
```

then `calibrate_lighthouse.py :: _estimate_geometry()` runs:

```
match samples → IPPE initial guess → least-squares solve
→ align to world frame (origin / x-axis / xy-plane)
→ scale_fixed_point(REFERENCE_DIST = 1.0 m)        ← sets METRIC scale
→ returns dict[int, Pose]   (Pose has .translation [m] and .rotation_matrix)
```

So the **metres** (from the 1.0 m reference distance) and the **XY orientation +
position** (from the origin / x-axis / xy-plane you physically defined) come
straight out of calibration. We just need to surface them and hand them to the
firmware.

### 2.2 New script: `utils/calibration/calibrate_export.py`

A thin wrapper that produces firmware-ready output and prints the human-readable
world-frame summary.

**Responsibilities:**
1. Obtain BS poses — either:
   - **real:** from `_estimate_geometry(origin, x_axis, xy_plane, samples)`, or
   - **`--synthetic`:** from a hardcoded geometry (see §2.4), bypassing the
     `lbees.indoor.*` imports so it runs today with only `numpy`/`scipy`.
2. Convert each `Pose` → `position [m]` + `R` (3×3) using scipy (authoritative —
   keeps the quaternion-order question off the MCU).
3. Print the world-frame summary (the "meter + xy info"):
   ```
   World frame:  origin=(0,0,0)  +X → x-axis sample  scale = 1.000 m (ref dist)
   BS0 (geos:0 → poly 8/9,  BS4) : pos=[1.264, 0.345, 0.094] m
   BS1 (geos:1 → poly 20/21,BS10): pos=[1.280, 0.254,-0.092] m
   boresight check: R0·(1,0,0) = (..)   R1·(1,0,0) = (..)   ← eyeball BS aiming
   ```
4. Write the firmware header `bs_poses_cal.h` (see §3).

**CLI shape (proposed):**
```bash
# now, no hardware:
python calibrate_export.py --synthetic -o bs_poses_cal.h

# later, from a finished calibration / measurements:
python calibrate_export.py --measurements measurements.json \
    --origin 0 0 0 --x-axis 1 0 0 --xy-plane 0 1 0 -o bs_poses_cal.h
```

### 2.3 World frame → what XY actually means

Because you choose origin / x-axis / xy-plane during acquisition, **you** define
the frame:
- origin sample → world `(0,0,0)`
- x-axis sample → defines **+X** direction (and, with `REFERENCE_DIST`, the scale)
- xy-plane samples → define the **XY plane** (so +Z is the room's up/normal)

The script prints this so the numbers are interpretable, and so the
**world → NED** mapping for MAVLink (§6.3) can be defined deliberately.

### 2.4 Synthetic geometry (the `--synthetic` mode)

Hardcode a known two-base-station setup, e.g. (must match the firmware injector,
§5e):
```
BS0: position (0,0,0),   boresight +X toward the volume
BS1: position (1,0,0),   boresight +X toward the volume
```
Emit it through the same conversion + writer as the real path. This closes the
loop: script bakes poses → firmware ray-crosses → recovers the known 1×1 m
square, in metres.

---

## 3. Firmware ingest format

The script writes a generated header — the pose analogue of your `CAL_BS*`
constants:

```c
// bs_poses_cal.h  — generated by calibrate_export.py, DO NOT hand-edit
//   World frame: origin at calibration origin, +X → x-axis sample, metres.
#define BS_POSE_SOURCE "synthetic"        // or "calib 2026-05-26"

static const lh2_bs_pose_t BS_POSES[NUM_BS] = {
  { .origin = {1.264f, 0.345f, 0.094f},   // geos:0  (poly 8/9,  BS4)
    .R = {{...},{...},{...}} },
  { .origin = {1.280f, 0.254f,-0.092f},   // geos:1  (poly 20/21, BS10)
    .R = {{...},{...},{...}} },
};
```

`R` is computed in Python (scipy), so the quaternion-order trap never reaches
the firmware. The firmware just `#include`s it.

---

## 4. The ray-crossing math

Per base station, the two calibrated sweep angles → a ray in the BS local frame,
then into the world:

```
1. (a0, a1) sweep angles ──Bitcraze──► (horiz, vert)        # lighthouse_bs_vector.from_lh2
2. d_local = normalize( (1, tan(horiz), tan(vert)) )        # local +X = boresight
3. d_world = R_bs · d_local        ray origin = position_bs # calibrated pose
```

A sensor seen by BS0 and BS1 → two world rays. The sensor sits at the **closest
point between those two skew lines** (midpoint of nearest approach). One sample
per BS → one metric 3D point. 4 sensors → 4 points → centroid → VPE.

Closest-point-of-two-rays: for rays `p0 + s·d0` and `p1 + t·d1`, solve the 2×2
least-squares system for `(s, t)`, take the midpoint of the two nearest points;
the half-gap between them is a free quality metric.

---

## 5. Files to add / change

| # | File | Change | When |
|---|---|---|---|
| a | `utils/calibration/calibrate_export.py` *(new)* | wraps `_estimate_geometry()`; `--synthetic` mode; prints world-frame summary; writes `bs_poses_cal.h` | now |
| b | `solve3d/ray_cross.{h,c}` *(new)* | `lh2_bs_pose_t` + `ray_cross_solve(BS_POSES, horiz/vert per sensor) → points` | now |
| c | `angle_decoder.{h,c}` | add Bitcraze `(horiz,vert)` EMA output (current `az/el` elevation formula differs — angle_decoder.c:63) | now |
| d | `main_real.c` | `#include "bs_poses_cal.h"`; swap `solve3d_run` → `ray_cross_solve` | now |
| e | `main_real.c` (synthetic block) | generate sweep angles via Bitcraze forward model + the **same** synthetic poses the script bakes, so it round-trips | now |
| f | `CMakeLists.txt` | add `ray_cross.c` to both real + synthetic targets | now |

The synthetic-mode script (5a) and the synthetic firmware injector (5e) **share
the same hardcoded geometry** → closed loop, testable today without hardware.

---

## 6. Risks to confirm

1. **Quaternion order** — `lighthouse_geometry_types.py:107` documents `[x,y,z,w]`
   (scipy scalar-last) and feeds it to `Rotation.from_quat`, but `test_output.yaml`'s
   first element looks like a scalar `w`. Resolved by doing quat→R in Python; add
   the boresight sanity print (§2.2) to catch a wrong order.
2. **BS index ↔ geos key** — confirm `geos:0 ↔ poly 8/9 (BS4)` and
   `geos:1 ↔ poly 20/21 (BS10)`; the script comment must state the mapping it used.
3. **World → NED** — ray-crossing outputs the *calibration* world frame; the VPE
   to ArduPilot still needs the documented world→NED mapping (the z-handling we
   discussed). The script's world-frame printout makes this explicit to define.
4. **Import drift** — `calibrate_cli.py` imports `lbees.indoor.*`, which may not
   resolve standalone; the `--synthetic` path must avoid those imports so it runs
   today with just `numpy`/`scipy`.

---

## 7. Migration path

1. **Now (no hardware):**
   `python calibrate_export.py --synthetic -o bs_poses_cal.h`
   → build `crossing_beams_synthetic` → clean **metric** 1×1 m square out the VPE.
2. **When sensors arrive:**
   run `calibrate_cli.py` with the drone (origin / x-axis / xy-plane / free-move)
   → feed poses to `calibrate_export.py` → new `bs_poses_cal.h`
   → build `crossing_beams`. Same firmware, same format, real poses.
