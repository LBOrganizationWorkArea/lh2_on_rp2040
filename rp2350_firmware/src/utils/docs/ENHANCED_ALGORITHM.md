# The Enhanced Solver — calibrated-pose triangulation

> **Module:** `solve3d/solve3d.c` (`solve3d_calib_run`) + `cv/cv.c` (`triangulate_points`)

This is the algorithm that turns two base-station angle measurements into a
metric 3D point. It is an **enhancement of the original** epipolar pipeline, not
a replacement: it keeps the parts that were correct and paper-validated, and
fixes only the part that was broken.

---

## 1. What the original did, and why it was unstable

The original `solve_3d_scene` (ported from `data_processing.py`) ran a classic
two-view structure-from-motion pipeline:

```
angles_to_pixels → find_fundamental_mat → recover_pose → triangulate_points
                   └──────────── pose ESTIMATION ────────┘   └─ triangulation ─┘
```

It **estimated** the relative pose `(R, t)` between the two base stations from
the point correspondences (the fundamental matrix). Two problems for this rig:

1. **Scale ambiguity.** The fundamental matrix recovers `t` only up to scale
   (epipolar geometry cannot determine baseline length). The triangulated points
   therefore had an arbitrary, per-frame scale.
2. **Ill-conditioning.** The four sensors are nearly coplanar and barely move,
   which is a near-degenerate configuration for the 8-point algorithm, so the
   estimated pose — and hence the scale and sign — jumped frame to frame.

That is what produced the observed "position swinging 0→170 in a second."

**Crucial observation:** the triangulation step (`triangulate_points`, a DLT) was
never the problem, and the base stations are *fixed and surveyed*. So we should
not be estimating their pose at all.

---

## 2. The enhancement

Keep the validated front/back of the pipeline, drop the estimation in the middle:

```
KEEP    angles_to_pixels            (projection)
KEEP    triangulate_points          (DLT triangulation — the OpenCV-derived core)
REPLACE find_fundamental_mat + recover_pose
   WITH projection matrices built directly from the CALIBRATED poses
```

Because the poses come from the calibration pipeline — which is **metric**
(scaled by a known reference distance) and **world-aligned** — the triangulated
points come out metric and stable. No history buffer, no per-frame estimation.

This is the same operation as "find where the two beams cross," but expressed in
the projective-camera formalism the original code already used, so the
paper-tested DLT does the actual work.

---

## 3. Conventions

### Base-station pose (from calibration)

```c
typedef struct { float origin[3]; float R[3][3]; } lh2_bs_pose_t;
```
`R` maps the base-station-local frame to the world frame (its columns are the
local axes in world coordinates). **Local +X is the boresight** (Bitcraze
convention), local +Y / +Z are the horizontal / vertical sweep axes.

### Angles

`angle_decoder` provides EMA-filtered Bitcraze angles per (sensor, base station):
```
horiz = atan2(local_y, local_x)        vert = atan2(local_z, local_x)
```
For a world point `X`, with `rel = R^T·(X − origin)` giving its local coords,
those are the horizontal/vertical angles the base station observes.

---

## 4. Step by step (`solve3d_calib_run`)

For each sensor that has fresh angles from **both** base stations:

### 4.1 Project to image pixels

The original projection (`angles_to_pixels`) is a z=1 pinhole:
```
angles_to_pixels(az, el) = ( tan(az),  tan(el)/cos(az) )
```
The calibrated angles are Bitcraze `(horiz, vert)` with boresight along +X. The
elevation fed to the pinhole is therefore
```
el = atan( tan(vert) · cos(horiz) )
```
so that `angles_to_pixels(horiz, el) = (tan horiz, tan vert)` — exactly the
projection the matrices below invert. (Reusing `angles_to_pixels` rather than
hand-writing the projection keeps the original, documented function in the loop.)

### 4.2 Build each base station's world→image projection matrix

A pinhole expects the optical (depth) axis to be the third image coordinate, but
the boresight is local +X, so we permute the pose axes:

```
P row 0 (image u) ← R column 1   (local +Y)
P row 1 (image v) ← R column 2   (local +Z)
P row 2 (depth)   ← R column 0   (local +X, boresight)
P[:,3]            ← −(axis · origin)   for each row
```

For any world point `X`:
```
P · [X; 1] = ( local_y, local_z, local_x )   of (X − origin)
projected   = ( local_y/local_x, local_z/local_x ) = ( tan horiz, tan vert )
```
which matches the pixel from §4.1 — the matrix and the projection are consistent
by construction.

### 4.3 Triangulate (unchanged DLT)

```
triangulate_points(P0, P1, px0, px1)  →  X  (world coordinates)
```
`triangulate_points` builds the 4×4 DLT system from the two projection
constraints and takes its null vector via Jacobi SVD — the OpenCV-derived
routine, used verbatim. Because `P0`/`P1` map **world** points to images, the
result is directly in the world frame; no back-transform is needed.

4 sensors → 4 world points → centroid → MAVLink VPE.

---

## 5. Why it's metric and stable (vs. the original)

| | Original (fundamental matrix) | Enhanced (calibrated poses) |
|---|---|---|
| Relative pose | estimated every frame (up to scale) | **known**, metric, fixed |
| Scale | arbitrary / drifting | **real metres** (calibration reference) |
| Conditioning | degenerate for coplanar sensors | not applicable — no estimation |
| History needed | 8–32 samples | **1 sample per base station** |
| Reused validated code | `triangulate_points`, `angles_to_pixels` | **same** `triangulate_points`, `angles_to_pixels` |
| Changed | — | only the pose source |

---

## 6. Offline verification

The projection-matrix + DLT convention was checked against the synthetic
geometry (two stations 1 m apart, both boresight +Z, body on a 1×1 m square at
z = 2 m): the triangulation recovers each known point exactly, and the computed
angles match the firmware's serial output (e.g. `BS0 h=0.00 v=−5.99 | BS1
v=21.55` for the point `(0.21, 0, 2)`). On-chip, the `crossing_beams_synthetic`
build closes the same loop in real time — see `SYNTHETIC_CAPTURE.md`.

---

## 7. What still needs real-hardware calibration

- **World → NED mapping** for MAVLink (only the Z sign is handled today).
- **BS index ↔ calibration key** mapping (`geos:0 ↔ poly 8/9`, etc.).
- The small modelling difference between the angle decoder's reconstruction and
  the calibration's angle convention is exact in synthetic mode (shared model);
  on hardware it is a calibration nuance to validate.
