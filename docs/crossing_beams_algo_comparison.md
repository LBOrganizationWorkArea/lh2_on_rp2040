# Crossing Beams — Algorithm Comparison

This document traces the lineage of the 3D positioning solver from the original Python
reference code through the lab's exploration scripts to the current RP2350 firmware.

---

## Lineage

```
alvarado23lighthouse/data_processing.py   ← original Python reference
        │                                    (OpenCV: FM + recoverPose + DLT)
        │
        ├─ ported to C + calibrated poses ──► rp2350_firmware/ (solve3d.c + cv.c)
        │                                      current firmware — best approach
        │
        └─ independent lab exploration ──────► origin/latest_crossing_beams (4 Python scripts)
                                               skew-lines crossing beam method
                                                    ↑
                                   same algorithm as Taffanel 2021 (arXiv:2104.11523)
```

---

## 1 — Shared pipeline (all versions)

Every implementation follows the same front-end:

```
LFSR counts  →  sweep angles (A·lfsr + B)  →  az/el  →  pixels  →  triangulation  →  centroid
```

### Angle decoding (identical everywhere)

```
angle_deg = A · lfsr + B          per sweep (linear calibration)
az        = (a0 + a1) / 2
diff_rad  = radians((a0 − a1) / 2)
el        = atan( tan(diff_rad) / tan(30°) / cos(az) )

Swap guard: if |a0 − a1| > 90°  →  diff = 2·(B0 − B1) − diff
```

### Pixel projection (identical in reference Python and firmware)

```
px = [ tan(az),   tan(el) / cos(az) ]      ← normalised +Z pinhole
```

`LH2_angles_to_pixels()` in the reference Python; `angles_to_pixels()` in `solve3d.c`.

### EMA smoothing

| | α |
|---|---|
| Python branch scripts | 0.15 |
| Firmware | 0.20 |

### Freshness / quality filtering

All versions discard observations older than **0.5 s**. The Python C.B. scripts additionally
reject a solution when the ray-gap `δ > 0.1 m` (Taffanel 2021 threshold).

---

## 2 — Original reference: `alvarado23lighthouse/data_processing.py`

The firmware's solver is a direct C port of this file. It uses OpenCV's full epipolar pipeline
to estimate the relative pose of the two base stations from accumulated point correspondences
and then triangulates.

```python
def solve_3d_scene(pts_a, pts_b):
    # Step 1 — estimate fundamental matrix from many point pairs
    F, _ = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)

    # Step 2 — recover relative rotation and translation (up to scale)
    _, R_star, t_star, _ = cv2.recoverPose(F, pts_a, pts_b)

    # Step 3 — build projection matrices
    P1 = np.hstack([I,          zeros])          # camera 1 at origin
    P2 = np.hstack([R_star.T,  -R_star.T @ t_star])   # camera 2

    # Step 4 — DLT triangulation
    # BUG: pts_b and pts_a are swapped → ~165 mm reconstruction error
    point3D = cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)
    return point3D[:3] / point3D[3]
```

**Limitations of this approach:**

| Issue | Effect |
|---|---|
| `findFundamentalMat` needs many simultaneous correspondences | Not per-frame; requires data accumulation |
| `recoverPose` gives pose up to scale | No absolute metric positions without external reference |
| `pts_b/pts_a` swap | ~165 mm systematic reconstruction error (documented in branch test suite) |
| No real BS position knowledge | Output is in an arbitrary relative frame |

---

## 3 — Current firmware: calibrated DLT (`solve3d.c` + `cv.c`)

The firmware replaces both `findFundamentalMat + recoverPose` with pre-computed projection
matrices built from the calibrated base-station poses in `bs_poses_cal.h`, and fixes the
pts swap bug. Everything else is a faithful C port of the reference.

### 3.1 Projection matrix from calibrated poses (`solve3d.c:_bs_projection`)

```c
// BS-local +X = boresight; +Y = horiz sweep; +Z = vert sweep
for (int r = 0; r < 3; r++) {
    P[0][r] = R[r][1];   // u ← local +Y
    P[1][r] = R[r][2];   // v ← local +Z
    P[2][r] = R[r][0];   // depth ← local +X (boresight)
}
P[0][3] = -(R[:,1] · origin)
P[1][3] = -(R[:,2] · origin)
P[2][3] = -(R[:,0] · origin)
```

`P · [X_world; 1]` = `(tan horiz, tan vert, depth)` — exactly what `angles_to_pixels()` produces.

Gains over the reference:
- **Absolute metric scale** — comes from the calibration, not from data
- **Per-frame** — no need to accumulate many point correspondences
- **Known world frame** — output is directly in the calibrated world coordinate system

### 3.2 Fixed point pairing (`solve3d.c:solve3d_calib_run`)

```c
angles_to_pixels(h0, el0, pa[0]);   // BS0 → pa
angles_to_pixels(h1, el1, pb[0]);   // BS1 → pb
triangulate_points(P0, P1, pa, pb, 1, X);  // pa with P0, pb with P1  ✓
```

### 3.3 DLT triangulation (`cv.c`)

Pure-C Jacobi SVD (derived from OpenCV, no heap allocation):

```
4×4 system — one constraint per pixel coordinate per base station:
  A[0] = u0·P0[2] − P0[0]
  A[1] = v0·P0[2] − P0[1]
  A[2] = u1·P1[2] − P1[0]
  A[3] = v1·P1[2] − P1[1]

Solve A·X = 0  via Jacobi SVD (max 30 iterations)
Solution: last row of Vt (null vector), dehomogenised by X[3]
```

---

## 4 — Python branch: crossing beam / skew-lines (`origin/latest_crossing_beams`)

Four scripts written independently by the lab team. They implement the **Crossing Beam (C.B.)
method** from Taffanel 2021 (Section II-B), which is geometrically different from the DLT
approach.

### 4.1 Core algorithm

Instead of projecting to a pinhole image plane, the C.B. method builds a 3D unit ray in
spherical coordinates and finds the closest point between two skew lines:

```python
# Step 1 — spherical unit ray
vx = sin(az) * cos(el)
vy = cos(az) * cos(el)
vz = sin(el)

# Step 2 — optional: rotate ray by BS Euler angles (Z-X-Y)
d = rotate_zxy(v, yaw, pitch, roll)

# Step 3 — find closest point between two skew lines (Taffanel eq. 2–3)
b = d1 · d2
t1 = (b·e − d_dot) / (1 − b²)
t2 = (e − b·d_dot) / (1 − b²)

c1 = o1 + t1·d1
c2 = o2 + t2·d2
p_s = (c1 + c2) / 2           # position estimate
δ   = ‖c1 − c2‖              # ray-gap quality metric
```

Filter: discard if `δ > 0.1 m` (recommended in Taffanel 2021).

### 4.2 Four variants

| File | BS positions | Rotation applied | Notes |
|---|---|---|---|
| `crossing_beams.py` | (0,0,0) and (1,0,0) | None | Baseline, parallel beams |
| `2crossing_beams.py` | (0,0,−0.5) and (1,0,−0.5) | Euler Z-X-Y per BS | Full terminal display |
| `for_calibration.py` | (−0.5,0,0) and (+0.5,0,0) | ±1.5° yaw correction | Manual fine-tuning variant |
| `edu_cbeams.py` | Placeholders (identity R) | Full `R @ d` (numpy) | Different serial format and poly→BS mapping |

### 4.3 `edu_cbeams.py` specifics

- **Serial format**: Said's raw output — `{sensor_id=N} || {base=N} || {poly=N} || {lfsr=N}`
- **Poly→BS mapping**: `bs = poly // 2`, `sweep = poly % 2`
  (differs from firmware: `poly ∈ {8,9} → BS0`, `poly ∈ {20,21} → BS1`)
- **Vert formula**: `vert = atan(sin(dt) / tan30° · sqrt(1 + tan²(h)))`
  — algebraically equivalent to the firmware formula for small angles but sign convention may differ
- **No EMA** — computes once per complete 4-LFSR set and resets buffer
- **BS poses are placeholder** — file explicitly warns this must be replaced with real values

---

## 5 — Taffanel 2021 benchmark (arXiv:2104.11523)

This paper measures the same C.B. method and an EKF on LH2 hardware. Key results:

### Precision (jitter, stationary)

| Method | LH1 | LH2 |
|---|---|---|
| Crossing Beam | **0.6 mm** | **0.3 mm** |
| EKF | 3.9 mm | 0.7 mm |
| MoCap (reference) | 0.1 mm | 0.1 mm |

C.B. is ~5× more precise than EKF.

### Accuracy (mean Euclidean error, flight)

Both methods achieve **1–4 cm** mean error with outliers up to ~5 cm. LH2 > LH1.

### EKF measurement model (not implemented here, documented for reference)

Each raw sweep plane `p` is used directly as an EKF measurement:

```
α_p = arctan(y_s / x_s)  +  arcsin( z_s·tan(t_p) / sqrt(x_s² + y_s²) )

where:
  (x_s, y_s, z_s) = EKF state position of sensor s
  t_p             = tilt angle of light plane (0 for LH1; ±π/6 for LH2)
  r_s             = sqrt(x_s² + y_s²)

Jacobian:
  g_p = ( (−y_s − x_s·z_s·q_p)/r_s²,
           (x_s  − y_s·z_s·q_p)/r_s²,
           q_p )
  where q_p = tan(t_p) / sqrt(r_s² − (z_s·tan(t_p))²)

Rotated to global frame:
  g'_p = R_b · R_d⁻¹ · g_p
```

The EKF updates on **any single sweep plane from any single BS** — more robust under
partial occlusion, but at the cost of 5× worse jitter.

---

## 6 — What is the best way to obtain positions

For this system (RP2350, no IMU fusion, MAVLink ODOMETRY to Pixhawk) the recommendation is:

**Use the current firmware DLT approach.** It is strictly better than the Python C.B. scripts:

| Criterion | Python C.B. | Firmware DLT |
|---|---|---|
| Scale | Approximate (hard-coded constants) | Absolute metric (from calibration) |
| World frame | Arbitrary | Calibrated |
| Per-frame? | Yes | Yes |
| Algebraic optimality | Geometric midpoint | SVD optimal point |

The only reason the firmware currently gives wrong positions is that `bs_poses_cal.h` contains
**synthetic placeholder geometry** instead of real calibrated poses. Fix:

```bash
# 1. Export real calibration to firmware header
python utils/calibration/calibrate_export.py \
    --yaml utils/calibration/lab.yaml \
    -o rp2350_firmware/src/bs_poses_cal.h

# 2. Rebuild both targets
make -C rp2350_firmware/src/build crossing_beams crossing_beams_synthetic -j$(nproc)
```

The EKF from Taffanel 2021 could be added later if partial-occlusion robustness becomes
a requirement, but it would not improve accuracy — only visibility coverage.

---

## 7 — Full comparison table

| Aspect | Reference Python | Python branch (C.B.) | Firmware (DLT) |
|---|---|---|---|
| **Triangulation** | FM + recoverPose + DLT | Skew-lines midpoint | DLT SVD |
| **BS pose source** | Estimated from data | Hard-coded (approximate) | Calibrated `origin + R[3×3]` |
| **Scale** | Relative | Set by position constants | Absolute metric |
| **Per-frame** | No (needs batch) | Yes | Yes |
| **Pixel formula** | `[tan az, tan el / cos az]` ✓ | Spherical unit vector | `[tan h, tan v · cos h]` ✓ |
| **pts swap bug** | Yes | N/A | Fixed |
| **EMA α** | None | 0.15 | 0.20 |
| **Ray-gap filter** | None | `δ < 0.1 m` | None |
| **Jitter (Taffanel)** | — | 0.3–0.6 mm | — |
| **Flight accuracy** | — | ~1–4 cm mean | — |
| **Language** | Python (OpenCV) | Python (pure math) | C (no heap, no OpenCV) |
| **Output** | Offline CSV | Terminal | MAVLink ODOMETRY @ 10 Hz |

---

## References

- Taffanel et al., "Lighthouse Positioning System: Dataset, Accuracy, and Precision for UAV
  Research," arXiv:2104.11523, 2021 — benchmarks C.B. vs EKF; equations 1–9
- Alvarado et al., `alvarado23lighthouse` — original Python reference implementation
  (`twoLH-3D_scene/functions/data_processing.py`)
- `rp2350_firmware/src/solve3d/solve3d.c` — firmware DLT solver
- `rp2350_firmware/src/cv/cv.c` — pure-C Jacobi SVD
- `rp2350_firmware/src/angle_decoder/angle_decoder.c` — LFSR → az/el pipeline
- `origin/latest_crossing_beams` — lab Python exploration scripts
