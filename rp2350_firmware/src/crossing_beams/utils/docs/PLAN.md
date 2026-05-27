# Crossing Beams — 3D Scene Solver for RP2350
## Implementation Plan

---

## 1. Goal

Port the full Python `solve_3d_scene` pipeline from
`utils/user_interface/saids_implementation/data_processing.py`
to **pure C**, running on the RP2350 with no host PC, no cv2, no numpy.

The algorithm must be a faithful translation of the Python version:

```python
# Python reference (data_processing.py)
pixel_a = LH2_angles_to_pixels(az_a, el_a)   # project lighthouse A angles → 2D point
pixel_b = LH2_angles_to_pixels(az_b, el_b)   # project lighthouse B angles → 2D point

point3d, t, R = solve_3d_scene(pts_a, pts_b)  # fundamental matrix → R,t → triangulate
```

The `crossing_beams/` directory is **fully self-contained**: it carries its own
copy of the lh2 hardware layer rather than linking against the rest of the
firmware tree. The only external dependencies are the Pico SDK and the standard
C library.

---

## 2. What Gets Copied from `src/lh2/`

All six files from the existing lh2 library are copied verbatim into
`crossing_beams/lh2/`. **No modifications to any of these files.**

| Source file | What it does |
|---|---|
| `lh2.h` | Public API: `db_lh2_t`, `db_lh2_init()`, `db_lh2_process_location()` |
| `lh2.c` | Hardware layer: PIO init, DMA rings, calls into `lh2_decoder` |
| `lh2_decoder.h` | Internal decoder API: `_demodulate_light()`, `_determine_polynomial()`, `_lfsr_index_search()` |
| `lh2_decoder.c` | Bit demodulation, polynomial search, LFSR index search |
| `lh2_checkpoints.h` | Pre-computed LFSR checkpoint table (lookup acceleration) |
| `ts4231_capture.pio` | PIO program for TS4231 light capture (compiled by pioasm) |

### Why copy instead of linking against `lh2_lib`?

- `crossing_beams` becomes a drop-in standalone project — buildable and
  testable independently without touching any other part of the firmware tree.
- Avoids CMake target dependency chains that would force a full top-level
  rebuild whenever `crossing_beams` changes.
- Makes the include paths inside `crossing_beams/` unambiguous: everything
  in `lh2/` is local.

### What the copied lh2 layer delivers

```
TS4231 sensor
    │  (PIO DMA capture — ts4231_capture.pio)
    ▼
lh2.c  →  db_lh2_init() / db_lh2_process_location()
    │
    │  internal calls:
    ├── lh2_decoder.c :: _demodulate_light()       SPI bytes → 64 demodulated bits
    ├── lh2_decoder.c :: _determine_polynomial()   bits → which of the 32 LFSR polynomials
    └── lh2_decoder.c :: _lfsr_index_search()      17-bit seed → LFSR count
    │
    ▼
db_lh2_t.locations[sweep][slot]
    .selected_polynomial   (0–31, identifies which lighthouse and which sweep plane)
    .lfsr_location         (raw count — the number we feed into calibration)
    .data_ready            (flag: new unread data available)
```

---

## 3. Proposed File Structure

```
rp2350_firmware/src/crossing_beams/
│
├── PLAN.md                          ← this document
├── CMakeLists.txt                   ← standalone build target "crossing_beams"
├── main.c                           ← entry point; wires all modules together
│
├── lh2/                             ← COPIED verbatim from src/lh2/
│   ├── lh2.h
│   ├── lh2.c
│   ├── lh2_decoder.h
│   ├── lh2_decoder.c
│   ├── lh2_checkpoints.h
│   └── ts4231_capture.pio
│
├── angle_decoder/                   ← NEW: C port of utils/angle_lib/angle_decoder.py
│   ├── angle_decoder.h
│   └── angle_decoder.c
│
├── cv/                              ← NEW: C equivalents of the three cv2 functions used
│   ├── cv.h
│   └── cv.c
│
└── solve3d/                         ← NEW: C port of data_processing.py::solve_3d_scene()
    ├── solve3d.h
    └── solve3d.c
```

### Dependency graph

```
main.c
  ├── lh2/lh2.h                           (db_lh2_t, db_lh2_init, db_lh2_process_location)
  │       ├── lh2/lh2.c                   (hardware init, PIO DMA, calls lh2_decoder)
  │       │       ├── lh2/lh2_decoder.c   (bit decode, poly search, LFSR search)
  │       │       ├── lh2/lh2_checkpoints.h
  │       │       └── lh2/ts4231_capture.pio
  │       └── lh2/lh2_decoder.h
  │
  ├── angle_decoder/angle_decoder.h        (reads db_lh2_t → emits ema_az, ema_el)
  │       └── lh2/lh2.h                   (db_lh2_t type only)
  │
  └── solve3d/solve3d.h                    (consumes az/el → emits 3D points)
          └── cv/cv.h                       (find_fundamental_mat, recover_pose, triangulate)
                  └── <math.h>              (sinf, cosf, sqrtf — hardware FPU on RP2350)
```

All arrows point inward: no module in `crossing_beams/` includes anything
from outside this directory.

---

## 4. Module Details

---

### 4.1 `lh2/` — Hardware + LFSR layer (copied, unmodified)

These files are the hardware foundation. They are **not modified**. The only
thing that changes relative to the original is the include paths used by
`angle_decoder.c` and `main.c`, which reference `"lh2/lh2.h"` (local) instead
of the `lh2_lib` CMake target.

For reference, the key output from this layer is the `db_lh2_t` struct:

```c
// From lh2/lh2.h (copied verbatim)
typedef struct {
    db_lh2_raw_data_t  raw_data [LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];
    db_lh2_location_t  locations[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];
    //                  .selected_polynomial  (0–31)
    //                  .lfsr_location        (raw count)
    absolute_time_t    timestamps[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];
    db_lh2_data_ready_state_t data_ready[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];
    uint8_t            sensor;
} db_lh2_t;
```

The two public functions called by `main.c`:

```c
void db_lh2_init(db_lh2_t *lh2, uint8_t sensor, uint8_t gpio_d, uint8_t gpio_e);
void db_lh2_process_location(db_lh2_t *lh2);
```

---

### 4.2 `angle_decoder/` — LFSR counts → azimuth + elevation

**Python reference:** `utils/angle_lib/angle_decoder.py`

**Responsibility:** Sits directly on top of the lh2 layer. Reads
`db_lh2_t.locations`, applies linear calibration coefficients to convert LFSR
counts to sweep angles, reconstructs azimuth and elevation from a pair of
sweeps, and applies an EMA filter. Exposes smooth `(ema_az, ema_el)` for every
`(sensor, basestation)` pair.

**Key types:**

```c
// Calibration for one basestation
// angle_deg = A0 * lfsr + B0  (sweep 0)
// angle_deg = A1 * lfsr + B1  (sweep 1)
typedef struct {
    float A0, B0;
    float A1, B1;
} lh2_cal_t;

// Decoded + filtered angles for one (sensor × basestation) pair
typedef struct {
    float    raw_sweep[2];      // pending raw angles, NaN until filled
    bool     has_sweep[2];      // whether each sweep has arrived
    float    ema_az;            // EMA-smoothed azimuth   [degrees]
    float    ema_el;            // EMA-smoothed elevation [degrees]
    bool     valid;             // true once at least one complete pair decoded
    uint64_t last_update_us;    // timestamp of last successful decode [µs]
} lh2_angles_t;
```

**Key functions:**

```c
void angle_decoder_init(lh2_angles_t out[NUM_SENSORS][NUM_BS],
                        const lh2_cal_t cal[NUM_BS]);

// Call every loop iteration after db_lh2_process_location().
// Scans all data_ready slots, converts, filters, clears flags.
void angle_decoder_update(db_lh2_t         lh2[NUM_SENSORS],
                          lh2_angles_t     out[NUM_SENSORS][NUM_BS],
                          const lh2_cal_t  cal[NUM_BS],
                          uint64_t         now_us);
```

**Algorithm inside `angle_decoder_update`:**

```
poly → basestation index mapping (from angle_decoder.py):
    poly 8  or 9  → bs index 0  (physical BS 4)
    poly 20 or 21 → bs index 1  (physical BS 10)
    anything else → skip

For each sensor s, each sweep, each slot:
    if data_ready != RAW_DATA_AVAILABLE: continue
    poly  = locations[sweep][slot].selected_polynomial
    bs    = poly_to_bs_index(poly)          // skip if -1
    lfsr  = locations[sweep][slot].lfsr_location

    raw_angle = cal[bs].A{sweep} * lfsr + cal[bs].B{sweep}
    store in out[s][bs].raw_sweep[sweep], set has_sweep[sweep]

    if has_sweep[0] AND has_sweep[1]:
        a0 = raw_sweep[0],  a1 = raw_sweep[1]
        az_raw = (a0 + a1) / 2.0
        diff   = a0 - a1
        if |diff| > 90°:
            diff = 2*(cal[bs].B0 - cal[bs].B1) - diff    // swap guard

        diff_rad = radians(diff / 2)
        az_rad   = radians(az_raw)
        el_raw   = degrees( atan( tan(diff_rad) / TAN_30 / cos(az_rad) ) )

        EMA:
            ema_az = α * az_raw + (1-α) * ema_az
            ema_el = α * el_raw + (1-α) * ema_el

        mark valid, store last_update_us, clear has_sweep[0..1]

    data_ready = NO_NEW_DATA
```

**Constants:**
- `TAN_30 = 0.57735027f`
- `EMA_ALPHA = 0.2f`

---

### 4.3 `cv/` — C equivalents of the three cv2 calls

**Python reference:** `cv2.findFundamentalMat`, `cv2.recoverPose`,
`cv2.triangulatePoints` as called in `data_processing.py :: solve_3d_scene()`

This module implements exactly those three functions in C, plus the mathematical
primitives they depend on. The algorithms below are derived directly from the
OpenCV 4.x source code (see §12 for links).

---

#### 4.3.0 Mathematical primitives (internal, `static`)

These are the building blocks for everything above them.

##### `jacobi_svd_3x3(A, U, S, Vt)`
**Source:** `opencv/modules/core/src/lapack.cpp` — `JacobiSVDImpl_`

OpenCV uses a **one-sided Jacobi SVD**: it operates on the columns of `Aᵀ`
(stored as rows), applying Givens plane rotations to pairs of columns until
all off-diagonal inner products are below a convergence threshold.

```
Input:  A[3][3]
Output: U[3][3],  S[3] (singular values, descending),  Vt[3][3]

Initialise: At = Aᵀ,  V = I₃,  W[i] = dot(At[i], At[i])  (column norms²)

Outer loop (max 30 iterations, break if no pair changed):
  For each column pair (i, j), i < j:
    p = dot(At[i], At[j])
    if |p| ≤ eps * sqrt(W[i] * W[j]):  continue   // already orthogonal

    // 2×2 symmetric eigenvalue subproblem
    beta  = W[i] - W[j]
    gamma = hypot(2*p, beta)
    if beta < 0:
        s = sqrt( (gamma - beta) / (2*gamma) )
        c = p / (gamma * s)
    else:
        c = sqrt( (gamma + beta) / (2*gamma) )
        s = p / (gamma * c)

    // Apply Givens rotation to columns i and j of At and V
    for k in 0..2:
        t0 = At[i][k];  t1 = At[j][k]
        At[i][k] =  c*t0 + s*t1
        At[j][k] = -s*t0 + c*t1
    for k in 0..2:
        t0 = V[k][i];  t1 = V[k][j]
        V[k][i] =  c*t0 + s*t1
        V[k][j] = -s*t0 + c*t1

    W[i] = dot(At[i], At[i]);  W[j] = dot(At[j], At[j]);  changed = true

S[i]    = sqrt(W[i])            (singular values)
U[:,i]  = At[i] / S[i]          (left singular vectors, columns of U)
Vt[i,:] = V[:,i]                (rows of Vt = columns of V)
```

Convergence threshold: `eps = 2^-23` (single precision machine epsilon).  
Same loop structure handles any `n×n`; instantiated for n=3 and n=4.

---

##### `sym_mineig_9(M[9][9], v[9])`
**Source:** `opencv/modules/core/src/lapack.cpp` — same `JacobiSVDImpl_` pattern,
applied to a 9×9 symmetric matrix formed as `AᵀA`.

Finds the eigenvector of the symmetric matrix `M` corresponding to its
**smallest eigenvalue** (= null vector of the original design matrix `A`).

```
Run jacobi_svd_9 on the matrix M viewed as A = M, skip computing U.
The right singular vector corresponding to the smallest singular value
of M (= smallest eigenvalue, since M is symmetric positive-semidefinite)
is the last row of Vt.
→ v = Vt[8]   (zero-indexed last row)
```

In practice, since M is only 9×9, the Jacobi loop converges in ≤ 5 sweeps.

---

##### Matrix helpers (all inline, operate on stack arrays)

```
mat3_mul(A, B, C)          C = A×B                (3×3 × 3×3)
mat3_transpose(A, At)      component-wise transpose
mat3_det(A)                cofactor expansion, returns float
mat3_diag_mul(U, S, Vt, R) R = U * diag(S) * Vt   (avoids building diag matrix)
mat34_mul_vec4(P, X, y)    y = P×X                (3×4 × 4×1 → 3×1)
vec3_cross(a, b, c)        c = a × b
vec3_dot(a, b)             returns float
vec3_norm(v)               returns float
vec3_normalize(v)          in-place, v /= ||v||
```

---

#### 4.3.1 `find_fundamental_mat(pts_a, pts_b, n, F_out)`

**Replaces:** `cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)`  
**Source:** `opencv/modules/calib3d/src/fundam.cpp` — `FundamentalMat8Point::run8Point()`

```
Inputs : pts_a[n][2], pts_b[n][2]   — n ≥ 8 pixel-projected point pairs
Output : F_out[3][3]                — 3×3 fundamental matrix

Step 1 — Isotropic Hartley normalisation (identical to OpenCV fundam.cpp)

    // Centre each cloud on the origin
    cx1 = mean(pts_a[:,0]),  cy1 = mean(pts_a[:,1])
    cx2 = mean(pts_b[:,0]),  cy2 = mean(pts_b[:,1])

    // Scale so mean distance to centroid = sqrt(2)
    scale1 = sqrt(2) / mean_i( sqrt((pts_a[i][0]-cx1)² + (pts_a[i][1]-cy1)²) )
    scale2 = sqrt(2) / mean_i( sqrt((pts_b[i][0]-cx2)² + (pts_b[i][1]-cy2)²) )

    T1 = [[scale1,      0, -scale1*cx1],
          [     0, scale1, -scale1*cy1],
          [     0,      0,           1]]

    T2 = [[scale2,      0, -scale2*cx2],
          [     0, scale2, -scale2*cy2],
          [     0,      0,           1]]

    x1_n[i] = (pts_a[i][0] - cx1) * scale1
    y1_n[i] = (pts_a[i][1] - cy1) * scale1
    x2_n[i] = (pts_b[i][0] - cx2) * scale2
    y2_n[i] = (pts_b[i][1] - cy2) * scale2

Step 2 — Accumulate 9×9 normal equations matrix (direct from OpenCV source)

    M[9][9] = 0
    for i in 0..n:
        r[9] = { x2*x1, x2*y1, x2,
                 y2*x1, y2*y1, y2,
                  x1,    y1,   1  }    // where x1=x1_n[i], etc.
        M += outer_product(r, r)       // M = Aᵀ A, built without storing A

Step 3 — Null vector via smallest eigenvector of M

    sym_mineig_9(M, f)         // f[9] = eigenvector for smallest eigenvalue
    F_full[3][3] = reshape(f)  // row-major

Step 4 — Enforce rank 2 (identical to OpenCV)

    jacobi_svd_3x3(F_full, U, S, Vt)
    S[2] = 0.0f                        // zero the smallest singular value
    mat3_diag_mul(U, S, Vt, F_norm)   // F_norm = U * diag(S) * Vt

Step 5 — Denormalise (identical to OpenCV)

    F_out = T2ᵀ * F_norm * T1
    F_out /= F_out[2][2]               // normalise so F[2][2] = 1
```

> **FM_LMEDS vs. 8-point:**  
> OpenCV's FM_LMEDS wraps the 8-point kernel inside a robust LMedS estimator
> that draws random 8-point subsets, scores each with the symmetric epipolar
> distance (below), and picks the model with the lowest median error.
> Error per point from `FMEstimatorCallback::computeError()` in `fundam.cpp`:
> ```
> a2 = F[0]*x1 + F[1]*y1 + F[2]
> b2 = F[3]*x1 + F[4]*y1 + F[5]
> c2 = F[6]*x1 + F[7]*y1 + F[8]
> d2 = x2*a2 + y2*b2 + c2          // epipolar line residual for point 2
>
> a1 = F[0]*x2 + F[3]*y2 + F[6]
> b1 = F[1]*x2 + F[4]*y2 + F[7]
> c1 = F[2]*x2 + F[5]*y2 + F[8]
> d1 = x1*a1 + y1*b1 + c1          // epipolar line residual for point 1
>
> err = max(d1² / (a1²+b1²),  d2² / (a2²+b2²))   // symmetric epipolar distance
> ```
> The C implementation starts with the deterministic 8-point. A lightweight
> RANSAC/LMedS wrapper can be added later via `pico_rand()` if needed.

---

#### 4.3.2 `recover_pose(F, pts_a, pts_b, n, R_out, t_out)`

**Replaces:** `cv2.recoverPose(F, pts_a, pts_b)`  
**Source:** `opencv/modules/calib3d/src/five-point.cpp` — `decomposeEssentialMat` + `recoverPose`

```
Inputs : F[3][3], pts_a[n][2], pts_b[n][2]
Outputs: R_out[3][3], t_out[3]  — unit translation

Since K = I  →  E = F

Step 1 — Clean up E so its two non-zero singular values are equal

    jacobi_svd_3x3(E, U, S, Vt)
    s      = (S[0] + S[1]) * 0.5f
    S[0]   = S[1] = s
    S[2]   = 0.0f
    mat3_diag_mul(U, S, Vt, E)   // recompose clean E

Step 2 — Build W matrix (from OpenCV decomposeEssentialMat)

    W = [[ 0, -1,  0],
         [ 1,  0,  0],
         [ 0,  0,  1]]

Step 3 — Four candidate (R, t) pairs (from OpenCV decomposeEssentialMat)

    R1 = U * W  * Vt          // mat3_mul twice
    R2 = U * Wᵀ * Vt
    t_pos[3] = { U[0][2], U[1][2], U[2][2] }   // third column of U
    t_neg[3] = { -t_pos[0], -t_pos[1], -t_pos[2] }

    // Fix reflection: det(R) must be +1
    if mat3_det(R1) < 0:  negate all elements of R1
    if mat3_det(R2) < 0:  negate all elements of R2

    Candidates = { (R1, t_pos), (R1, t_neg), (R2, t_pos), (R2, t_neg) }

Step 4 — Cheirality check (from OpenCV recoverPose)

    best_count = -1
    for each candidate (R, t):
        P1[3×4] = [I₃ | 0]                     // camera A at origin
        P2[3×4] = [Rᵀ | -(Rᵀ t)]              // camera B

        count = 0
        for i in 0..n:
            triangulate one point pair → X_h[4]  // single-point DLT
            X = X_h[0..2] / X_h[3]

            depth1 = X[2]                        // Z in camera-A frame
            depth2 = (R[2][0]*X[0] + R[2][1]*X[1] + R[2][2]*X[2]) + t[2]

            if depth1 > 0 AND depth2 > 0:  count++

        if count > best_count:
            best_count = count
            R_out = R,  t_out = t
```

---

#### 4.3.3 `triangulate_points(P1, P2, pts_a, pts_b, n, pts3d_out)`

**Replaces:** `cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)`  
**Source:** `opencv/modules/calib3d/src/triangulate.cpp` — `triangulateCorrPoints()`

> **Point ordering fix:** The Python source has a documented FIXME — `pts_a`
> and `pts_b` are swapped relative to their projection matrices. The C
> implementation uses the **correct** order: `pts_a` with `P1`, `pts_b` with `P2`.

```
Inputs : P1[3][4], P2[3][4], pts_a[n][2], pts_b[n][2]
Output : pts3d_out[n][3]

For each i in 0..n:
    x1 = pts_a[i][0],  y1 = pts_a[i][1]
    x2 = pts_b[i][0],  y2 = pts_b[i][1]

    // Build 4×4 DLT matrix  (direct from OpenCV triangulate.cpp)
    // OpenCV iterates k=0..3 as column index of P, producing a 4×4
    // matrix whose columns are the four homogeneous-point components.
    // Equivalent standard form — rows are:
    //   row 0:  x1*P1[2,:] - P1[0,:]
    //   row 1:  y1*P1[2,:] - P1[1,:]
    //   row 2:  x2*P2[2,:] - P2[0,:]
    //   row 3:  y2*P2[2,:] - P2[1,:]

    for k in 0..3:
        A[k][0] = x1*P1[2][k] - P1[0][k]
        A[k][1] = y1*P1[2][k] - P1[1][k]
        A[k][2] = x2*P2[2][k] - P2[0][k]
        A[k][3] = y2*P2[2][k] - P2[1][k]

    // Null vector of A via SVD
    // OpenCV calls hal::SVD64f and takes the last row of V.
    // We use our jacobi_svd_4x4; last row of Vt is the null vector.
    jacobi_svd_4x4(A, U, S, Vt)
    Xh[4] = Vt[3]    // last row of Vt = right singular vector for smallest S

    // Euclidean divide (homogeneous → Cartesian)
    pts3d_out[i][0] = Xh[0] / Xh[3]
    pts3d_out[i][1] = Xh[1] / Xh[3]
    pts3d_out[i][2] = Xh[2] / Xh[3]
```

---

### 4.4 `solve3d/` — `solve_3d_scene` in C

**Python reference:** `data_processing.py :: LH2_angles_to_pixels()` and `solve_3d_scene()`

**Responsibility:** Takes smoothed `(az, el)` angle pairs from `angle_decoder`,
projects them onto the z=1 image plane, accumulates a history buffer, then
runs the full epipolar pipeline and returns 3D points.

**Key types:**

```c
// One measurement: both lighthouses saw the same sensor at the same time
typedef struct {
    float   px_a[2];    // [tan(az_a), tan(el_a)/cos(az_a)]
    float   px_b[2];    // [tan(az_b), tan(el_b)/cos(az_b)]
    uint8_t sensor_id;
} lh2_sample_t;

// One 3D output point
typedef struct {
    float   xyz[3];     // metres (scale ambiguous until D_BS known)
    uint8_t sensor_id;
} lh2_point3d_t;

// Solver context — ring buffer + cached pose
#define SOLVE3D_MAX_SAMPLES 32
typedef struct {
    lh2_sample_t history[SOLVE3D_MAX_SAMPLES];
    int          n_samples;     // current fill level (0..MAX_SAMPLES)
    int          head;          // ring-buffer write head
    float        R[3][3];       // rotation cached from last successful solve
    float        t[3];          // translation cached (unit length)
    bool         pose_valid;
} solve3d_ctx_t;
```

**Key functions:**

```c
void solve3d_init(solve3d_ctx_t *ctx);

// Direct port of LH2_angles_to_pixels():
//   px[0] = tan(az_rad)
//   px[1] = tan(el_rad) / cos(az_rad)
void angles_to_pixels(float az_rad, float el_rad, float px_out[2]);

// Add one sample to the ring buffer (overwrites oldest when full).
void solve3d_push_sample(solve3d_ctx_t *ctx, const lh2_sample_t *s);

// Run solve_3d_scene on the current history buffer.
// Writes up to ctx->n_samples points into pts3d_out.
// Returns the number of points written, or 0 on failure.
int solve3d_run(solve3d_ctx_t *ctx, lh2_point3d_t *pts3d_out);
```

**Algorithm inside `solve3d_run` — direct port of `solve_3d_scene`:**

```
Requires n_samples >= 8.

Step 1  Unpack history → pts_a[n][2], pts_b[n][2]

Step 2  F = find_fundamental_mat(pts_a, pts_b, n)

Step 3  R, t = recover_pose(F, pts_a, pts_b, n)

Step 4  Build projection matrices (identical to Python):
            P1 = [I(3×3) | 0(3×1)]                    (camera A at origin)
            P2 = [R^T    | -R^T * t]                   (camera B)

Step 5  pts3d = triangulate_points(P1, P2, pts_a, pts_b, n)

Step 6  Cache R, t in ctx->R, ctx->t; set pose_valid = true.
        Copy sensor_ids from history into pts3d_out.
        Return n.
```

---

### 4.5 `main.c` — Clean entry point

**Responsibility:** Hardware init, dual-core setup, and main loop.
Does **no math** — only orchestrates the four modules above.

```c
// ─── globals ────────────────────────────────────────────────────────────────
db_lh2_t      g_lh2   [NUM_SENSORS];         // populated by lh2 library
lh2_angles_t  g_angles[NUM_SENSORS][NUM_BS]; // populated by angle_decoder
solve3d_ctx_t g_solver;                      // populated by solve3d

// ─── core 1 (sensors 2 & 3) ─────────────────────────────────────────────────
void core1_entry(void) {
    db_lh2_init(&g_lh2[2], 2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&g_lh2[3], 3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);
    while (true) {
        db_lh2_process_location(&g_lh2[2]);
        db_lh2_process_location(&g_lh2[3]);
    }
}

// ─── core 0 ─────────────────────────────────────────────────────────────────
int main(void) {
    set_sys_clock_khz(128000, true);
    stdio_init_all();

    db_lh2_init(&g_lh2[0], 0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&g_lh2[1], 1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);
    angle_decoder_init(g_angles, CAL);
    solve3d_init(&g_solver);
    multicore_launch_core1(core1_entry);

    while (true) {
        // ① keep LH2 processing running continuously
        db_lh2_process_location(&g_lh2[0]);
        db_lh2_process_location(&g_lh2[1]);

        // ② decode new LFSR counts → angles
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        angle_decoder_update(g_lh2, g_angles, CAL, now_us);

        // ③ periodic: push samples, solve, print  (~10 Hz)
        if (elapsed_us > PRINT_INTERVAL_US) {
            for each sensor s:
                if g_angles[s][0] and g_angles[s][1] are fresh:
                    lh2_sample_t smp;
                    angles_to_pixels(g_angles[s][0].ema_az, ..., smp.px_a);
                    angles_to_pixels(g_angles[s][1].ema_az, ..., smp.px_b);
                    smp.sensor_id = s;
                    solve3d_push_sample(&g_solver, &smp);

            if g_solver.n_samples >= 8:
                lh2_point3d_t pts[SOLVE3D_MAX_SAMPLES];
                int n = solve3d_run(&g_solver, pts);
                for i in 0..n: printf("P,%d,%.4f,%.4f,%.4f\n", ...);
                print centroid of all pts;
        }
    }
}
```

---

## 5. Data Flow Summary

```
TS4231 × 4
  │  PIO DMA  (ts4231_capture.pio)
  ▼
lh2/lh2.c  +  lh2/lh2_decoder.c
  →  db_lh2_t[4].locations[sweep][slot].lfsr_location
                                        .selected_polynomial
                                        .data_ready
  │
  ▼  angle_decoder_update()
angle_decoder/
  →  lh2_angles_t[4][2]
         .ema_az, .ema_el   [degrees, EMA-filtered]
         .valid, .last_update_us
  │
  ▼  angles_to_pixels()  +  solve3d_push_sample()
solve3d/  history ring buffer
  →  lh2_sample_t[32]
         .px_a[2] = [tan(az_a), tan(el_a)/cos(az_a)]
         .px_b[2] = [tan(az_b), tan(el_b)/cos(az_b)]
  │
  ▼  solve3d_run()
  │
  ├── cv :: find_fundamental_mat()   →  F [3×3]
  ├── cv :: recover_pose()           →  R [3×3],  t [3]
  └── cv :: triangulate_points()     →  pts3d [n×3]
  │
  ▼
main.c  →  "P,<sensor>,<x>,<y>,<z>"  over USB-serial
```

---

## 6. CMakeLists.txt Notes

Because the lh2 sources are copied locally, the CMakeLists.txt for
`crossing_beams` compiles them directly — it does **not** call
`add_subdirectory` on the parent `src/lh2` folder or link `lh2_lib`.

```cmake
add_executable(crossing_beams
    main.c
    lh2/lh2.c
    lh2/lh2_decoder.c
    angle_decoder/angle_decoder.c
    cv/cv.c
    solve3d/solve3d.c
)

pico_generate_pio_header(crossing_beams
    ${CMAKE_CURRENT_LIST_DIR}/lh2/ts4231_capture.pio
)

target_include_directories(crossing_beams PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}       # so #include "lh2/lh2.h" works
)

target_link_libraries(crossing_beams
    pico_stdlib
    hardware_pio
    hardware_dma
    pico_multicore
)

pico_add_extra_outputs(crossing_beams)
pico_enable_stdio_usb(crossing_beams  1)
pico_enable_stdio_uart(crossing_beams 0)
```

Wire into the top-level build by adding one line to
`rp2350_firmware/CMakeLists.txt`:
```cmake
add_subdirectory(src/crossing_beams)
```

---

## 7. Memory Budget

| Item | Size | Notes |
|---|---|---|
| `db_lh2_t × 4` | ~2 KB | lh2 library structs |
| `lh2_angles_t[4][2]` | ~144 B | 4 sensors × 2 BS |
| `solve3d_ctx_t` history | `32 × 20 B` ≈ 640 B | ring buffer |
| `pts3d` output (stack) | `32 × 16 B` = 512 B | allocated / freed each cycle |
| `cv` working arrays | ~600 B | A^T A (9×9), SVD temps — stack only |
| **Total extra** | **< 5 KB** | Well within 520 KB SRAM |

---

## 8. Calibration Constants

From `utils/user_interface/tools/history_calibration.txt` (most recent entries):

```c
// BS index 0 → physical BS 4 — calibrated 2026-05-05
{ .A0 =  0.00315641f, .B0 = -121.7511f,
  .A1 =  0.00307607f, .B1 = -234.6501f }

// BS index 1 → physical BS 10 — calibrated 2026-05-04
{ .A0 =  0.00327992f, .B0 = -126.1425f,
  .A1 =  0.00317364f, .B1 = -236.6446f }
```

Compiled in as defaults. Can be overwritten in flash for field recalibration.

---

## 9. Implementation Phases

| Phase | Files | Deliverable |
|---|---|---|
| 0 | `lh2/` | Copy six files verbatim from `src/lh2/` |
| 1 | `angle_decoder/` | LFSR → az/el + EMA, verified against Python `angle_decoder.py` |
| 2 | `cv/` (internals) | `svd_3x3`, `sym_mineig_9x9`, `sym_mineig_4x4`, matrix helpers — testable with gcc |
| 3 | `cv/` (surface) | `find_fundamental_mat`, `recover_pose`, `triangulate_points` — verified on known inputs |
| 4 | `solve3d/` | Full pipeline end-to-end, output matches Python `solve_3d_scene` on logged data |
| 5 | `main.c` + `CMakeLists.txt` | Builds clean UF2, runs on hardware |

---

## 10. Verification Strategy

- **Phases 1–4:** Compile with `gcc -std=c11 -lm` on a desktop, no Pico SDK
  needed. Feed in LFSR counts from logged serial captures and compare output
  against the Python scripts.
- **Phase 5:** Flash to hardware. Pipe USB serial output into
  `utils/user_interface/display_real_time.py` for visual live comparison.

---

## 11. Serial Output Format

```
A,<sensor>,<bs>,<az_deg>,<el_deg>      angle update (diagnostic, 10 Hz)
P,<sensor>,<x>,<y>,<z>                 3D point for one sensor
C,<n_active>,<cx>,<cy>,<cz>            centroid over all active sensors
```

All floating-point values: 4 decimal places, distances in metres.

---

## 12. OpenCV Source References

The `cv/` module algorithms are derived directly from the following files in
the **OpenCV 4.x** repository. These are the exact files studied to produce
the pseudocode in §4.3.

| Our function | OpenCV source file | Key function name |
|---|---|---|
| `find_fundamental_mat` | [`modules/calib3d/src/fundam.cpp`][fundam] | `run8Point()`, `FMEstimatorCallback::computeError()` |
| `recover_pose` | [`modules/calib3d/src/five-point.cpp`][fivepoint] | `decomposeEssentialMat()`, `recoverPose()` |
| `triangulate_points` | [`modules/calib3d/src/triangulate.cpp`][triangulate] | `triangulateCorrPoints()` |
| `jacobi_svd_3x3/4x4` | [`modules/core/src/lapack.cpp`][lapack] | `JacobiSVDImpl_<float>` |

[fundam]:     https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/fundam.cpp
[fivepoint]:  https://github.com/opencv/opencv/blob/master/modules/calib3d/src/five-point.cpp
[triangulate]:https://github.com/opencv/opencv/blob/master/modules/calib3d/src/triangulate.cpp
[lapack]:     https://github.com/opencv/opencv/blob/master/modules/core/src/lapack.cpp

### Key decisions traced to the source

| Decision | Where in OpenCV |
|---|---|
| `A = Σ r·rᵀ` instead of storing the full n×9 matrix | `fundam.cpp:run8Point()` — avoids O(n) allocation |
| Smallest eigenvector via `eigen()`, not full SVD | `fundam.cpp:run8Point()` — symmetric matrix, cheaper |
| Rank-2 via SVD with `w[2] = 0` | `fundam.cpp:run8Point()` — explicit zero |
| Symmetric epipolar distance = `max(d1²/‖l1‖², d2²/‖l2‖²)` | `fundam.cpp:FMEstimatorCallback::computeError()` |
| W matrix `[[0,-1,0],[1,0,0],[0,0,1]]` | `five-point.cpp:decomposeEssentialMat()` |
| Cheirality by depth sign in both cameras | `five-point.cpp:recoverPose()` |
| DLT column loop over k=0..3 (column index of P) | `triangulate.cpp:triangulateCorrPoints()` |
| Solution = last row of V (not last column) | `triangulate.cpp` — `matrV(3,0..3)` |
| One-sided Jacobi, convergence `\|p\| ≤ eps·√(a·b)` | `lapack.cpp:JacobiSVDImpl_` |
| `β<0` branch for `c,s` Givens rotation | `lapack.cpp:JacobiSVDImpl_` |
