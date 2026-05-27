# Math Self-Test Plan

**Goal**: Prove to your boss that the 3D position math runs correctly on the RP2350.

Two files. Same input data. Same pipeline. Compare outputs.

```
angles_data[]          ← hardcoded az/el recordings (same values in both)
      │
      ├── main_test.c        angles_to_pixels → solve3d_run → USB serial CSV
      │
      └── validate_solve3d.py   same math in Python/OpenCV → stdout CSV

diff the two CSVs → PASS if pairwise distances match within tolerance
```

No lighthouses. No PIO. No angle_decoder. Just the math.

---

## 1. Input data

A table of previously recorded angle measurements. Each row is one (sensor,
position) observation where both lighthouses saw that sensor simultaneously:

```
N  sensor_id  az_a_deg  el_a_deg  az_b_deg  el_b_deg
```

- `az_a / el_a`: azimuth + elevation from BS0 [degrees]
- `az_b / el_b`: azimuth + elevation from BS1 [degrees]
- `sensor_id`: which of the 4 physical sensors (0–3)

**Minimum viable dataset**: ≥ 8 rows with genuinely different angles
(i.e. the sensor board was moved to at least 8 different positions in the room).
If you have previously recorded serial output (`A,<sensor>,<bs>,<az>,<el>` lines),
parse those directly — you'll have far more than enough.

If no recordings are available yet: synthesize from a known virtual scene (see
Appendix A) and use those as the hardcoded values.

The same numeric values are copied verbatim into both files.

---

## 2. `main_test.c` (RP2350)

### What it does

```
1. Hardcode the angle table as two float arrays:
      pts_a[N][2]  and  pts_b[N][2]  and  sensor_ids[N]
   (computed from az/el via angles_to_pixels — can be pre-computed on PC
    and baked in as literals, OR computed at runtime from raw az/el floats)

2. Call angles_to_pixels(az_a_rad, el_a_rad, sample.px_a) for each row
   Call angles_to_pixels(az_b_rad, el_b_rad, sample.px_b) for each row
   Call solve3d_push_sample(&ctx, &sample)               for each row

3. n = solve3d_run(&ctx, pts3d)

4. Print results as CSV over USB serial:
      i, sensor_id, x, y, z

5. Loop forever, reprinting every 5 s
```

### What it does NOT do

- No `db_lh2_init`, no PIO, no DMA, no multicore
- No `angle_decoder_update` — angles are injected directly as floats
- No `pico_multicore`, no `hardware_pio`, no `hardware_dma`

### Dependencies (only two source files needed)

```
main_test.c
cv/cv.c
solve3d/solve3d.c
```

`angle_decoder.c` and all `lh2/` files are **not compiled**.

---

## 3. `validate_solve3d.py` (PC)

### What it does

Mirrors the C pipeline exactly, using OpenCV as the reference implementation.
Does **not** use the existing `solve_3d_scene()` from `data_processing.py` because
that function has a known pts_a/pts_b swap bug (see FIXME comments in that file).
Instead, implements the corrected pipeline step by step:

```python
import numpy as np, cv2

# ── same angle table as main_test.c ──────────────────────────────────────────
AZ_A = [...]   # degrees, N entries
EL_A = [...]
AZ_B = [...]
EL_B = [...]
SENSOR_IDS = [...]

# ── Step 1: angles_to_pixels (mirrors angles_to_pixels() in solve3d.c) ───────
def lh2_angles_to_pixels(az_deg, el_deg):
    az = np.radians(az_deg)
    el = np.radians(el_deg)
    return np.array([np.tan(az), np.tan(el) / np.cos(az)])

pts_a = np.array([lh2_angles_to_pixels(az, el) for az, el in zip(AZ_A, EL_A)])
pts_b = np.array([lh2_angles_to_pixels(az, el) for az, el in zip(AZ_B, EL_B)])

# ── Step 2: fundamental matrix ────────────────────────────────────────────────
F, _ = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)

# ── Step 3: recover pose ──────────────────────────────────────────────────────
_, R, t, _ = cv2.recoverPose(F, pts_a, pts_b)

# ── Step 4: projection matrices — P2 = [R | t]  (OpenCV convention) ──────────
# NOTE: uses [R|t] directly, NOT [R.T | -R.T t].
# This matches the C code in solve3d.c and is the correct OpenCV convention.
P1 = np.hstack([np.eye(3), np.zeros((3,1))])
P2 = np.hstack([R, t])

# ── Step 5: triangulate — pts_a with P1, pts_b with P2 (correct order) ───────
pts4d = cv2.triangulatePoints(P1, P2, pts_a.T, pts_b.T)
pts3d = (pts4d[:3] / pts4d[3]).T     # shape (N, 3)

# ── Print CSV ─────────────────────────────────────────────────────────────────
print("i,sensor_id,x,y,z")
for i, (sid, pt) in enumerate(zip(SENSOR_IDS, pts3d)):
    print(f"{i},{sid},{pt[0]:.6f},{pt[1]:.6f},{pt[2]:.6f}")
```

---

## 4. Comparison

### Collect outputs

```bash
# RP2350: capture USB serial to file
python -m serial.tools.miniterm /dev/ttyACM0 115200 | tee pico_out.csv

# PC: run Python script
python validate_solve3d.py > python_out.csv
```

### Compare pairwise distances

Raw XYZ coordinates cannot be compared directly because:
- Scale is ambiguous (epipolar geometry only recovers translation direction)
- The coordinate frame may be reflected or rotated

Instead compare **all pairwise distances** normalized to the first pair:

```python
# compare_results.py
import numpy as np, pandas as pd

pico   = pd.read_csv("pico_out.csv")[['x','y','z']].values
python = pd.read_csv("python_out.csv")[['x','y','z']].values

def dist_matrix(pts):
    n = len(pts)
    D = np.zeros((n,n))
    for i in range(n):
        for j in range(n):
            D[i,j] = np.linalg.norm(pts[i] - pts[j])
    return D

D_pico   = dist_matrix(pico)
D_python = dist_matrix(python)

# Normalise both by same reference distance (pair 0-1)
ref_pico   = D_pico[0,1]
ref_python = D_python[0,1]

D_pico_n   = D_pico   / ref_pico
D_python_n = D_python / ref_python

max_err = np.max(np.abs(D_pico_n - D_python_n))
print(f"Max normalised distance error: {max_err*100:.2f}%")
print("PASS" if max_err < 0.02 else "FAIL")
```

### Pass condition

```
max normalised pairwise distance error < 2 %
```

This threshold accommodates float32 vs float64 rounding and the
algorithmic difference (Jacobi SVD in C vs `np.linalg.svd` in Python).

---

## 5. CMakeLists.txt addition

Append to `crossing_beams/CMakeLists.txt`:

```cmake
# ---- Math self-test (no hardware) ----
add_executable(crossing_beams_test
    main_test.c
    cv/cv.c
    solve3d/solve3d.c
)
target_include_directories(crossing_beams_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(crossing_beams_test
    pico_stdlib
)
pico_enable_stdio_usb(crossing_beams_test  1)
pico_enable_stdio_uart(crossing_beams_test 0)
pico_add_extra_outputs(crossing_beams_test)
target_compile_options(crossing_beams_test PRIVATE
    -Wall -Wno-format -Wno-unused-function -Wno-maybe-uninitialized -O2
)
```

`solve3d.h` includes `cv/cv.h` only — no lh2 or angle_decoder headers needed.

---

## 6. Serial output format (`main_test.c`)

```
=== LH2 SOLVE3D SELF-TEST ===
N=32 samples loaded
solve3d_run: 32 points

i,sensor_id,x,y,z
0,0, 0.123456, 0.234567, 1.345678
1,1, 0.156789,-0.012345, 1.378901
...
31,3, 0.198765, 0.187654, 1.412345

=== END (reprinting in 5s) ===
```

One line per point, CSV-parseable directly. Reprints every 5 s for late
USB connections.

---

## Appendix A — Synthetic angles (if no recordings available)

If you have no previously recorded data yet, generate synthetic angles from a
known scene and use those as the hardcoded input. This also lets you check
absolute positions (not just relative distances).

**Scene**: 4 sensors at corners of a 5 cm square, measured from 8 body
positions as the board moves in a gentle arc (varied X, Y, Z so the 2D
projections spread across the image plane — pure horizontal translation is
degenerate, see test notes below).

**Body positions** (world frame, metres):

| k | X_body | Y_body | Z_body |
|---|--------|--------|--------|
| 0 | −0.15  |  0.00  | 1.90   |
| 1 | −0.10  |  0.04  | 1.95   |
| 2 | −0.05  |  0.00  | 2.00   |
| 3 |  0.00  | −0.04  | 2.05   |
| 4 |  0.05  |  0.00  | 2.10   |
| 5 |  0.10  |  0.04  | 2.05   |
| 6 |  0.15  |  0.00  | 2.00   |
| 7 |  0.20  | −0.04  | 1.95   |

**Sensors in body frame** (metres): S0=(0,0,0), S1=(0.05,0,0),
S2=(0.05,0.05,0), S3=(0,0.05,0).

**Lighthouses**: BS0=(0,0,0)→+Z, BS1=(1,0,0)→+Z.

**Per sensor s, per BS b, per pose k**:
```
P_world = body_pos[k] + sensor_body[s]
P_cam   = P_world - BS_pos[b]          # parallel cameras, no rotation
az_rad  = atan2(P_cam.x, P_cam.z)
el_rad  = atan2(P_cam.y, sqrt(P_cam.x²+P_cam.z²))
```

Convert to degrees, paste into both files.

With synthetic data the Python script can also verify absolute geometry:
`diagonal / side` should equal `√2` for the recovered 5 cm square.

---

## Notes

- **Why ≥ 8 distinct positions matter**: 4 coplanar sensors pushed repeatedly give
  ≤ 4 unique rows in the constraint matrix — `find_fundamental_mat` returns garbage.
  The ring buffer must contain samples from ≥ 8 genuinely different positions.

- **Why pure X translation is degenerate**: all sensor Y-projections stay constant
  (`px[1] = sy/Z = const`) — all image points land on 2 horizontal lines and F
  is rank-deficient. The trajectory needs variation in Y and Z as well.

- **Convention**: the Python validator uses `P2=[R|t]` (OpenCV standard), matching
  the C code. The existing `solve_3d_scene()` in `data_processing.py` uses
  `P2=[R.T|-R.T t]` with a pts_a/pts_b swap — those two things cancel out in some
  cases but are both bugs. Do not use that function for validation.
