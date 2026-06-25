# Rotation Matrix in `lh2_bs_pose_t`

## Convention

`R` (3×3, `float`) is a **local-to-world** rotation matrix stored in `lh2_bs_pose_t`
(defined in `rp2350_firmware/src/solve3d/solve3d.h`).

Its **columns** are the base-station's local axes expressed as world-frame vectors:

```
R = [ col0          | col1              | col2            ]
    [ local +X      | local +Y          | local +Z        ]
    [ (boresight)   | (horiz sweep)     | (vert sweep)    ]
```

To transform a point from local to world frame: `v_world = R * v_local`.

## What each cell means

```
         col 0           col 1           col 2
       (boresight)   (horiz sweep)   (vert sweep)
      ┌──────────────────────────────────────────┐
row 0 │  R[0][0]       R[0][1]         R[0][2]  │  → world X component
row 1 │  R[1][0]       R[1][1]         R[1][2]  │  → world Y component
row 2 │  R[2][0]       R[2][1]         R[2][2]  │  → world Z component
      └──────────────────────────────────────────┘
```

Each column is a unit vector. For example, `R[0][0], R[1][0], R[2][0]` are the world
X, Y, Z components of the direction the base station is pointing (its boresight).

## Concrete example — synthetic geometry

Both synthetic base stations look straight up (boresight = world +Z).
That maps local +X → world +Z, which is a −90° rotation about world Y (`Ry(-90°)`):

```
    ┌  0   0   1 ┐
R = │  0   1   0 │
    └ -1   0   0 ┘

col 0 = (0, 0, 1)  → boresight points world +Z  ✓
col 1 = (0, 1, 0)  → horiz axis stays world +Y
col 2 = (-1, 0, 0) → vert axis points world −X
```

For a real calibrated base station the values are fractional (cosines of the angles
between axes), but the structure is identical.

## How R is used to build the projection matrix P

`_bs_projection()` in `solve3d.c` builds the world→image projection matrix `P` (3×4)
used by the DLT triangulator. The standard pinhole convention puts depth on row 2, but
here depth is local +X (boresight = column 0 of R). The rows of P are therefore
**re-ordered columns of R**:

```c
P[0][r] = bs->R[r][1];  // image-u  ← local +Y (horiz sweep, R col 1)
P[1][r] = bs->R[r][2];  // image-v  ← local +Z (vert sweep,  R col 2)
P[2][r] = bs->R[r][0];  // depth    ← local +X (boresight,   R col 0)
```

The translation column (index 3) is the standard `−R_permuted · origin`:

```c
P[0][3] = -(R[:,1] · origin)
P[1][3] = -(R[:,2] · origin)
P[2][3] = -(R[:,0] · origin)
```

The result is that `P · [X_world; 1]` gives `(tan(horiz), tan(vert), depth)`,
which matches the output of `angles_to_pixels()`.

## Where R comes from

`calibrate_export.py` reads the calibrated quaternion for each base station from
`lab.yaml` and converts it via `scipy.spatial.transform.Rotation.from_quat(...).as_matrix()`.
The resulting matrix is hardcoded into `bs_poses_cal.h` — the firmware never handles
quaternions at runtime.
