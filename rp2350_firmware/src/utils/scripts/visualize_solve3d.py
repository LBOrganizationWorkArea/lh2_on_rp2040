#!/usr/bin/env python3
"""
visualize_solve3d.py — reconstruction error graph for the solve3d self-test.

Runs the identical virtual scene as main_test.c / validate_solve3d.py,
reconstructs 3D points, aligns them to ground truth (Procrustes), and plots
the per-point error in millimetres.

Usage
-----
    python visualize_solve3d.py                  # Python reconstruction vs ground truth
    python visualize_solve3d.py pico_out.csv     # Pico reconstruction vs ground truth

Dependencies: numpy, opencv-python, matplotlib
"""

import sys
import os
import numpy as np
import cv2
import matplotlib
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────────────────────
# Virtual scene  (keep in sync with main_test.c and validate_solve3d.py)
# ──────────────────────────────────────────────────────────────────────────────

BODY_POS = np.array([
    [-0.15,  0.00, 1.90],
    [-0.10,  0.04, 1.95],
    [-0.05,  0.00, 2.00],
    [ 0.00, -0.04, 2.05],
    [ 0.05,  0.00, 2.10],
    [ 0.10,  0.04, 2.05],
    [ 0.15,  0.00, 2.00],
    [ 0.20, -0.04, 1.95],
], dtype=np.float64)

SENSOR_BODY = np.array([
    [0.000, 0.000],   # S0: bottom-left
    [0.050, 0.000],   # S1: bottom-right
    [0.050, 0.050],   # S2: top-right
    [0.000, 0.050],   # S3: top-left
], dtype=np.float64)

BS0 = np.array([0.0, 0.0, 0.0])
BS1 = np.array([1.0, 0.0, 0.0])

N_POSES    = 8
N_SENSORS  = 4
N_SAMPLES  = N_POSES * N_SENSORS   # 32
THRESHOLD_MM = 2.0                 # pass/fail line [mm]

SENSOR_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231']
SENSOR_LABELS = ['S0', 'S1', 'S2', 'S3']

# ──────────────────────────────────────────────────────────────────────────────
# Build input data
# ──────────────────────────────────────────────────────────────────────────────

pts_a_list, pts_b_list, sensor_list, gt_list = [], [], [], []

for k in range(N_POSES):
    for s in range(N_SENSORS):
        bx, by, bz = BODY_POS[k]
        sx, sy     = SENSOR_BODY[s]
        P = np.array([bx + sx, by + sy, bz])

        for BS, rows in [(BS0, pts_a_list), (BS1, pts_b_list)]:
            Q      = P - BS
            az     = np.arctan2(Q[0], Q[2])
            el     = np.arctan2(Q[1], np.sqrt(Q[0]**2 + Q[2]**2))
            cos_az = np.cos(az)
            rows.append([np.tan(az),
                         np.tan(el) / cos_az if abs(cos_az) > 1e-9 else 0.0])

        sensor_list.append(s)
        gt_list.append(P)

pts_a      = np.array(pts_a_list, dtype=np.float64)   # (32, 2)
pts_b      = np.array(pts_b_list, dtype=np.float64)   # (32, 2)
sensor_ids = np.array(sensor_list, dtype=int)
gt_pts     = np.array(gt_list,     dtype=np.float64)   # (32, 3) — metres

# ──────────────────────────────────────────────────────────────────────────────
# Epipolar pipeline
# ──────────────────────────────────────────────────────────────────────────────

def solve_scene(pts_a, pts_b):
    a = np.ascontiguousarray(pts_a, dtype=np.float32).reshape(-1, 1, 2)
    b = np.ascontiguousarray(pts_b, dtype=np.float32).reshape(-1, 1, 2)
    F, _ = cv2.findFundamentalMat(a, b, cv2.FM_8POINT)
    if F is None:
        raise RuntimeError("findFundamentalMat returned None")
    _, R, t, _ = cv2.recoverPose(F, a, b)
    P1    = np.hstack([np.eye(3), np.zeros((3, 1))])
    P2    = np.hstack([R, t])
    pts4d = cv2.triangulatePoints(P1, P2, a.reshape(-1, 2).T, b.reshape(-1, 2).T)
    return (pts4d[:3] / pts4d[3]).T   # (N, 3)

py_pts3d = solve_scene(pts_a, pts_b)

# ──────────────────────────────────────────────────────────────────────────────
# Optional: load Pico CSV
# ──────────────────────────────────────────────────────────────────────────────

pico_pts3d = None
pico_csv   = sys.argv[1] if len(sys.argv) > 1 else None

if pico_csv:
    if not os.path.exists(pico_csv):
        print(f"WARNING: {pico_csv} not found — showing Python result instead",
              file=sys.stderr)
    else:
        import pandas as pd
        df = pd.read_csv(pico_csv, skipinitialspace=True)
        pico_pts3d = df[['x', 'y', 'z']].values.astype(float)
        print(f"Loaded {len(pico_pts3d)} points from {pico_csv}")

# ──────────────────────────────────────────────────────────────────────────────
# Procrustes alignment: scale + rotate reconstructed → ground-truth frame
#
# Epipolar geometry gives a point cloud that is correct up to a global scale
# and an arbitrary rotation/reflection.  We recover those here so we can
# report error in physical millimetres.
# ──────────────────────────────────────────────────────────────────────────────

def procrustes_align(src, dst):
    """
    Find scale s, rotation R, translation t that minimise ||s*R*src + t - dst||.
    Returns the aligned src and the per-point errors in the same units as dst.
    """
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c  = src - mu_src
    dst_c  = dst - mu_dst

    # Scale: ratio of RMS norms
    s = np.sqrt((dst_c**2).sum() / (src_c**2).sum())

    # Rotation via SVD
    H   = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    R   = Vt.T @ U.T
    # Fix reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    aligned = s * (src_c @ R.T) + mu_dst
    errors  = np.linalg.norm(aligned - dst, axis=1)   # per-point [same units as dst]
    return aligned, errors


# Choose which reconstruction to plot
pts_to_plot = pico_pts3d if pico_pts3d is not None else py_pts3d
label       = "Pico"      if pico_pts3d is not None else "Python"

_, errors_m = procrustes_align(pts_to_plot, gt_pts)
errors_mm   = errors_m * 1000.0   # convert metres → mm

# ──────────────────────────────────────────────────────────────────────────────
# Plot
# ──────────────────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(12, 5))
fig.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.12)

x = np.arange(N_SAMPLES)

# Bars coloured by sensor_id
for s in range(N_SENSORS):
    mask = sensor_ids == s
    ax.bar(x[mask], errors_mm[mask],
           color=SENSOR_COLORS[s], label=SENSOR_LABELS[s],
           width=0.7, zorder=2)

# Threshold and statistics
ax.axhline(THRESHOLD_MM, color='#cc0000', linewidth=1.2, linestyle='--',
           label=f'threshold  {THRESHOLD_MM} mm', zorder=3)
ax.axhline(errors_mm.mean(), color='#555555', linewidth=1.0, linestyle=':',
           label=f'mean  {errors_mm.mean():.3f} mm', zorder=3)

# Pose-group separators and labels
for k in range(N_POSES):
    ax.axvline(k * N_SENSORS - 0.5, color='#cccccc', linewidth=0.6, zorder=1)
    ax.text(k * N_SENSORS + 1.5, ax.get_ylim()[1] if False else THRESHOLD_MM * 1.08,
            f'P{k}', ha='center', va='bottom', fontsize=8, color='#666666')

ax.set_xlim(-0.5, N_SAMPLES - 0.5)
ax.set_xticks(x)
ax.set_xticklabels([f'{s}' for s in sensor_ids], fontsize=6)
ax.set_xlabel('Sample index  (tick label = sensor ID)', fontsize=9)
ax.set_ylabel('Reconstruction error  [mm]', fontsize=9)

verdict = 'PASS' if errors_mm.max() < THRESHOLD_MM else 'FAIL'
ax.set_title(
    f'solve3d reconstruction error — {label}  [{verdict}]\n'
    f'max = {errors_mm.max():.3f} mm    mean = {errors_mm.mean():.3f} mm    '
    f'threshold = {THRESHOLD_MM} mm',
    fontsize=10
)

ax.legend(fontsize=8, loc='upper right')
ax.grid(axis='y', linewidth=0.4, alpha=0.6)

# Annotate pose labels at top of their group
for k in range(N_POSES):
    ax.text(k * N_SENSORS + 1.5, errors_mm.max() * 1.05,
            f'P{k}', ha='center', va='bottom', fontsize=7, color='#888888')

print(f"\nReconstruction error  ({label} vs ground truth, after Procrustes):")
print(f"  max  = {errors_mm.max():.4f} mm")
print(f"  mean = {errors_mm.mean():.4f} mm")
print(f"  [{verdict}]\n")

plt.tight_layout()
plt.show()
