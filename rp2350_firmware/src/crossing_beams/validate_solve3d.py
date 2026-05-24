#!/usr/bin/env python3
"""
validate_solve3d.py — Python reference implementation of the RP2350 solve3d test.

Uses the identical virtual scene as main_test.c.
Implements the corrected solve_3d_scene pipeline:
  - P2 = [R | t]  (OpenCV standard convention)
  - pts_a paired with P1, pts_b paired with P2  (correct ordering)

This intentionally does NOT call the existing solve_3d_scene() in
data_processing.py — that function has a pts_a/pts_b swap matched by a
[R.T | -R.T t] convention, both of which are bugs that partially cancel.
This script does it cleanly to match the C code.

Usage:
    python validate_solve3d.py > python_out.csv
    python compare_results.py pico_out.csv python_out.csv
"""

import sys
import numpy as np
import cv2

# ---------------------------------------------------------------------------
# Virtual scene  (keep in sync with main_test.c)
# ---------------------------------------------------------------------------

BODY_POS = np.array([
    [-0.15,  0.00, 1.90],
    [-0.10,  0.04, 1.95],
    [-0.05,  0.00, 2.00],
    [ 0.00, -0.04, 2.05],
    [ 0.05,  0.00, 2.10],
    [ 0.10,  0.04, 2.05],
    [ 0.15,  0.00, 2.00],
    [ 0.20, -0.04, 1.95],
], dtype=np.float64)  # shape (8, 3)

# Sensor offsets in body frame [metres].  Z_body = 0 for all.
#   S3 (0,5) --- S2 (5,5)
#    |               |
#   S0 (0,0) --- S1 (5,0)   [cm]
SENSOR_BODY = np.array([
    [0.000, 0.000],   # S0
    [0.050, 0.000],   # S1
    [0.050, 0.050],   # S2
    [0.000, 0.050],   # S3
], dtype=np.float64)  # shape (4, 2)

BS0 = np.array([0.0, 0.0, 0.0])
BS1 = np.array([1.0, 0.0, 0.0])

# ---------------------------------------------------------------------------
# Geometry helpers  (mirror of main_test.c)
# ---------------------------------------------------------------------------

def world_pos(pose_k, sensor_s):
    """World position of sensor s at body pose k (identity body rotation)."""
    bx, by, bz = BODY_POS[pose_k]
    sx, sy     = SENSOR_BODY[sensor_s]
    return np.array([bx + sx, by + sy, bz])

def angles_from_bs(P_world, BS_pos):
    """Azimuth and elevation in radians for P_world seen from BS_pos → +Z."""
    P = P_world - BS_pos
    az = np.arctan2(P[0], P[2])
    el = np.arctan2(P[1], np.sqrt(P[0]**2 + P[2]**2))
    return az, el

def lh2_angles_to_pixels(az_rad, el_rad):
    """Mirror of angles_to_pixels() in solve3d.c:
       px = [tan(az), tan(el) / cos(az)]
    """
    cos_az = np.cos(az_rad)
    px_x = np.tan(az_rad)
    px_y = np.tan(el_rad) / cos_az if abs(cos_az) > 1e-9 else 0.0
    return np.array([px_x, px_y])

# ---------------------------------------------------------------------------
# Build pixel correspondence arrays  (8 poses × 4 sensors = 32 samples)
# ---------------------------------------------------------------------------

pts_a_list  = []
pts_b_list  = []
sensor_list = []

for k in range(8):      # poses
    for s in range(4):  # sensors
        P = world_pos(k, s)
        az_a, el_a = angles_from_bs(P, BS0)
        az_b, el_b = angles_from_bs(P, BS1)
        pts_a_list.append(lh2_angles_to_pixels(az_a, el_a))
        pts_b_list.append(lh2_angles_to_pixels(az_b, el_b))
        sensor_list.append(s)

pts_a      = np.array(pts_a_list,  dtype=np.float64)  # (32, 2)
pts_b      = np.array(pts_b_list,  dtype=np.float64)  # (32, 2)
sensor_ids = np.array(sensor_list, dtype=int)

# ---------------------------------------------------------------------------
# Corrected solve_3d_scene pipeline  (mirrors solve3d.c exactly)
# ---------------------------------------------------------------------------

# Step 1: fundamental matrix
F, mask = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)
if F is None:
    print("FAIL: findFundamentalMat returned None", file=sys.stderr)
    sys.exit(1)

# Step 2: recover pose
_, R, t, _ = cv2.recoverPose(F, pts_a, pts_b)

# Step 3: projection matrices
#   P1 = [I | 0]
#   P2 = [R | t]   — OpenCV [R|t] convention, matches solve3d.c
#
#   Do NOT use [R.T | -R.T @ t].  That is the convention in the existing
#   data_processing.py, which compensates for a pts_a/pts_b swap.
#   This script fixes both: P2=[R|t] and pts_a→P1, pts_b→P2.
P1 = np.hstack([np.eye(3, dtype=np.float64), np.zeros((3, 1))])
P2 = np.hstack([R,                            t              ])

# Step 4: triangulate — pts_a with P1, pts_b with P2 (correct pairing)
pts4d = cv2.triangulatePoints(P1, P2, pts_a.T, pts_b.T)
pts3d = (pts4d[:3] / pts4d[3]).T   # (32, 3)

# ---------------------------------------------------------------------------
# Print CSV  (same format as main_test.c)
# ---------------------------------------------------------------------------

print("i,sensor_id,x,y,z")
for i, (sid, pt) in enumerate(zip(sensor_ids, pts3d)):
    print(f"{i},{sid},{pt[0]:.6f},{pt[1]:.6f},{pt[2]:.6f}")
