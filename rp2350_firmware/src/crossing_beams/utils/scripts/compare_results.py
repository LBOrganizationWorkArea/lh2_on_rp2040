#!/usr/bin/env python3
"""
compare_results.py — diff pico_out.csv vs python_out.csv

Usage:
    python compare_results.py pico_out.csv python_out.csv

How it works:
    Computes all pairwise distances for both point clouds, normalises both
    distance matrices by the same reference pair (points 0 and 1), then
    checks that the two normalised matrices agree within 2 %.

    This is scale-, rotation-, and reflection-invariant: it only checks
    that the C and Python implementations produce the same 3D *shape*,
    which is all epipolar geometry can guarantee (absolute scale is unknown).
"""

import sys
import numpy as np
import pandas as pd


THRESHOLD = 0.02   # 2 % max normalised distance error


def load_csv(path):
    try:
        df = pd.read_csv(path, skipinitialspace=True)
        return df[['x', 'y', 'z']].values.astype(float), df['sensor_id'].values
    except Exception as e:
        print(f"Error reading {path}: {e}", file=sys.stderr)
        sys.exit(1)


def dist_matrix(pts):
    """Upper-triangle pairwise distance matrix (full NxN, symmetric)."""
    diff = pts[:, np.newaxis, :] - pts[np.newaxis, :, :]   # (N,N,3)
    return np.sqrt((diff**2).sum(axis=2))                   # (N,N)


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} pico_out.csv python_out.csv")
        sys.exit(1)

    pico_pts,   pico_ids   = load_csv(sys.argv[1])
    python_pts, python_ids = load_csv(sys.argv[2])

    n = len(pico_pts)

    if n != len(python_pts):
        print(f"FAIL: point counts differ  (pico={n}  python={len(python_pts)})")
        sys.exit(1)

    if not np.array_equal(pico_ids, python_ids):
        print("WARNING: sensor_id columns differ — proceeding anyway")

    D_pico   = dist_matrix(pico_pts)
    D_python = dist_matrix(python_pts)

    # Normalise by the distance between points 0 and 4.
    # Points 0 and 1 are both sensor 0 (at different poses) and can be very
    # close if the trajectory step is small; use 0-4 (one full arc step).
    ref_i, ref_j = 0, 4
    ref_pico   = D_pico  [ref_i, ref_j]
    ref_python = D_python[ref_i, ref_j]

    if ref_pico < 1e-9 or ref_python < 1e-9:
        print(f"FAIL: reference distance (pair {ref_i}-{ref_j}) is near zero")
        sys.exit(1)

    Dn_pico   = D_pico   / ref_pico
    Dn_python = D_python / ref_python

    err = np.abs(Dn_pico - Dn_python)

    # Only upper triangle (i < j) to avoid double-counting
    mask = np.triu(np.ones((n, n), dtype=bool), k=1)
    err_upper = err[mask]

    max_err    = err_upper.max()
    mean_err   = err_upper.mean()
    worst_flat = err_upper.argmax()

    # Map flat upper-triangle index back to (i, j)
    triu_idx = np.argwhere(mask)
    worst_i, worst_j = triu_idx[worst_flat]

    # ---- Print summary ---------------------------------------------------
    print(f"Points compared : {n}")
    print(f"Reference pair  : ({ref_i}, {ref_j})")
    print(f"  pico   ref dist = {ref_pico:.6f}")
    print(f"  python ref dist = {ref_python:.6f}")
    print(f"  ratio           = {ref_pico/ref_python:.6f}  (expected ≈ 1.000)")
    print()
    print(f"Normalised distance error (upper triangle, {len(err_upper)} pairs):")
    print(f"  max  = {max_err*100:.3f}%   at pair ({worst_i}, {worst_j})")
    print(f"  mean = {mean_err*100:.3f}%")
    print(f"  threshold = {THRESHOLD*100:.0f}%")
    print()

    # ---- Per-pose geometry check (diagonal/side = sqrt(2)) ---------------
    # Samples are ordered: pose 0 → sensors 0,1,2,3; pose 1 → 0,1,2,3; etc.
    print("Per-pose geometry check  (diagonal/side should be √2 = 1.4142):")
    print(f"  {'pose':>4}  {'side(pico)':>10}  {'diag(pico)':>10}  "
          f"{'ratio':>7}  {'√2 dev':>8}  {'side(py)':>10}  {'diag(py)':>10}  "
          f"{'ratio':>7}  {'√2 dev':>8}")

    sqrt2 = np.sqrt(2.0)
    pose_ok = True
    for k in range(8):
        base = k * 4
        # Sensor indices within this pose: base+0=S0, base+1=S1, base+2=S2, base+3=S3
        s0, s1, s2, s3 = base, base+1, base+2, base+3

        def check(D):
            side = 0.5 * (D[s0, s1] + D[s0, s3])   # average of two sides
            diag = 0.5 * (D[s0, s2] + D[s1, s3])   # average of two diagonals
            ratio = diag / side if side > 1e-9 else float('nan')
            dev   = abs(ratio - sqrt2) / sqrt2
            return side, diag, ratio, dev

        ps, pd_, pr, pdev = check(D_pico)
        ys, yd_, yr, ydev = check(D_python)
        flag = "OK" if pdev < THRESHOLD and ydev < THRESHOLD else "FAIL"
        if flag == "FAIL":
            pose_ok = False
        print(f"  {k:>4}  {ps:>10.5f}  {pd_:>10.5f}  {pr:>7.4f}  "
              f"{pdev*100:>7.2f}%  {ys:>10.5f}  {yd_:>10.5f}  {yr:>7.4f}  "
              f"{ydev*100:>7.2f}%  [{flag}]")

    print()

    # ---- Final verdict ---------------------------------------------------
    overall_pass = (max_err < THRESHOLD) and pose_ok
    if overall_pass:
        print("*** PASS ***")
    else:
        if max_err >= THRESHOLD:
            print(f"*** FAIL  (max normalised distance error {max_err*100:.2f}% > {THRESHOLD*100:.0f}%) ***")
        if not pose_ok:
            print("*** FAIL  (at least one pose failed the √2 geometry check) ***")

        # Print worst offenders
        sorted_idx = np.argsort(err_upper)[::-1]
        print("\nTop 5 worst pairs:")
        count = 0
        for fi in sorted_idx:
            if count >= 5:
                break
            i, j = triu_idx[fi]
            print(f"  ({i:2d},{j:2d})  pico={Dn_pico[i,j]:.4f}  "
                  f"python={Dn_python[i,j]:.4f}  err={err[i,j]*100:.2f}%")
            count += 1

        sys.exit(1)


if __name__ == "__main__":
    main()
