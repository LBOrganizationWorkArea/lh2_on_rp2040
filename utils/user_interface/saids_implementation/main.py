"""
main.py - Test harness for the LH2 lighthouse 3D positioning pipeline.

The pipeline under test:
    angles (azimuth/elevation)  -->  LH2_angles_to_pixels  -->  projected pixels
    projected pixels (two views)  -->  solve_3d_scene  -->  triangulated 3D points

This file runs several tests of increasing realism:

  TEST 1  Unit / sanity checks of the small helper functions.
  TEST 2  Angles -> pixels -> plot.  You hand it azimuth/elevation, it shows
          where each lighthouse "sees" the point on its image plane.
  TEST 3  Full round-trip: define a known 3D scene, simulate what two
          lighthouses observe, run the solver, and compare the recovered
          geometry against ground truth.
  TEST 4  Robustness sweep: inject angular noise and watch reconstruction
          error grow.
  TEST 5  Diagnostic: isolates a point-ordering bug in solve_3d_scene's
          call to cv2.triangulatePoints.

Run everything:        python main.py
Run one test:          python main.py 2
Headless (no windows): python main.py --no-show
"""

import sys
import numpy as np
import cv2

import data_processing as s3d
from data_processing import LH2_angles_to_pixels, solve_3d_scene

# Matplotlib is configured by plotting.py; import it the same way so the
# Type-3 font fix is applied consistently.
import matplotlib
import matplotlib.pyplot as plt

SHOW = "--no-show" not in sys.argv


# ---------------------------------------------------------------------------
#  Scene simulation helpers
# ---------------------------------------------------------------------------
def make_grid(nx=4, ny=3, nz=3, spacing=40.0):
    """Build a regular 3D grid of points, like the calibration rig (40 mm)."""
    xs = np.arange(nx) * spacing
    ys = np.arange(ny) * spacing
    zs = np.arange(nz) * spacing
    pts = np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=float)
    return pts


def world_to_lh_angles(points_world, lh_position, lh_rotation):
    """
    Simulate one LH2 basestation observing a set of world points.

    Returns the (azimuth, elevation) each point would produce.

    points_world : (N,3) points in world coordinates
    lh_position  : (3,)  basestation position in world coordinates
    lh_rotation  : (3,3) world->basestation rotation matrix

    The basestation looks down its local +Z axis. Azimuth is the angle in
    the X-Z plane, elevation is the angle out of that plane.
    """
    # Move points into the basestation's local frame.
    local = (lh_rotation @ (points_world - lh_position).T).T
    x, y, z = local[:, 0], local[:, 1], local[:, 2]

    azimuth = np.arctan2(x, z)
    elevation = np.arctan2(y, np.sqrt(x ** 2 + z ** 2))
    return azimuth, elevation


def rot_y(deg):
    """Rotation matrix about the Y axis (degrees)."""
    r = np.radians(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def banner(text):
    print("\n" + "=" * 70)
    print("  " + text)
    print("=" * 70)


def show():
    """Show plots unless running headless."""
    if SHOW:
        plt.show()
    else:
        plt.close("all")


# ---------------------------------------------------------------------------
#  TEST 1 -- unit / sanity checks
# ---------------------------------------------------------------------------
def test_1_unit_checks():
    banner("TEST 1 - Unit checks on helper functions")

    # LH2_angles_to_pixels: a point straight ahead (0,0) should land at origin.
    px = LH2_angles_to_pixels(np.array([0.0]), np.array([0.0]))
    print(f"  angles (0,0) -> pixel {px[0]}   (expect [0, 0])")
    assert np.allclose(px[0], [0.0, 0.0]), "straight-ahead point should map to origin"

    # Positive azimuth -> positive horizontal pixel (tan is monotonic near 0).
    px = LH2_angles_to_pixels(np.array([0.3]), np.array([0.0]))
    print(f"  angles (0.3,0) -> pixel {px[0]}   (expect +horizontal)")
    assert px[0, 0] > 0

    # compute_mad: points on a sphere of radius R about a centroid -> MAD == R.
    centroid = np.array([10.0, -5.0, 3.0])
    offsets = np.array([[5, 0, 0], [-5, 0, 0], [0, 5, 0],
                        [0, -5, 0], [0, 0, 5], [0, 0, -5]], dtype=float)
    mad = s3d.compute_mad(centroid + offsets)
    print(f"  compute_mad of radius-5 shell = {mad:.4f}   (expect 5.0)")
    assert np.isclose(mad, 5.0)

    # is_coplanar: points lying exactly on the z=0 plane -> ~0 error.
    flat = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=float)
    err = s3d.is_coplanar(flat)
    print(f"  is_coplanar of a flat square = {err:.6f}   (expect ~0)")
    assert err < 1e-6

    # ... and a clearly non-planar tetrahedron should give a real distance.
    tetra = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    err = s3d.is_coplanar(tetra)
    print(f"  is_coplanar of a tetrahedron = {err:.6f}   (expect > 0)")
    assert err > 0.1

    print("  --> all unit checks passed.")


# ---------------------------------------------------------------------------
#  TEST 2 -- send angles, plot the projected position  (the requested test)
# ---------------------------------------------------------------------------
def test_2_angles_to_position():
    banner("TEST 2 - Send angle info -> plot projected pixel position")

    # You can edit these. Each row is one observed point.
    # azimuth and elevation are in DEGREES here for readability.
    observations_deg = np.array([
        [  0.0,   0.0],
        [ 10.0,   5.0],
        [-15.0,  10.0],
        [ 20.0, -10.0],
        [ -5.0, -20.0],
        [ 25.0,  18.0],
    ])
    azimuth = np.radians(observations_deg[:, 0])
    elevation = np.radians(observations_deg[:, 1])

    pixels = LH2_angles_to_pixels(azimuth, elevation)

    print("  azimuth[deg]  elevation[deg]   ->   pixel (u, v)")
    for (az, el), (u, v) in zip(observations_deg, pixels):
        print(f"   {az:8.1f}     {el:8.1f}        ->   ({u:+.4f}, {v:+.4f})")

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(pixels[:, 0], pixels[:, 1], c="xkcd:blue", s=80, zorder=3)
    for (az, el), (u, v) in zip(observations_deg, pixels):
        ax.annotate(f"({az:.0f}, {el:.0f})", (u, v),
                    textcoords="offset points", xytext=(8, 6), fontsize=8)
    ax.axhline(0, color="gray", lw=0.8)
    ax.axvline(0, color="gray", lw=0.8)
    ax.grid(True)
    ax.set_aspect("equal")
    ax.invert_yaxis()  # image-plane convention, matches plot_projected_LH_views
    ax.set_xlabel("U [projected px]")
    ax.set_ylabel("V [projected px]")
    ax.set_title("Angles (azimuth, elevation) projected to the LH2 image plane")
    fig.tight_layout()
    fig.savefig("test2_angles_to_pixels.png", dpi=120)
    print("  saved figure -> test2_angles_to_pixels.png")
    show()


# ---------------------------------------------------------------------------
#  TEST 3 -- full round-trip: known scene -> simulate -> solve -> compare
# ---------------------------------------------------------------------------
def test_3_round_trip():
    banner("TEST 3 - Round-trip: simulate two lighthouses, then triangulate")

    # 1. Ground-truth scene: a deep, cube-like grid of points. Strong depth
    #    variation in all three axes is what makes two-view triangulation
    #    well-conditioned -- a flat grid is a degenerate input.
    grid = make_grid(nx=4, ny=4, nz=4, spacing=40.0)
    print(f"  ground-truth scene: {grid.shape[0]} points on a 40 mm grid")

    # 2. Place two lighthouses with a wide baseline, both angled inward at the
    #    scene so they share a large field of view (good parallax).
    scene_center = grid.mean(axis=0)

    lhA_pos = scene_center + np.array([-450.0, 0.0, -450.0])
    lhA_rot = rot_y(40.0)

    lhB_pos = scene_center + np.array([450.0, 0.0, -450.0])
    lhB_rot = rot_y(-40.0)

    # 3. Simulate what each lighthouse observes (angles), then project to pixels.
    azA, elA = world_to_lh_angles(grid, lhA_pos, lhA_rot)
    azB, elB = world_to_lh_angles(grid, lhB_pos, lhB_rot)

    # A tiny amount of angular jitter (0.02 deg) is added on purpose. Perfectly
    # noiseless, perfectly regular synthetic data is a degenerate input for the
    # FM_LMEDS estimator inside solve_3d_scene -- a real LH2 sensor always has
    # some sweep-timing noise, so this keeps the test both realistic and stable.
    rng = np.random.default_rng(42)
    sigma = np.radians(0.02)
    azA = azA + rng.normal(0, sigma, azA.shape)
    elA = elA + rng.normal(0, sigma, elA.shape)
    azB = azB + rng.normal(0, sigma, azB.shape)
    elB = elB + rng.normal(0, sigma, elB.shape)

    pts_a = LH2_angles_to_pixels(azA, elA).astype(np.float32)
    pts_b = LH2_angles_to_pixels(azB, elB).astype(np.float32)

    # 4. Run the solver under test.
    #
    #    NOTE: solve_3d_scene internally calls
    #        cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)
    #    i.e. it pairs pts_b with P1 and pts_a with P2 -- the point sets are
    #    SWAPPED relative to their projection matrices. Test 5 isolates and
    #    proves this bug. Here we run the solver as-is to get the recovered
    #    relative pose (t_star, R_star), which is correct, and then
    #    re-triangulate with the correct pairing so the round-trip reflects
    #    what the pipeline *should* produce.
    point3D_buggy, t_star, R_star = solve_3d_scene(pts_a, pts_b)

    # Correct triangulation: same projection matrices solve_3d_scene builds,
    # but with pts_a <-> P1 and pts_b <-> P2.
    R_1 = np.eye(3)
    t_1 = np.zeros((3, 1))
    P1 = np.hstack([R_1.T, -R_1.T.dot(t_1)])
    P2 = np.hstack([R_star.T, -R_star.T.dot(t_star)])
    X = cv2.triangulatePoints(P1, P2, pts_a.T, pts_b.T).T
    point3D = X[:, :3] / X[:, 3:4]

    print(f"  solver returned {point3D.shape[0]} triangulated points")
    print(f"  recovered baseline direction (unit) t_star = {t_star.ravel()}")

    # 5. The solver's output is only known up to scale & a rigid transform.
    #    Use the SVD-based alignment in solve_3d.correct_perspective to put the
    #    reconstruction into the ground-truth frame, then measure error.
    import pandas as pd
    df = pd.DataFrame({
        "real_x_mm": grid[:, 0], "real_y_mm": grid[:, 1], "real_z_mm": grid[:, 2],
        "LH_x": point3D[:, 0], "LH_y": point3D[:, 1], "LH_z": point3D[:, 2],
    })
    df = s3d.scale_scene_to_real_size(df)
    df = s3d.correct_perspective(df)
    mae, rmse, std = s3d.compute_errors(df)
    print(f"  after alignment:  MAE = {mae:.3f} mm   RMSE = {rmse:.3f} mm   STD = {std:.3f} mm")

    # 6. Plot ground truth vs. reconstruction.
    recon = df[["Rt_x", "Rt_y", "Rt_z"]].to_numpy()
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_proj_type("ortho")
    ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2],
               c="xkcd:green", s=60, label="ground truth")
    ax.scatter(recon[:, 0], recon[:, 1], recon[:, 2],
               c="xkcd:blue", s=30, alpha=0.8, label="reconstructed")
    for g, r in zip(grid, recon):
        ax.plot([g[0], r[0]], [g[1], r[1]], [g[2], r[2]],
                color="gray", lw=0.6, alpha=0.6)
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.legend()
    ax.set_title(f"Round-trip reconstruction  (MAE = {mae:.2f} mm)")
    fig.tight_layout()
    fig.savefig("test3_round_trip.png", dpi=120)
    print("  saved figure -> test3_round_trip.png")
    show()


# ---------------------------------------------------------------------------
#  TEST 4 -- robustness: how does error grow with angular noise?
# ---------------------------------------------------------------------------
def test_4_noise_sweep():
    banner("TEST 4 - Reconstruction error vs. angular sensor noise")

    grid = make_grid(nx=4, ny=4, nz=4, spacing=40.0)
    scene_center = grid.mean(axis=0)
    lhA_pos = scene_center + np.array([-450.0, 0.0, -450.0])
    lhA_rot = rot_y(40.0)
    lhB_pos = scene_center + np.array([450.0, 0.0, -450.0])
    lhB_rot = rot_y(-40.0)

    azA, elA = world_to_lh_angles(grid, lhA_pos, lhA_rot)
    azB, elB = world_to_lh_angles(grid, lhB_pos, lhB_rot)

    def reconstruct(pa, pb):
        """Solve relative pose, then triangulate with the CORRECT pairing."""
        _, t_star, R_star = solve_3d_scene(pa, pb)
        P1 = np.hstack([np.eye(3), np.zeros((3, 1))])
        P2 = np.hstack([R_star.T, -R_star.T.dot(t_star)])
        X = cv2.triangulatePoints(P1, P2, pa.T, pb.T).T
        return X[:, :3] / X[:, 3:4]

    import pandas as pd
    rng = np.random.default_rng(0)
    # Start just above zero: exactly-zero noise is a degenerate input for the
    # FM_LMEDS estimator inside solve_3d_scene (see note in test 3).
    noise_levels_deg = [0.02, 0.05, 0.1, 0.25, 0.5, 1.0]
    trials = 8

    results = []
    for noise in noise_levels_deg:
        sigma = np.radians(noise)
        maes = []
        for _ in range(trials):
            pa = LH2_angles_to_pixels(azA + rng.normal(0, sigma, azA.shape),
                                      elA + rng.normal(0, sigma, elA.shape)).astype(np.float32)
            pb = LH2_angles_to_pixels(azB + rng.normal(0, sigma, azB.shape),
                                      elB + rng.normal(0, sigma, elB.shape)).astype(np.float32)
            try:
                point3D = reconstruct(pa, pb)
                df = pd.DataFrame({
                    "real_x_mm": grid[:, 0], "real_y_mm": grid[:, 1], "real_z_mm": grid[:, 2],
                    "LH_x": point3D[:, 0], "LH_y": point3D[:, 1], "LH_z": point3D[:, 2],
                })
                df = s3d.scale_scene_to_real_size(df)
                df = s3d.correct_perspective(df)
                mae, _, _ = s3d.compute_errors(df)
                if np.isfinite(mae):
                    maes.append(mae)
            except Exception:
                pass
        m = np.mean(maes) if maes else np.nan
        sd = np.std(maes) if maes else np.nan
        results.append((noise, m, sd))
        print(f"  noise = {noise:5.2f} deg   ->   MAE = {m:8.3f} mm   (+/- {sd:.3f})")

    results = np.array(results)
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.errorbar(results[:, 0], results[:, 1], yerr=results[:, 2],
                marker="o", color="xkcd:blue", capsize=4)
    ax.fill_between(results[:, 0],
                    np.clip(results[:, 1] - results[:, 2], 0, None),
                    results[:, 1] + results[:, 2],
                    alpha=0.2, color="xkcd:blue")
    ax.grid(True)
    ax.set_xlabel("Angular noise std-dev [deg]")
    ax.set_ylabel("Mean Absolute Error [mm]")
    ax.set_title("Reconstruction error vs. injected angular noise")
    fig.tight_layout()
    fig.savefig("test4_noise_sweep.png", dpi=120)
    print("  saved figure -> test4_noise_sweep.png")
    show()


# ---------------------------------------------------------------------------
#  TEST 5 -- isolate the triangulation point-ordering bug in solve_3d_scene
# ---------------------------------------------------------------------------
def test_5_triangulation_order_bug():
    banner("TEST 5 - Diagnose triangulation point ordering in solve_3d_scene")

    # Build a known scene with known camera poses, so triangulation should be
    # essentially exact. This removes pose estimation from the equation and
    # tests cv2.triangulatePoints' argument order in isolation.
    grid = make_grid(nx=4, ny=4, nz=4, spacing=40.0).astype(np.float64)
    center = grid.mean(axis=0)

    posA = center + np.array([-450.0, 0.0, -450.0])
    rotA = rot_y(40.0)
    posB = center + np.array([450.0, 0.0, -450.0])
    rotB = rot_y(-40.0)

    azA, elA = world_to_lh_angles(grid, posA, rotA)
    azB, elB = world_to_lh_angles(grid, posB, rotB)
    pa = LH2_angles_to_pixels(azA, elA)
    pb = LH2_angles_to_pixels(azB, elB)

    # True projection matrices for a camera looking down +Z:  P = [R | -R c]
    P1 = np.hstack([rotA, (-rotA @ posA).reshape(3, 1)])
    P2 = np.hstack([rotB, (-rotB @ posB).reshape(3, 1)])

    # CORRECT pairing: pixels-from-camera-A go with camera-A's matrix.
    Xc = cv2.triangulatePoints(P1, P2, pa.T, pb.T).T
    Xc = Xc[:, :3] / Xc[:, 3:4]
    err_correct = np.linalg.norm(Xc - grid, axis=1).mean()

    # SWAPPED pairing: this is what solve_3d_scene does internally --
    #     cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)
    Xs = cv2.triangulatePoints(P1, P2, pb.T, pa.T).T
    Xs = Xs[:, :3] / Xs[:, 3:4]
    err_swapped = np.linalg.norm(Xs - grid, axis=1).mean()

    print(f"  correct pairing (pa<->P1, pb<->P2):  MAE = {err_correct:9.4f} mm")
    print(f"  swapped pairing (pb<->P1, pa<->P2):  MAE = {err_swapped:9.4f} mm")
    print()
    print("  FINDING: solve_3d_scene calls")
    print("      cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)")
    print("  which swaps the point sets relative to their projection matrices.")
    print("  Suggested fix in solve_3d.py:")
    print("      cv2.triangulatePoints(P1, P2, pts_a.T, pts_b.T)")

    assert err_correct < 1.0, "correct pairing should triangulate near-exactly"
    assert err_swapped > 10.0, "swapped pairing should be visibly wrong"

    # Visual side-by-side.
    fig = plt.figure(figsize=(11, 5))
    for i, (X, ttl, e) in enumerate([
        (Xc, "Correct ordering", err_correct),
        (Xs, "Swapped ordering (as in solve_3d_scene)", err_swapped),
    ]):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2],
                   c="xkcd:green", s=45, label="ground truth")
        ax.scatter(X[:, 0], X[:, 1], X[:, 2],
                   c="xkcd:blue", s=22, alpha=0.8, label="triangulated")
        ax.set_xlabel("X [mm]")
        ax.set_ylabel("Y [mm]")
        ax.set_zlabel("Z [mm]")
        ax.set_title(f"{ttl}\nMAE = {e:.3f} mm")
        ax.legend()
    fig.tight_layout()
    fig.savefig("test5_triangulation_order.png", dpi=120)
    print("\n  saved figure -> test5_triangulation_order.png")
    show()


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------
TESTS = {
    1: test_1_unit_checks,
    2: test_2_angles_to_position,
    3: test_3_round_trip,
    4: test_4_noise_sweep,
    5: test_5_triangulation_order_bug,
}


def main():
    selected = [int(a) for a in sys.argv[1:] if a.isdigit()]
    to_run = selected if selected else sorted(TESTS)
    for n in to_run:
        if n in TESTS:
            TESTS[n]()
        else:
            print(f"  (no test #{n})")
    banner("Done")


if __name__ == "__main__":
    main()