/**
 * @file   cv.h
 * @brief  Pure-C equivalents of the three OpenCV functions used by solve_3d_scene.
 *
 * Implements, for embedded use (no OpenCV, no numpy):
 *   - find_fundamental_mat()   ← cv2.findFundamentalMat(pts_a, pts_b, FM_LMEDS)
 *   - recover_pose()           ← cv2.recoverPose(F, pts_a, pts_b)
 *   - triangulate_points()     ← cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)
 *
 * Algorithms derived from OpenCV 4.x source:
 *   modules/calib3d/src/fundam.cpp       — find_fundamental_mat
 *   modules/calib3d/src/five-point.cpp   — recover_pose
 *   modules/calib3d/src/triangulate.cpp  — triangulate_points
 *   modules/core/src/lapack.cpp          — jacobi_svd internals
 *
 * No dynamic allocation. All temporaries live on the call stack.
 * Only dependency: <math.h> (sinf, cosf, sqrtf, fabsf, hypotf, atanf).
 */

#ifndef CV_H
#define CV_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Estimate the fundamental matrix from n ≥ 8 point correspondences.
 *
 * Implements the Hartley normalised 8-point algorithm (deterministic).
 * This is the 8-point kernel that OpenCV uses inside FM_LMEDS.
 *
 * @param pts_a  [n][2] pixel-projected points from lighthouse A
 * @param pts_b  [n][2] pixel-projected points from lighthouse B
 * @param n      number of correspondences (must be ≥ 8)
 * @param F_out  [3][3] output fundamental matrix
 *
 * @return 1 on success, 0 if n < 8 or degenerate configuration
 */
int find_fundamental_mat(const float pts_a[][2],
                         const float pts_b[][2],
                         int         n,
                         float       F_out[3][3]);

/**
 * @brief  Decompose F into R and t and select the physically valid solution.
 *
 * Assumes K = I (calibrated image coordinates — exactly how LH2 pixels are
 * defined here, so E = F). Performs cheirality check on all four R/t candidates.
 *
 * @param F      [3][3] fundamental matrix (= essential matrix when K=I)
 * @param pts_a  [n][2] points used for cheirality test
 * @param pts_b  [n][2] points used for cheirality test
 * @param n      number of test points
 * @param R_out  [3][3] output rotation matrix
 * @param t_out  [3]    output translation (unit length)
 */
void recover_pose(const float F[3][3],
                  const float pts_a[][2],
                  const float pts_b[][2],
                  int         n,
                  float       R_out[3][3],
                  float       t_out[3]);

/**
 * @brief  Triangulate n point pairs given two projection matrices.
 *
 * Uses DLT (Direct Linear Transform) with SVD null-space extraction,
 * mirroring OpenCV's triangulateCorrPoints().
 *
 * NOTE: uses pts_a with P1 and pts_b with P2 (correct order).
 * The Python source has pts_a/pts_b swapped — this C version fixes that.
 *
 * @param P1         [3][4] projection matrix for lighthouse A
 * @param P2         [3][4] projection matrix for lighthouse B
 * @param pts_a      [n][2] pixel-projected points from lighthouse A
 * @param pts_b      [n][2] pixel-projected points from lighthouse B
 * @param n          number of point pairs
 * @param pts3d_out  [n][3] output 3D points in Cartesian (X/W, Y/W, Z/W)
 */
void triangulate_points(const float P1[3][4],
                        const float P2[3][4],
                        const float pts_a[][2],
                        const float pts_b[][2],
                        int         n,
                        float       pts3d_out[][3]);

#endif /* CV_H */
