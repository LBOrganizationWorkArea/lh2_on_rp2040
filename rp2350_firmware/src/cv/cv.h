/**
 * @file   cv.h
 * @brief  DLT triangulation in pure C (no OpenCV, no numpy).
 *
 * Provides the one OpenCV routine the crossing-beams solver still needs:
 *   - triangulate_points()  ← cv2.triangulatePoints(P1, P2, pts_a, pts_b)
 *
 * Algorithm derived from OpenCV 4.x:
 *   modules/calib3d/src/triangulate.cpp  — triangulateCorrPoints (DLT)
 *   modules/core/src/lapack.cpp          — Jacobi SVD null-space extraction
 *
 * The fundamental-matrix / pose-recovery routines that used to live here
 * (find_fundamental_mat, recover_pose) were removed: the solver now uses
 * calibrated base-station poses instead of estimating them, so only the
 * triangulation back-end is required.
 *
 * No dynamic allocation. All temporaries live on the call stack.
 */

#ifndef CV_H
#define CV_H

/**
 * @brief  Triangulate n point pairs given two projection matrices.
 *
 * DLT (Direct Linear Transform) with SVD null-space extraction. Works with any
 * pair of 3×4 projection matrices; if they map world points to image points,
 * the output is in world coordinates.
 *
 * @param P1         [3][4] projection matrix for lighthouse A
 * @param P2         [3][4] projection matrix for lighthouse B
 * @param pts_a      [n][2] image points from lighthouse A (used with P1)
 * @param pts_b      [n][2] image points from lighthouse B (used with P2)
 * @param n          number of point pairs
 * @param pts3d_out  [n][3] output 3D points (X/W, Y/W, Z/W)
 */
void triangulate_points(const float P1[3][4],
                        const float P2[3][4],
                        const float pts_a[][2],
                        const float pts_b[][2],
                        int         n,
                        float       pts3d_out[][3]);

#endif /* CV_H */
