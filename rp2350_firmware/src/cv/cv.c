/**
 * @file   cv.c
 * @brief  DLT triangulation in pure C — see cv.h.
 *
 * Derived from OpenCV 4.x:
 *   Jacobi SVD:         modules/core/src/lapack.cpp :: JacobiSVDImpl_<float>
 *   triangulate_points: modules/calib3d/src/triangulate.cpp :: triangulateCorrPoints
 *
 * No heap allocation. All temporaries are stack-allocated.
 */

#include "cv.h"

#include <math.h>
#include <string.h>

/** Single-precision machine epsilon (2^-23 ≈ 1.19e-7) */
#define CV_EPS_F  1.1920929e-7f

/** Maximum Jacobi SVD outer iterations */
#define JACOBI_MAX_ITER 30

// ===========================================================================
// One-sided Jacobi SVD (n×n)
//
//   Derived from OpenCV modules/core/src/lapack.cpp :: JacobiSVDImpl_<float>.
//   Applies Givens rotations to pairs of columns of Aᵀ until all off-diagonal
//   inner products fall below threshold; accumulates right singular vectors V.
// ===========================================================================

static void _jacobi_svd_nxn(float *At,   /* n×n row-major = columns of A */
                             float *V,    /* n×n row-major, V (not Vt)    */
                             float *S,    /* n singular values            */
                             float *W,    /* n column norms² — scratch    */
                             int    n)
{
    for (int i = 0; i < n*n; i++) V[i] = 0.0f;
    for (int i = 0; i < n;   i++) V[i*n + i] = 1.0f;

    for (int i = 0; i < n; i++) {
        float s = 0.0f;
        for (int k = 0; k < n; k++) s += At[i*n + k] * At[i*n + k];
        W[i] = s;
    }

    for (int iter = 0; iter < JACOBI_MAX_ITER; iter++) {
        int changed = 0;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {

                float p = 0.0f;
                for (int k = 0; k < n; k++)
                    p += At[i*n + k] * At[j*n + k];

                float eps_thresh = CV_EPS_F * sqrtf(W[i] * W[j]);
                if (fabsf(p) <= eps_thresh) continue;

                float beta  = W[i] - W[j];
                float gamma = hypotf(2.0f * p, beta);
                float c, s;
                if (beta < 0.0f) {
                    s = sqrtf((gamma - beta) / (2.0f * gamma));
                    c = p / (gamma * s);
                } else {
                    c = sqrtf((gamma + beta) / (2.0f * gamma));
                    s = p / (gamma * c);
                }

                for (int k = 0; k < n; k++) {
                    float t0 = At[i*n + k];
                    float t1 = At[j*n + k];
                    At[i*n + k] =  c*t0 + s*t1;
                    At[j*n + k] = -s*t0 + c*t1;
                }
                for (int k = 0; k < n; k++) {
                    float t0 = V[k*n + i];
                    float t1 = V[k*n + j];
                    V[k*n + i] =  c*t0 + s*t1;
                    V[k*n + j] = -s*t0 + c*t1;
                }

                W[i] = 0.0f;
                for (int k = 0; k < n; k++) W[i] += At[i*n + k] * At[i*n + k];
                W[j] = 0.0f;
                for (int k = 0; k < n; k++) W[j] += At[j*n + k] * At[j*n + k];

                changed = 1;
            }
        }
        if (!changed) break;
    }

    for (int i = 0; i < n; i++)
        S[i] = sqrtf(W[i]);
}

/* ---- 4×4 SVD (the only size triangulation needs) ---- */
static void jacobi_svd_4x4(const float A[4][4],
                            float Vt[4][4])
{
    float At[16], V[16], S[4], W[4];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            At[i*4 + j] = A[j][i];

    _jacobi_svd_nxn(At, V, S, W, 4);

    /* Sort singular values descending so the null vector lands last. */
    for (int i = 0; i < 3; i++) {
        for (int j = i+1; j < 4; j++) {
            if (S[j] > S[i]) {
                float tmp = S[i]; S[i] = S[j]; S[j] = tmp;
                for (int k = 0; k < 4; k++) {
                    float t = At[i*4+k]; At[i*4+k] = At[j*4+k]; At[j*4+k] = t;
                    t = V[k*4+i]; V[k*4+i] = V[k*4+j]; V[k*4+j] = t;
                }
            }
        }
    }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Vt[i][j] = V[j*4 + i];
}

// ===========================================================================
// triangulate_points — DLT for n point pairs
//
//   Derived from OpenCV modules/calib3d/src/triangulate.cpp.
// ===========================================================================

static void _triangulate_one(const float P1[3][4],
                             const float P2[3][4],
                             float x1, float y1,
                             float x2, float y2,
                             float Xout[4])
{
    /* 4×4 DLT system: each row is one projection constraint.
     *   row 0: x1·P1[2] − P1[0]
     *   row 1: y1·P1[2] − P1[1]
     *   row 2: x2·P2[2] − P2[0]
     *   row 3: y2·P2[2] − P2[1]
     * The homogeneous solution is the null vector (last row of Vt). */
    float A[4][4];
    for (int k = 0; k < 4; k++) {
        A[0][k] = x1 * P1[2][k] - P1[0][k];
        A[1][k] = y1 * P1[2][k] - P1[1][k];
        A[2][k] = x2 * P2[2][k] - P2[0][k];
        A[3][k] = y2 * P2[2][k] - P2[1][k];
    }

    float Vt4[4][4];
    jacobi_svd_4x4(A, Vt4);
    for (int k = 0; k < 4; k++)
        Xout[k] = Vt4[3][k];
}

void triangulate_points(const float P1[3][4],
                        const float P2[3][4],
                        const float pts_a[][2],
                        const float pts_b[][2],
                        int         n,
                        float       pts3d_out[][3])
{
    for (int i = 0; i < n; i++) {
        float Xh[4];
        _triangulate_one(P1, P2,
                         pts_a[i][0], pts_a[i][1],
                         pts_b[i][0], pts_b[i][1],
                         Xh);

        if (fabsf(Xh[3]) > CV_EPS_F) {
            pts3d_out[i][0] = Xh[0] / Xh[3];
            pts3d_out[i][1] = Xh[1] / Xh[3];
            pts3d_out[i][2] = Xh[2] / Xh[3];
        } else {
            pts3d_out[i][0] = 0.0f;
            pts3d_out[i][1] = 0.0f;
            pts3d_out[i][2] = 0.0f;
        }
    }
}
