/**
 * @file   cv.c
 * @brief  Pure-C equivalents of the OpenCV functions used by solve_3d_scene.
 *
 * Algorithms faithfully derived from OpenCV 4.x source code:
 *
 *   Jacobi SVD:           modules/core/src/lapack.cpp :: JacobiSVDImpl_<float>
 *   find_fundamental_mat: modules/calib3d/src/fundam.cpp :: run8Point()
 *   recover_pose:         modules/calib3d/src/five-point.cpp :: decomposeEssentialMat + recoverPose
 *   triangulate_points:   modules/calib3d/src/triangulate.cpp :: triangulateCorrPoints
 *
 * No heap allocation. All temporaries are stack-allocated.
 */

#include "cv.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Internal constants
// ---------------------------------------------------------------------------

/** Single-precision machine epsilon (2^-23 ≈ 1.19e-7) */
#define CV_EPS_F  1.1920929e-7f

/** Maximum Jacobi SVD outer iterations */
#define JACOBI_MAX_ITER 30

// ===========================================================================
// §1  Low-level matrix helpers
// ===========================================================================

/** C = A × B  (3×3) */
static void mat3_mul(const float A[3][3],
                     const float B[3][3],
                     float       C[3][3])
{
    float tmp[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[k][j];
            tmp[i][j] = s;
        }
    memcpy(C, tmp, sizeof(tmp));
}

/** At = Aᵀ  (3×3) */
static void mat3_transpose(const float A[3][3], float At[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            At[i][j] = A[j][i];
}

/** Determinant of a 3×3 matrix */
static float mat3_det(const float A[3][3])
{
    return A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
         - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
         + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
}

/** R = U * diag(S) * Vt  (avoids building the explicit diagonal matrix) */
static void mat3_diag_mul(const float U[3][3],
                          const float S[3],
                          const float Vt[3][3],
                          float       R[3][3])
{
    /* First form tmp = diag(S) * Vt, then R = U * tmp */
    float tmp[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tmp[i][j] = S[i] * Vt[i][j];
    mat3_mul(U, tmp, R);
}

/** y = P × X  (3×4 matrix times 4-vector) */
static void mat34_mul_vec4(const float P[3][4],
                           const float X[4],
                           float       y[3])
{
    for (int i = 0; i < 3; i++) {
        float s = 0.0f;
        for (int k = 0; k < 4; k++)
            s += P[i][k] * X[k];
        y[i] = s;
    }
}

/** c = a × b  (3-vector cross product) */
static void vec3_cross(const float a[3], const float b[3], float c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/** Dot product of two 3-vectors */
static float vec3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/** Euclidean norm of a 3-vector */
static float vec3_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/** Normalise a 3-vector in place */
static void vec3_normalize(float v[3])
{
    float n = vec3_norm(v);
    if (n > CV_EPS_F) {
        v[0] /= n; v[1] /= n; v[2] /= n;
    }
}

// ===========================================================================
// §2  One-sided Jacobi SVD
//
//   Derived from OpenCV modules/core/src/lapack.cpp :: JacobiSVDImpl_<float>
//
//   Operates on columns of Aᵀ (stored as rows of At).  Applies Givens plane
//   rotations to pairs of columns until all off-diagonal inner products are
//   below the convergence threshold.
//
//   Two instantiations: n=3 (for fundamental matrix work) and n=4 (for DLT).
// ===========================================================================

static void _jacobi_svd_nxn(float *At,   /* n×n, row-major = columns of A */
                             float *V,    /* n×n, row-major, V (not Vt) */
                             float *S,    /* n singular values */
                             float *W,    /* n column norms²  — scratch */
                             int    n)
{
    /* Initialise V = Iₙ */
    for (int i = 0; i < n*n; i++) V[i] = 0.0f;
    for (int i = 0; i < n;   i++) V[i*n + i] = 1.0f;

    /* Initial column norms² */
    for (int i = 0; i < n; i++) {
        float s = 0.0f;
        for (int k = 0; k < n; k++) s += At[i*n + k] * At[i*n + k];
        W[i] = s;
    }

    for (int iter = 0; iter < JACOBI_MAX_ITER; iter++) {
        int changed = 0;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {

                /* p = dot(At[i], At[j]) */
                float p = 0.0f;
                for (int k = 0; k < n; k++)
                    p += At[i*n + k] * At[j*n + k];

                float eps_thresh = CV_EPS_F * sqrtf(W[i] * W[j]);
                if (fabsf(p) <= eps_thresh) continue;

                /* 2×2 symmetric eigenvalue sub-problem */
                float beta  = W[i] - W[j];
                float gamma = hypotf(2.0f * p, beta);
                float c, s;
                if (beta < 0.0f) {
                    /* OpenCV β<0 branch */
                    s = sqrtf((gamma - beta) / (2.0f * gamma));
                    c = p / (gamma * s);
                } else {
                    c = sqrtf((gamma + beta) / (2.0f * gamma));
                    s = p / (gamma * c);
                }

                /* Apply Givens rotation to columns i and j of At */
                for (int k = 0; k < n; k++) {
                    float t0 = At[i*n + k];
                    float t1 = At[j*n + k];
                    At[i*n + k] =  c*t0 + s*t1;
                    At[j*n + k] = -s*t0 + c*t1;
                }

                /* Apply same rotation to V (accumulate right singular vectors) */
                for (int k = 0; k < n; k++) {
                    float t0 = V[k*n + i];
                    float t1 = V[k*n + j];
                    V[k*n + i] =  c*t0 + s*t1;
                    V[k*n + j] = -s*t0 + c*t1;
                }

                /* Update column norms² */
                W[i] = 0.0f;
                for (int k = 0; k < n; k++) W[i] += At[i*n + k] * At[i*n + k];
                W[j] = 0.0f;
                for (int k = 0; k < n; k++) W[j] += At[j*n + k] * At[j*n + k];

                changed = 1;
            }
        }
        if (!changed) break;
    }

    /* Extract singular values and build U, Vt */
    for (int i = 0; i < n; i++)
        S[i] = sqrtf(W[i]);
}

/* ---- 3×3 SVD ---- */
static void jacobi_svd_3x3(const float A[3][3],
                            float U[3][3],
                            float S[3],
                            float Vt[3][3])
{
    float At[9], V[9], W[3];

    /* At = Aᵀ stored row-major (each row = one column of A) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            At[i*3 + j] = A[j][i];

    _jacobi_svd_nxn(At, V, S, W, 3);

    /* Sort singular values descending (bubble — only 3 elements) */
    for (int i = 0; i < 2; i++) {
        for (int j = i+1; j < 3; j++) {
            if (S[j] > S[i]) {
                float tmp = S[i]; S[i] = S[j]; S[j] = tmp;
                /* swap columns i,j of At and V */
                for (int k = 0; k < 3; k++) {
                    float t = At[i*3+k]; At[i*3+k] = At[j*3+k]; At[j*3+k] = t;
                    t = V[k*3+i]; V[k*3+i] = V[k*3+j]; V[k*3+j] = t;
                }
            }
        }
    }

    /* U[:,i] = At[i] / S[i]  (columns of U are left singular vectors) */
    for (int i = 0; i < 3; i++) {
        float inv_s = (S[i] > CV_EPS_F) ? 1.0f / S[i] : 0.0f;
        for (int k = 0; k < 3; k++)
            U[k][i] = At[i*3 + k] * inv_s;
    }

    /* Special case: when S[2] ≈ 0 (essential matrix always has a zero singular
     * value), U[:,2] = At[2]/S[2] = 0/0 → set to zero above.  Recover it as
     * the cross product of the first two columns so that U is a proper rotation
     * matrix (det = +1).  This matches OpenCV's handling of the zero singular
     * value in decomposeEssentialMat. */
    if (S[2] <= CV_EPS_F) {
        U[0][2] = U[1][0]*U[2][1] - U[2][0]*U[1][1];
        U[1][2] = U[2][0]*U[0][1] - U[0][0]*U[2][1];
        U[2][2] = U[0][0]*U[1][1] - U[1][0]*U[0][1];
    }

    /* Vt[i,:] = V[:,i] transposed  (Vt rows = V columns) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Vt[i][j] = V[j*3 + i];
}

/* ---- 4×4 SVD ---- */
static void jacobi_svd_4x4(const float A[4][4],
                            float U[4][4],
                            float S[4],
                            float Vt[4][4])
{
    float At[16], V[16], W[4];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            At[i*4 + j] = A[j][i];

    _jacobi_svd_nxn(At, V, S, W, 4);

    /* Sort descending */
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

    for (int i = 0; i < 4; i++) {
        float inv_s = (S[i] > CV_EPS_F) ? 1.0f / S[i] : 0.0f;
        for (int k = 0; k < 4; k++)
            U[k][i] = At[i*4 + k] * inv_s;
    }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Vt[i][j] = V[j*4 + i];
}

// ===========================================================================
// §3  sym_mineig_9 — smallest eigenvector of a 9×9 symmetric matrix
//
//   Used in find_fundamental_mat to find the null vector of AᵀA.
//   Delegates to _jacobi_svd_nxn and picks the column of V with the
//   smallest singular value (= smallest eigenvalue of the symmetric M).
// ===========================================================================

static void sym_mineig_9(const float M[9][9], float v[9])
{
    /* At = Mᵀ = M  (symmetric), stored row-major so each row is a column of M */
    float At[81], V[81], S[9], W[9];

    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
            At[i*9 + j] = M[i][j];

    _jacobi_svd_nxn(At, V, S, W, 9);

    /* Find the column of V corresponding to the smallest singular value */
    int min_idx = 0;
    for (int i = 1; i < 9; i++)
        if (S[i] < S[min_idx]) min_idx = i;

    /* v = V[:,min_idx]  (column min_idx of V) */
    for (int k = 0; k < 9; k++)
        v[k] = V[k*9 + min_idx];
}

// ===========================================================================
// §4  find_fundamental_mat
//
//   Hartley normalised 8-point algorithm.
//   Derived from OpenCV modules/calib3d/src/fundam.cpp :: run8Point().
// ===========================================================================

int find_fundamental_mat(const float pts_a[][2],
                         const float pts_b[][2],
                         int         n,
                         float       F_out[3][3])
{
    if (n < 8) return 0;

    /* ---- Step 1: Isotropic Hartley normalisation ---- */

    /* Compute centroids */
    float cx1 = 0.0f, cy1 = 0.0f;
    float cx2 = 0.0f, cy2 = 0.0f;
    for (int i = 0; i < n; i++) {
        cx1 += pts_a[i][0]; cy1 += pts_a[i][1];
        cx2 += pts_b[i][0]; cy2 += pts_b[i][1];
    }
    cx1 /= n; cy1 /= n;
    cx2 /= n; cy2 /= n;

    /* Mean distances to centroids */
    float mean_dist1 = 0.0f, mean_dist2 = 0.0f;
    for (int i = 0; i < n; i++) {
        float dx1 = pts_a[i][0] - cx1, dy1 = pts_a[i][1] - cy1;
        float dx2 = pts_b[i][0] - cx2, dy2 = pts_b[i][1] - cy2;
        mean_dist1 += sqrtf(dx1*dx1 + dy1*dy1);
        mean_dist2 += sqrtf(dx2*dx2 + dy2*dy2);
    }
    mean_dist1 /= n;
    mean_dist2 /= n;

    /* Guard against degenerate clouds */
    if (mean_dist1 < CV_EPS_F || mean_dist2 < CV_EPS_F) return 0;

    float scale1 = 1.4142135f / mean_dist1;  /* sqrt(2) */
    float scale2 = 1.4142135f / mean_dist2;

    /* Normalisation transforms T1, T2 (used for denormalisation at the end) */
    float T1[3][3] = {
        {scale1, 0.0f,   -scale1 * cx1},
        {0.0f,   scale1, -scale1 * cy1},
        {0.0f,   0.0f,    1.0f        }
    };
    float T2[3][3] = {
        {scale2, 0.0f,   -scale2 * cx2},
        {0.0f,   scale2, -scale2 * cy2},
        {0.0f,   0.0f,    1.0f        }
    };

    /* ---- Step 2: Accumulate 9×9 normal equations M = AᵀA ---- */

    float M[9][9];
    memset(M, 0, sizeof(M));

    for (int i = 0; i < n; i++) {
        float x1 = (pts_a[i][0] - cx1) * scale1;
        float y1 = (pts_a[i][1] - cy1) * scale1;
        float x2 = (pts_b[i][0] - cx2) * scale2;
        float y2 = (pts_b[i][1] - cy2) * scale2;

        float r[9] = {
            x2*x1, x2*y1, x2,
            y2*x1, y2*y1, y2,
            x1,    y1,    1.0f
        };

        /* M += outer(r, r) */
        for (int p = 0; p < 9; p++)
            for (int q = 0; q < 9; q++)
                M[p][q] += r[p] * r[q];
    }

    /* ---- Step 3: Null vector via smallest eigenvector of M ---- */

    float f[9];
    sym_mineig_9(M, f);

    /* Reshape into 3×3 (row-major) */
    float F_full[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F_full[i][j] = f[i*3 + j];

    /* ---- Step 4: Enforce rank 2 ---- */

    float U[3][3], S[3], Vt[3][3];
    jacobi_svd_3x3(F_full, U, S, Vt);
    S[2] = 0.0f;   /* zero the smallest singular value */

    float F_norm[3][3];
    mat3_diag_mul(U, S, Vt, F_norm);

    /* ---- Step 5: Denormalise: F = T2ᵀ * F_norm * T1 ---- */

    float T2t[3][3];
    mat3_transpose(T2, T2t);

    float tmp[3][3];
    mat3_mul(T2t, F_norm, tmp);
    mat3_mul(tmp, T1, F_out);

    /* Normalise by Frobenius norm (OpenCV convention).
     * Do NOT normalise by F[2][2]: for a horizontal camera baseline the
     * exact F has F[2][2] = 0, so dividing by it amplifies numerical noise
     * by many orders of magnitude. */
    float frob = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            frob += F_out[i][j] * F_out[i][j];
    frob = sqrtf(frob);
    if (frob > CV_EPS_F) {
        float inv = 1.0f / frob;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                F_out[i][j] *= inv;
    }

    return 1;
}

// ===========================================================================
// §5  recover_pose
//
//   K = I  ⟹  E = F.
//   Derived from OpenCV five-point.cpp :: decomposeEssentialMat + recoverPose.
// ===========================================================================

/* Helper: triangulate a single point pair given P1, P2 and return depth pair */
static void _triangulate_one(const float P1[3][4],
                              const float P2[3][4],
                              float x1, float y1,
                              float x2, float y2,
                              float Xout[4])
{
    /* Build 4×4 DLT matrix in standard form:
     *   rows = equations (one per point/projection pair)
     *   cols = homogeneous coordinate components [X, Y, Z, W]
     *
     *   row 0: x1*P1[2,:] - P1[0,:]    ← derived from x1 = (P1[0]*X)/(P1[2]*X)
     *   row 1: y1*P1[2,:] - P1[1,:]
     *   row 2: x2*P2[2,:] - P2[0,:]
     *   row 3: y2*P2[2,:] - P2[1,:]
     *
     * k iterates over the columns (= homogeneous coordinate components).
     * This matches the convention in OpenCV triangulate.cpp (j-loop over k).
     */
    float A[4][4];
    for (int k = 0; k < 4; k++) {
        A[0][k] = x1 * P1[2][k] - P1[0][k];
        A[1][k] = y1 * P1[2][k] - P1[1][k];
        A[2][k] = x2 * P2[2][k] - P2[0][k];
        A[3][k] = y2 * P2[2][k] - P2[1][k];
    }

    float U4[4][4], S4[4], Vt4[4][4];
    jacobi_svd_4x4(A, U4, S4, Vt4);
    /* Last row of Vt = last right singular vector of A = null vector of A.
     * (Corresponds to the smallest singular value, which is sorted last.) */
    for (int k = 0; k < 4; k++)
        Xout[k] = Vt4[3][k];
}

void recover_pose(const float F[3][3],
                  const float pts_a[][2],
                  const float pts_b[][2],
                  int         n,
                  float       R_out[3][3],
                  float       t_out[3])
{
    /* Since K = I,  E = F */
    float E[3][3];
    memcpy(E, F, sizeof(E));

    /* ---- Step 1: SVD of E (single pass — mirrors OpenCV five-point.cpp)
     *
     * OpenCV does NOT re-decompose after averaging the two non-zero singular
     * values.  It uses the U from this single decomposition for both:
     *   - t = U[:,2]  (third left singular vector = null vector direction)
     *   - R = U * W * Vt  (or U * Wᵀ * Vt)
     *
     * A second decomposition would set S[2] = 0 which triggers the
     * "inv_s = 0" guard in jacobi_svd, zeroing U[:,2] entirely and
     * making t = [0, 0, 0].
     * -------------------------------------------------------------------- */
    float U[3][3], S3[3], Vt[3][3];
    jacobi_svd_3x3(E, U, S3, Vt);
    /* (The averaging step is kept for numerical conditioning but we do not
     * re-decompose — U and Vt from above are what matter.) */
    float s_avg = (S3[0] + S3[1]) * 0.5f;
    (void)s_avg;  /* singular values not used after this point */

    /* ---- Step 2: W matrix (from OpenCV decomposeEssentialMat) ---- */
    float W[3][3] = {
        { 0.0f, -1.0f,  0.0f},
        { 1.0f,  0.0f,  0.0f},
        { 0.0f,  0.0f,  1.0f}
    };
    float Wt[3][3] = {
        { 0.0f,  1.0f,  0.0f},
        {-1.0f,  0.0f,  0.0f},
        { 0.0f,  0.0f,  1.0f}
    };

    /* ---- Step 3: Four candidate (R, t) pairs ---- */
    float UW[3][3], UWt[3][3];
    mat3_mul(U, W,  UW);
    mat3_mul(U, Wt, UWt);

    float R1[3][3], R2[3][3];
    mat3_mul(UW,  Vt, R1);
    mat3_mul(UWt, Vt, R2);

    /* Fix reflections: det(R) must be +1 */
    if (mat3_det(R1) < 0.0f)
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R1[i][j] = -R1[i][j];
    if (mat3_det(R2) < 0.0f)
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R2[i][j] = -R2[i][j];

    /* t_pos = third column of U */
    float t_pos[3] = { U[0][2], U[1][2], U[2][2] };
    float t_neg[3] = { -t_pos[0], -t_pos[1], -t_pos[2] };

    /* Candidate table: (R, t) */
    const float (*R_cands[4])[3] = { R1, R1, R2, R2 };
    const float (*t_cands[4])    = { t_pos, t_neg, t_pos, t_neg };

    /* ---- Step 4: Cheirality check ---- */

    /* P1 = [I | 0]  (camera A at origin) */
    float P1[3][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f}
    };

    int best_count = -1;

    for (int c = 0; c < 4; c++) {
        const float (*Rc)[3] = R_cands[c];
        const float *tc      = t_cands[c];

        /* P2 = [Rc | tc]  (OpenCV convention: world-to-cam2 rotation + translation)
         *
         * Camera-B projects a world point X as:  Rc * X + tc.
         * Depth in camera B = (Rc * X + tc)[2] = Rc[2,:] * X + tc[2].
         *
         * Using this convention (not the transposed form) keeps the
         * triangulation and the depth check consistent, and matches
         * exactly how OpenCV's recoverPose cheirality check works.
         */
        float P2[3][4];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++)
                P2[i][j] = Rc[i][j];   /* direct R */
            P2[i][3] = tc[i];           /* direct t */
        }

        /* Count points with positive depth in both cameras */
        int count = 0;
        for (int i = 0; i < n; i++) {
            float Xh[4];
            _triangulate_one(P1, P2,
                             pts_a[i][0], pts_a[i][1],
                             pts_b[i][0], pts_b[i][1],
                             Xh);
            if (fabsf(Xh[3]) < CV_EPS_F) continue;

            float X[3] = {
                Xh[0] / Xh[3],
                Xh[1] / Xh[3],
                Xh[2] / Xh[3]
            };

            /* depth1 = Z in camera-A frame (P1=[I|0]) */
            float depth1 = X[2];

            /* depth2 = (Rc * X + tc)[2]  — consistent with P2=[Rc|tc] above */
            float depth2 = Rc[2][0]*X[0] + Rc[2][1]*X[1] + Rc[2][2]*X[2] + tc[2];

            if (depth1 > 0.0f && depth2 > 0.0f) count++;
        }

        if (count > best_count) {
            best_count = count;
            memcpy(R_out, Rc, sizeof(float[3][3]));
            t_out[0] = tc[0]; t_out[1] = tc[1]; t_out[2] = tc[2];
        }
    }

    /* Ensure unit translation */
    vec3_normalize(t_out);
}

// ===========================================================================
// §6  triangulate_points
//
//   DLT triangulation for n point pairs.
//   Derived from OpenCV modules/calib3d/src/triangulate.cpp
//
//   NOTE: fixed pts_a/pts_b swap present in the Python source.
//   This C version correctly uses pts_a with P1 and pts_b with P2.
// ===========================================================================

void triangulate_points(const float P1[3][4],
                        const float P2[3][4],
                        const float pts_a[][2],
                        const float pts_b[][2],
                        int         n,
                        float       pts3d_out[][3])
{
    for (int i = 0; i < n; i++) {
        float x1 = pts_a[i][0], y1 = pts_a[i][1];
        float x2 = pts_b[i][0], y2 = pts_b[i][1];

        float Xh[4];
        _triangulate_one(P1, P2, x1, y1, x2, y2, Xh);

        if (fabsf(Xh[3]) > CV_EPS_F) {
            pts3d_out[i][0] = Xh[0] / Xh[3];
            pts3d_out[i][1] = Xh[1] / Xh[3];
            pts3d_out[i][2] = Xh[2] / Xh[3];
        } else {
            /* Degenerate: put point at infinity → zero */
            pts3d_out[i][0] = 0.0f;
            pts3d_out[i][1] = 0.0f;
            pts3d_out[i][2] = 0.0f;
        }
    }
}
