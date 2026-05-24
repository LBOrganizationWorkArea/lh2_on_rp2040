/**
 * @file   solve3d.c
 * @brief  C port of data_processing.py :: solve_3d_scene()
 *
 * Python reference (data_processing.py):
 *
 *   pts_a    = [LH2_angles_to_pixels(az_a, el_a) for each sensor sample]
 *   pts_b    = [LH2_angles_to_pixels(az_b, el_b) for each sensor sample]
 *   F        = cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_LMEDS)[0]
 *   R, t, *_ = cv2.recoverPose(F, pts_a, pts_b)
 *   P1       = np.hstack([np.eye(3), np.zeros((3,1))])
 *   P2       = np.hstack([R.T, -R.T @ t])
 *   point3d  = cv2.triangulatePoints(P1, P2, pts_b.T, pts_a.T)[:3]  ← swapped
 */

#include "solve3d.h"
#include "../cv/cv.h"

#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void solve3d_init(solve3d_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->n_samples  = 0;
    ctx->head       = 0;
    ctx->pose_valid = false;
}

void angles_to_pixels(float az_rad, float el_rad, float px_out[2])
{
    /* Direct port of LH2_angles_to_pixels() from data_processing.py:
     *   px = [tan(az), tan(el) / cos(az)]
     * This projects the lighthouse azimuth/elevation onto the z=1 image plane
     * of a pinhole camera with identity intrinsics. */
    float cos_az = cosf(az_rad);
    px_out[0] = tanf(az_rad);
    px_out[1] = (fabsf(cos_az) > 1e-9f)
                    ? (tanf(el_rad) / cos_az)
                    : 0.0f;
}

void solve3d_push_sample(solve3d_ctx_t *ctx, const lh2_sample_t *s)
{
    ctx->history[ctx->head] = *s;
    ctx->head = (ctx->head + 1) % SOLVE3D_MAX_SAMPLES;
    if (ctx->n_samples < SOLVE3D_MAX_SAMPLES)
        ctx->n_samples++;
}

int solve3d_run(solve3d_ctx_t *ctx, lh2_point3d_t *pts3d_out)
{
    int n = ctx->n_samples;
    if (n < SOLVE3D_MIN_SAMPLES) return 0;

    /* ---- Step 1: Unpack history into flat point arrays ---- */

    /* History is a ring buffer; read out in order from oldest to newest. */
    float pts_a[SOLVE3D_MAX_SAMPLES][2];
    float pts_b[SOLVE3D_MAX_SAMPLES][2];

    /* The oldest entry is at index (head) when buffer is full,
     * or at index 0 when not yet full. */
    int start = (n == SOLVE3D_MAX_SAMPLES) ? ctx->head : 0;

    for (int i = 0; i < n; i++) {
        int idx = (start + i) % SOLVE3D_MAX_SAMPLES;
        pts_a[i][0] = ctx->history[idx].px_a[0];
        pts_a[i][1] = ctx->history[idx].px_a[1];
        pts_b[i][0] = ctx->history[idx].px_b[0];
        pts_b[i][1] = ctx->history[idx].px_b[1];
    }

    /* ---- Step 2: Fundamental matrix ---- */

    float F[3][3];
    if (!find_fundamental_mat(pts_a, pts_b, n, F)) return 0;

    /* ---- Step 3: Recover R, t ---- */

    float R[3][3], t[3];
    recover_pose(F, pts_a, pts_b, n, R, t);

    /* Cache the pose */
    memcpy(ctx->R, R, sizeof(R));
    memcpy(ctx->t, t, sizeof(t));
    ctx->pose_valid = true;

    /* ---- Step 4: Build projection matrices ---- */

    /* P1 = [I₃ | 0]  — camera A at the origin */
    float P1[3][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f}
    };

    /* P2 = [R | t]  — camera B in OpenCV convention.
     *
     * recover_pose returns R (world-to-cam2 rotation) and t (unit translation
     * vector in cam2 frame), both in OpenCV's standard [R|t] convention.
     * The cheirality check in cv.c uses P2=[Rc|tc] consistently.
     *
     * Using [R|t] directly here keeps triangulation consistent with the
     * cheirality check and with how pts_b was generated.
     *
     * (The Python reference uses np.hstack([R.T, -R.T @ t]) which has a
     * documented pts_a/pts_b swap — the C version uses the correct ordering
     * with [R|t] instead.)
     */
    float P2[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            P2[i][j] = R[i][j];   /* direct R */
        P2[i][3] = t[i];           /* direct t */
    }

    /* ---- Step 5: Triangulate ---- */

    float pts3d[SOLVE3D_MAX_SAMPLES][3];
    triangulate_points(P1, P2, pts_a, pts_b, n, pts3d);

    /* ---- Step 6: Copy to output with sensor IDs ---- */

    for (int i = 0; i < n; i++) {
        int idx = (start + i) % SOLVE3D_MAX_SAMPLES;
        pts3d_out[i].xyz[0]    = pts3d[i][0];
        pts3d_out[i].xyz[1]    = pts3d[i][1];
        pts3d_out[i].xyz[2]    = pts3d[i][2];
        pts3d_out[i].sensor_id = ctx->history[idx].sensor_id;
    }

    return n;
}
