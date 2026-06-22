/**
 * @file   solve3d.c
 * @brief  Calibrated-pose crossing-beams solver — see solve3d.h.
 *
 * Keeps the original pipeline's projection (angles_to_pixels) and DLT
 * triangulation (cv triangulate_points); replaces the estimated relative pose
 * with projection matrices built from the calibrated base-station poses.
 */

#include "solve3d.h"
#include "../cv/cv.h"

#include <math.h>

// ---------------------------------------------------------------------------
// Projection (unchanged from data_processing.py)
// ---------------------------------------------------------------------------

void angles_to_pixels(float az_rad, float el_rad, float px_out[2])
{
    /* px = [tan(az), tan(el)/cos(az)] — normalised +Z pinhole projection. */
    float cos_az = cosf(az_rad);
    px_out[0] = tanf(az_rad);
    px_out[1] = (fabsf(cos_az) > 1e-9f) ? (tanf(el_rad) / cos_az) : 0.0f;
}

// ---------------------------------------------------------------------------
// Build a base station's world→image projection matrix P (3×4)
//
// The calibrated pose has local +X = boresight. A standard pinhole expects the
// optical (depth) axis to be the third image coordinate, so we map:
//     image-u (row 0) ← local +Y axis  (R column 1)
//     image-v (row 1) ← local +Z axis  (R column 2)
//     depth   (row 2) ← local +X axis  (R column 0, boresight)
// Then for a world point X:  P·[X;1] = (localY, localZ, localX) of (X−origin),
// so the projected pixel = (localY/localX, localZ/localX) = (tan horiz, tan vert).
// ---------------------------------------------------------------------------

static void _bs_projection(const lh2_bs_pose_t *bs, float P[3][4])
{
    const float *o = bs->origin;

    for (int r = 0; r < 3; r++) {
        P[0][r] = bs->R[r][1];   /* u-axis  = local +Y */
        P[1][r] = bs->R[r][2];   /* v-axis  = local +Z */
        P[2][r] = bs->R[r][0];   /* depth   = local +X (boresight) */
    }
    P[0][3] = -(bs->R[0][1]*o[0] + bs->R[1][1]*o[1] + bs->R[2][1]*o[2]);
    P[1][3] = -(bs->R[0][2]*o[0] + bs->R[1][2]*o[1] + bs->R[2][2]*o[2]);
    P[2][3] = -(bs->R[0][0]*o[0] + bs->R[1][0]*o[1] + bs->R[2][0]*o[2]);
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

int solve3d_calib_run(const lh2_bs_pose_t bs[NUM_BS],
                      const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                      uint64_t            now_us,
                      lh2_point3d_t      *pts_out)
{
    float P0[3][4], P1[3][4];
    _bs_projection(&bs[0], P0);
    _bs_projection(&bs[1], P1);

    int n = 0;
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (!angle_decoder_is_fresh(angles, s, now_us)) {
            continue;
        }

        /* Project both base stations' angles to image pixels.
         *
         * angles_to_pixels() expects (azimuth, elevation) of a +Z-looking
         * pinhole. The calibrated angles are Bitcraze (horiz, vert) with the
         * boresight along +X, so the elevation fed to the pinhole is
         *   el = atan( tan(vert) · cos(horiz) )
         * which makes angles_to_pixels(horiz, el) = (tan horiz, tan vert) —
         * exactly the projection the matrices in _bs_projection() invert. */
        float h0 = angles[s][0].ema_horiz, v0 = angles[s][0].ema_vert;
        float h1 = angles[s][1].ema_horiz, v1 = angles[s][1].ema_vert;

        float pa[1][2], pb[1][2];
        angles_to_pixels(h0, atanf(tanf(v0) * cosf(h0)), pa[0]);
        angles_to_pixels(h1, atanf(tanf(v1) * cosf(h1)), pb[0]);

        float X[1][3];
        triangulate_points(P0, P1, pa, pb, 1, X);

        pts_out[n].xyz[0]    = X[0][0];
        pts_out[n].xyz[1]    = X[0][1];
        pts_out[n].xyz[2]    = X[0][2];
        pts_out[n].sensor_id = (uint8_t)s;
        n++;
    }

    return n;
}
