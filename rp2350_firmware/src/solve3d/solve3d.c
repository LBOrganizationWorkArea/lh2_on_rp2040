/**
 * @file   solve3d.c
 * @brief  Crossing-beams skew-lines solver (Taffanel 2021, Section II-B, eq. 2–3).
 *
 * Replaces the DLT triangulator with the midpoint-of-closest-approach on two
 * skew rays built from calibrated base-station poses.  The calibration data
 * format (lh2_bs_pose_t: origin + R) and public API are unchanged.
 *
 * Algorithm:
 *   1. Build a unit ray in each BS-local frame from (horiz, vert) angles:
 *        d_loc = [1, tan(h), tan(v)] / norm   (boresight=+X, horiz=+Y, vert=+Z)
 *   2. Rotate to world frame:   d_world = R @ d_loc
 *   3. Find closest-approach parameters on the two skew lines (Taffanel eq. 2–3)
 *   4. Midpoint of the closest-approach segment = position estimate
 *   5. Discard if ray gap > CB_MAX_GAP_M (quality filter from Taffanel 2021)
 */

#include "solve3d.h"
#include <math.h>

/* Rays further apart than this are discarded (Taffanel 2021 threshold: 0.1 m). */
#define CB_MAX_GAP_M 0.10f

int solve3d_calib_run(const lh2_bs_pose_t bs[NUM_BS],
                      const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                      uint64_t            now_us,
                      lh2_point3d_t      *pts_out)
{
    int n = 0;
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (!angle_decoder_is_fresh(angles, s, now_us))
            continue;

        float h0 = angles[s][0].ema_horiz, v0 = angles[s][0].ema_vert;
        float h1 = angles[s][1].ema_horiz, v1 = angles[s][1].ema_vert;

        /* Local ray: boresight=+X, horiz=+Y, vert=+Z  →  [1, tan(h), tan(v)]. */
        float d0l[3] = {1.0f, tanf(h0), tanf(v0)};
        float d1l[3] = {1.0f, tanf(h1), tanf(v1)};

        float n0 = sqrtf(d0l[0]*d0l[0] + d0l[1]*d0l[1] + d0l[2]*d0l[2]);
        float n1 = sqrtf(d1l[0]*d1l[0] + d1l[1]*d1l[1] + d1l[2]*d1l[2]);
        if (n0 < 1e-9f || n1 < 1e-9f) continue;
        d0l[0] /= n0;  d0l[1] /= n0;  d0l[2] /= n0;
        d1l[0] /= n1;  d1l[1] /= n1;  d1l[2] /= n1;

        /* Rotate to world frame: d_world = R @ d_local  (R is local→world). */
        float d0[3], d1[3];
        for (int r = 0; r < 3; r++) {
            d0[r] = bs[0].R[r][0]*d0l[0] + bs[0].R[r][1]*d0l[1] + bs[0].R[r][2]*d0l[2];
            d1[r] = bs[1].R[r][0]*d1l[0] + bs[1].R[r][1]*d1l[1] + bs[1].R[r][2]*d1l[2];
        }

        /* Skew-lines closest-approach (Taffanel 2021 eq. 2–3).
         * d0 and d1 are unit vectors so a = c = 1. */
        const float *o0 = bs[0].origin;
        const float *o1 = bs[1].origin;

        float w[3]  = {o0[0]-o1[0], o0[1]-o1[1], o0[2]-o1[2]};
        float b     = d0[0]*d1[0] + d0[1]*d1[1] + d0[2]*d1[2];
        float dw    = d0[0]*w[0]  + d0[1]*w[1]  + d0[2]*w[2];
        float ew    = d1[0]*w[0]  + d1[1]*w[1]  + d1[2]*w[2];

        float denom = 1.0f - b*b;
        if (fabsf(denom) < 1e-6f) continue;   /* parallel rays */

        float t0 = (b*ew - dw) / denom;
        float t1 = (ew - b*dw) / denom;

        /* Closest points on each ray. */
        float c0[3] = {o0[0]+t0*d0[0], o0[1]+t0*d0[1], o0[2]+t0*d0[2]};
        float c1[3] = {o1[0]+t1*d1[0], o1[1]+t1*d1[1], o1[2]+t1*d1[2]};

        /* Quality filter: discard if rays don't nearly intersect. */
        float gx = c0[0]-c1[0], gy = c0[1]-c1[1], gz = c0[2]-c1[2];
        if (gx*gx + gy*gy + gz*gz > CB_MAX_GAP_M * CB_MAX_GAP_M) continue;

        pts_out[n].xyz[0]    = (c0[0]+c1[0]) * 0.5f;
        pts_out[n].xyz[1]    = (c0[1]+c1[1]) * 0.5f;
        pts_out[n].xyz[2]    = (c0[2]+c1[2]) * 0.5f;
        pts_out[n].sensor_id = (uint8_t)s;
        n++;
    }

    return n;
}
