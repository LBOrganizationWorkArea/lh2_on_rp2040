/**
 * @file   ray_cross.c
 * @brief  Direct ray-crossing 3D solver — see ray_cross.h.
 */

#include "ray_cross.h"

#include <math.h>

/* ---- helpers -------------------------------------------------------------- */

static inline float _dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* ---- public API ----------------------------------------------------------- */

void bs_ray_dir(const lh2_bs_pose_t *bs, float horiz, float vert, float dir_out[3])
{
    /* Local-frame ray: boresight is +X (Bitcraze convention). */
    float dl[3] = { 1.0f, tanf(horiz), tanf(vert) };

    float norm = sqrtf(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2]);
    float inv  = (norm > 1e-9f) ? (1.0f / norm) : 0.0f;
    dl[0] *= inv;
    dl[1] *= inv;
    dl[2] *= inv;

    /* World direction = R · d_local. */
    for (int i = 0; i < 3; i++) {
        dir_out[i] = bs->R[i][0] * dl[0]
                   + bs->R[i][1] * dl[1]
                   + bs->R[i][2] * dl[2];
    }
}

float ray_closest_point(const float o0[3], const float d0[3],
                        const float o1[3], const float d1[3],
                        float p_out[3])
{
    float w0[3] = { o0[0] - o1[0], o0[1] - o1[1], o0[2] - o1[2] };

    float a = _dot3(d0, d0);   /* = 1 for unit dirs, kept general */
    float b = _dot3(d0, d1);
    float c = _dot3(d1, d1);
    float d = _dot3(d0, w0);
    float e = _dot3(d1, w0);

    float denom = a * c - b * b;

    float s, t;
    if (fabsf(denom) < 1e-9f) {
        /* Rays near-parallel: pin ray 0, project onto ray 1. */
        s = 0.0f;
        t = (c > 1e-9f) ? (e / c) : 0.0f;
    } else {
        s = (b * e - c * d) / denom;
        t = (a * e - b * d) / denom;
    }

    float gap2 = 0.0f;
    for (int i = 0; i < 3; i++) {
        float p0 = o0[i] + s * d0[i];
        float p1 = o1[i] + t * d1[i];
        p_out[i] = 0.5f * (p0 + p1);
        float diff = p0 - p1;
        gap2 += diff * diff;
    }
    return sqrtf(gap2);
}

int ray_cross_solve(const lh2_bs_pose_t bs[NUM_BS],
                    const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                    uint64_t            now_us,
                    lh2_point3d_t      *pts_out)
{
    int n = 0;

    for (int s = 0; s < NUM_SENSORS; s++) {
        if (!angle_decoder_is_fresh(angles, s, now_us)) {
            continue;
        }

        float d0[3], d1[3];
        bs_ray_dir(&bs[0], angles[s][0].ema_horiz, angles[s][0].ema_vert, d0);
        bs_ray_dir(&bs[1], angles[s][1].ema_horiz, angles[s][1].ema_vert, d1);

        float p[3];
        ray_closest_point(bs[0].origin, d0, bs[1].origin, d1, p);

        pts_out[n].xyz[0]    = p[0];
        pts_out[n].xyz[1]    = p[1];
        pts_out[n].xyz[2]    = p[2];
        pts_out[n].sensor_id = (uint8_t)s;
        n++;
    }

    return n;
}
