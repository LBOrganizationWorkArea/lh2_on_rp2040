/**
 * @file   solve3d.c
 * @brief  Calibrated-pose crossing-beams solver — see solve3d.h.
 *
 * Each sensor is triangulated from every calibrated base station that sees it
 * (least-squares closest point to their rays), with the calibrated poses.
 */

#include "solve3d.h"

#include <math.h>
#include <stdbool.h>

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
// Multi-station triangulation
//
// Every base station that sees a sensor gives a ray: from the station origin along
//     d = R · normalize(1, tan horiz, tan vert)      (local +X = boresight)
// The sensor is the point closest to all the rays in the least-squares sense:
//     Σ (I − d dᵀ) · x = Σ (I − d dᵀ) · o
// a 3×3 system, well conditioned as long as the rays are not parallel. For two
// stations this is the midpoint of closest approach — what the calibration tools use
// (utils/calibration/calibrate_bitcraze.py, triangulate_sensors), so the firmware and
// the calibration agree on the geometry. Solved in double: cheap, no precision worry.
// ---------------------------------------------------------------------------

/** Ray direction (unit, world frame) of one station's (horiz, vert) angles. */
static void _ray_direction(const lh2_bs_pose_t *bs, float horiz, float vert, double d[3])
{
    double local[3] = { 1.0, tan((double)horiz), tan((double)vert) };
    double n = sqrt(local[0] * local[0] + local[1] * local[1] + local[2] * local[2]);
    for (int r = 0; r < 3; r++) {
        d[r] = (bs->R[r][0] * local[0] + bs->R[r][1] * local[1] + bs->R[r][2] * local[2]) / n;
    }
}

/**
 * Least-squares point of @p n rays (origins @p o, unit directions @p d), skipping ray @p skip.
 * Returns false if the rays are (nearly) parallel.
 */
static bool _closest_point(const double o[][3], const double d[][3], int n, int skip, double x[3])
{
    double A[3][3] = { { 0 } }, b[3] = { 0 };
    for (int k = 0; k < n; k++) {
        if (k == skip) continue;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double m = (i == j ? 1.0 : 0.0) - d[k][i] * d[k][j];
                A[i][j] += m;
                b[i] += m * o[k][j];
            }
        }
    }
    double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
               - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
               + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (fabs(det) < 1e-9) return false;
    for (int c = 0; c < 3; c++) {  /* Cramer's rule */
        double M[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) M[i][j] = (j == c) ? b[i] : A[i][j];
        x[c] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
              - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
              + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
    }
    return true;
}

/** Distance from point @p x to the ray (o, unit d). */
static double _ray_distance(const double o[3], const double d[3], const double x[3])
{
    double v[3] = { x[0] - o[0], x[1] - o[1], x[2] - o[2] };
    double t = v[0] * d[0] + v[1] * d[1] + v[2] * d[2];
    double e[3] = { v[0] - t * d[0], v[1] - t * d[1], v[2] - t * d[2] };
    return sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

int solve3d_calib_run(const lh2_bs_pose_t bs[NUM_BS],
                      const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                      uint64_t            now_us,
                      lh2_point3d_t      *pts_out)
{
    int n = 0;
    for (int s = 0; s < NUM_SENSORS; s++) {
        double o[NUM_BS][3], d[NUM_BS][3];
        int views = 0;
        for (int b = 0; b < NUM_BS; b++) {
            if (!bs_pose_valid(&bs[b]) || !angle_decoder_bs_fresh(angles, s, b, now_us)) continue;
            for (int r = 0; r < 3; r++) o[views][r] = bs[b].origin[r];
            _ray_direction(&bs[b], angles[s][b].ema_horiz, angles[s][b].ema_vert, d[views]);
            views++;
        }
        if (views < MIN_BS_FOR_POINT) continue;

        double x[3];
        if (!_closest_point(o, d, views, -1, x)) continue;

        /* With three or more stations, one inconsistent ray (mis-decoded sweep, reflection)
         * is dropped when it is far from the others' point. */
        if (views >= 3) {
            int worst = 0;
            double worst_d = 0.0;
            for (int k = 0; k < views; k++) {
                double dist = _ray_distance(o[k], d[k], x);
                if (dist > worst_d) { worst_d = dist; worst = k; }
            }
            double y[3];
            if (worst_d > SOLVE3D_OUTLIER_M && _closest_point(o, d, views, worst, y)) {
                x[0] = y[0]; x[1] = y[1]; x[2] = y[2];
                views--;
            }
        }

        pts_out[n].xyz[0]    = (float)x[0];
        pts_out[n].xyz[1]    = (float)x[1];
        pts_out[n].xyz[2]    = (float)x[2];
        pts_out[n].sensor_id = (uint8_t)s;
        pts_out[n].n_views   = (uint8_t)views;
        n++;
    }
    return n;
}
