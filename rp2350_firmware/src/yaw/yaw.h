/**
 * @file  yaw.h
 * @brief Yaw estimator — fuses rigid-body sensor geometry with velocity-derived
 *        heading using inverse-variance circular weighting.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "solve3d/solve3d.h"   /* lh2_point3d_t, NUM_BS */

/**
 * Persistent state for the velocity-derived heading estimator.
 * Zero-initialise before first call; var must start at 9.87f (π² = "unknown").
 */
typedef struct {
    float    yaw;       /**< last velocity-derived heading [rad] */
    float    var;       /**< angular variance [rad²]; 9.87 = unknown */
    float    prev_cx;
    float    prev_cy;
    uint64_t prev_us;
} yaw_vel_state_t;

/**
 * Fuse rigid-body sensor yaw + velocity-derived yaw into a single estimate.
 *
 * The sensor estimate uses world-frame positions of sensor pairs whose
 * body-frame displacement is known (body-X or body-Y).  Pairs are fused via
 * circular mean; variance comes from YAW_VAR_TABLE indexed by valid-pair count.
 *
 * The velocity estimate is derived from position differencing and has speed-
 * dependent variance: vel_var = 2·pos_var / (speed·dt)².  The faster the
 * drone moves, the more accurate the atan2(vy,vx) heading estimate.
 *
 * Both estimates are blended with inverse-variance weighting on the circle.
 * A 90° consistency gate prevents blending when estimates disagree (e.g.
 * lateral translation giving a heading perpendicular to the body axis).
 *
 * @param pts      Triangulated sensor positions from solve3d_calib_run().
 * @param n_pts    Number of entries in pts[].
 * @param cx,cy    Current position centroid [m].
 * @param now_us   Current time [µs].
 * @param pos_var  Position variance [m²] — used to scale velocity yaw noise.
 * @param vel      Velocity yaw state (updated in-place each call).
 * @param yaw_out  Fused heading [rad] (output).
 * @param var_out  Fused variance [rad²] (output).
 * @param q_out    Quaternion [w,x,y,z] for yaw_out (output, pure-Z rotation).
 */
void yaw_fuse(const lh2_point3d_t *pts, int n_pts,
              float cx, float cy, uint64_t now_us,
              float pos_var,
              yaw_vel_state_t *vel,
              float *yaw_out, float *var_out,
              float q_out[4]);
