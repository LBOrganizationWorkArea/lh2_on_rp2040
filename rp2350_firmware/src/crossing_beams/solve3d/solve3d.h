/**
 * @file   solve3d.h
 * @brief  C port of data_processing.py :: solve_3d_scene()
 *
 * Accumulates a ring buffer of (pixel_a, pixel_b, sensor_id) samples,
 * then runs the full epipolar pipeline:
 *
 *   1. Project LH2 az/el angles onto the z=1 image plane → 2D pixels
 *   2. find_fundamental_mat(pts_a, pts_b)  → F
 *   3. recover_pose(F, pts_a, pts_b)       → R, t
 *   4. Build P1, P2 projection matrices
 *   5. triangulate_points(P1, P2, pts_a, pts_b) → 3D points
 *
 * Depends on: cv/cv.h
 */

#ifndef SOLVE3D_H
#define SOLVE3D_H

#include <stdbool.h>
#include <stdint.h>

/** Maximum number of history samples kept in the ring buffer. */
#define SOLVE3D_MAX_SAMPLES 32

/** Minimum samples required before attempting a solve. */
#define SOLVE3D_MIN_SAMPLES 8

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/**
 * @brief  One measurement: both lighthouses observed the same sensor.
 *
 * px_a and px_b are the result of angles_to_pixels() — projection of
 * the respective (az, el) pair onto the z=1 image plane.
 */
typedef struct {
    float   px_a[2];     ///< [ tan(az_a), tan(el_a)/cos(az_a) ]
    float   px_b[2];     ///< [ tan(az_b), tan(el_b)/cos(az_b) ]
    uint8_t sensor_id;   ///< which TS4231 sensor produced this measurement
} lh2_sample_t;

/**
 * @brief  One 3D output point.
 */
typedef struct {
    float   xyz[3];      ///< Cartesian coordinates [metres, scale-ambiguous until D_BS known]
    uint8_t sensor_id;   ///< sensor that produced this point
} lh2_point3d_t;

/**
 * @brief  Solver context — ring buffer + cached pose from last solve.
 */
typedef struct {
    lh2_sample_t history[SOLVE3D_MAX_SAMPLES];  ///< circular history buffer
    int          n_samples;    ///< current fill level (0 .. MAX_SAMPLES)
    int          head;         ///< ring-buffer write head (next free slot)
    float        R[3][3];      ///< cached rotation from last successful solve
    float        t[3];         ///< cached unit translation
    bool         pose_valid;   ///< true once at least one solve succeeded
} solve3d_ctx_t;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Initialise a solver context.
 */
void solve3d_init(solve3d_ctx_t *ctx);

/**
 * @brief  Project LH2 azimuth + elevation onto the z=1 image plane.
 *
 * Direct port of data_processing.py :: LH2_angles_to_pixels():
 *
 *   px[0] = tan(az_rad)
 *   px[1] = tan(el_rad) / cos(az_rad)
 *
 * @param az_rad   azimuth   in radians
 * @param el_rad   elevation in radians
 * @param px_out   output: [px_x, px_y]
 */
void angles_to_pixels(float az_rad, float el_rad, float px_out[2]);

/**
 * @brief  Add one sample to the ring buffer; overwrites the oldest when full.
 */
void solve3d_push_sample(solve3d_ctx_t *ctx, const lh2_sample_t *s);

/**
 * @brief  Run solve_3d_scene on the current history buffer.
 *
 * Requires ctx->n_samples >= SOLVE3D_MIN_SAMPLES.
 *
 * On success, writes ctx->n_samples points into pts3d_out,
 * caches R and t in the context, and returns ctx->n_samples.
 *
 * Returns 0 if there are too few samples or find_fundamental_mat fails.
 *
 * @param ctx        solver context (updated in place)
 * @param pts3d_out  caller-allocated array of at least SOLVE3D_MAX_SAMPLES entries
 * @return           number of 3D points written, or 0 on failure
 */
int solve3d_run(solve3d_ctx_t *ctx, lh2_point3d_t *pts3d_out);

#endif /* SOLVE3D_H */
