/**
 * @file   solve3d.h
 * @brief  3D solver for the crossing-beams system — calibrated-pose variant.
 *
 * Started as the epipolar pipeline of data_processing.py (angles_to_pixels →
 * fundamental matrix → recover_pose → DLT), whose estimated pose was scale-ambiguous
 * and unstable. The base stations are fixed and calibrated, so their poses are used
 * directly; since the move to more than two stations each sensor is triangulated from
 * every station that sees it (least-squares closest point to the rays, see solve3d.c).
 *
 * Depends on: angle_decoder (lh2_angles_t). angles_to_pixels() is kept for the
 * original pipeline's tools.
 */

#ifndef SOLVE3D_H
#define SOLVE3D_H

#include <stdbool.h>
#include <stdint.h>

#include "angle_decoder/angle_decoder.h"   /* lh2_angles_t, NUM_SENSORS, NUM_BS */

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** One 3D output point in the calibrated world frame [metres]. */
typedef struct {
    float   xyz[3];
    uint8_t sensor_id;
    uint8_t n_views;    ///< base stations used for this point (2..NUM_BS)
} lh2_point3d_t;

/** A ray farther than this from the point of the others is dropped (3+ stations) [m]. */
#define SOLVE3D_OUTLIER_M 0.05
/* Rays older than this are left out (they used to count for up to FRESHNESS_US = 500 ms: log
 * 2026-10-26, 23 % of the rays used were > 100 ms old, the position lagged and its stamp
 * jumped). Matches EMA_RESET_US: after that long without data the angle filter restarts anyway. */
#define SOLVE3D_RAY_MAX_AGE_US 100000ULL

/**
 * @brief  A base station's pose in the calibrated world frame.
 *
 * R maps the base-station-local frame to the world frame (columns are the
 * local axes expressed in world coordinates). Local +X is the boresight.
 */
typedef struct {
    float origin[3];    ///< base-station position [m]
    float R[3][3];      ///< base-station-local → world rotation
} lh2_bs_pose_t;

/** False for an all-zero pose: that station is not calibrated yet (bs_poses_cal.h). */
static inline bool bs_pose_valid(const lh2_bs_pose_t *bs)
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            if (bs->R[r][c] != 0.0f) return true;
    return false;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Project lighthouse azimuth/elevation onto the z=1 image plane.
 *
 * From data_processing.py :: LH2_angles_to_pixels():
 *   px = [ tan(az),  tan(el) / cos(az) ]
 * i.e. the normalised pinhole projection of a +Z-looking camera. Retained
 * unchanged from the original pipeline.
 */
void angles_to_pixels(float az_rad, float el_rad, float px_out[2]);

/**
 * @brief  Triangulate a metric 3D point for every sensor seen by >= MIN_BS_FOR_POINT stations.
 *
 * For each sensor, every calibrated station (bs_pose_valid) whose filtered angles were captured
 * at most SOLVE3D_RAY_MAX_AGE_US before now_us gives a ray, its angles carried forward to now_us
 * with the filter's angular rate: all rays describe the same moment, now_us (the ODOMETRY stamp).
 * The point is the least-squares closest point to those rays. With three or more rays, one
 * that is farther than SOLVE3D_OUTLIER_M from the others' point is dropped.
 *
 * @param bs       array of NUM_BS base-station poses (all-zero entries = not calibrated)
 * @param angles   decoded angle table [NUM_SENSORS][NUM_BS]
 * @param now_us   the moment the point describes [µs, capture clock]
 * @param pts_out  caller buffer of at least NUM_SENSORS points
 * @return         number of points written (0 .. NUM_SENSORS)
 */
int solve3d_calib_run(const lh2_bs_pose_t bs[NUM_BS],
                      const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                      uint64_t            now_us,
                      lh2_point3d_t      *pts_out);

#endif /* SOLVE3D_H */
