/**
 * @file   solve3d.h
 * @brief  3D solver for the crossing-beams system — calibrated-pose skew-lines variant.
 *
 * Implements the Crossing Beam method from Taffanel 2021 (arXiv:2104.11523,
 * Section II-B, eq. 2–3): builds a unit ray in each base-station's local frame
 * from (horiz, vert) angles, rotates it to the world frame using the calibrated
 * rotation matrix R, then finds the midpoint of closest approach between the two
 * skew rays.  Rays with a gap > 0.1 m are discarded as low-quality.
 *
 * Depends on: angle_decoder (lh2_angles_t).
 */

#ifndef SOLVE3D_H
#define SOLVE3D_H

#include <stdint.h>

#include "angle_decoder/angle_decoder.h"   /* lh2_angles_t, NUM_SENSORS, NUM_BS */

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** One 3D output point in the calibrated world frame [metres]. */
typedef struct {
    float   xyz[3];
    uint8_t sensor_id;
} lh2_point3d_t;

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

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Triangulate a metric 3D point for every sensor both base stations see.
 *
 * For each sensor with fresh angles from both base stations:
 *   1. Build a unit ray in each BS-local frame from (horiz, vert) radians.
 *   2. Rotate to world frame using the calibrated R matrix.
 *   3. Find the midpoint of closest approach (skew-lines method).
 *   4. Discard if ray gap > 0.1 m.
 *
 * @param bs       array of NUM_BS calibrated base-station poses
 * @param angles   decoded angle table [NUM_SENSORS][NUM_BS]
 * @param now_us   current time [µs] for the freshness check
 * @param pts_out  caller buffer of at least NUM_SENSORS points
 * @return         number of points written (0 .. NUM_SENSORS)
 */
int solve3d_calib_run(const lh2_bs_pose_t bs[NUM_BS],
                      const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                      uint64_t            now_us,
                      lh2_point3d_t      *pts_out);

#endif /* SOLVE3D_H */
