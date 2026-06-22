/**
 * @file   solve3d.h
 * @brief  3D solver for the crossing-beams system — calibrated-pose variant.
 *
 * This is the original epipolar-geometry pipeline from data_processing.py,
 * *enhanced* to use KNOWN base-station poses instead of estimating them.
 *
 * The original solve_3d_scene() did:
 *     angles_to_pixels → find_fundamental_mat → recover_pose → triangulate_points
 * The pose estimation (find_fundamental_mat + recover_pose) is scale-ambiguous
 * and ill-conditioned for a near-coplanar sensor cloud, which is what made the
 * output unstable. Since the base stations are fixed and calibrated, we keep the
 * validated front/back of the pipeline:
 *     angles_to_pixels  +  triangulate_points   (the DLT — unchanged)
 * and replace the estimated pose with projection matrices built directly from
 * the calibrated poses. The result is metric and stable.
 *
 * Depends on: cv/cv.h (triangulate_points), angle_decoder (lh2_angles_t).
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
 * @brief  Project lighthouse azimuth/elevation onto the z=1 image plane.
 *
 * From data_processing.py :: LH2_angles_to_pixels():
 *   px = [ tan(az),  tan(el) / cos(az) ]
 * i.e. the normalised pinhole projection of a +Z-looking camera. Retained
 * unchanged from the original pipeline.
 */
void angles_to_pixels(float az_rad, float el_rad, float px_out[2]);

/**
 * @brief  Triangulate a metric 3D point for every sensor both base stations see.
 *
 * For each sensor with fresh angles from both base stations:
 *   1. project the calibrated (horiz, vert) angles to pixels (angles_to_pixels),
 *   2. build each base station's world→image projection matrix from its pose,
 *   3. DLT-triangulate the pair (cv triangulate_points) → world point.
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
