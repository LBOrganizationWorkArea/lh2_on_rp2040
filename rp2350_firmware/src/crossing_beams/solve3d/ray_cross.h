/**
 * @file   ray_cross.h
 * @brief  Direct ray-crossing 3D solver using calibrated lighthouse poses.
 *
 * Replaces the scale-ambiguous fundamental-matrix pipeline (solve3d.c) with
 * exact, metric triangulation: each base station + angle pair defines a ray in
 * world space, and the sensor sits at the closest point between the two rays.
 *
 * Requires known base-station poses (position + orientation), produced by the
 * calibration pipeline and baked into bs_poses_cal.h (see RAY_CROSSING_PLAN.md).
 *
 * Angle convention: Bitcraze (horiz, vert) in radians, where the base station's
 * local +X axis is its boresight. The local ray direction is
 *   d_local = normalize( (1, tan(horiz), tan(vert)) )
 * rotated into world space by the pose rotation R.
 */

#ifndef RAY_CROSS_H
#define RAY_CROSS_H

#include <stdint.h>

#include "angle_decoder/angle_decoder.h"   /* lh2_angles_t, NUM_SENSORS, NUM_BS */
#include "solve3d/solve3d.h"               /* lh2_point3d_t                      */

/**
 * @brief  One base station's pose in the calibrated world frame.
 */
typedef struct {
    float origin[3];    ///< base-station position [metres]
    float R[3][3];      ///< base-station-local → world rotation matrix
} lh2_bs_pose_t;

/**
 * @brief  World-frame unit ray direction for a (horiz, vert) angle pair.
 *
 * @param bs     base-station pose
 * @param horiz  Bitcraze horizontal angle [rad]
 * @param vert   Bitcraze vertical   angle [rad]
 * @param dir_out  output: unit direction in world frame
 */
void bs_ray_dir(const lh2_bs_pose_t *bs, float horiz, float vert, float dir_out[3]);

/**
 * @brief  Closest point between two skew rays (midpoint of nearest approach).
 *
 * @param o0,d0  first  ray origin + (unit) direction
 * @param o1,d1  second ray origin + (unit) direction
 * @param p_out  output: midpoint of the two nearest points
 * @return       gap between the two nearest points [m] — a free quality metric
 */
float ray_closest_point(const float o0[3], const float d0[3],
                        const float o1[3], const float d1[3],
                        float p_out[3]);

/**
 * @brief  Solve a metric 3D point for every sensor that both base stations see.
 *
 * For each sensor with fresh angles from both base stations, builds the two
 * world rays and intersects them. Writes one lh2_point3d_t per solved sensor.
 *
 * @param bs       array of NUM_BS base-station poses
 * @param angles   decoded angle table [NUM_SENSORS][NUM_BS]
 * @param now_us   current time [µs] — used for the freshness check
 * @param pts_out  caller buffer of at least NUM_SENSORS points
 * @return         number of points written (0 .. NUM_SENSORS)
 */
int ray_cross_solve(const lh2_bs_pose_t bs[NUM_BS],
                    const lh2_angles_t  angles[NUM_SENSORS][NUM_BS],
                    uint64_t            now_us,
                    lh2_point3d_t      *pts_out);

#endif /* RAY_CROSS_H */
