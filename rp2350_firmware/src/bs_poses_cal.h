/* bs_poses_cal.h — HAND-MEASURED + empirically-corrected roll (calibrate_export.py
 * pipeline is not usable, see .claude/rules/scale_calibration_bug.md). Do not
 * run calibrate_export.py over this. */
/*
 * World frame: origin at BS4, +X toward BS10, metres.
 * BS-local +X = boresight. R is base-station-local -> world.
 *
 * Roll: boresight still world -Z (pointing down), but rolled 90 deg about
 * that axis vs. the naive Ry(90) mount: local Y (horiz sweep) -> world -X,
 * local Z (vert sweep) -> world +Y. Direction confirmed empirically on
 * 2026-07-07: moving a sensor from under BS4 to under BS10 (true world +X)
 * showed up entirely on the OLD output's Y channel, decreasing — so local Y
 * maps to -X, not +X.
 */
#ifndef BS_POSES_CAL_H
#define BS_POSES_CAL_H

#include "solve3d/solve3d.h"   /* lh2_bs_pose_t, NUM_BS */

#define BS_POSE_SOURCE "manual:2026-07-07-rolled"

static const lh2_bs_pose_t BS_POSES[NUM_BS] = {
    {  /* BS0  (poly 8/9,  BS4) — origin, pointing straight down, rolled 90 deg */
        .origin = {0.000000f, 0.000000f, 3.450000f},
        .R = { {0.000000f, -1.000000f, 0.000000f},
               {0.000000f, 0.000000f, 1.000000f},
               {-1.000000f, 0.000000f, 0.000000f} },
    },
    {  /* BS1  (poly 20/21, BS10) — 2.26 m from BS4 along X, pointing straight down, rolled 90 deg */
        .origin = {2.260000f, 0.000000f, 3.450000f},
        .R = { {0.000000f, -1.000000f, 0.000000f},
               {0.000000f, 0.000000f, 1.000000f},
               {-1.000000f, 0.000000f, 0.000000f} },
    },
};

#endif /* BS_POSES_CAL_H */
