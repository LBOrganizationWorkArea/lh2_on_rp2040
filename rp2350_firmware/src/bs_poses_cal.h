/* bs_poses_cal.h — HAND-MEASURED + empirically-corrected roll (calibrate_export.py
 * pipeline is not usable, see .claude/rules/scale_calibration_bug.md). Do not
 * run calibrate_export.py over this. */
/*
 * World frame: +X along BS4 -> BS10, Z up, metres. Both stations sit at
 * y = -0.70 m, 3.45 m up, tilted 30 deg from straight down toward -Y.
 * BS-local +X = boresight. R is base-station-local -> world.
 *
 * R = Rx(-30 deg) * (straight-down roll-0 pose): local Y (horizontal sweep)
 * -> world +X, boresight (0, -sin30, -cos30), local Z (vertical sweep)
 * (0, -cos30, sin30).
 */
#ifndef BS_POSES_CAL_H
#define BS_POSES_CAL_H

#include "solve3d/solve3d.h"   /* lh2_bs_pose_t, NUM_BS */

#define BS_POSE_SOURCE "manual:2026-10-09-tilt30-y-0.70"

static const lh2_bs_pose_t BS_POSES[NUM_BS] = {
    {  /* BS0  (poly 8/9, BS4) — origin, tilted 30 deg from straight down toward -Y, roll 0 deg */
        .origin = {0.000000f, -0.700000f, 3.450000f},
        .R = { {0.000000f, 1.000000f, 0.000000f},
               {-0.500000f, 0.000000f, -0.866025f},
               {-0.866025f, 0.000000f, 0.500000f} },
    },
    {  /* BS1  (poly 20/21, BS10) — 2.26 m from BS4 along X, tilted 30 deg from straight down toward -Y, roll 0 deg */
        .origin = {2.260000f, -0.700000f, 3.450000f},
        .R = { {0.000000f, 1.000000f, 0.000000f},
               {-0.500000f, 0.000000f, -0.866025f},
               {-0.866025f, 0.000000f, 0.500000f} },
    },
};

#endif /* BS_POSES_CAL_H */
