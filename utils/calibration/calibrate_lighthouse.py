#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
  _     _    _ __  __ _____ _   _  ____  _    _  _____   ____  ______ ______  _____ 
 | |   | |  | |  \/  |_   _| \ | |/ __ \| |  | |/ ____| |  _ \|  ____|  ____|/ ____|
 | |   | |  | | \  / | | | |  \| | |  | | |  | | (___   | |_) | |__  | |__  | (___  
 | |   | |  | | |\/| | | | | . ` | |  | | |  | |\___ \  |  _ <|  __| |  __|  \___ \ 
 | |___| |__| | |  | |_| |_| |\  | |__| | |__| |____) | | |_) | |____| |____ ____) |
 |______\____/|_|  |_|_____|_| \_|\____/ \____/|_____/  |____/|______|______|_____/ 
                                                                                    
Giorgio Rinolfi
Victor Bianchi
Antoine el Kahi
Eduardo Gonzalez

Lighthouse calibration CLI - reuses the calibration pipeline from the wizard.

This script collects lighthouse measurements and produces calibrated base station
geometry through the same pipeline as the Qt-based calibration wizard, but as a
standalone command-line tool.

Workflow:
1. Collect/load measurements
2. Match them into pose samples
3. Build initial geometry guess
4. Solve geometry (least squares optimization)
5. Align to world frame
6. Scale using reference measurement
7. Save base station geometry
"""



import numpy as np

from lbees.indoor.angle_lib.lighthouse_types import (
    LhMeasurement, LhCfPoseSample, Pose, LhDeck4SensorPositions
)

from calibration_lib.lighthouse_sample_matcher import LighthouseSampleMatcher
from calibration_lib.lighthouse_initial_estimator import LighthouseInitialEstimator
from calibration_lib.lighthouse_geometry_solver import LighthouseGeometrySolver
from calibration_lib.lighthouse_system_aligner import LighthouseSystemAligner
from calibration_lib.lighthouse_system_scaler import LighthouseSystemScaler
from calibration_lib.lighthouse_config_manager import LighthouseConfigFileManager

REFERENCE_DIST = 1.0


class EstimateGeometryThread():
    def __init__(self, origin, x_axis, xy_plane, samples):
        super(EstimateGeometryThread, self).__init__()

        self.origin = origin
        self.x_axis = x_axis
        self.xy_plane = xy_plane
        self.samples = samples
        self.bs_poses = {}

    def run(self):
        try:
            self.bs_poses = self._estimate_geometry(self.origin, self.x_axis, self.xy_plane, self.samples)
            self.finished.emit()
        except Exception as ex:
            print(ex)
            self.failed.emit()

    def get_poses(self):
        return self.bs_poses

    def _estimate_geometry(self, origin: LhCfPoseSample,
                           x_axis: list[LhCfPoseSample],
                           xy_plane: list[LhCfPoseSample],
                           samples: list[LhCfPoseSample]) -> dict[int, Pose]:
        """Estimate the geometry of the system based on samples recorded by a Crazyflie"""
        matched_samples = [origin] + x_axis + xy_plane + LighthouseSampleMatcher.match(samples, min_nr_of_bs_in_match=2)
        initial_guess, cleaned_matched_samples = LighthouseInitialEstimator.estimate(matched_samples,
                                                                                     LhDeck4SensorPositions.positions)

        solution = LighthouseGeometrySolver.solve(initial_guess,
                                                  cleaned_matched_samples,
                                                  LhDeck4SensorPositions.positions)
        if not solution.success:
            raise Exception("No lighthouse base station geometry solution could be found!")

        start_x_axis = 1
        start_xy_plane = 1 + len(x_axis)
        origin_pos = solution.cf_poses[0].translation
        x_axis_poses = solution.cf_poses[start_x_axis:start_x_axis + len(x_axis)]
        x_axis_pos = list(map(lambda x: x.translation, x_axis_poses))
        xy_plane_poses = solution.cf_poses[start_xy_plane:start_xy_plane + len(xy_plane)]
        xy_plane_pos = list(map(lambda x: x.translation, xy_plane_poses))

        # Align the solution
        bs_aligned_poses, transformation = LighthouseSystemAligner.align(
            origin_pos, x_axis_pos, xy_plane_pos, solution.bs_poses)

        cf_aligned_poses = list(map(transformation.rotate_translate_pose, solution.cf_poses))

        # Scale the solution
        bs_scaled_poses, cf_scaled_poses, scale = LighthouseSystemScaler.scale_fixed_point(bs_aligned_poses,
                                                                                           cf_aligned_poses,
                                                                                           [REFERENCE_DIST, 0, 0],
                                                                                           cf_aligned_poses[1])

        return bs_scaled_poses

