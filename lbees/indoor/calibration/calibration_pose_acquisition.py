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

"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from calibration_lib.lighthouse_types import (
    LhMeasurement, LhCfPoseSample, Pose, LhDeck4SensorPositions
)

from calibration_lib.lighthouse_sample_matcher import LighthouseSampleMatcher
from calibration_lib.lighthouse_initial_estimator import LighthouseInitialEstimator
from calibration_lib.lighthouse_geometry_solver import LighthouseGeometrySolver
from calibration_lib.lighthouse_system_aligner import LighthouseSystemAligner
from calibration_lib.lighthouse_system_scaler import LighthouseSystemScaler
from calibration_lib.lighthouse_config_manager import LighthouseConfigFileManager

from angle_lib.angle_decoder import AngleDecoder
from angle_lib.angle import Angle

class CalibrationObtainMeasuraments:
    def __init__(self):
        self.origin, self.x_axis, self.xy_plane, self.samples = None, None, [], []
        self.origin_samples = 100
        self.x_axis_samples = 100
        self.xy_plane_positions = 5
        self.xy_plane_samples = 100
        self.xyz_samples = 100000

        self.decoder = AngleDecoder()


    def record_origin(self):
        origin = Angle(0.0, 0, np.zeros((4, 2)))

        for sample in self.origin_samples:
            cur_decoded = self.decoder.decode()
            origin.angles += cur_decoded.angles

        origin /= self.origin_samples

        self.origin = origin

    def record_x_axis(self):
        x_axis = Angle(0.0, 0, np.zeros((4, 2)))

        for sample in self.x_axis_samples:
            cur_decoded = self.decoder.decode()
            x_axis.angles += cur_decoded.angles

        x_axis /= self.x_axis_samples

        self.x_axis = x_axis

    def record_xy_plane(self):
        for sample in self.xy_plane_samples:
            cur_decoded = self.decoder.decode()
            pos_samples.angles += cur_decoded.angles

        pos_samples /= self.xy_plane_samples
        self.xy_plane.append(pos_samples) #FIXME: do this break??

    def record_samples(self):
        self.samples = []

        for sample in self.xyz_samples:
            cur_decoded = self.decoder.decode()
            self.samples.append(cur_decoded)

