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

from lbees.indoor.angle_lib.lighthouse_types import (
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
        self.origin, self.x_axis, self.xy_plane, self.samples = None, [], [], []
        self.origin_samples = 100
        self.x_axis_samples = 100
        self.xy_plane_positions = 5
        self.xy_plane_samples = 100
        self.xyz_samples = 100000

        self.decoder = AngleDecoder()


    def record_origin(self):
        self.origin = self.decoder.decode()

    def record_x_axis(self):
        for sample in self.x_axis_samples:
            cur_decoded = self.decoder.decode()
            self.x_axis.append(cur_decoded)

    def record_xy_plane(self):
        cur_pos_samples = []
        for sample in self.xy_plane_samples:
            cur_pos_samples.append(self.decoder.decode())

        self.xy_plane.append(cur_pos_samples)

    def record_samples(self):
        self.samples = []

        for sample in self.xyz_samples:
            cur_decoded = self.decoder.decode()
            self.samples.append(cur_decoded)

