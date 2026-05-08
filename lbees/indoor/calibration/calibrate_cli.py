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

from lbees.indoor.calibration.calibrate_lighthouse import LighthouseCalibrator
from calibration.calibration_pose_acquisition import CalibrationObtainMeasuraments 


def main():
    sampler = CalibrationObtainMeasuraments()

    # === DATA ACQUISITION ===
    input("Start origin calibration. Place the drone at the origin position and press Enter...")
    sampler.record_origin()

    input("Start x-axis calibration. Place the drone on the x-axis position and press Enter...")
    sampler.record_x_axis()

    for pose in sampler.xy_plane_positions:
        input("Start xy-plane calibration. Place the drone on the xy-plane position and press Enter...")
        sampler.record_xy_plane()
    
    print("Start free movement calibration. Move the drone around.")
    sampler.record_samples()

    # === CALIBRATION ===
    calibrator = LighthouseCalibrator()
    calibrator.calibrate(sampler.origin, sampler.x_axis, sampler.xy_plane, sampler.samples)

if __name__ == "__main__":
    main()


