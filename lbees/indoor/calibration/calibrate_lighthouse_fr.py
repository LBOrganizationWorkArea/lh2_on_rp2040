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

class LighthouseCalibrator:
    """Orchestrates the calibration pipeline."""

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.sensor_positions = LhDeck4SensorPositions.positions

    def log(self, message):
        """Print if verbose mode is enabled."""
        if self.verbose:
            print(message)

    def calibrate(self, measurements_file, output_file,
                  origin=None, x_axis=None, xy_plane=None, reference_distance=None):
        """
        Run the full calibration pipeline.

        :param measurements_file: JSON file containing lighthouse measurements
        :param output_file: Output file for calibrated geometry
        :param origin: World frame origin position, or None for default (0,0,0)
        :param x_axis: List of positions on the X-axis, or None for default
        :param xy_plane: List of positions in the XY plane, or None for default
        :param reference_distance: Reference distance for scaling, or None for diagonal scaling
        :return: True if successful
        """

        try:

            """
            ================================================================================
            MATHEMATICAL DIFFERENCES: STEP 3 (XY plane) vs STEP 4 (XYZ space)
            ================================================================================

            STEP 3 - XY data colection - averaging
            ---------------------------------------------------
            Reader: LighthouseSweepAngleAverageReader (instantiated at base, line 145)
            Mathematical Operation: AVERAGING (statistical reduction)

                angles_calibrated[bs_id] = data[1]  # Average of angles
                
                Formula: angles_calibrated = (1/N) * Σ(raw_angles)
                
                - Aggregation in REAL TIME during collection
                - Returns a single LhCfPoseSample per measurement
                - Reduces dimensionality: less data to store
                - Loss of temporal information (timestamps are discarded)

            Output: LhCfPoseSample with averaged angles per BS


            STEP 4 - XYZ data colection - timestamping
            ---------------------------------------------------
            Reader: LighthouseSweepAngleReader (instantiated here, line 276)
            Mathematical Operation: TEMPORAL MATCHING (grouping by time proximity)

                measurement = LhMeasurement(timestamp=now, base_station_id=bs_id, angles=angles)
                self.recorded_angles_result.append(measurement)
                
            Then in STEP 5, in _estimate_geometry (line 355):
                
                LighthouseSampleMatcher.match(samples, max_time_diff=0.020)
                
                Formula: group if |t_i - t_{i-1}| ≤ 0.020s (20 milliseconds)
                
                - Collects RAW data with timestamps
                - Groups samples from multiple base stations that were measured
                approximately at the same instant (20ms window)
                - Preserves temporal and sequential information
                - Maintains CF "movement" over time

            Output: List of LhMeasurement → converted to LhCfPoseSample with temporal matching


            DIRECT COMPARISON
            ------------------
            Aspect                   | Step 3              | Step 4
            -------------------------|--------------------|--------------------
            Reader                   | AverageReader       | SweepAngleReader
            Aggregation              | Arithmetic mean     | Temporal matching
            Time window              | N/A (realtime avg)  | 20ms (max_time_diff)
            Intermediate type        | LhCfPoseSample      | LhMeasurement
            When processing?         | Real time           | Step 5 (_estimate_geometry)
            Preserves movement?      | No (mean loses it)  | Yes (timestamps preserve order)
            Use case                 | Initial calibration | Data for optimization


            PRACTICAL CONSEQUENCE
            ---------------------
            - Step 3: Reduces noise/outliers via averaging, but loses dynamics
            - Step 4: Maintains CF movement dynamics, enables spatial optimization
                    in LighthouseGeometrySolver

            NOT just a "record vs process" difference - it's a REAL MATHEMATICAL DIFFERENCE
            in data reduction/aggregation algorithms!

            ================================================================================
            """

            # Step 1: Load and parse measurements
            self.log("Loading measurements...")
            measurements = self._load_measurements(measurements_file)
            if not measurements:
                print("Error: No measurements loaded")
                return False
            print(f"Loaded {len(measurements)} measurements")

            # Step 2: Match measurements into pose samples
            self.log("Matching measurements...")
            matched_samples = LighthouseSampleMatcher.match(
                measurements,
                max_time_diff=0.020,
                min_nr_of_bs_in_match=2
            )
            print(f"Matched into {len(matched_samples)} pose samples")
            if len(matched_samples) < 2:
                print("Error: Need at least 2 matched samples")
                return False

            # Step 3: Build initial geometry guess using IPPE
            self.log("Building initial geometry guess...")
            initial_guess, cleaned_samples = LighthouseInitialEstimator.estimate(
                matched_samples,
                self.sensor_positions
            )
            print(f"Initial estimate has {len(initial_guess.bs_poses)} base stations")
            print(f"Cleaned to {len(cleaned_samples)} valid samples")

            # Step 4: Solve geometry using least squares optimization
            self.log("Solving geometry...")
            solution = LighthouseGeometrySolver.solve(
                initial_guess,
                cleaned_samples,
                self.sensor_positions
            )
            if not solution.success:
                print("Warning: Solver did not converge to a solution")
                print(f"  Mean error: {solution.error_info.get('mean_error', 'N/A'):.4f}")
            else:
                print("Solver converged successfully")
                print(f"  Mean error: {solution.error_info.get('mean_error', 0.0):.4f}")
                print(f"  Max error: {solution.error_info.get('max_error', 0.0):.4f}")
                print(f"  Std error: {solution.error_info.get('std_error', 0.0):.4f}")

            # Step 5: Align to world frame
            self.log("Aligning to world frame...")
            if origin is None:
                origin = np.array([0.0, 0.0, 0.0])
            if x_axis is None:
                x_axis = [np.array([1.0, 0.0, 0.0])]
            if xy_plane is None:
                xy_plane = [np.array([0.0, 1.0, 0.0])]

            aligned_bs_poses, transformation = LighthouseSystemAligner.align(
                origin, x_axis, xy_plane, solution.bs_poses
            )
            print("Alignment complete")

            # Step 6: Scale using reference measurement
            self.log("Scaling system...")
            if reference_distance is not None:
                # Scale using a fixed reference point
                scaled_bs_poses, scaled_cf_poses, scale_factor = LighthouseSystemScaler.scale_fixed_point(
                    aligned_bs_poses,
                    solution.cf_poses,
                    origin,
                    solution.cf_poses[0]
                )
            else:
                # Scale using diagonal spacing on the lighthouse deck (default: 1 meter reference)
                scaled_bs_poses, scaled_cf_poses, scale_factor = LighthouseSystemScaler.scale_diagonals(
                    aligned_bs_poses,
                    solution.cf_poses,
                    cleaned_samples,
                    LhDeck4SensorPositions.diagonal_distance
                )
            print(f"Scaling complete (scale factor: {scale_factor:.4f})")

            # Step 7: Convert poses to geometry format and save
            self.log("Saving configuration...")
            geos = self._poses_to_geometries(scaled_bs_poses)
            LighthouseConfigFileManager.write(
                output_file,
                geos=geos,
                calibs={},  # No calibration data in this script
                system_type=LighthouseConfigFileManager.SYSTEM_TYPE_V2
            )
            print(f"Saved calibrated geometry to {output_file}")
            return True

        except Exception as e:
            print(f"Error during calibration: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _load_measurements(self, file_path):
        """
        Load measurements from a JSON file.

        Expected format:
        [
          {"timestamp": 0.123, "base_station_id": 0, "angles": [[h0, v0], [h1, v1], [h2, v2], [h3, v3]]},
          ...
        ]
        """
        from calibration_lib.lighthouse_bs_vector import LighthouseBsVector, LighthouseBsVectors

        try:
            with open(file_path, 'r') as f:
                data = json.load(f)

            measurements = []
            for item in data:
                bs_vectors = LighthouseBsVectors([
                    LighthouseBsVector(angle[0], angle[1])
                    for angle in item['angles']
                ])
                measurement = LhMeasurement(
                    timestamp=item['timestamp'],
                    base_station_id=item['base_station_id'],
                    angles=bs_vectors
                )
                measurements.append(measurement)

            return measurements
        except Exception as e:
            print(f"Error loading measurements: {e}")
            return []

    def _poses_to_geometries(self, bs_poses):
        """
        Convert Pose objects to LighthouseBsGeometry objects.

        Stores the basestation position and orientation computed during calibration.
        """
        from calibration_lib.lighthouse_geometry_types import LighthouseBsGeometry

        geos = {}
        for bs_id, pose in bs_poses.items():
            geo = LighthouseBsGeometry()
            geo.set_from_pose(pose.matrix_vec)
            geos[bs_id] = geo

        return geos


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate lighthouse base station geometry from measurements'
    )
    parser.add_argument(
        'measurements',
        help='JSON file containing lighthouse measurements'
    )
    parser.add_argument(
        '-o', '--output',
        default='lighthouse_config.yaml',
        help='Output configuration file (default: lighthouse_config.yaml)'
    )
    parser.add_argument(
        '--origin',
        nargs=3, type=float,
        help='World frame origin [x y z] (default: 0 0 0)'
    )
    parser.add_argument(
        '--x-axis',
        nargs=3, type=float,
        help='Position on positive X-axis [x y z] (default: 1 0 0)'
    )
    parser.add_argument(
        '--xy-plane',
        nargs=3, type=float,
        help='Position in XY plane [x y z] (default: 0 1 0)'
    )
    parser.add_argument(
        '--reference-distance',
        type=float,
        help='Reference distance for scaling (default: use sensor diagonal)'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Verbose output'
    )

    args = parser.parse_args()

    # Validate input file
    measurements_path = Path(args.measurements)
    if not measurements_path.exists():
        print(f"Error: Measurements file not found: {args.measurements}")
        sys.exit(1)

    # Prepare optional arguments
    origin = None
    if args.origin:
        origin = np.array(args.origin)

    x_axis = None
    if args.x_axis:
        x_axis = [np.array(args.x_axis)]

    xy_plane = None
    if args.xy_plane:
        xy_plane = [np.array(args.xy_plane)]

    # Run calibration
    calibrator = LighthouseCalibrator(verbose=args.verbose)
    success = calibrator.calibrate(
        str(measurements_path),
        args.output,
        origin=origin,
        x_axis=x_axis,
        xy_plane=xy_plane,
        reference_distance=args.reference_distance
    )

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
