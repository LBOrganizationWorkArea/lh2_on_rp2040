#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np


STABLE_BASESTATIONS = [4, 10]


def point_to_features(point):
    """
    Convert one calibration point into a feature vector.

    We keep only stable basestations 4 and 10.
    Feature order:
    sensor 0..3
    basestation 4,10
    sweep 0,1
    => 4 * 2 * 2 = 16 features
    """
    values = {}

    for m in point["measurements"]:
        sensor = int(m["sensor"])
        basestation = int(m["basestation"])
        sweep = int(m["sweep"])
        lfsr = float(m["median_lfsr_location"])

        if basestation not in STABLE_BASESTATIONS:
            continue

        key = (sensor, basestation, sweep)
        values[key] = lfsr

    features = []
    feature_names = []

    for sensor in range(4):
        for basestation in STABLE_BASESTATIONS:
            for sweep in range(2):
                key = (sensor, basestation, sweep)
                feature_names.append({
                    "sensor": sensor,
                    "basestation": basestation,
                    "sweep": sweep,
                })

                if key not in values:
                    raise ValueError(
                        f"Missing channel in point {point['name']}: "
                        f"sensor={sensor}, basestation={basestation}, sweep={sweep}"
                    )

                features.append(values[key])

    return np.array(features, dtype=float), feature_names


def main():
    parser = argparse.ArgumentParser(description="Fit relative 2D model from calibration file.")
    parser.add_argument("--input", default="config/calibration_relative_2d.json")
    parser.add_argument("--output", default="config/model_relative_2d.json")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    with open(input_path, "r") as f:
        calibration = json.load(f)

    X_raw = []
    Y = []
    feature_names = None

    for point in calibration["points"]:
        features, names = point_to_features(point)

        if feature_names is None:
            feature_names = names

        X_raw.append(features)
        Y.append([float(point["x_m"]), float(point["y_m"])])

    X_raw = np.vstack(X_raw)
    Y = np.array(Y, dtype=float)

    # Use point 0 as reference.
    x0_features = X_raw[0].copy()

    # Delta features relative to first calibration point.
    X_delta = X_raw - x0_features

    # Add bias column.
    X_design = np.hstack([
        np.ones((X_delta.shape[0], 1)),
        X_delta
    ])

    # Least squares:
    # X_design @ W = Y
    W, residuals, rank, singular_values = np.linalg.lstsq(X_design, Y, rcond=None)

    Y_pred = X_design @ W
    errors = Y_pred - Y
    rmse = np.sqrt(np.mean(errors ** 2, axis=0))

    model = {
        "description": "Relative 2D linear model from Lighthouse LFSR deltas to x,y position.",
        "input_calibration": str(input_path),
        "stable_basestations": STABLE_BASESTATIONS,
        "feature_names": feature_names,
        "reference_features": x0_features.tolist(),
        "weights": W.tolist(),
        "training": {
            "num_points": int(len(calibration["points"])),
            "rank": int(rank),
            "residuals": residuals.tolist() if hasattr(residuals, "tolist") else [],
            "rmse_x_m": float(rmse[0]),
            "rmse_y_m": float(rmse[1]),
            "points": []
        }
    }

    for i, point in enumerate(calibration["points"]):
        model["training"]["points"].append({
            "name": point["name"],
            "target_x_m": float(Y[i, 0]),
            "target_y_m": float(Y[i, 1]),
            "predicted_x_m": float(Y_pred[i, 0]),
            "predicted_y_m": float(Y_pred[i, 1]),
            "error_x_m": float(errors[i, 0]),
            "error_y_m": float(errors[i, 1]),
        })

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        json.dump(model, f, indent=2)

    print("Model fitted.")
    print("=" * 60)
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print(f"Calibration points: {len(calibration['points'])}")
    print(f"Features: {len(feature_names)}")
    print(f"RMSE x: {rmse[0]:.4f} m")
    print(f"RMSE y: {rmse[1]:.4f} m")
    print("=" * 60)

    print("Training points:")
    for p in model["training"]["points"]:
        print(
            f"{p['name']}: "
            f"target=({p['target_x_m']:+.3f}, {p['target_y_m']:+.3f}) m | "
            f"pred=({p['predicted_x_m']:+.3f}, {p['predicted_y_m']:+.3f}) m | "
            f"err=({p['error_x_m']:+.3f}, {p['error_y_m']:+.3f}) m"
        )


if __name__ == "__main__":
    main()
