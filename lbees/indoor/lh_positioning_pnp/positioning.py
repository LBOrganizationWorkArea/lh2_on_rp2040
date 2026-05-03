import json
from pathlib import Path

import cv2
import numpy as np

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

DISTANCE_BETWEEN_SENSORS = 0.1  #FIXME: fill this distance with the correct one guys
WORK_DIR = Path(__file__).resolve().parent
ANGLES_FILE = WORK_DIR / "angles.json"
POINT_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")

# 1. Object points perfectly following the IPPE_SQUARE specification
# squareLength = 0.1 meters (10 cm)
# Half squareLength = 0.05 meters
object_points = np.array([
    [-DISTANCE_BETWEEN_SENSORS/2,  DISTANCE_BETWEEN_SENSORS/2, 0.0],  # point 0: top-left
    [ DISTANCE_BETWEEN_SENSORS/2,  DISTANCE_BETWEEN_SENSORS/2, 0.0],  # point 1: top-right
    [ DISTANCE_BETWEEN_SENSORS/2, -DISTANCE_BETWEEN_SENSORS/2, 0.0],  # point 2: bottom-right
    [-DISTANCE_BETWEEN_SENSORS/2, -DISTANCE_BETWEEN_SENSORS/2, 0.0]   # point 3: bottom-left
], dtype=float)

def load_angles_json(path):
    """
    Read 4 measured angle pairs from JSON.

    Expected JSON format:
        {
          "unit": "radians",
          "points": {
            "top_left": {"theta": 0.10, "phi": -0.05},
            "top_right": {"theta": -0.20, "phi": 0.10},
            "bottom_right": {"theta": 0.05, "phi": 0.20},
            "bottom_left": {"theta": -0.10, "phi": -0.15}
          }
        }

    The returned array order matches object_points:
        0: top-left
        1: top-right
        2: bottom-right
        3: bottom-left
    """
    if not path.exists():
        raise FileNotFoundError(f"Angles JSON not found: {path}")

    with open(path, "r") as file:
        data = json.load(file)

    unit = data.get("unit", "radians")
    if unit != "radians":
        raise ValueError(f"Angles unit must be radians. Got: {unit}")

    points = data.get("points")
    if not isinstance(points, dict):
        raise ValueError("Angles JSON must contain a points object")

    angle_pairs = []
    for point_name in POINT_ORDER:
        point = points.get(point_name)
        if not isinstance(point, dict):
            raise ValueError(f"Angles JSON is missing point: {point_name}")

        if "theta" not in point or "phi" not in point:
            raise ValueError(f"Point {point_name} must contain theta and phi")

        angle_pairs.append([float(point["theta"]), float(point["phi"])])

    return np.array(angle_pairs, dtype=float)


def angles_to_image_points(angles):
    image_points = []
    for theta, phi in angles:
        x = np.tan(theta)
        y = np.tan(phi) / np.cos(theta)
        image_points.append([x, y])

    return np.array(image_points, dtype=float)


def main():
    # 2. Your measured angles mapped to virtual image coordinates.
    angles = load_angles_json(ANGLES_FILE)
    image_points = angles_to_image_points(angles)

    # 3. Virtual Camera setup
    camera_matrix = np.eye(3, dtype=float)
    dist_coeffs = np.zeros(4, dtype=float)

    # 4. Solve Pose using IPPE_SQUARE
    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )

    if success:
        R, _ = cv2.Rodrigues(rotation_vector)

        # Calculate base station position relative to the center of the sensor board
        base_station_position = -np.dot(R.T, translation_vector)

        print("Base Station Position (X, Y, Z in meters):")
        print(base_station_position.flatten())
    else:
        print("Pose estimation failed.")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, ValueError) as error:
        print(f"Cannot estimate pose: {error}")
