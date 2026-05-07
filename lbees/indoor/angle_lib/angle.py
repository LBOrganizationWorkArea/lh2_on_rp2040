import numpy as np

class Angle:
    def __init__(self, timestamp: float, base_station_id: int, angles: np.ndarray):
        self.timestamp = timestamp
        self.base_station_id = base_station_id
        self.angles = angles
