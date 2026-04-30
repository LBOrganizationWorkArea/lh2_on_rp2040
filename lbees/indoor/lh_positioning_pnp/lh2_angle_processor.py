import math
import re
from collections import defaultdict

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

TICKS_PER_REV = 833333
POINT_BY_SENSOR = {
    0: "top_left",
    1: "top_right",
    2: "bottom_right",
    3: "bottom_left",
}

LINE_RE = re.compile(
    r"^LH2,"
    r"(?P<sensor>\d+),"
    r"(?P<sweep>\d+),"
    r"(?P<basestation>\d+),"
    r"(?P<polynomial>-?\d+),"
    r"(?P<lfsr_location>-?\d+)"
    r"$"
)


def parse_lh2_csv_line(line):
    match = LINE_RE.match(line.strip())
    if not match:
        return None

    return {
        "sensor": int(match.group("sensor")),
        "sweep": int(match.group("sweep")),
        "basestation": int(match.group("basestation")),
        "polynomial": int(match.group("polynomial")),
        "lfsr_location": int(match.group("lfsr_location")),
    }


def lfsr_to_degrees(lfsr_location, ticks_per_rev=TICKS_PER_REV):
    return (((lfsr_location % ticks_per_rev) / ticks_per_rev) * 120.0) - 60.0


def lfsr_to_radians(lfsr_location, ticks_per_rev=TICKS_PER_REV):
    return math.radians(lfsr_to_degrees(lfsr_location, ticks_per_rev))


class AngleAverager:
    def __init__(self, basestation, point_by_sensor=None, ticks_per_rev=TICKS_PER_REV):
        self.basestation = basestation
        self.point_by_sensor = point_by_sensor or POINT_BY_SENSOR
        self.ticks_per_rev = ticks_per_rev
        self._samples = defaultdict(list)

    def add_line(self, line):
        data = parse_lh2_csv_line(line)
        if data is None:
            return False

        if data["basestation"] != self.basestation:
            return False

        if data["sensor"] not in self.point_by_sensor:
            return False

        if data["sweep"] not in (0, 1):
            return False

        angle = lfsr_to_radians(data["lfsr_location"], self.ticks_per_rev)
        key = (data["sensor"], data["sweep"])
        self._samples[key].append(angle)
        return True

    def count(self, sensor, sweep):
        return len(self._samples[(sensor, sweep)])

    def ready(self, samples_per_angle):
        for sensor in self.point_by_sensor:
            for sweep in (0, 1):
                if self.count(sensor, sweep) < samples_per_angle:
                    return False

        return True

    def to_angles_json(self):
        points = {}

        for sensor, point_name in self.point_by_sensor.items():
            theta_samples = self._samples[(sensor, 0)]
            phi_samples = self._samples[(sensor, 1)]

            if not theta_samples or not phi_samples:
                raise ValueError(f"Missing angle samples for sensor {sensor}")

            points[point_name] = {
                "theta": float(sum(theta_samples) / len(theta_samples)),
                "phi": float(sum(phi_samples) / len(phi_samples)),
                "samples": {
                    "theta": len(theta_samples),
                    "phi": len(phi_samples),
                },
            }

        return {
            "unit": "radians",
            "basestation": self.basestation,
            "points": points,
        }
