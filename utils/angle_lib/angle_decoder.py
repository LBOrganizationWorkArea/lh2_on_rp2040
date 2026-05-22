"""
lh2_decoder.py
==============
LH2 angle decoding -- nothing else.

Single responsibility: receive raw LH2Frame objects (produced by lh2_serial),
reconstruct a complete "V" (sweep 0 + sweep 1), apply the calibration
coefficients and the EMA filter, and expose the smoothed azimuth / elevation
for each (sensor, base station) pair.

This module opens NO serial port. It only knows about LH2Frame objects.
"""

import math
import time
from pathlib import Path

# --- CONFIGURATION ---
LOG_FILE = Path("~/lbees/2indoor_ubuntu/history_calibration.txt")

TAN_30 = 0.577350269
SENSORS = [0, 1, 2, 3]
BASE_STATIONS = [4, 10]

# EMA filter (temporal smoothing of the raw angles)
EMA_ALPHA = 0.2


def load_bs_coefficients(target_bs):
    """Load the 4 calibration coefficients of a base station from the history file."""
    coeffs = None
    if not LOG_FILE.exists():
        return None
    with open(LOG_FILE, "r") as f:
        for line in f:
            if "DATE_TIME" in line or line.startswith("-") or not line.strip():
                continue
            parts = [p.strip() for p in line.split("|")]
            if len(parts) >= 7:
                try:
                    if int(parts[1]) == target_bs:
                        coeffs = (
                            float(parts[3]),
                            float(parts[4]),
                            float(parts[5]),
                            float(parts[6]),
                        )
                except ValueError:
                    continue
    return coeffs


class LH2Decoder:
    """
    Turns raw LH2Frame objects into smoothed azimuth/elevation angles.

    Usage:
        decoder = LH2Decoder()              # loads the calibration coefficients
        decoder.feed(frame)                 # for each frame received from the reader
        # ... or in a batch:
        decoder.feed_all(reader.read_frames())
        angles = decoder.get_angles(s, bs)  # -> dict or None
    """

    def __init__(self, ema_alpha=EMA_ALPHA):
        self.ema_alpha = ema_alpha

        # Calibration coefficients
        self.coeffs = {
            4: load_bs_coefficients(4),
            10: load_bs_coefficients(10),
        }
        if not self.coeffs[4] or not self.coeffs[10]:
            raise RuntimeError("Coefficients not found in the calibration history.")

        # Angle state: one sub-dictionary per (sensor, base station) pair
        self.state = {
            s: {
                bs: {
                    "angle_0": None,
                    "angle_1": None,
                    "ema_az": None,
                    "ema_el": None,
                    "last_update": 0,
                }
                for bs in BASE_STATIONS
            }
            for s in SENSORS
        }

    # ------------------------------------------------------------------
    # Input: frame ingestion
    # ------------------------------------------------------------------
    def feed_all(self, frames):
        """Ingest a list of LH2Frame objects (as returned by the serial reader)."""
        for frame in frames:
            self.feed(frame)

    def feed(self, frame):
        """Ingest a single raw LH2Frame and update the state if it is valid."""
        if frame.s_id not in SENSORS:
            return

        bs_id = (
            4 if frame.poly in (8, 9)
            else (10 if frame.poly in (20, 21) else None)
        )
        if bs_id is None:
            return

        coeffs = self.coeffs[bs_id]
        slot = self.state[frame.s_id][bs_id]

        # "V" reconstruction: store each sweep separately
        if frame.sweep == 0:
            slot["angle_0"] = (coeffs[0] * frame.lfsr) + coeffs[1]
        elif frame.sweep == 1:
            slot["angle_1"] = (coeffs[2] * frame.lfsr) + coeffs[3]

        # Once both sweeps are in, compute azimuth/elevation
        if slot["angle_0"] is not None and slot["angle_1"] is not None:
            self._compute_angles(frame.s_id, bs_id, coeffs)

    # ------------------------------------------------------------------
    # Core decoding
    # ------------------------------------------------------------------
    def _compute_angles(self, s_id, bs_id, coeffs):
        """Turn a complete V into smoothed azimuth/elevation angles (EMA)."""
        slot = self.state[s_id][bs_id]
        a0 = slot["angle_0"]
        a1 = slot["angle_1"]

        azimut_brut = (a0 + a1) / 2.0
        diff = a0 - a1
        swap_constant = 2.0 * (coeffs[1] - coeffs[3])

        # Handle the sweep swap when the angular difference is too large
        if abs(diff) > 90.0:
            diff = swap_constant - diff

        diff_rad = math.radians(diff / 2.0)
        azimut_rad = math.radians(azimut_brut)
        y_projection = math.tan(diff_rad) / TAN_30 * (1.0 / math.cos(azimut_rad))
        elevation_brute = math.degrees(math.atan(y_projection))

        # EMA filter
        if slot["ema_az"] is None:
            slot["ema_az"] = azimut_brut
            slot["ema_el"] = elevation_brute
        else:
            a = self.ema_alpha
            slot["ema_az"] = a * azimut_brut + (1.0 - a) * slot["ema_az"]
            slot["ema_el"] = a * elevation_brute + (1.0 - a) * slot["ema_el"]

        slot["last_update"] = time.time()
        slot["angle_0"] = None
        slot["angle_1"] = None

    # ------------------------------------------------------------------
    # Output: access to the decoded angles
    # ------------------------------------------------------------------
    def get_angles(self, s_id, bs_id):
        """
        Return {'az', 'el', 'last_update'} for a (sensor, base station) pair,
        or None if no angle has been decoded yet.
        """
        slot = self.state[s_id][bs_id]
        if slot["ema_az"] is None:
            return None
        return {
            "az": slot["ema_az"],
            "el": slot["ema_el"],
            "last_update": slot["last_update"],
        }

    def is_fresh(self, s_id, bs_id, max_age=0.5, now=None):
        """Return True if the angles of a pair are less than `max_age` seconds old."""
        if now is None:
            now = time.time()
        return (now - self.state[s_id][bs_id]["last_update"]) < max_age