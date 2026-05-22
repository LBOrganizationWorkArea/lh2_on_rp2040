"""
lh2_serial.py
=============
Raw serial acquisition -- nothing else.

Single responsibility: open the serial port, read the available lines,
recognize "LH2,..." frames and return them as raw fields
(s_id, sweep, poly, lfsr). No trigonometry, no coefficients, no EMA.

Angle decoding is fully delegated to the lh2_decoder module.
"""

import serial

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


class LH2Frame:
    """A raw LH2 frame: parsed off the wire but NOT yet decoded into an angle."""

    __slots__ = ("s_id", "sweep", "poly", "lfsr")

    def __init__(self, s_id, sweep, poly, lfsr):
        self.s_id = s_id
        self.sweep = sweep
        self.poly = poly
        self.lfsr = lfsr

    def __repr__(self):
        return (
            f"LH2Frame(s_id={self.s_id}, sweep={self.sweep}, "
            f"poly={self.poly}, lfsr={self.lfsr})"
        )


class LH2SerialReader:
    """
    Reads the serial port and yields raw LH2 frames.

    Usage:
        reader = LH2SerialReader()          # opens the port
        for frame in reader.read_frames():  # call in a loop
            ...                             # frame is an LH2Frame
    """

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()

    def read_frames(self):
        """
        Drain the serial buffer and return the list of valid LH2 frames.
        Non-LH2 or malformed lines are silently ignored.
        """
        frames = []
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            frame = self._parse_line(line)
            if frame is not None:
                frames.append(frame)
        return frames

    @staticmethod
    def _parse_line(line):
        """Parse a raw line into an LH2Frame, or return None if invalid."""
        if not line.startswith("LH2,"):
            return None

        parts = line.split(",")
        if len(parts) != 6:
            return None

        try:
            return LH2Frame(
                s_id=int(parts[1]),
                sweep=int(parts[2]),
                poly=int(parts[4]),
                lfsr=int(parts[5]),
            )
        except ValueError:
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()