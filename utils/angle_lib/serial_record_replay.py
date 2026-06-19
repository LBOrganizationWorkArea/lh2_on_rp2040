"""
Record and replay LH2 serial frames.

This module builds on utils.angle_lib.serial_read.LH2SerialReader so you can
capture a live session once and later simulate the same input stream without
the hardware.

The recording format is JSONL. Each line stores the elapsed time since the
start of the capture and the raw LH2 frame fields.
"""

import json
import time
from pathlib import Path

try:
    from .serial_read import BAUD_RATE, SERIAL_PORT, LH2Frame, LH2SerialReader
except ImportError:
    from serial_read import BAUD_RATE, SERIAL_PORT, LH2Frame, LH2SerialReader


RECORDING_VERSION = 1

# Hardcoded runtime configuration.
# Set MODE to "record" or "replay".
MODE = "record"
RECORDING_PATH = Path("utils/angle_lib/lh2_recording.jsonl")
REPLAY_PATH = Path("utils/angle_lib/lh2_recording.jsonl")
REPLAY_SPEED = 1.0
REPLAY_LOOP = False
RECORD_PORT = SERIAL_PORT
RECORD_BAUD = BAUD_RATE


class LH2FrameRecorder:
    """Capture frames from a live serial reader and write them to JSONL."""

    def __init__(self, output_path, port=SERIAL_PORT, baud=BAUD_RATE):
        self.output_path = Path(output_path)
        self.reader = LH2SerialReader(port=port, baud=baud)
        self.started_at = time.monotonic()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_file = self.output_path.open("w", encoding="utf-8")

    def poll(self):
        """Read available frames and append them to the recording file."""
        frames = self.reader.read_frames()
        now = time.monotonic() - self.started_at

        for frame in frames:
            event = {
                "version": RECORDING_VERSION,
                "elapsed": now,
                "frame": {
                    "s_id": frame.s_id,
                    "sweep": frame.sweep,
                    "poly": frame.poly,
                    "lfsr": frame.lfsr,
                },
            }
            self.output_file.write(json.dumps(event, ensure_ascii=True) + "\n")
            self.output_file.flush()

        return frames

    def close(self):
        self.reader.close()
        if self.output_file and not self.output_file.closed:
            self.output_file.close()


class LH2FrameReplayReader:
    """Replay a previously recorded capture through the same read_frames API."""

    def __init__(self, recording_path, speed=1.0, loop=False):
        self.recording_path = Path(recording_path)
        self.speed = speed
        self.loop = loop
        self.events = self._load_events()
        self.index = 0
        self.started_at = time.monotonic()
        self.finished = False

    def _load_events(self):
        events = []
        with self.recording_path.open("r", encoding="utf-8") as recording_file:
            for line in recording_file:
                line = line.strip()
                if not line:
                    continue

                event = json.loads(line)
                if int(event.get("version", 0)) != RECORDING_VERSION:
                    continue

                frame_data = event["frame"]
                events.append(
                    (
                        float(event["elapsed"]),
                        LH2Frame(
                            s_id=int(frame_data["s_id"]),
                            sweep=int(frame_data["sweep"]),
                            poly=int(frame_data["poly"]),
                            lfsr=int(frame_data["lfsr"]),
                        ),
                    )
                )

        return events

    def read_frames(self):
        """Return the frames that should have arrived by the current replay time."""
        if self.finished:
            return []

        if not self.events:
            self.finished = True
            return []

        while True:
            if self.index >= len(self.events):
                if self.loop:
                    self.index = 0
                    self.started_at = time.monotonic()
                else:
                    self.finished = True
                    return []

            elapsed = (time.monotonic() - self.started_at) * self.speed
            next_elapsed, _ = self.events[self.index]
            wait_seconds = (next_elapsed - elapsed) / max(self.speed, 1e-6)

            if wait_seconds > 0:
                time.sleep(min(wait_seconds, 0.05))
                continue

            frames = []
            while self.index < len(self.events):
                event_elapsed, frame = self.events[self.index]
                elapsed = (time.monotonic() - self.started_at) * self.speed
                if event_elapsed > elapsed:
                    break
                frames.append(frame)
                self.index += 1

            if frames:
                return frames

    def close(self):
        return None


def main():
    if MODE == "replay":
        reader = LH2FrameReplayReader(REPLAY_PATH, speed=REPLAY_SPEED, loop=REPLAY_LOOP)
    elif MODE == "record":
        reader = LH2FrameRecorder(RECORDING_PATH, port=RECORD_PORT, baud=RECORD_BAUD)
    else:
        raise SystemExit('MODE must be either "record" or "replay".')

    try:
        while True:
            frames = reader.read_frames()
            for frame in frames:
                print(frame)

            if getattr(reader, "finished", False):
                break

            if MODE == "record":
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()


if __name__ == "__main__":
    main()