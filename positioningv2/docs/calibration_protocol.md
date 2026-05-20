# Calibration protocol

## Objective

Calibrate the position and orientation of the Lighthouse base stations without needing to place them at the exact same position every time.

## Principle

The Lighthouse base stations can be moved.

At each new setup, we collect measurements from the fixed TS4231 sensors on the drone.

The known relative positions of the sensors are stored in:

config/sensors_layout.json

The calibration estimates the Lighthouse geometry and saves it in:

config/geometry.json

## Basic protocol

1. Place the Lighthouse base stations so that they can see the drone sensors.
2. Connect the RP2040/RP2350 board to the computer.
3. Start the live viewer.
4. Check that all sensors receive data.
5. Record a capture.
6. Run the calibration script.
7. Check the generated geometry.json.

## Commands

Live view:

python3 tools/live_view.py --port /dev/ttyACM0

Record data:

python3 tools/serial_logger.py --port /dev/ttyACM0 --output data/captures/test_001.csv

Run calibration:

python3 tools/calibrate_geometry.py --capture data/captures/test_001.csv

## Notes

The real calibration algorithm will be added later.

For now, the goal is to create a clean project structure and define the expected data flow.
