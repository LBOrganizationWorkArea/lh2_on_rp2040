# positioningv2

Positioning system for a drone using Lighthouse base stations and multiple fixed TS4231 sensors.

## Goal

Build a positioning system that does not require placing the Lighthouse base stations at the exact same position every time.

Instead, the system recalibrates the Lighthouse geometry at each setup.

## Drone setup

The TS4231 sensors are fixed on the drone body.

Their positions must be measured manually and stored in:

config/sensors_layout.json

## Main idea

1. Multiple TS4231 sensors receive Lighthouse sweeps.
2. The firmware decodes timing and sweep information.
3. The PC reads the data through USB serial.
4. A calibration script computes the Lighthouse geometry.
5. The geometry is saved in config/geometry.json.
6. The system estimates the drone position and orientation.
