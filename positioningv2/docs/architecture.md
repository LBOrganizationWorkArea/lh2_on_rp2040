# positioningv2 architecture

## Goal

The system estimates the position and orientation of a drone using Lighthouse base stations and multiple fixed TS4231 sensors.

## Drone reference frame

origin = drone center
x = front of the drone
y = left of the drone
z = up

Each TS4231 sensor has a fixed 3D position in this frame.

These positions are stored in:

config/sensors_layout.json

## Data flow

TS4231 sensors
↓
RP2040 / RP2350 firmware
↓
USB serial
↓
Python logger
↓
CSV capture
↓
Calibration
↓
geometry.json
↓
Tracking

## Main components

### Firmware

The firmware reads the TS4231 sensors and extracts Lighthouse sweep information.

Expected output:

timestamp_us,sensor_id,base_station_id,sweep_id,angle_rad,quality

### Sensor layout

The file config/sensors_layout.json describes the fixed position of each TS4231 sensor on the drone body.

### Geometry

The file config/geometry.json describes the position and orientation of each Lighthouse base station.

### Tools

- serial_logger.py: records serial data.
- plot_angles.py: visualizes captured angles.
- calibrate_geometry.py: computes Lighthouse geometry.
- live_view.py: displays live serial data.
