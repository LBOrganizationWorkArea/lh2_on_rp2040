# Serial output format

The positioningv2 firmware sends decoded Lighthouse v2 data over USB serial.

## CSV format

LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location

## Example

LH2,12345678,0,0,4,8,51232

## Fields

- LH2: fixed prefix used to identify valid Lighthouse data.
- time_us: timestamp from the Pico since boot, in microseconds.
- sensor: TS4231 sensor ID.
- sweep: Lighthouse sweep ID, usually 0 or 1.
- basestation: Lighthouse base station ID decoded from the polynomial.
- polynomial: raw Lighthouse polynomial ID.
- lfsr_location: decoded raw Lighthouse sweep position.

## Notes

This is not yet a final 3D position.

The PC side must convert these measurements into drone position and orientation using:

- the fixed sensor positions on the drone;
- the Lighthouse geometry;
- a calibration/tracking algorithm.
