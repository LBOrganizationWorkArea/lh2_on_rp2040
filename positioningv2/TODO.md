# TODO positioningv2

## Step 1 — Project setup

- [x] Create clean project structure.
- [x] Copy old positioning scripts to legacy.
- [x] Add config files.
- [x] Add Python tool skeletons.
- [x] Add documentation.

## Step 2 — Drone sensor layout

- [ ] Define drone reference frame.
- [ ] Measure each TS4231 sensor position.
- [ ] Update config/sensors_layout.json with real drone values.

## Step 3 — Firmware

- [ ] Choose RP2040 or RP2350 board.
- [ ] Connect TS4231 sensors.
- [ ] Read sensor signals.
- [ ] Decode Lighthouse sweeps.
- [ ] Output serial data.

## Step 4 — Logging

- [ ] Read serial data from PC.
- [ ] Save captures to CSV.
- [ ] Check data quality.

## Step 5 — Calibration

- [ ] Use known sensor positions.
- [ ] Estimate Lighthouse geometry.
- [ ] Save geometry.json.
- [ ] Validate repeatability.

## Step 6 — Tracking

- [ ] Estimate drone position.
- [ ] Estimate drone orientation.
- [ ] Add live visualization.
