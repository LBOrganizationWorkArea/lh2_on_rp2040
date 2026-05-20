# positioningv5mocap

Goal: use motion capture to calibrate Lighthouse V2 measurements for drone positioning.

With mocap, we do not need a manual `-30 / 0 / +30 deg` angle calibration. Instead, we record:

- LH2 serial measurements from the RP2040
- mocap pose of the drone over time
- known 4-sensor layout on the drone

Then we optimize, for each Lighthouse:

- Lighthouse position and orientation
- LH2 sweep conversion coefficients

This should produce the calibration needed for later PnP/live positioning.

## Expected LH2 Serial Format

```text
LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location
```

## Expected Mocap CSV

The mocap file should contain one row per pose:

```text
pc_time_s,x_m,y_m,z_m,qx,qy,qz,qw
```

Use the same PC clock if possible. If the mocap system cannot export PC time, add a sync event and we will estimate a time offset later.

## Basic Workflow

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv5mocap
py .\tools\01_record_lh2.py --port COM3 --output data\lh2_record.csv
py .\tools\03_fit_mocap_lh2.py --lh2 data\lh2_record.csv --mocap data\mocap.csv
py .\tools\04_validate_mocap_lh2.py
```

Move the drone through a rich 3D volume during recording:

- left/right/front/back
- up/down
- yaw changes
- small roll/pitch changes if safe

Do not move the Lighthouse base stations during or after calibration.
