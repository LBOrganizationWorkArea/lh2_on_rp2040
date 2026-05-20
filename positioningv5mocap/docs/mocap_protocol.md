# Mocap Calibration Protocol

## Recording

Record two streams at the same time:

- LH2 serial with `tools/01_record_lh2.py`
- mocap pose CSV from your motion-capture system

The best case is when both files use the same PC clock in `pc_time_s`.

## Mocap CSV Format

Preferred:

```text
pc_time_s,x_m,y_m,z_m,qx,qy,qz,qw
```

Alternative:

```text
pc_time_s,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg
```

The pose must be the drone body frame. The sensor offsets are then applied from `config/sensors_layout.json`.

## Movement

Use a rich motion, not a flat square:

- move left/right/front/back
- move up/down
- rotate yaw
- add small roll/pitch if safe
- keep all sensors visible to both Lighthouse base stations as much as possible

Record at least 20-60 seconds.

## Commands

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv5mocap
py .\tools\01_record_lh2.py --port COM3 --output data\lh2_record.csv
py .\tools\02_check_recordings.py --lh2 data\lh2_record.csv --mocap data\mocap.csv
py .\tools\03_fit_mocap_lh2.py --lh2 data\lh2_record.csv --mocap data\mocap.csv --basestations 4,10
py .\tools\04_validate_mocap_lh2.py
```

## Reading Results

Good calibration should have small angular residuals. As a rough first target:

- median below 1 degree: promising
- RMSE around 1-2 degrees: usable for iteration
- much larger: check time sync, sensor layout, mocap frame, or LH2 conversion model
