# positioningv6_anglecalib

This version uses the calibrated angle coefficients from the colleague scripts.

The important formula is no longer:

```text
angle = ((lfsr / 833333) * 120) - 60
```

It is now:

```text
sweep0_angle_deg = A0 * lfsr + B0
sweep1_angle_deg = A1 * lfsr + B1
azimuth_deg = (sweep0_angle_deg + sweep1_angle_deg) / 2
elevation_deg = (sweep0_angle_deg - sweep1_angle_deg) / (2 * tan(30 deg))
```

The coefficients are loaded from:

```text
config/history_calibration.txt
```

## Test

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv6_anglecalib
py .\tools\01_live_calibrated_angles.py --port COM3
```

This first checks whether calibrated azimuth/elevation are stable and believable for all 4 sensors and both base stations.
