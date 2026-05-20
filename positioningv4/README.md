# positioningv4

Goal: estimate the drone center position `x,y` on the floor with Lighthouse V2 and 4 TS4231 sensors mounted as a 12.5 cm square.

This version intentionally starts with a 2D floor-plane calibration:

1. Put the Lighthouse base stations wherever you want.
2. Do not move them after calibration.
3. Move the drone to many known floor positions.
4. Capture the Lighthouse measurements at each known drone position.
5. Fit one 2D floor mapping per Lighthouse.
6. In live mode, every visible sensor gives a possible world `x,y`; the drone center is the robust average of all visible sensor estimates.

This avoids relying on the small 12.5 cm sensor square alone to recover the Lighthouse geometry. The real baseline comes from moving the whole drone over many floor positions.

## Quick Start

Use the Windows Python launcher because this machine has `py` available:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv4
py .\tools\01_capture_floor_calibration.py --port COM3
py .\tools\02_fit_floor_maps.py
py .\tools\03_validate_floor_maps.py
py .\tools\04_live_xy.py --port COM3
```

Edit `config\floor_poses.json` to match the physical points you actually mark on the floor.

## Serial Format

The scripts expect the RP2040 firmware to print:

```text
LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location
```

This matches the `lh2_on_rp2040` style output already used in the project.
