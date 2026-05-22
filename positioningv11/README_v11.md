# positioningv11 workflow

v11 is the alternate calibration path.

It assumes the Lighthouse positions are measured by hand and fixed in the room frame. The calibration then solves only Lighthouse orientation, sweep signs/offsets, and the correct LH2A angle family.

Use this path when the physical Lighthouse positions are trusted more than the unconstrained point fit. The current v10 work remains the active test path; v11 is kept for the "known Lighthouse positions" approach.

## 1. Enter measured Lighthouse positions

Edit:

```text
config/lighthouse_positions.json
```

Example shape:

```json
{
  "basestations": [
    {"basestation": 4, "x_m": -0.50, "y_m": 1.20, "z_m": 1.70},
    {"basestation": 10, "x_m": 0.50, "y_m": 1.20, "z_m": 1.70}
  ]
}
```

Use your measured room coordinates. If you know the spacing between Lighthouses and their height, choose the room origin and enter their coordinates consistently.

## 2. Check LH2A reception

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv11
py .\tools\17_diagnose_lh2a_families.py --port COM3 --baudrate 115200 --duration 5 --cluster-deg 8
```

Expected: `LH2A` counts, `channels=16`, and usually 2 families per channel.

## 3. Capture known anchor points with angle families

Start with a few ground/box anchors:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume --only P00_bas_avant_gauche,P04_bas_centre,P08_bas_arriere_droite,P11_boite45_centre
```

Then capture all points when the first fit looks sane:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume
```

Output:

```text
config/wand_calibration_poses_3d_lh2a_families.json
```

Validate before fitting:

```powershell
py .\tools\20_validate_lh2a_family_capture.py --max-spread-deg 0.5
```

Recapture any bad pose with:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 1.0 --resume --recapture P00_bas_avant_gauche --only P00_bas_avant_gauche
```

## 4. Fit orientations with fixed Lighthouse positions

```powershell
py .\tools\20_fit_known_lighthouse_positions.py
```

Output:

```text
config/lighthouse_geometry_known_positions.json
```

If the RMSE is high, try:

```powershell
py .\tools\20_fit_known_lighthouse_positions.py --model-variants all
```

The RMSE is an angular error in degrees. Check both RMSE and whether the solved orientations produce stable live positions.
