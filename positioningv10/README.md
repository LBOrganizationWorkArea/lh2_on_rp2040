# positioningv10 calibration workflow

Run every command from this folder:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv10
```

v10 is the current live calibration branch. It uses direct Lighthouse V2 angles from the firmware (`LH2A`) and keeps both angle families when a channel has two stable candidates. The fit then chooses the family that best matches the room geometry.

## Current status

What works now:

- firmware emits direct angle lines:

```text
LH2A,time_us,sensor,sweep,basestation,polynomial,lfsr_location,angle_urad
```

- `02_live_angles.py` can display paired `LH2P` angles or direct `LH2A` angles.
- `17_diagnose_lh2a_families.py` confirms reception and family spread.
- `18_capture_lh2a_family_poses.py` captures known 3D anchor poses and stores candidate angle families.
- `20_validate_lh2a_family_capture.py` validates spread/channel coverage before fitting.
- `19_fit_lh2a_family_geometry.py` fits the Lighthouse geometry from the captured family candidates.

Known current observation:

- With only 4 poses, the RMSE can look good but the unconstrained Lighthouse positions can drift to impossible coordinates.
- With 12 poses, dirty/high-spread captures increase the RMSE. Filtering or recapturing points with tighter spread improves the fit.
- The next practical step is to recapture the high-spread points and then add wand waving refinement.

## 1. Check direct LH2A reception

Use this before capturing points:

```powershell
py .\tools\17_diagnose_lh2a_families.py --port COM3 --baudrate 115200 --duration 5 --cluster-deg 8
```

Good signs:

- `LH2A` count is high.
- Every `sensor/bs/sweep` channel has samples.
- Most channels show 2 stable families.
- Spread is low for each family.

For a live angle view:

```powershell
py .\tools\02_live_angles.py --port COM3 --baudrate 115200 --debug --window 0.5 --angle-outlier-deg 5 --prefer-direct-lh2a
```

## 2. Capture known 3D anchor points

The known pose file is:

```text
config/wand_3d_points.json
```

Capture one or more poses with direct LH2A family candidates:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume --only P00_bas_avant_gauche
```

For the initial four-point sanity set:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume --only P00_bas_avant_gauche,P04_bas_centre,P08_bas_arriere_droite,P11_boite45_centre
```

Then add broader coverage:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume --only P01_bas_avant_centre,P02_bas_avant_droite,P03_bas_centre_gauche,P05_bas_centre_droite,P06_bas_arriere_gauche,P07_bas_arriere_centre,P09_boite45_avant_gauche,P13_boite45_arriere_droite
```

The output is:

```text
config/wand_calibration_poses_3d_lh2a_families.json
```

The capture script saves after every pose and creates backups before overwriting an existing output. To recapture one bad pose without losing the others:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 1.0 --resume --recapture P07_bas_arriere_centre --only P07_bas_arriere_centre
```

## 3. Validate capture quality

Basic validation:

```powershell
py .\tools\20_validate_lh2a_family_capture.py
```

Strict validation, useful before fitting:

```powershell
py .\tools\20_validate_lh2a_family_capture.py --max-spread-deg 0.5
```

Good signs:

- `channels=16/16`
- `two-family=16/16`
- `max_spread` below the threshold

Current useful lesson: points around `1deg` to `2deg` spread can raise the 12-pose fit RMSE. Recapture those with `--cluster-deg 1.0` or `--cluster-deg 0.7`.

## 4. Fit Lighthouse geometry

The wrapper uses the LH2A family capture file by default:

```powershell
py .\tools\19_fit_lh2a_family_geometry.py
```

For the current physical setup, use constrained approximate Lighthouse placement:

```powershell
py .\tools\19_fit_lh2a_family_geometry.py --lighthouse-z 0.70 --bs4-guess -0.75,1.80 --bs10-guess 0.75,2.40 --position-window 0.60
```

To diagnose only one basestation:

```powershell
py .\tools\19_fit_lh2a_family_geometry.py --only-bs 4 --lighthouse-z 0.70 --bs4-guess -0.75,1.90 --position-window 0.70 --model-variants all --coarse-nfev 80 --refine-top-k 24 --max-nfev 500
```

To fit only with clean poses:

```powershell
py .\tools\19_fit_lh2a_family_geometry.py --max-pose-spread-deg 0.5 --lighthouse-z 0.70 --bs4-guess -0.75,1.80 --bs10-guess 0.75,2.40 --position-window 0.60
```

The output is:

```text
config/lighthouse_geometry_lh2a_families.json
```

Interpretation:

- RMSE is an angular RMSE in degrees, not a position error in meters.
- Low RMSE with impossible Lighthouse translation is not a good calibration.
- Prefer a physically plausible solution with slightly higher RMSE over an unconstrained solution that places a Lighthouse far outside the room.

## 5. Next step: wand waving refinement

After the fixed-point fit is physically plausible, record a moving wand wave:

```powershell
py .\tools\09_record_wand_wave.py --port COM3 --baudrate 115200 --duration 60
```

The intended next refinement is:

- use clean known points to anchor the room frame;
- use wand waving to add dense moving observations;
- refine Lighthouse orientations, offsets, and family selection with much more coverage.

## Legacy point-plus-wave path

Older v10 scripts are still present:

```powershell
py .\tools\07_capture_wand_poses.py --port COM3 --baudrate 115200 --duration 4 --resume
py .\tools\08_fit_wand_geometry.py
py .\tools\10_refine_geometry_points_plus_wave.py
```

These were kept as references, but the current active path is the direct `LH2A` family workflow above.

## ROS2 bridge

A progressive ROS2 bridge lives in:

```text
ros2_ws/src/lh2_ros_bridge
```

It keeps the current Python tools as the parsing/calibration/fit reference and adds ROS topics around them.
