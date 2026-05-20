# positioningv10 workflow

Run commands from this folder:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv10
```

This v10 pipeline is centered on the **wand 3D calibration**:

```text
firmware LH2P -> calibrated angles -> wand points on the floor/boxes -> Lighthouse geometry -> live 3D drone position
```

The measured points anchor the coordinate frame. The floor points define `z=0`, and the higher points make the Lighthouse geometry observable in 3D.

## 1. Check serial

```powershell
py .\tools\01_live_view.py --port COM3 --baudrate 115200
```

Expected firmware output:

```text
LH2P;...
```

With the partial-pair firmware, missing sensors are sent as `0;0` inside the same `LH2P` format. The v10 parser ignores these missing sensors.

## 2. Check angles

```powershell
py .\tools\02_live_angles.py --port COM3 --baudrate 115200 --debug
```

This checks that:

- `LH2P` lines are parsed
- polynomials are mapped to the correct axes
- factory calibration files are loaded from `config`
- parasite angle families are filtered
- missing `0;0` sensors do not create fake angles

## 3. Capture the wand points

The main point file is:

```text
config/wand_3d_points.json
```

It contains floor points and higher points. Edit this file if your measured positions are different.

Capture:

```powershell
py .\tools\07_capture_wand_poses.py --port COM3 --baudrate 115200 --duration 4 --resume
```

Output:

```text
config/wand_calibration_poses_3d.json
```

Recommended capture rules:

- keep the drone orientation fixed unless the point file says otherwise
- keep the Lighthouses fixed after starting capture
- accept partial captures when needed, but prefer at least 2 sensors and 1-2 basestations
- recapture weak points with `--resume`

To capture only a few points:

```powershell
py .\tools\07_capture_wand_poses.py --port COM3 --baudrate 115200 --resume --only P04_bas_centre,P11_boite45_centre
```

## 4. Fit the Lighthouse geometry

```powershell
py .\tools\08_fit_wand_geometry.py
```

Output:

```text
config/lighthouse_geometry_wand_3d.json
```

This is the main geometry file used by validation and live position.

## 5. Validate

```powershell
py .\tools\06_validate_geometry.py
```

For a quick floor-only check:

```powershell
py .\tools\06_validate_geometry.py --planar-2d --fixed-z 0.0
```

## 6. Live 3D position

```powershell
py .\tools\05_live_position.py --port COM3 --baudrate 115200
```

For a floor-only test:

```powershell
py .\tools\05_live_position.py --port COM3 --baudrate 115200 --planar-2d --fixed-z 0.0
```

## Legacy 2D Floor Path

The old 9-point floor calibration is still available if needed:

```powershell
py .\tools\03_capture_calibration_poses.py --port COM3 --baudrate 115200 --duration 4 --output config\calibration_poses_2d.json
py .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --poses config\calibration_poses_2d.json --output config\lighthouse_geometry_lh2_guided_ultrafast.json --max-nfev 300
```

But this is no longer the main path. The main path is wand 3D.

## ROS2 bridge

A progressive ROS2 bridge lives in:

```text
ros2_ws/src/lh2_ros_bridge
```

It keeps the current Python tools as the parsing/calibration/fit reference and adds ROS topics around them. See `docs/ros2_bridge.md`.
