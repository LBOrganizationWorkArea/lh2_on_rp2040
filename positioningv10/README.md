# positioningv10 workflow

Run commands from this folder:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv10
```

The main v10 calibration uses **both**:

```text
1. anchored known points -> define the room/floor coordinate frame
2. wand wave motion -> add dense moving observations and coverage checks
```

The known points are what anchor the result to the real room. The wand wave is not a replacement for the points; it complements them by giving many more observations while moving through the volume.

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

This checks that `LH2P` lines are parsed, factory calibration is loaded, parasite angle families are filtered, and missing `0;0` sensors do not create fake angles.

## 3. Capture anchored points

The point file is:

```text
config/wand_3d_points.json
```

It contains floor points and higher box points. Edit this file if the measured positions are different.

Capture the anchored points:

```powershell
py .\tools\07_capture_wand_poses.py --port COM3 --baudrate 115200 --duration 4 --resume
```

Output:

```text
config/wand_calibration_poses_3d.json
```

To recapture only a few points:

```powershell
py .\tools\07_capture_wand_poses.py --port COM3 --baudrate 115200 --resume --only P04_bas_centre,P11_boite45_centre
```

## 4. Fit anchored Lighthouse geometry

```powershell
py .\tools\08_fit_wand_geometry.py
```

Output:

```text
config/lighthouse_geometry_wand_3d.json
```

This geometry is anchored by the known points.

## 5. Record wand wave

After the point-based geometry exists, record moving observations through the useful volume:

```powershell
py .\tools\09_record_wand_wave.py --port COM3 --baudrate 115200 --duration 60
```

Output:

```text
config/wand_wave_record.json
```

This file is used to check coverage and later refine the calibration with moving constraints. The wave should pass through places where the drone will actually fly.

## 6. Validate

Validate against anchored points:

```powershell
py .\tools\06_validate_geometry.py
```

Quick floor-only check:

```powershell
py .\tools\06_validate_geometry.py --planar-2d --fixed-z 0.0
```

## 7. Live 3D position

```powershell
py .\tools\05_live_position.py --port COM3 --baudrate 115200
```

Floor-only test:

```powershell
py .\tools\05_live_position.py --port COM3 --baudrate 115200 --planar-2d --fixed-z 0.0
```

## Legacy 2D Floor Path

The old 9-point floor calibration is still available if needed:

```powershell
py .\tools\03_capture_calibration_poses.py --port COM3 --baudrate 115200 --duration 4 --output config\calibration_poses_2d.json
py .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --poses config\calibration_poses_2d.json --output config\lighthouse_geometry_lh2_guided_ultrafast.json --max-nfev 300
```

But the intended v10 flow is points + wand wave together.

## ROS2 bridge

A progressive ROS2 bridge lives in:

```text
ros2_ws/src/lh2_ros_bridge
```

It keeps the current Python tools as the parsing/calibration/fit reference and adds ROS topics around them. See `docs/ros2_bridge.md`.
