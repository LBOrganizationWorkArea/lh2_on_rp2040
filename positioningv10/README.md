# positioningv10 workflow

Run commands from this folder:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv10
```

## 1. Check raw serial

```powershell
py .\tools\01_live_view.py --port COM3 --baudrate 115200
```

The firmware should print only `LH2P;...` lines.

## 2. Check angles

```powershell
py .\tools\02_live_angles.py --port COM3 --baudrate 115200
```

This parser:

- accepts `LH2P` with `;`
- identifies axes from polynomials, not from the raw sweep fields
- converts offsets with Bitcraze LH2 periods
- applies factory calibration from `config`
- rejects the weaker parasite offset family inside each time window

## 3. Capture the 9 ground points

```powershell
py .\tools\03_capture_calibration_poses.py --port COM3 --baudrate 115200 --duration 4 --output config\calibration_poses_2d.json
```

The default pose list is the 9 point floor pattern: center, four cardinal points, and four diagonals.

## 4. Fit lighthouse geometry

```powershell
py .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --poses config\calibration_poses_2d.json --output config\lighthouse_geometry_lh2_guided_ultrafast.json --max-nfev 300
```

When the pose file contains `calibrated_angle_rad`, the solver treats the measurements as already factory corrected and does not embed factory calibration again in the geometry.

## 5. Validate

```powershell
py .\tools\06_validate_geometry.py --planar-2d --fixed-z 0.0
```

## 6. Live position

```powershell
py .\tools\05_live_position.py --port COM3 --baudrate 115200 --planar-2d --fixed-z 0.0
```

For full 3D pose later, capture calibration points at different heights and run live without `--planar-2d`.

## ROS2 bridge

A progressive ROS2 bridge lives in:

```text
ros2_ws/src/lh2_ros_bridge
```

It keeps the current Python tools as the parsing/calibration/fit reference and
adds ROS topics around them. See `docs/ros2_bridge.md`.
