# ROS2 bridge for LH2P

This bridge keeps the existing `tools/` scripts as the reference implementation.
ROS2 is only wrapped around the serial stream, parsing, calibration recording, and
future live position publishing.

## Firmware serial contract

The final RP2040 firmware should emit only useful `LH2P` lines:

```text
LH2P;bs;sweep0;sweep1;poly0;poly1;block0;block1;delta;o0a;o0b;o1a;o1b;o2a;o2b;o3a;o3b
```

`firmware/lh2_on_rp2040/src/main.c` now has:

```c
#define LH2_OUTPUT_HEARTBEAT 0
#define LH2_OUTPUT_BOOT_BANNER 0
```

Leave both at `0` for the final flight/ROS workflow. Set `LH2_OUTPUT_HEARTBEAT`
back to `1` only for temporary firmware diagnostics.

## Build the ROS2 package

From a ROS2-enabled terminal:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv10\ros2_ws
colcon build --symlink-install
.\install\setup.ps1
```

The package is intentionally plain `ament_python` and uses standard message
types first:

- `/lh2/raw_line`: `std_msgs/String`
- `/lh2/parsed`: `std_msgs/String`, JSON payload
- `/lh2/calibration_command`: `std_msgs/String`, JSON or pose name
- `/lh2/calibration_status`: `std_msgs/String`, JSON status
- `/lh2/position`: `geometry_msgs/PointStamped`
- `/coord`: `std_msgs/Float32MultiArray`, planned as `x,y,z,vx,vy,vz`

## Run the first bridge stack

```powershell
ros2 launch lh2_ros_bridge lh2_bridge.launch.py port:=COM3 baudrate:=115200 output:=C:\Users\elkah\lh2_positioning\positioningv10\config\calibration_poses_2d_ros.json
```

Or start each node manually as shown below.

## Run serial -> raw topic

```powershell
ros2 run lh2_ros_bridge lh2_serial_node --ros-args -p port:=COM3 -p baudrate:=115200
```

The serial node publishes only lines beginning with `LH2P;`. Anything else is
ignored, so old debug output cannot contaminate ROS topics.

## Run parser

```powershell
ros2 run lh2_ros_bridge lh2_parser_node --ros-args -p factory_calibs:=auto
```

The parser imports `tools/lh2v10.py` and `tools/lh2_factory_model.py`, then
publishes one JSON message per `LH2P` frame. The JSON includes the raw fields,
sensor offsets, validity of the polynomial axes, and converted observations with
`raw_angle_rad` / `calibrated_angle_rad`.

## Record calibration poses with ROS2

Start the recorder:

```powershell
ros2 run lh2_ros_bridge lh2_calibration_recorder_node --ros-args `
  -p output:=C:\Users\elkah\lh2_positioning\positioningv10\config\calibration_poses_2d_ros.json `
  -p duration_s:=4.0 `
  -p basestations:=4,10
```

Capture a known default pose by name:

```powershell
ros2 topic pub --once /lh2/calibration_command std_msgs/String "{data: P0_center}"
```

Capture a custom pose:

```powershell
ros2 topic pub --once /lh2/calibration_command std_msgs/String "{data: '{\"command\":\"capture\",\"name\":\"P1_right_40cm\",\"x_m\":0.40,\"y_m\":0.0,\"z_m\":0.0,\"yaw_deg\":0.0}'}"
```

The recorder writes a JSON with the same top-level shape as
`tools/03_capture_calibration_poses.py`, including `poses[].measurements` and
`poses[].missing_channels`.

## Fit and validate outside ROS

Keep using the existing solver:

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv10
py .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --poses config\calibration_poses_2d_ros.json --output config\lighthouse_geometry.json --lighthouse-z 1.70 --bs4-guess 0.40,3.00 --bs10-guess -0.20,3.00 --max-nfev 500
py .\tools\06_validate_geometry.py --geometry config\lighthouse_geometry.json --poses config\calibration_poses_2d_ros.json --planar-2d
```

## Position node status

`lh2_position_node` currently owns the future ROS output topics and checks that
the geometry file exists. The numerical live solver remains in
`tools/05_live_position.py` until the bridge has proven stable.

```powershell
ros2 run lh2_ros_bridge lh2_position_node --ros-args -p geometry:=C:\Users\elkah\lh2_positioning\positioningv10\config\lighthouse_geometry.json
```
