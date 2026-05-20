# positioningv7_selfcalib_wand

Goal: use the drone itself as a calibration wand.

The 4 TS4231 sensors are a rigid square of side 12.5 cm. With calibrated LH2 angles, each Lighthouse can run PnP and estimate the drone pose in its own frame.

If the same drone pose is seen by BS4 and BS10 at the same time:

```text
drone pose in BS4 frame
drone pose in BS10 frame
```

then we can estimate the relative transform between BS4 and BS10. After that, live poses from both base stations can be expressed in one common frame.

This does not need external motion capture.

## Workflow

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv7_selfcalib_wand
py .\tools\01_record_wand_pnp.py --port COM3
py .\tools\02_fit_bs_relative.py
py .\tools\03_live_global_pose.py --port COM3
```

Move the drone like a wand for 30-60 seconds:

- left/right/front/back
- up/down
- yaw rotations
- small pitch/roll rotations

Keep both Lighthouses visible.

## Frame

The first implementation uses BS4 as the world frame.

So output `x,y,z` means:

```text
drone center position in BS4 virtual camera frame
```

Later, we can add a floor/world alignment step.
