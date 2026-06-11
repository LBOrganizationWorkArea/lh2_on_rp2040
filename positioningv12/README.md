# positioningv12 User Guide

v12 follows a "Lighthouse frame first" workflow.

The principle is:

1. BS4 is fixed as the origin of the Lighthouse coordinate frame.
2. Wand waving is used to calibrate BS10 relative to BS4.
3. A few known points are then used to transform the Lighthouse frame into the room frame.
4. During live operation, the drone pose is estimated through PnP/angle optimization in the Lighthouse frame and then converted into the room frame.

This workflow is cleaner than forcing the Lighthouses directly into the room frame from the beginning. Relative Lighthouse calibration does not initially need to know the floor, room center, or global orientation.

## 0. Open the v12 directory

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv12
```

## 1. Check LH2A

```powershell
py .\tools\17_diagnose_lh2a_families.py --port COM3 --baudrate 115200 --duration 5 --cluster-deg 8
```

Expected results:

- many `LH2A` messages;
- all 16 channels visible;
- two stable families per channel;
- low spread while the drone/wand is stationary.

## 2. Record wand waving in the Lighthouse frame

Move the drone/wand throughout the useful volume while varying:

- left/right position;
- forward/backward position;
- height;
- smooth rotations.

Command:

```powershell
py .\tools\21_record_lh2a_wave.py --port COM3 --baudrate 115200 --duration 60 --window 0.20 --period 0.25
```

Output:

```text
config/lh2a_wave_record.json
```

This file does not contain known positions. It only contains angle families for each frame.

## 3. Fit the relative Lighthouse geometry

Goals:

- keep BS4 fixed at `(0,0,0)`;
- use BS4 to define the axes of the Lighthouse frame;
- estimate BS10 relative to BS4;
- treat drone poses during the wave as internal variables;
- select consistent `LH2A` families.

Planned command:

```powershell
py .\tools\22_fit_relative_lighthouse_frame.py --wave config\lh2a_wave_record.json
```

Planned output:

```text
config/lighthouse_relative_geometry.json
```

Note: this step is the core of v12. The script is provided as the workflow entry point; the bundle-adjustment optimization will be finalized after the first wave capture.

## 4. Capture known points to anchor the room frame

Once the relative Lighthouse geometry is plausible, capture a few known points. These points are not used to locate BS10. They are used to calculate:

```text
Lighthouse frame -> room frame
```

Command:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 1.0 --resume --only P00_bas_avant_gauche,P04_bas_centre,P08_bas_arriere_droite,P11_boite45_centre
```

Then validate the capture:

```powershell
py .\tools\20_validate_lh2a_family_capture.py --max-spread-deg 0.5
```

## 5. Calculate the room anchor

Planned command:

```powershell
py .\tools\23_anchor_lighthouse_frame_to_room.py --geometry config\lighthouse_relative_geometry.json --points config\wand_calibration_poses_3d_lh2a_families.json
```

Planned output:

```text
config/lighthouse_to_room_transform.json
```

This transformation converts positions calculated in the Lighthouse frame into the room frame.

## 6. Live pose / PnP

During live operation:

1. read the `LH2A` angles;
2. use the relative Lighthouse geometry;
3. solve the drone pose using the fixed sensor layout;
4. convert the pose into the room frame with `lighthouse_to_room_transform.json`.

Planned command:

```powershell
py .\tools\24_live_pnp_lighthouse_frame.py --port COM3 --baudrate 115200
```

## Why two Lighthouses help

One Lighthouse mainly provides an angular direction. Two Lighthouses provide two viewpoints, resulting in much stronger triangulation.

With four fixed sensors on the drone:

```text
2 Lighthouses + 4 sensors = a much more robust 6D pose
```

BS4 is used as the reference frame. BS10 provides the second viewpoint.

## Recommended order

1. Diagnose `LH2A`.
2. Record a clean wave.
3. Fit the relative `BS4 -> BS10` geometry.
4. Capture 4 to 8 clean known points.
5. Calculate the room anchor.
6. Test live PnP.
7. Record a longer wave and refine the calibration.
