# Lighthouse V2 indoor positioning workflow

## Recommended calibration

The simplest reliable method is fixed known poses:

1. Put both Lighthouse V2 base stations wherever you want.
2. Do not move them after calibration.
3. Put the drone at several known positions on the floor.
4. Capture LH2 measurements for each pose.
5. Estimate the base-station geometry once.
6. Reuse that geometry for every drone that has the same sensor layout.

The current default frame is:

- origin: drone center at `P0_center`
- `+x`: right from the drone's initial direction
- `+y`: front from the drone's initial direction
- `+z`: up

With your setup, a good starting guess is:

- BS4: left/front, about `x=-0.50`, `y=+1.20`, `z=+1.20`
- BS10: right/front, about `x=+0.50`, `y=+1.20`, `z=+1.20`

## Commands

Install dependencies in the Python environment you use:

```powershell
pip install -r requirements.txt
```

Capture fixed calibration poses:

```powershell
python .\tools\03_capture_calibration_poses.py --port COM3 --duration 4
```

For real 3D positioning, capture poses at more than one height:

```powershell
python .\tools\03_capture_calibration_poses.py --port COM3 --duration 4 --pose-file .\config\calibration_poses_3d_example.json --output .\config\calibration_poses_3d.json
```

Estimate the Lighthouse geometry:

```powershell
python .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --poses .\config\calibration_poses_3d.json --lighthouse-z 1.20 --drone-z 0.00
```

If `config\lighthouse_factory_calibration_bs4.json` and `config\lighthouse_factory_calibration_bs10.json` exist, the estimator loads them automatically and includes the factory correction model in the saved geometry. To be explicit, use:

```powershell
python .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --factory-calibs "4=.\config\lighthouse_factory_calibration_bs4.json,10=.\config\lighthouse_factory_calibration_bs10.json"
```

To compare with the old uncorrected model:

```powershell
python .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --factory-calibs none
```

Run live positioning:

```powershell
python .\tools\05_live_position.py --port COM3
```

Validate the geometry before flying:

```powershell
python .\tools\06_validate_geometry.py
```

For a drone kept level, use:

```powershell
python .\tools\05_live_position.py --port COM3 --position-only
```

## Motion-capture option

If you later record poses with a motion-capture system, store each calibration pose with:

- `x_m`, `y_m`, `z_m`
- `roll_deg`, `pitch_deg`, `yaw_deg`
- `measurements`

The corrected geometry estimator now accepts those 3D pose fields. This lets you do a wand-style calibration later, but fixed known poses are simpler to validate first.
