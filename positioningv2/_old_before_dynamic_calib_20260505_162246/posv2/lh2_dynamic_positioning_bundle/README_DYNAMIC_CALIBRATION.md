# LH2 Dynamic Calibration / Indoor Positioning

Goal: do **one dynamic calibration** by moving the drone with 4 known TS4231 sensors for 60-90 seconds, solve the two Lighthouse poses, save them, then reuse the geometry for live 2D positioning as long as the basestations do not move.

## Folder location on your PC
Recommended root:

```powershell
C:\Users\elkah\lh2_positioning\positioningv2
```

## Workflow

1. Edit `config/sensors_layout.json` with real sensor positions in meters.
2. Put the two Lighthouse basestations anywhere, but keep them fixed.
3. Run raw capture:

```powershell
py tools\capture_dynamic_calibration_raw.py --port COM3 --seconds 90 --out data\captures\dynamic_raw_001.jsonl
```

4. Convert raw capture to angle observations CSV. This needs your firmware output to contain decoded `theta` and `phi` angles, or you must adapt the parser:

```powershell
py tools\convert_raw_to_observations.py --raw data\captures\dynamic_raw_001.jsonl --out data\captures\dynamic_obs_001.csv
```

5. Filter observations:

```powershell
py tools\filter_dynamic_observations.py --input data\captures\dynamic_obs_001.csv --output data\captures\dynamic_obs_001_filtered.csv
```

6. Solve Lighthouse geometry:

```powershell
py tools\solve_dynamic_lighthouse_geometry.py --layout config\sensors_layout.json --observations data\captures\dynamic_obs_001_filtered.csv --config config\calibration_config.json --out config\lighthouse_geometry_dynamic.json
```

7. Live position using saved geometry:

```powershell
py tools\live_position_from_geometry.py --layout config\sensors_layout.json --geometry config\lighthouse_geometry_dynamic.json --observations data\captures\one_frame_obs.csv
```

## Required Python packages

```powershell
py -m pip install numpy scipy pandas pyserial
```

## Important

The solve script expects this CSV format:

```csv
timestamp,sensor_id,lighthouse_id,theta,phi,valid
0.000,S0,4,0.123,-0.045,1
0.000,S1,4,0.130,-0.050,1
0.000,S0,10,-0.210,-0.030,1
```

Angles must be in radians.
