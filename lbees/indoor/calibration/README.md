# Bitcrazy Calibration Procedure

## Calibration Steps

<img src="images/Untitled.jpg" alt="Imagem 1" style="width: 100%; height: auto; display: block;" />

<img src="images/Untitled (1).jpg" alt="Imagem 2" style="width: 100%; height: auto; display: block; margin-top: 16px;" />

<img src="images/Untitled (2).jpg" alt="Imagem 3" style="width: 100%; height: auto; display: block; margin-top: 16px;" />

# Lighthouse Calibration Tools

Standalone calibration tools that reuse the calibration pipeline from the Qt-based wizard as a command-line interface and Python library.

## Overview

This toolkit provides two main ways to calibrate your lighthouse base station geometry:

1. **CLI Tool** (`calibrate_lighthouse_cli.py`) - for automated calibration workflows
2. **Python API** (`calibrate_lighthouse_example.py`) - for integration into your own tools

Both use the same underlying calibration pipeline:
- Match angle measurements into pose samples
- Build initial geometry estimate using IPPE
- Refine with least-squares optimization
- Align to world frame
- Scale using reference measurements

## Installation

The calibration scripts use only the existing lighthouse modules in this directory. No additional installation needed if you have the base dependencies.

**Requirements:**
- numpy
- scipy
- pyyaml (for saving configuration)
- cflib (for geometry and calibration types)

## Usage

### CLI: Basic Calibration

The simplest way to run calibration:

```bash
./calibrate_lighthouse_cli.py measurements.json -o lighthouse_config.yaml
```

This will:
1. Load angle measurements from `measurements.json`
2. Run the full calibration pipeline
3. Save the result to `lighthouse_config.yaml`

### CLI: With Custom World Frame

Define your own world frame using reference points:

```bash
./calibrate_lighthouse_cli.py measurements.json \
  --origin 0 0 0 \
  --x-axis 1 0 0 \
  --xy-plane 0 1 0 \
  -o lighthouse_config.yaml
```

### CLI: Verbose Output

For debugging and understanding the calibration process:

```bash
./calibrate_lighthouse_cli.py measurements.json -o lighthouse_config.yaml -v
```

### Python API: Programmatic Usage

For integration into your own applications:

```python
from calibrate_lighthouse_cli import LighthouseCalibrator
import numpy as np

calibrator = LighthouseCalibrator(verbose=True)
success = calibrator.calibrate(
    'measurements.json',
    'output_config.yaml',
    origin=np.array([0, 0, 0]),
    x_axis=[np.array([1, 0, 0])],
    xy_plane=[np.array([0, 1, 0])]
)
```

See `calibrate_lighthouse_example.py` for more detailed examples.

## Measurement Format

Measurements should be saved as JSON with the following structure:

```json
[
  {
    "timestamp": 0.123,
    "base_station_id": 0,
    "angles": [
      [horizontal, vertical],
      [horizontal, vertical],
      [horizontal, vertical],
      [horizontal, vertical]
    ]
  },
  ...
]
```

Where:
- `timestamp`: Time when measurement was taken (float, in seconds)
- `base_station_id`: Which lighthouse base station (0, 1, ...) 
- `angles`: 4 sensor angle pairs in **radians**
  - Each pair is [horizontal_angle, vertical_angle]
  - Horizontal: 0 = straight ahead, positive = left, negative = right
  - Vertical: 0 = straight ahead, positive = up, negative = down

### Collecting Measurements from Crazyflie

Use the `LighthouseSweepAngleAverageReader` from the calibration module to collect measurements. Here's a minimal example:

```python
from lbees.indoor.calibration import LighthouseSweepAngleAverageReader
from lbees.indoor.calibration.lighthouse_types import LhMeasurement
import json
import time

measurements = []
cf = your_crazyflie_instance  # Already connected

def measurement_ready(averages):
    for bs_id, (count, angles) in averages.items():
        measurements.append({
            'timestamp': time.time(),
            'base_station_id': bs_id,
            'angles': [
                [v.lh_v1_horiz_angle, v.lh_v1_vert_angle]
                for v in angles
            ]
        })

reader = LighthouseSweepAngleAverageReader(cf, measurement_ready)
reader.nr_of_samples_required = 50

# Move the Crazyflie around and collect measurements
reader.start_angle_collection()
# ... move CF to different positions ...
# When enough samples collected, measurement_ready() is called automatically

# Save to file
with open('measurements.json', 'w') as f:
    json.dump(measurements, f)
```

## Calibration Pipeline Details

### 1. Match Measurements
Aggregates angle measurements that occur at approximately the same time into pose samples. Parameters:
- `max_time_diff`: Maximum time span to group measurements (default 20ms)
- `min_nr_of_bs_in_match`: Minimum base stations per sample (default 2)

### 2. Initial Estimate (IPPE)
Uses Infinitesimal Plane-Based Pose Estimation to compute an initial guess of base station poses. Automatically handles the two mirror solutions from IPPE and picks the correct one by clustering.

### 3. Geometry Solving
Iterative least-squares optimization to refine poses. Minimizes the error between measured angles and projected angles based on estimated poses. Converges to a solution or stops after max iterations.

### 4. Alignment
Transforms the coordinate system so that:
- Origin is at specified position
- X-axis points through specified x_axis points
- XY-plane contains specified xy_plane points

This allows you to define your own world frame.

### 5. Scaling
Adjusts the scale of the entire system to match real-world measurements. Two methods:
- **Diagonal spacing** (default): Uses the known physical spacing between lighthouse deck sensors as reference
- **Fixed point**: Uses a known distance to a measured position

## Output Format

The output is a YAML file containing base station geometry that can be loaded into the Crazyflie:

```yaml
type: lighthouse_system_configuration
version: '1'
systemType: 2
geos:
  0:
    # Base station 0 geometry
    position: [x, y, z]
    ...
  1:
    # Base station 1 geometry
    ...
calibs: {}
```

## Examples

### Complete Calibration Workflow

See `calibrate_lighthouse_example.py` for:
- Basic calibration with default parameters
- Custom world frame definition
- File-based calibration workflow
- Collecting measurements from actual Crazyflie

Run the examples:
```bash
python calibrate_lighthouse_example.py
```

### Integration with Existing Code

The calibration modules can be imported and used independently:

```python
from lbees.indoor.calibration.lighthouse_sample_matcher import LighthouseSampleMatcher
from lbees.indoor.calibration.lighthouse_initial_estimator import LighthouseInitialEstimator
from lbees.indoor.calibration.lighthouse_geometry_solver import LighthouseGeometrySolver
from lbees.indoor.calibration.lighthouse_system_aligner import LighthouseSystemAligner
from lbees.indoor.calibration.lighthouse_system_scaler import LighthouseSystemScaler

# Use them in your own applications
```

## Architecture

The calibration pipeline is built from reusable modules:

- **`lighthouse_types.py`** - Core data structures (Pose, LhMeasurement, LhCfPoseSample)
- **`lighthouse_sample_matcher.py`** - Aggregates measurements into pose samples
- **`lighthouse_initial_estimator.py`** - IPPE-based initial pose estimation
- **`lighthouse_geometry_solver.py`** - Least-squares geometry refinement
- **`lighthouse_system_aligner.py`** - World frame alignment
- **`lighthouse_system_scaler.py`** - System scaling
- **`lighthouse_bs_vector.py`** - Angle representation and conversions
- **`ippe_cf.py`** - IPPE solver interface

Each module is independent and can be used in isolation.

## Troubleshooting

### "Too little data, no reference" Error
You need measurements from at least 2 base stations. Ensure your measurements contain observations from multiple base stations with at least 2-3 samples each.

### Solver Did Not Converge
This often means:
- Too few measurements
- Measurements have high noise
- Base stations are too far apart
- Try collecting more measurements at different positions

### Large Error Values
If mean error is unusually large:
- Check that your angle measurements are in radians, not degrees
- Ensure sensor positions are correct (device-specific)
- Verify timeline - measurements should span a few seconds

### Unrealistic Base Station Positions
If base stations appear upside-down or mirrored:
- Use custom `--x-axis` and `--xy-plane` to specify your physical setup
- Or collect more samples to improve disambiguation

## Contributing

To improve the calibration:
- Test with real measurements from your environment
- Report measurement quality or convergence issues
- Save example measurement files for debugging

## License

GNU General Public License v3.0 - See LICENSE file
