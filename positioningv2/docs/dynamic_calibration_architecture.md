# Dynamic Lighthouse Calibration Architecture

This pipeline estimates a fixed Lighthouse geometry from many drone observations collected during free motion.

## Data Flow

1. The receiver produces raw decoded Lighthouse 2.0 LFSR observations, not angles:

   ```text
   LH2,sensor,sweep,basestation,polynomial,lfsr
   ```

   Older firmware can also print:

   ```text
   sen_0 (8-12906 9-53998 21-54125 21-18082)
   ```

2. The PC parser converts each raw entry into:

   ```text
   pc_time,sensor_id,lighthouse_id,polynomial,sweep,lfsr
   ```

   `polynomial` is the Lighthouse V2 polynomial id. `lfsr` is the decoded LFSR location. The stable sweep id is computed on the PC as `polynomial & 1`; the firmware sweep column is not trusted.

3. A conversion step groups raw rows into short time windows and pairs sweep 0 + sweep 1 for each sensor and Lighthouse.
4. For debugging, current v7 coefficients can convert LFSR pairs into provisional sweep angles and approximate azimuth/elevation.
5. `scripts/07_estimate_lighthouse_geometry_sweeps_2d.py` performs the first global calibration from ordered sweep0/sweep1 measurements.
6. The solved Lighthouse poses are saved to `config/lighthouse_geometry.json`.
7. Live positioning keeps Lighthouse poses fixed and solves only drone `x`, `y`, and `yaw` for each new frame.

## Current Decoder Layer

The current conversion is:

```text
sweep0_deg = A0 * lfsr0 + B0
sweep1_deg = A1 * lfsr1 + B1
azimuth    = (sweep0_deg + sweep1_deg) / 2
elevation  = (sweep0_deg - sweep1_deg) / (2 * tan(30 deg))
```

This is useful for inspecting data and for test solvers, but it is still an approximation. The clean long-term target is to predict the Lighthouse V2 sweep-plane measurement directly from the geometry and compare it to the decoded LFSR/sweep measurement.

The first sweep-based dynamic solver does not use the `normal/swapped` azimuth/elevation mode. It compares ordered sweep measurements:

```text
poly 8 or 20 -> sweep0
poly 9 or 21 -> sweep1
```

It currently predicts sweeps using the inverse of the v7 azimuth/elevation approximation:

```text
sweep0 = azimuth + tan(30 deg) * elevation
sweep1 = azimuth - tan(30 deg) * elevation
```

This is cleaner than solving on the guessed azimuth/elevation columns, but it is still not the final physical Lighthouse 2.0 plane model.

## First Geometry Model

The first version uses an angular-camera approximation:

```text
p_lh = R_lh.T @ (p_world - t_lh)
azimuth = atan2(p_lh.y, p_lh.x)
elevation = atan2(p_lh.z, sqrt(p_lh.x^2 + p_lh.y^2))
```

This is not the full Valve Lighthouse 2.0 sweep-plane model. It is intentionally modular so `predict_angles()` can later be replaced by `predict_lh2_sweep_measurement()`.

## Planar Drone Assumption

For the first implementation:

- `drone_z` is fixed.
- roll is fixed to zero.
- pitch is fixed to zero.
- each drone frame has only `x`, `y`, and `yaw`.

Known sensor positions in the drone body frame provide metric scale.

## Gauge Freedom

Without an external reference, the solved map has gauge freedom. A globally translated or rotated map can explain the same angular observations. This implementation fixes the first drone frame to:

```text
x=0, y=0, yaw=0
```

That defines the world frame for the saved geometry. Future versions can use stronger anchors:

- known drone start position,
- IMU yaw prior,
- known Lighthouse height,
- known distance between floor markers,
- height sensor,
- partial Lighthouse pose constraints.

## Practical Limits

The first solver is a baseline, not the final physical model. Expected next improvements:

- true Lighthouse 2.0 sweep-plane prediction,
- robust outlier rejection per observation,
- use raw LFSR/sweep observations directly instead of approximate azimuth/elevation,
- IMU yaw prior,
- EKF filtering,
- real-time serial frame builder,
- automatic health checks for missing sensors.

If Lighthouse basestations move, `config/lighthouse_geometry.json` is no longer valid and calibration must be repeated.
