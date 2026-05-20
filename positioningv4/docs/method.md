# positioningv4 Method

The 4 sensors are only 12.5 cm apart, so they are not enough by themselves to recover a strong Lighthouse geometry.

The v4 calibration uses a larger baseline: the drone is moved to many known positions on the floor.

For each known drone pose:

1. The drone center `x,y` is known.
2. The sensor offsets are known from `config/sensors_layout.json`.
3. Therefore each sensor has a known floor point.
4. Each Lighthouse measurement gives an image-like point from sweep 0 and sweep 1.
5. A homography is fitted per Lighthouse:

```text
Lighthouse image point -> floor x,y
```

In live mode:

1. Each visible sensor and base station gives one estimated sensor floor point.
2. The sensor offset is subtracted to estimate the drone center.
3. The center candidates are combined with a robust median.

## Important Limit

This first version assumes the drone yaw is the same as during calibration. If the drone rotates, subtracting fixed sensor offsets will be wrong.

The next improvement is to estimate `x,y,yaw` together from the four sensor world points.
