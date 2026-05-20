# Raw Lighthouse LFSR Format

The Pico firmware is kept as a receiver. It does not output final position or final angles.

## CSV Firmware Format

```text
LH2,sensor,sweep,basestation,polynomial,lfsr
LH2,2,0,4,8,15493
```

Some diagnostic firmware includes a firmware timestamp:

```text
LH2,time_us,sensor,sweep,basestation,polynomial,lfsr
```

Meaning:

- `sensor`: TS4231 sensor id on the drone.
- `polynomial`: Lighthouse V2 polynomial id decoded by the firmware.
- `lfsr`: decoded LFSR location for that sweep.
- `basestation`: usually `polynomial >> 1`.
- `sweep`: should be treated as diagnostic only. The PC code recomputes it as `polynomial & 1`.

For this setup:

```text
poly 8  -> BS4  sweep0
poly 9  -> BS4  sweep1
poly 20 -> BS10 sweep0
poly 21 -> BS10 sweep1
```

## Compact Firmware Format

```text
sen_0 (8-12906 9-53998 21-54125 21-18082)
```

Each `A-B` pair means:

- `A`: polynomial id.
- `B`: LFSR location.

The PC parser expands this into one raw observation per pair.

## PC Pipeline

1. Record raw serial into `data/captures/*_lfsr_raw.csv`.
2. Pair sweep 0 and sweep 1 by time window, sensor, and Lighthouse.
3. Convert paired LFSR values with calibration coefficients when needed.
4. Use those paired measurements in the dynamic calibration or live positioning model.

The current angle conversion is provisional. It is kept separate so the final Lighthouse 2.0 sweep-plane model can replace it without changing the recorder.
