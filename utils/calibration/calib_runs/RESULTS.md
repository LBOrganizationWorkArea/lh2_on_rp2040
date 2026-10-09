# Lighthouse calibration runs — 2026-10-09

Setup: 2 LH2 base stations, tape baseline BS0–BS1 = 2.26 m, mounted 3.45 m above
the floor. Static captures were taken with the drone on a **60 cm chair**, so
Z = 0 is the chair plane and the expected station height is about 2.85 m.

## Runs

| Directory | What it is |
|---|---|
| `run1/` | First wizard run. Its 1 m x-axis mark was misplaced (solver puts it at ~1.07–1.23 m), so its scale is not usable. |
| `run2/` | Second wizard run, recorded with the current firmware constants (`CAL_BS*` in `main.c`). |
| `run2_period/` | `run2` re-converted with the period model (`reconvert_run.py --model period`). |
| `run2_factory/` | `run2` re-converted with the period model plus the base stations' factory calibration (`--model factory`). **Best result.** |

Each solved directory has `solve.log` and `geometry.yaml` (from `calibrate_bitcraze.py --baseline 2.26`).

## Results (run2, scale from the 1 m mark)

| Angle model | Residual mean/max | Board fit | Tilt BS0/BS1 | Baseline | Height |
|---|---|---|---|---|---|
| Fitted `CAL_BS*` (current firmware) | 6.97 / 25.6 mm | 5.54 mm (fails) | 19.3° / 22.6° | 2.111 m (−6.6 %) | 2.66 m |
| Period model | 5.17 / 19.3 mm | 4.89 mm | 25.7° / 30.6° | 2.214 m (−2.0 %) | 2.73 m |
| **Period + factory calibration** | **5.09 / 17.1 mm** | **4.67 mm** | **22.3° / 27.6°** | **2.216 m (−1.9 %)** | **2.78–2.80 m** |
| Independent reference | | < 5 mm | accelerometers 22.2° / 27.5° | tape 2.26 m | ~2.85 m |

`run1` with the factory model gives tilts 22.6° / 27.8°, agreeing with `run2` to 0.3°.

## Findings

1. **Sweep-slot bug (fixed in firmware).** `lh2.c` puts a pulse in whichever slot is free, not
   by laser plane. The old swap correction only fixed the `B` offset, leaving swapped sensors off
   by ~1° horizontal and 6–10° vertical → 0 % plausible static captures. Fixed in
   `angle_decoder.c` by re-converting the swapped counts with the right plane's constants.
2. **Fitted `CAL_BS*` constants distort large angles.** Physics (DotBots, Alvarado, Bitcraze) gives
   one slope per station, `360° · 8 / period`, and planes 120° apart. The fitted constants have
   slopes 1–5 % off with A0 ≠ A1, and plane separations of 113° / 110°. Their error varies by
   2–3° (horizontal) and ~6.5° (vertical) across the field of view.
3. **Factory calibration (OOTX) decoded** with `lh2_ootx.py` → `lh2_factory_calibration.json`:

   | | BS0 `D1A56BEC` (mode 5) | BS1 `66DB72B9` (mode 11) |
   |---|---|---|
   | phase, plane 0 / 1 | 0.000° / +0.017° | 0.000° / +0.100° |
   | tilt, plane 0 / 1 | −2.72° / +2.65° | −2.67° / +2.65° |
   | gibmag, plane 0 / 1 | −0.05° / −0.19° | +0.30° / +0.15° |
   | accelerometer tilt | 22.2° | 27.5° |

   The planes are really 120° apart (phase ≤ 0.1°), but tilted ~27.3° instead of 30°.
4. **With period + factory calibration, the geometry matches every independent check:**
   station tilt equals the stations' own accelerometers to 0.1°, the baseline and height are
   within 2 % of the tape and expected values.
5. **Remaining ~5 mm board residual** is not explained by the angle model. Candidates: real
   photodiode positions vs nominal 40 mm, S2 occasional mis-decoded sweeps, inter-sensor timing
   during motion.
6. **Precision:** the solver's formal 1σ (now printed) is a lower bound; run-to-run differences
   are several times larger. Use repeated runs or a bootstrap for real uncertainty.

## Tools

- `capture_lh2.py` — wizard capture (writes `session.json` before the sweep).
- `calibrate_bitcraze.py` — solver; prints 1σ precision and writes it to `geometry.yaml`.
- `lh2_ootx.py` — decodes the factory calibration from the Pico USB `O,` stream.
- `reconvert_run.py` — re-derives a recorded session's angles with the period / factory model.
