/**
 * @file   angle_decoder.h
 * @brief  LH2 angle decoder — LFSR counts → azimuth + elevation (EMA filtered)
 *
 * C port of utils/angle_lib/angle_decoder.py.
 * Sits on top of the lh2 hardware layer: reads db_lh2_t.locations,
 * converts LFSR counts to sweep angles using linear calibration coefficients,
 * reconstructs azimuth + elevation, and applies an EMA filter.
 *
 * Dependency: lh2/lh2.h  (db_lh2_t type only — no other crossing_beams headers)
 */

#ifndef ANGLE_DECODER_H
#define ANGLE_DECODER_H

#include <stdbool.h>
#include <stdint.h>
#include "../lh2/lh2.h"

// ---------------------------------------------------------------------------
// Compile-time constants
// ---------------------------------------------------------------------------
#define NUM_SENSORS 4   ///< Number of TS4231 photodiode sensors
#define NUM_BS      2   ///< Number of basestations (BS4 → index 0, BS10 → index 1)

#define EMA_ALPHA   0.2f         ///< EMA smoothing factor (α in paper)
#define TAN_30      0.57735027f  ///< tan(30°) — used in elevation reconstruction
#define FRESHNESS_US 500000ULL   ///< Maximum age before an angle is considered stale [µs]

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/**
 * @brief  Linear calibration coefficients for one base station.
 *
 *   sweep 0 angle [deg] = A0 * lfsr + B0
 *   sweep 1 angle [deg] = A1 * lfsr + B1
 */
typedef struct {
    float A0, B0;   ///< sweep-0 coefficients
    float A1, B1;   ///< sweep-1 coefficients
} lh2_cal_t;

/**
 * @brief  Decoded + EMA-filtered angles for one (sensor × basestation) pair.
 */
typedef struct {
    float    raw_sweep[2];       ///< pending raw angle per sweep [deg]; NAN until received
    bool     has_sweep[2];       ///< true once the corresponding raw_sweep is valid
    float    ema_az;             ///< EMA-smoothed azimuth   [degrees] (legacy solve3d path)
    float    ema_el;             ///< EMA-smoothed elevation [degrees] (legacy solve3d path)
    float    ema_horiz;          ///< EMA-smoothed Bitcraze horizontal angle [radians] (ray_cross)
    float    ema_vert;           ///< EMA-smoothed Bitcraze vertical   angle [radians] (ray_cross)
    bool     valid;              ///< true once at least one complete pair has been decoded
    uint64_t last_update_us;     ///< timestamp of last successful decode [µs since boot]
} lh2_angles_t;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Initialise the angle state table and load calibration.
 *
 * Must be called once before the first angle_decoder_update().
 *
 * @param out  [NUM_SENSORS][NUM_BS] table to initialise
 * @param cal  calibration coefficients for each basestation
 */
void angle_decoder_init(lh2_angles_t out[NUM_SENSORS][NUM_BS],
                        const lh2_cal_t cal[NUM_BS]);

/**
 * @brief  Scan all data_ready slots, decode new LFSR counts, update EMA angles.
 *
 * Call every loop iteration, immediately after db_lh2_process_location().
 * Clears data_ready flags for every slot it consumes.
 *
 * @param lh2    array of NUM_SENSORS lh2 instances
 * @param out    angle state table (updated in place)
 * @param cal    calibration coefficients
 * @param now_us current time in microseconds since boot
 */
void angle_decoder_update(db_lh2_t        lh2[NUM_SENSORS],
                          lh2_angles_t    out[NUM_SENSORS][NUM_BS],
                          const lh2_cal_t cal[NUM_BS],
                          uint64_t        now_us);

/**
 * @brief  Return true if both basestations for sensor @p s have fresh angles.
 *
 * @param out    angle state table
 * @param s      sensor index [0, NUM_SENSORS)
 * @param now_us current time in microseconds since boot
 */
bool angle_decoder_is_fresh(const lh2_angles_t out[NUM_SENSORS][NUM_BS],
                            int s,
                            uint64_t now_us);

#endif /* ANGLE_DECODER_H */
