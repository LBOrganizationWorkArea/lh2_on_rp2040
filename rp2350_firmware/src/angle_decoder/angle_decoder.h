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
/// Number of base stations. Index → physical station (channel = polynomial pair / 2):
///   0 = BS4 (poly 8/9), 1 = BS10 (poly 20/21), 2 = BS0 (poly 0/1), 3 = BS1 (poly 2/3).
/// BS4 / BS10 keep indices 0 / 1 so existing calibrations and recorded sessions stay valid.
#define NUM_BS      4
#define MIN_BS_FOR_POINT 2   ///< stations that must see a sensor to triangulate it

#define EMA_ALPHA   0.2f         ///< EMA smoothing factor (α in paper)
/* A pair farther than this from its EMA is a mis-decoded sweep (flight log 2026-10-21: one
 * wrong ray in ~13 % of snapshots) and is left out of the EMA; ANGLE_GATE_N in a row are real
 * motion and restart it. After EMA_RESET_US without data the EMA restarts too, instead of
 * blending in an old angle. */
#define ANGLE_GATE_RAD  0.052f   ///< 3 deg
#define ANGLE_GATE_N    3
#define EMA_RESET_US    100000ULL
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
    float phase;     ///< plane offset error [rad]
    float tilt;      ///< plane tilt error vs the nominal ±30° [rad]
    float gibmag;    ///< gibbous (azimuth wobble) magnitude [rad]
    float gibphase;  ///< gibbous phase [rad]
} lh2_plane_cal_t;

typedef struct {
    float A0, B0;   ///< sweep-0 coefficients
    float A1, B1;   ///< sweep-1 coefficients
    lh2_plane_cal_t plane[2];  ///< base-station factory calibration of plane 0 / 1 (all zero: none)
} lh2_cal_t;

/**
 * @brief  Decoded + EMA-filtered angles for one (sensor × basestation) pair.
 */
typedef struct {
    float    raw_sweep[2];       ///< pending raw angle per sweep [deg]; NAN until received
    uint32_t raw_lfsr[2];        ///< LFSR count behind raw_sweep (re-calibrated if the slots are swapped)
    bool     has_sweep[2];       ///< true once the corresponding raw_sweep is valid
    float    ema_az;             ///< EMA-smoothed azimuth   [degrees] (legacy solve3d path)
    float    ema_el;             ///< EMA-smoothed elevation [degrees] (legacy solve3d path)
    float    ema_horiz;          ///< EMA-smoothed Bitcraze horizontal angle [radians] (solver)
    float    ema_vert;           ///< EMA-smoothed Bitcraze vertical   angle [radians] (solver)
    float    raw_horiz;          ///< Instantaneous horizontal angle [radians]
    float    raw_vert;           ///< Instantaneous vertical angle [radians]
    bool     valid;              ///< true once at least one complete pair has been decoded
    uint8_t  gated;              ///< consecutive pairs left out of the EMA (ANGLE_GATE_RAD)
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
 * @brief  Enable/disable the OOTX bit stream on USB stdio.
 *
 * When enabled, every decoded sweep prints
 *   O,<sensor>,<bs index>,<polynomial>,<lfsr>,<capture time us>
 * The polynomial's parity is the base station's OOTX ("slow") bit for that
 * rotor turn; utils/calibration/lh2_ootx.py decodes the factory calibration
 * from it. Off by default (it is ~800 lines/s).
 */
void angle_decoder_set_ootx_stream(bool enabled);

/**
 * @brief  True if base station @p b has a fresh angle pair for sensor @p s.
 */
bool angle_decoder_bs_fresh(const lh2_angles_t out[NUM_SENSORS][NUM_BS],
                            int s, int b, uint64_t now_us);

/**
 * @brief  True if at least MIN_BS_FOR_POINT base stations have fresh angles for sensor @p s.
 *
 * @param out    angle state table
 * @param s      sensor index [0, NUM_SENSORS)
 * @param now_us current time in microseconds since boot
 */
bool angle_decoder_is_fresh(const lh2_angles_t out[NUM_SENSORS][NUM_BS],
                            int s,
                            uint64_t now_us);

/**
 * @brief  Angle pairs decoded per base station since boot (all sensors), for the
 *         decoder health line printed by main (STAT,...).
 */
uint32_t angle_decoder_pairs(int bs);

#endif /* ANGLE_DECODER_H */
