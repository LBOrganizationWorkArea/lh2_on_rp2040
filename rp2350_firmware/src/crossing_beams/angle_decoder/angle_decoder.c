/**
 * @file   angle_decoder.c
 * @brief  LH2 angle decoder — LFSR counts → azimuth + elevation (EMA filtered)
 *
 * Direct C port of utils/angle_lib/angle_decoder.py.
 *
 * Polynomial → basestation index mapping (from Python _determine_polynomial /
 * angle_decoder.py):
 *   poly 8  or 9  → bs index 0  (physical base station 4)
 *   poly 20 or 21 → bs index 1  (physical base station 10)
 *   anything else → skip
 */

#include "angle_decoder.h"

#define _USE_MATH_DEFINES  /* MSVC */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/** Convert a polynomial number to a basestation index (0 or 1), or -1. */
static inline int _poly_to_bs(uint8_t poly) {
    if (poly == 8 || poly == 9)   return 0;  // physical BS 4
    if (poly == 20 || poly == 21) return 1;  // physical BS 10
    return -1;
}

/**
 * @brief  Attempt to compute azimuth + elevation from two accumulated sweeps.
 *
 * Called whenever both has_sweep[0] and has_sweep[1] are true.
 * Updates ema_az, ema_el, valid, last_update_us and resets has_sweep.
 */
static void _finalize_angles(lh2_angles_t *slot,
                             const lh2_cal_t *cal,
                             uint64_t now_us)
{
    float a0 = slot->raw_sweep[0];
    float a1 = slot->raw_sweep[1];

    float az_raw = (a0 + a1) * 0.5f;
    float diff   = a0 - a1;

    /* Swap guard: if |diff| > 90° the two sweeps were interchanged.
     * Correction mirrors the Python LH2Decoder._compute_angles():
     *   diff = 2*(B0 - B1) - diff
     */
    if (fabsf(diff) > 90.0f) {
        diff = 2.0f * (cal->B0 - cal->B1) - diff;
    }

    float diff_rad = (diff * 0.5f) * ((float)M_PI / 180.0f);
    float az_rad   = az_raw              * ((float)M_PI / 180.0f);

    /* elevation = atan( tan(diff/2) / TAN_30 / cos(az) )  [Python reference] */
    float y_proj  = tanf(diff_rad) / TAN_30 / cosf(az_rad);
    float el_raw  = atanf(y_proj) * (180.0f / (float)M_PI);

    /* EMA update */
    if (!slot->valid) {
        slot->ema_az = az_raw;
        slot->ema_el = el_raw;
    } else {
        slot->ema_az = EMA_ALPHA * az_raw + (1.0f - EMA_ALPHA) * slot->ema_az;
        slot->ema_el = EMA_ALPHA * el_raw + (1.0f - EMA_ALPHA) * slot->ema_el;
    }

    slot->valid          = true;
    slot->last_update_us = now_us;
    slot->has_sweep[0]   = false;
    slot->has_sweep[1]   = false;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void angle_decoder_init(lh2_angles_t out[NUM_SENSORS][NUM_BS],
                        const lh2_cal_t cal[NUM_BS])
{
    (void)cal;  /* not used at init time — kept in signature for symmetry */
    memset(out, 0, sizeof(lh2_angles_t) * NUM_SENSORS * NUM_BS);
    for (int s = 0; s < NUM_SENSORS; s++) {
        for (int b = 0; b < NUM_BS; b++) {
            out[s][b].raw_sweep[0] = 0.0f;
            out[s][b].raw_sweep[1] = 0.0f;
            out[s][b].has_sweep[0] = false;
            out[s][b].has_sweep[1] = false;
            out[s][b].ema_az       = 0.0f;
            out[s][b].ema_el       = 0.0f;
            out[s][b].valid        = false;
            out[s][b].last_update_us = 0;
        }
    }
}

void angle_decoder_update(db_lh2_t        lh2[NUM_SENSORS],
                          lh2_angles_t    out[NUM_SENSORS][NUM_BS],
                          const lh2_cal_t cal[NUM_BS],
                          uint64_t        now_us)
{
    for (int s = 0; s < NUM_SENSORS; s++) {
        for (int sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
            for (int slot = 0; slot < LH2_BASESTATION_COUNT; slot++) {
                /* Only consume slots that have fresh raw data */
                if (lh2[s].data_ready[sweep][slot] != DB_LH2_RAW_DATA_AVAILABLE) {
                    continue;
                }

                uint8_t  poly = lh2[s].locations[sweep][slot].selected_polynomial;
                uint32_t lfsr = lh2[s].locations[sweep][slot].lfsr_location;
                int bs_idx    = _poly_to_bs(poly);

                /* Mark slot consumed regardless of whether we use the data */
                lh2[s].data_ready[sweep][slot] = DB_LH2_NO_NEW_DATA;

                if (bs_idx < 0) {
                    continue;  /* unknown polynomial — skip */
                }

                const lh2_cal_t *c = &cal[bs_idx];
                lh2_angles_t    *ang = &out[s][bs_idx];

                /* Convert LFSR count to sweep angle */
                float raw_angle;
                if (sweep == 0) {
                    raw_angle = c->A0 * (float)lfsr + c->B0;
                } else {
                    raw_angle = c->A1 * (float)lfsr + c->B1;
                }

                ang->raw_sweep[sweep] = raw_angle;
                ang->has_sweep[sweep] = true;

                /* If both sweeps are now in, compute az/el */
                if (ang->has_sweep[0] && ang->has_sweep[1]) {
                    _finalize_angles(ang, c, now_us);
                }
            }
        }
    }
}

bool angle_decoder_is_fresh(const lh2_angles_t out[NUM_SENSORS][NUM_BS],
                            int s,
                            uint64_t now_us)
{
    if (s < 0 || s >= NUM_SENSORS) return false;
    for (int b = 0; b < NUM_BS; b++) {
        const lh2_angles_t *a = &out[s][b];
        if (!a->valid) return false;
        if ((now_us - a->last_update_us) > FRESHNESS_US) return false;
    }
    return true;
}
