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
#include <stdio.h>
#include <string.h>

#include "pico/time.h"

#define LFSR_LOG_INTERVAL_US 250000ULL

static uint64_t s_last_lfsr_log_us[NUM_SENSORS][LH2_BASESTATION_COUNT][LH2_SWEEP_COUNT];
static bool     s_ootx_stream = false;

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
 * @brief  Measured beam angle of one light plane for the ideal ray (1, y, z).
 *
 * Base-station factory model (OOTX calibration data, as used by the Crazyflie):
 * the plane is tilted by (nominal - tilt), offset by phase, and wobbles by
 * gibmag * cos(azimuth + gibphase). Curve / ogee terms are not used.
 */
static float _distorted_beam(float y, float z, float nominal_tilt, const lh2_plane_cal_t *p)
{
    float azimuth = atanf(y);
    float s = z * tanf(nominal_tilt - p->tilt) / sqrtf(1.0f + y * y);
    s = fminf(1.0f, fmaxf(-1.0f, s));
    return azimuth + asinf(s) - p->phase + p->gibmag * cosf(azimuth + p->gibphase);
}

/**
 * @brief  Remove the factory distortion from both beam angles [rad], in place.
 *
 * The model maps ideal to measured angles, so it is inverted by fixed-point
 * iteration starting from the measured angles (converges in 2-3 steps).
 */
static void _apply_factory_cal(const lh2_cal_t *cal, float *beam0, float *beam1)
{
    const float measured0 = *beam0, measured1 = *beam1;
    float ideal0 = measured0, ideal1 = measured1;
    for (int i = 0; i < 5; i++) {
        float y = tanf(0.5f * (ideal0 + ideal1));
        float z = sinf(ideal1 - ideal0) / (TAN_30 * (cosf(ideal0) + cosf(ideal1)));
        float error0 = measured0 - _distorted_beam(y, z, -(float)M_PI / 6.0f, &cal->plane[0]);
        float error1 = measured1 - _distorted_beam(y, z, (float)M_PI / 6.0f, &cal->plane[1]);
        ideal0 += error0;
        ideal1 += error1;
        if (fabsf(error0) < 1e-6f && fabsf(error1) < 1e-6f) break;
    }
    *beam0 = ideal0;
    *beam1 = ideal1;
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

    /* Swap guard: if |a0 - a1| > 90° the two sweeps were interchanged.
     * db_lh2's sweep slot is whichever slot was free first, not which laser
     * plane hit, so re-apply each plane's own calibration to the swapped
     * counts. Correcting only the B offset (diff = 2*(B0 - B1) - diff) leaves
     * an (A0 - A1) * lfsr error of ~1° horizontal / 6-10° vertical.
     */
    if (fabsf(a0 - a1) > 90.0f) {
        a0 = cal->A0 * (float)slot->raw_lfsr[1] + cal->B0;
        a1 = cal->A1 * (float)slot->raw_lfsr[0] + cal->B1;
    }

    /* Base-station factory calibration (lh2_factory_cal.h), on the plane angles. */
    float beam0 = a0 * ((float)M_PI / 180.0f);
    float beam1 = a1 * ((float)M_PI / 180.0f);
    _apply_factory_cal(cal, &beam0, &beam1);
    a0 = beam0 * (180.0f / (float)M_PI);
    a1 = beam1 * (180.0f / (float)M_PI);

    float az_raw = (a0 + a1) * 0.5f;
    float diff   = a0 - a1;

    float diff_rad = (diff * 0.5f) * ((float)M_PI / 180.0f);
    float az_rad   = az_raw              * ((float)M_PI / 180.0f);

    /* elevation = atan( tan(diff/2) / TAN_30 / cos(az) )  [Python reference] */
    float y_proj  = tanf(diff_rad) / TAN_30 / cosf(az_rad);
    float el_raw  = atanf(y_proj) * (180.0f / (float)M_PI);

    /* Bitcraze (horiz, vert) reconstruction [radians] for the calibrated-pose solver.
     * Uses the swap-corrected sweep angles (s0c, s1c) derived from az_raw + diff,
     * matching LighthouseBsVector.from_lh2():
     *   horiz = (s0 + s1) / 2
     *   vert  = atan2( sin(s1 - s0), tan(30°) * (cos s0 + cos s1) )
     */
    float s0c_rad = (az_raw + diff * 0.5f) * ((float)M_PI / 180.0f);  /* sweep 0 */
    float s1c_rad = (az_raw - diff * 0.5f) * ((float)M_PI / 180.0f);  /* sweep 1 */
    float horiz_rad = az_rad;
    float vert_rad  = atan2f(sinf(s1c_rad - s0c_rad),
                             TAN_30 * (cosf(s0c_rad) + cosf(s1c_rad)));

    slot->raw_horiz = horiz_rad;
    slot->raw_vert  = vert_rad;

    /* EMA update */
    if (!slot->valid) {
        slot->ema_az    = az_raw;
        slot->ema_el    = el_raw;
        slot->ema_horiz = horiz_rad;
        slot->ema_vert  = vert_rad;
    } else {
        slot->ema_az    = EMA_ALPHA * az_raw    + (1.0f - EMA_ALPHA) * slot->ema_az;
        slot->ema_el    = EMA_ALPHA * el_raw    + (1.0f - EMA_ALPHA) * slot->ema_el;
        slot->ema_horiz = EMA_ALPHA * horiz_rad + (1.0f - EMA_ALPHA) * slot->ema_horiz;
        slot->ema_vert  = EMA_ALPHA * vert_rad  + (1.0f - EMA_ALPHA) * slot->ema_vert;
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
            out[s][b].raw_lfsr[0]  = 0u;
            out[s][b].raw_lfsr[1]  = 0u;
            out[s][b].has_sweep[0] = false;
            out[s][b].has_sweep[1] = false;
            out[s][b].ema_az       = 0.0f;
            out[s][b].ema_el       = 0.0f;
            out[s][b].ema_horiz    = 0.0f;
            out[s][b].ema_vert     = 0.0f;
            out[s][b].raw_horiz    = 0.0f;
            out[s][b].raw_vert     = 0.0f;
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
                /* Only consume slots db_lh2_process_location() has decoded */
                if (lh2[s].data_ready[sweep][slot] != DB_LH2_PROCESSED_DATA_AVAILABLE) {
                    continue;
                }

                uint8_t  poly = lh2[s].locations[sweep][slot].selected_polynomial;
                uint32_t lfsr = lh2[s].locations[sweep][slot].lfsr_location;
                int bs_idx    = _poly_to_bs(poly);
                uint8_t physical_bs = poly >> 1;

                /* Mark slot consumed regardless of whether we use the data */
                lh2[s].data_ready[sweep][slot] = DB_LH2_NO_NEW_DATA;

                if ((physical_bs == 0 || physical_bs == 1) &&
                    (s_last_lfsr_log_us[s][slot][sweep] == 0 ||
                     now_us - s_last_lfsr_log_us[s][slot][sweep] >= LFSR_LOG_INTERVAL_US)) {
                    printf("L,%d,%u,%d,%u,%lu,%llu\n", s, physical_bs, sweep, poly,
                           (unsigned long)lfsr, (unsigned long long)now_us);
                    s_last_lfsr_log_us[s][slot][sweep] = now_us;
                }

                if (bs_idx < 0) {
                    continue;  /* unknown polynomial — skip */
                }

                if (s_ootx_stream) {
                    printf("O,%d,%d,%u,%lu,%llu\n", s, bs_idx, poly, (unsigned long)lfsr,
                           (unsigned long long)to_us_since_boot(lh2[s].timestamps[sweep][slot]));
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
                ang->raw_lfsr[sweep]  = lfsr;
                ang->has_sweep[sweep] = true;

                /* If both sweeps are now in, compute az/el */
                if (ang->has_sweep[0] && ang->has_sweep[1]) {
                    _finalize_angles(ang, c, now_us);
                }
            }
        }
    }
}

void angle_decoder_set_ootx_stream(bool enabled)
{
    s_ootx_stream = enabled;
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
