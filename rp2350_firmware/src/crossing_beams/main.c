/**
 * @file   main.c
 * @brief  LH2 3D Positioning — crossing-beams solver on RP2350
 *
 * Implements the full pipeline from raw TS4231 pulses to 3D points using
 * the fundamental-matrix / epipolar geometry approach (solve_3d_scene).
 *
 * Architecture (all math lives in the sub-modules):
 *
 *   lh2/             — hardware PIO+DMA capture (TS4231 → LFSR counts)
 *   angle_decoder/   — LFSR counts → EMA-filtered az/el per (sensor, BS)
 *   cv/              — find_fundamental_mat, recover_pose, triangulate_points
 *   solve3d/         — ring buffer + full epipolar pipeline
 *
 * Serial output (115200 baud, USB):
 *   A,<sensor>,<bs>,<az_deg>,<el_deg>   angle update
 *   P,<sensor>,<x>,<y>,<z>             3D point
 *   C,<n>,<cx>,<cy>,<cz>               centroid
 *
 * Hardware:
 *   Sensors 0,1 on core 0  (data pins 10,12; env pins 11,13)
 *   Sensors 2,3 on core 1  (data pins 18,20; env pins 19,21)
 *
 * @date   2026
 * @copyright Inria, 2026
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/clocks.h"

#include "lh2/lh2.h"
#include "angle_decoder/angle_decoder.h"
#include "solve3d/solve3d.h"

// ---------------------------------------------------------------------------
// GPIO pin assignments (matching rp2350_firmware/src/main.c)
// ---------------------------------------------------------------------------

#define LH2_0_DATA_PIN  10
#define LH2_0_ENV_PIN   (LH2_0_DATA_PIN + 1)
#define LH2_1_DATA_PIN  12
#define LH2_1_ENV_PIN   (LH2_1_DATA_PIN + 1)
#define LH2_2_DATA_PIN  18
#define LH2_2_ENV_PIN   (LH2_2_DATA_PIN + 1)
#define LH2_3_DATA_PIN  20
#define LH2_3_ENV_PIN   (LH2_3_DATA_PIN + 1)

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

/** Period between solve + print calls [µs] — ~10 Hz */
#define PRINT_INTERVAL_US  100000ULL

// ---------------------------------------------------------------------------
// Calibration constants
// (from utils/user_interface/tools/history_calibration.txt, most recent)
// ---------------------------------------------------------------------------

/** BS index 0 → physical BS 4 — calibrated 2026-05-05 */
#define CAL_BS0_A0  0.00315641f
#define CAL_BS0_B0  (-121.7511f)
#define CAL_BS0_A1  0.00307607f
#define CAL_BS0_B1  (-234.6501f)

/** BS index 1 → physical BS 10 — calibrated 2026-05-04 */
#define CAL_BS1_A0  0.00327992f
#define CAL_BS1_B0  (-126.1425f)
#define CAL_BS1_A1  0.00317364f
#define CAL_BS1_B1  (-236.6446f)

static const lh2_cal_t CAL[NUM_BS] = {
    { .A0 = CAL_BS0_A0, .B0 = CAL_BS0_B0, .A1 = CAL_BS0_A1, .B1 = CAL_BS0_B1 },
    { .A0 = CAL_BS1_A0, .B0 = CAL_BS1_B0, .A1 = CAL_BS1_A1, .B1 = CAL_BS1_B1 },
};

// ---------------------------------------------------------------------------
// Globals (shared between cores via volatile access)
// ---------------------------------------------------------------------------

static db_lh2_t      g_lh2[NUM_SENSORS];
static lh2_angles_t  g_angles[NUM_SENSORS][NUM_BS];
static solve3d_ctx_t g_solver;

// ---------------------------------------------------------------------------
// Core 1 entry — runs sensors 2 & 3
// ---------------------------------------------------------------------------

static void core1_entry(void)
{
    db_lh2_init(&g_lh2[2], 2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&g_lh2[3], 3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);

    while (true) {
        db_lh2_process_location(&g_lh2[2]);
        db_lh2_process_location(&g_lh2[3]);
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * @brief  Convert degrees to radians.
 */
static inline float _deg2rad(float deg)
{
    return deg * (3.14159265358979323846f / 180.0f);
}

/**
 * @brief  Print one solve3d result over USB serial.
 */
static void _print_results(const lh2_point3d_t *pts, int n)
{
    if (n <= 0) return;

    /* Per-sensor average and centroid accumulator */
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int   n_active = 0;

    /* Bucket points by sensor */
    float sensor_x[NUM_SENSORS] = {0};
    float sensor_y[NUM_SENSORS] = {0};
    float sensor_z[NUM_SENSORS] = {0};
    int   sensor_cnt[NUM_SENSORS] = {0};

    for (int i = 0; i < n; i++) {
        int s = pts[i].sensor_id;
        if (s >= 0 && s < NUM_SENSORS) {
            sensor_x[s] += pts[i].xyz[0];
            sensor_y[s] += pts[i].xyz[1];
            sensor_z[s] += pts[i].xyz[2];
            sensor_cnt[s]++;
        }
    }

    for (int s = 0; s < NUM_SENSORS; s++) {
        if (sensor_cnt[s] == 0) continue;
        float x = sensor_x[s] / sensor_cnt[s];
        float y = sensor_y[s] / sensor_cnt[s];
        float z = sensor_z[s] / sensor_cnt[s];
        printf("P,%d,%.4f,%.4f,%.4f\n", s, (double)x, (double)y, (double)z);
        sum_x += x; sum_y += y; sum_z += z;
        n_active++;
    }

    if (n_active > 0) {
        printf("C,%d,%.4f,%.4f,%.4f\n",
               n_active,
               (double)(sum_x / n_active),
               (double)(sum_y / n_active),
               (double)(sum_z / n_active));
    }
}

// ---------------------------------------------------------------------------
// Core 0 — main()
// ---------------------------------------------------------------------------

int main(void)
{
    /* ① Clock + stdio */
    set_sys_clock_khz(128000, true);
    stdio_init_all();
    sleep_ms(2000);   /* let USB enumerate */
    printf("=== LH2 Crossing-Beams 3D Solver ===\n");

    /* ② Init sensors 0 & 1 on core 0 */
    db_lh2_init(&g_lh2[0], 0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&g_lh2[1], 1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);

    /* ③ Init angle decoder and solver */
    angle_decoder_init(g_angles, CAL);
    solve3d_init(&g_solver);

    /* ④ Launch core 1 (sensors 2 & 3) */
    multicore_launch_core1(core1_entry);

    /* ⑤ Main loop */
    uint64_t last_print_us = 0;

    while (true) {
        /* Keep LH2 processing running */
        db_lh2_process_location(&g_lh2[0]);
        db_lh2_process_location(&g_lh2[1]);

        /* Decode new LFSR counts → EMA angles */
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        angle_decoder_update(g_lh2, g_angles, CAL, now_us);

        /* Periodic solve + print (~10 Hz) */
        if (now_us - last_print_us >= PRINT_INTERVAL_US) {
            last_print_us = now_us;

            /* Push fresh samples into the ring buffer */
            for (int s = 0; s < NUM_SENSORS; s++) {
                if (!angle_decoder_is_fresh(g_angles, s, now_us)) continue;

                lh2_sample_t smp;
                smp.sensor_id = (uint8_t)s;

                /* BS 0 → px_a,  BS 1 → px_b */
                float az_a_rad = _deg2rad(g_angles[s][0].ema_az);
                float el_a_rad = _deg2rad(g_angles[s][0].ema_el);
                float az_b_rad = _deg2rad(g_angles[s][1].ema_az);
                float el_b_rad = _deg2rad(g_angles[s][1].ema_el);

                angles_to_pixels(az_a_rad, el_a_rad, smp.px_a);
                angles_to_pixels(az_b_rad, el_b_rad, smp.px_b);

                solve3d_push_sample(&g_solver, &smp);

                /* Diagnostic angle line */
                printf("A,%d,0,%.2f,%.2f\n",
                       s,
                       (double)g_angles[s][0].ema_az,
                       (double)g_angles[s][0].ema_el);
                printf("A,%d,1,%.2f,%.2f\n",
                       s,
                       (double)g_angles[s][1].ema_az,
                       (double)g_angles[s][1].ema_el);
            }

            /* Attempt to solve if we have enough history */
            if (g_solver.n_samples >= SOLVE3D_MIN_SAMPLES) {
                lh2_point3d_t pts[SOLVE3D_MAX_SAMPLES];
                int n = solve3d_run(&g_solver, pts);
                _print_results(pts, n);
            }
        }
    }

    /* unreachable */
    return 0;
}
