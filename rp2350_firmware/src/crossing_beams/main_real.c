/**
 * @file   main_real.c
 * @brief  LH2 3D Positioning — crossing-beams solver on RP2350, dual-core.
 *
 * Core split (producer / consumer):
 *
 *   Core 1  (capture)  — owns ALL sensor I/O. Initialises all 4 TS4231
 *                        sensors (so every PIO IRQ is routed to core 1) and
 *                        loops db_lh2_process_location() for all 4, filling
 *                        g_lh2[].locations with decoded LFSR positions.
 *
 *   Core 0  (compute)  — reads g_lh2[], decodes angles, runs solve3d, emits
 *                        VISION_POSITION_ESTIMATE over UART0 to the Pixhawk,
 *                        and prints USB diagnostics.
 *
 * Why all init must live on core 1: db_lh2_init() calls irq_set_enabled(),
 * which arms the PIO IRQ on whichever core executes it. Putting all four
 * inits in core1_entry() guarantees every capture IRQ fires on core 1.
 *
 * Pipeline:
 *   lh2/             — PIO+DMA capture (TS4231 → LFSR counts)        [core 1]
 *   angle_decoder/   — LFSR counts → EMA-filtered az/el             [core 0]
 *   cv/ + solve3d/   — fundamental matrix / triangulation           [core 0]
 *   mavlink/         — VISION_POSITION_ESTIMATE over UART0          [core 0]
 *
 * Serial output (USB, 115200):
 *   A,<sensor>,<bs>,<az_deg>,<el_deg>   angle update
 *   P,<sensor>,<x>,<y>,<z>              3D point
 *   C,<n>,<cx>,<cy>,<cz>                centroid (also sent as VPE)
 *
 * Hardware pins (data, env):
 *   S0 10/11   S1 12/13   S2 18/19   S3 20/21
 *   MAVLink UART0 TX/RX on GPIO 0/1.
 *
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
#include "mavlink/mavlink.h"

// ---------------------------------------------------------------------------
// GPIO pin assignments
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

/** Period between solve + VPE-send + print calls [µs] — ~10 Hz */
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
// Shared state (core 1 writes g_lh2, core 0 reads it)
// ---------------------------------------------------------------------------

static db_lh2_t      g_lh2[NUM_SENSORS];
static lh2_angles_t  g_angles[NUM_SENSORS][NUM_BS];
static solve3d_ctx_t g_solver;

/** Set true by core 1 once all sensors are initialised; gates core 0's loop. */
static volatile bool g_capture_ready = false;

// ---------------------------------------------------------------------------
// Core 1 — all sensor capture
//
// Two builds of this function exist:
//   - default            : real PIO/DMA capture from 4 TS4231 sensors
//   - -DSYNTHETIC_CAPTURE : no hardware; core 1 fabricates the exact LFSR
//                           counts a real sensor would emit for a body
//                           tracing a 1×1 m square, so the whole dual-core
//                           pipeline can be exercised without sensors.
//                           See SYNTHETIC_CAPTURE.md.
// ---------------------------------------------------------------------------

#ifdef SYNTHETIC_CAPTURE

#define SYN_PI       3.14159265358979323846f
#define SYN_SEG_US   8000000ULL    /* 8 s per square edge          */
#define SYN_TICK_US  20000ULL      /* inject a fresh set every 20 ms (~LH2 rate) */

/* Base-station slot indices in the lh2 array (basestation = polynomial >> 1). */
#define SYN_SLOT_BS0 4u            /* poly 8/9  → slot 4  */
#define SYN_SLOT_BS1 10u           /* poly 20/21 → slot 10 */
#define SYN_POLY_BS0 8u
#define SYN_POLY_BS1 20u

/** Square corners (world frame, z-up), 1×1 m at z = 2 m. */
static const float SYN_CORNERS[4][3] = {
    {0.00f, 0.00f, 2.00f}, {1.00f, 0.00f, 2.00f},
    {1.00f, 1.00f, 2.00f}, {0.00f, 1.00f, 2.00f},
};

/** Sensor offsets on the rigid body [x, y] metres (flat 5×5 cm square). */
static const float SYN_SENSOR_OFF[NUM_SENSORS][2] = {
    {0.000f, 0.000f}, {0.050f, 0.000f}, {0.050f, 0.050f}, {0.000f, 0.050f},
};

/**
 * @brief  Fabricate both sweep slots for one (sensor, base-station) pair.
 *
 * Inverts angle_decoder's az/el reconstruction + linear calibration so that
 * when core 0 decodes these LFSR counts it recovers exactly (az_deg, el_deg):
 *
 *   a0 = az + diff/2 ,  a1 = az - diff/2
 *   diff/2 = atan( tan(el) * TAN_30 * cos(az) )          [decoder's el formula]
 *   lfsr_sweep = (angle_deg - B_sweep) / A_sweep         [inverse calibration]
 */
static void syn_write_bs(int s, uint8_t slot, uint8_t poly,
                         float A0, float B0, float A1, float B1,
                         float az_deg, float el_deg)
{
    float az  = az_deg * (SYN_PI / 180.0f);
    float el  = el_deg * (SYN_PI / 180.0f);
    float half_diff_rad = atanf(tanf(el) * TAN_30 * cosf(az));
    float diff_deg      = 2.0f * half_diff_rad * (180.0f / SYN_PI);

    float a0 = az_deg + diff_deg * 0.5f;   /* sweep 0 angle [deg] */
    float a1 = az_deg - diff_deg * 0.5f;   /* sweep 1 angle [deg] */

    uint32_t lfsr0 = (uint32_t)lroundf((a0 - B0) / A0);
    uint32_t lfsr1 = (uint32_t)lroundf((a1 - B1) / A1);

    g_lh2[s].locations[0][slot].selected_polynomial = poly;
    g_lh2[s].locations[0][slot].lfsr_location        = lfsr0;
    g_lh2[s].data_ready[0][slot]                     = DB_LH2_PROCESSED_DATA_AVAILABLE;

    g_lh2[s].locations[1][slot].selected_polynomial = poly;
    g_lh2[s].locations[1][slot].lfsr_location        = lfsr1;
    g_lh2[s].data_ready[1][slot]                     = DB_LH2_PROCESSED_DATA_AVAILABLE;
}

static void core1_entry(void)
{
    /* No db_lh2_init() — no hardware touched in synthetic mode. */
    g_capture_ready = true;

    uint64_t start_us = to_us_since_boot(get_absolute_time());
    uint64_t next_us  = start_us;

    while (true) {
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        if (now_us < next_us) {
            tight_loop_contents();
            continue;
        }
        next_us += SYN_TICK_US;

        /* Interpolate body position along the current square edge. */
        uint64_t elapsed = now_us - start_us;
        int   seg = (int)((elapsed / SYN_SEG_US) % 4);
        int   nxt = (seg + 1) % 4;
        float t   = (float)(elapsed % SYN_SEG_US) / (float)SYN_SEG_US;
        float bx  = SYN_CORNERS[seg][0] + t * (SYN_CORNERS[nxt][0] - SYN_CORNERS[seg][0]);
        float by  = SYN_CORNERS[seg][1] + t * (SYN_CORNERS[nxt][1] - SYN_CORNERS[seg][1]);
        float bz  = 2.00f;

        for (int s = 0; s < NUM_SENSORS; s++) {
            float wx = bx + SYN_SENSOR_OFF[s][0];
            float wy = by + SYN_SENSOR_OFF[s][1];
            float wz = bz;

            /* BS0 at world origin, facing +Z. */
            float az0 = atan2f(wx, wz)                       * (180.0f / SYN_PI);
            float el0 = atan2f(wy, sqrtf(wx * wx + wz * wz)) * (180.0f / SYN_PI);

            /* BS1 at (1, 0, 0), facing +Z. */
            float bsx = wx - 1.0f;
            float az1 = atan2f(bsx, wz)                        * (180.0f / SYN_PI);
            float el1 = atan2f(wy, sqrtf(bsx * bsx + wz * wz)) * (180.0f / SYN_PI);

            syn_write_bs(s, SYN_SLOT_BS0, SYN_POLY_BS0,
                         CAL_BS0_A0, CAL_BS0_B0, CAL_BS0_A1, CAL_BS0_B1, az0, el0);
            syn_write_bs(s, SYN_SLOT_BS1, SYN_POLY_BS1,
                         CAL_BS1_A0, CAL_BS1_B0, CAL_BS1_A1, CAL_BS1_B1, az1, el1);
        }
    }
}

#else  /* real hardware capture */

static void core1_entry(void)
{
    /* All four inits run here so every PIO IRQ is armed on core 1. */
    db_lh2_init(&g_lh2[0], 0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&g_lh2[1], 1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);
    db_lh2_init(&g_lh2[2], 2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&g_lh2[3], 3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);

    g_capture_ready = true;

    while (true) {
        for (int s = 0; s < NUM_SENSORS; s++) {
            db_lh2_process_location(&g_lh2[s]);
        }
    }
}

#endif /* SYNTHETIC_CAPTURE */

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline float _deg2rad(float deg)
{
    return deg * (3.14159265358979323846f / 180.0f);
}

/**
 * @brief  Print per-sensor 3D points + centroid, and return the centroid.
 *
 * @param pts   array of @p n solved points
 * @param n     number of points
 * @param cx,cy,cz  out: centroid (averaged over per-sensor means)
 * @return      number of active sensors contributing to the centroid (0 = none)
 */
static int _print_and_centroid(const lh2_point3d_t *pts, int n,
                               float *cx, float *cy, float *cz)
{
    if (n <= 0) return 0;

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

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int   n_active = 0;

    for (int s = 0; s < NUM_SENSORS; s++) {
        if (sensor_cnt[s] == 0) continue;
        float x = sensor_x[s] / sensor_cnt[s];
        float y = sensor_y[s] / sensor_cnt[s];
        float z = sensor_z[s] / sensor_cnt[s];
        printf("P,%d,%.4f,%.4f,%.4f\n", s, (double)x, (double)y, (double)z);
        sum_x += x; sum_y += y; sum_z += z;
        n_active++;
    }

    if (n_active == 0) return 0;

    *cx = sum_x / n_active;
    *cy = sum_y / n_active;
    *cz = sum_z / n_active;
    printf("C,%d,%.4f,%.4f,%.4f\n", n_active,
           (double)*cx, (double)*cy, (double)*cz);
    return n_active;
}

// ---------------------------------------------------------------------------
// Core 0 — main()
// ---------------------------------------------------------------------------

int main(void)
{
    /* ① Clock + peripherals */
    set_sys_clock_khz(128000, true);
    stdio_init_all();
    mavlink_init();          /* UART0 GPIO 0/1 @ 115200 → Pixhawk TELEM2 */
    sleep_ms(2000);          /* let USB enumerate */
    printf("=== LH2 Crossing-Beams 3D Solver (dual-core) ===\n");
#ifdef SYNTHETIC_CAPTURE
    printf("Core 1: SYNTHETIC capture (no sensors, 1x1m square)\n");
#else
    printf("Core 1: capture (4 sensors)  Core 0: solve + MAVLink VPE\n");
#endif

    /* ② Compute-side init (no sensor I/O here — that lives on core 1) */
    angle_decoder_init(g_angles, CAL);
    solve3d_init(&g_solver);

    /* ③ Launch capture core and wait for it to finish initialising sensors */
    multicore_launch_core1(core1_entry);
    while (!g_capture_ready) {
        tight_loop_contents();
    }
    printf("Capture core ready.\n");

    /* ④ Compute loop */
    uint64_t last_print_us = 0;

    while (true) {
        uint64_t now_us = to_us_since_boot(get_absolute_time());

        /* Decode whatever core 1 has produced since last pass */
        angle_decoder_update(g_lh2, g_angles, CAL, now_us);

        if (now_us - last_print_us < PRINT_INTERVAL_US) {
            continue;
        }
        last_print_us = now_us;

        /* Push fresh samples into the solver ring buffer */
        for (int s = 0; s < NUM_SENSORS; s++) {
            if (!angle_decoder_is_fresh(g_angles, s, now_us)) continue;

            lh2_sample_t smp;
            smp.sensor_id = (uint8_t)s;

            float az_a = _deg2rad(g_angles[s][0].ema_az);
            float el_a = _deg2rad(g_angles[s][0].ema_el);
            float az_b = _deg2rad(g_angles[s][1].ema_az);
            float el_b = _deg2rad(g_angles[s][1].ema_el);

            angles_to_pixels(az_a, el_a, smp.px_a);
            angles_to_pixels(az_b, el_b, smp.px_b);
            solve3d_push_sample(&g_solver, &smp);

            printf("A,%d,0,%.2f,%.2f\n", s,
                   (double)g_angles[s][0].ema_az,
                   (double)g_angles[s][0].ema_el);
            printf("A,%d,1,%.2f,%.2f\n", s,
                   (double)g_angles[s][1].ema_az,
                   (double)g_angles[s][1].ema_el);
        }

        /* Solve + emit if we have enough history */
        if (g_solver.n_samples < SOLVE3D_MIN_SAMPLES) {
            continue;
        }

        lh2_point3d_t pts[SOLVE3D_MAX_SAMPLES];
        int n = solve3d_run(&g_solver, pts);

        float cx, cy, cz;
        if (_print_and_centroid(pts, n, &cx, &cy, &cz) > 0) {
            /*
             * World → NED: negate Z (solver z-up → MAVLink z-down).
             * NOTE: solve3d output is scale-ambiguous (see solve3d.h) until the
             * basestation baseline distance D_BS is applied — VPE values are
             * not yet metric. Calibrate scale here before relying on position.
             */
            mavlink_send_vpe(now_us, cx, cy, -cz);
        }
    }

    return 0;   /* unreachable */
}
