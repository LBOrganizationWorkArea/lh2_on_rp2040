/**
 * @file   main.c
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
 *                        ODOMETRY over UART0 to the Pixhawk, and prints USB
 *                        diagnostics.
 *
 * Base-station poses are NOT hardcoded.  At boot, core 0 fetches the
 * LH2_BS* parameters from the FC via MAVLink PARAM_REQUEST_LIST and waits
 * until all 25 params are received before starting the solve loop.
 * The FC must have lh2_bs_params.lua running with the current room geometry.
 *
 * Pipeline:
 *   lh2/             — PIO+DMA capture (TS4231 → LFSR counts)        [core 1]
 *   angle_decoder/   — LFSR counts → EMA-filtered az/el             [core 0]
 *   cv/ + solve3d/   — fundamental matrix / triangulation           [core 0]
 *   mavlink/         — ODOMETRY (msg #331) over UART0               [core 0]
 *
 * Serial output (USB, 115200):
 *   A,<sensor>,<bs>,<az_deg>,<el_deg>   angle update
 *   P,<sensor>,<x>,<y>,<z>              3D point
 *   C,<n>,<cx>,<cy>,<cz>                centroid (also sent as ODOMETRY)
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

/** Diagnostic print interval [µs] — 10 Hz */
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
// Base-station poses
//
// Real hardware: populated at runtime from FC MAVLink params (LH2_BS*).
// Synthetic:     hardcoded so the synthetic build needs no FC.
// ---------------------------------------------------------------------------

static lh2_bs_pose_t BS_POSES[NUM_BS];   /* filled from FC params at boot */

// ---------------------------------------------------------------------------
// Shared state (core 1 writes g_lh2, core 0 reads it)
// ---------------------------------------------------------------------------

static db_lh2_t      g_lh2[NUM_SENSORS];
static lh2_angles_t  g_angles[NUM_SENSORS][NUM_BS];

/** Set true by core 1 once all sensors are initialised; gates core 0's loop. */
static volatile bool g_capture_ready = false;

/**
 * Set true by core 0 when the FC first reports a healthy EKF and DO_SET_HOME
 * has been sent.  Read by core 1 (synthetic only) to restart the square walk
 * from corner 0 so the mission begins from a known home.
 */
static volatile bool g_home_set = false;

// ---------------------------------------------------------------------------
// Core 1 — all sensor capture
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

static void syn_world_to_angles(const lh2_bs_pose_t *bs, const float w[3],
                                float *horiz, float *vert)
{
    float rel[3] = { w[0] - bs->origin[0], w[1] - bs->origin[1], w[2] - bs->origin[2] };
    float xl = bs->R[0][0]*rel[0] + bs->R[1][0]*rel[1] + bs->R[2][0]*rel[2];
    float yl = bs->R[0][1]*rel[0] + bs->R[1][1]*rel[1] + bs->R[2][1]*rel[2];
    float zl = bs->R[0][2]*rel[0] + bs->R[1][2]*rel[1] + bs->R[2][2]*rel[2];
    *horiz = atan2f(yl, xl);
    *vert  = atan2f(zl, xl);
}

static void syn_write_bs(int s, uint8_t slot, uint8_t poly,
                         float A0, float B0, float A1, float B1,
                         float horiz, float vert)
{
    float th = tanf(horiz);
    float tv = tanf(vert);
    float q  = tv / sqrtf(1.0f + th * th);
    float dT = asinf(q * TAN_30);

    float s0_deg = (horiz - dT) * (180.0f / SYN_PI);
    float s1_deg = (horiz + dT) * (180.0f / SYN_PI);

    uint32_t lfsr0 = (uint32_t)lroundf((s0_deg - B0) / A0);
    uint32_t lfsr1 = (uint32_t)lroundf((s1_deg - B1) / A1);

    g_lh2[s].locations[0][slot].selected_polynomial = poly;
    g_lh2[s].locations[0][slot].lfsr_location        = lfsr0;
    g_lh2[s].data_ready[0][slot]                     = DB_LH2_PROCESSED_DATA_AVAILABLE;

    g_lh2[s].locations[1][slot].selected_polynomial = poly;
    g_lh2[s].locations[1][slot].lfsr_location        = lfsr1;
    g_lh2[s].data_ready[1][slot]                     = DB_LH2_PROCESSED_DATA_AVAILABLE;
}

static void core1_entry(void)
{
    g_capture_ready = true;

    uint64_t start_us   = to_us_since_boot(get_absolute_time());
    uint64_t next_us    = start_us;
    bool     home_latch = false;

    while (true) {
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        if (now_us < next_us) { tight_loop_contents(); continue; }
        next_us += SYN_TICK_US;

        if (g_home_set && !home_latch) {
            start_us   = now_us;
            home_latch = true;
        }

        uint64_t elapsed = now_us - start_us;
        int   seg = (int)((elapsed / SYN_SEG_US) % 4);
        int   nxt = (seg + 1) % 4;
        float t   = (float)(elapsed % SYN_SEG_US) / (float)SYN_SEG_US;
        float bx  = SYN_CORNERS[seg][0] + t * (SYN_CORNERS[nxt][0] - SYN_CORNERS[seg][0]);
        float by  = SYN_CORNERS[seg][1] + t * (SYN_CORNERS[nxt][1] - SYN_CORNERS[seg][1]);
        float bz  = 2.00f;

        for (int s = 0; s < NUM_SENSORS; s++) {
            float w[3] = { bx + SYN_SENSOR_OFF[s][0],
                           by + SYN_SENSOR_OFF[s][1],
                           bz };
            float h0, v0, h1, v1;
            syn_world_to_angles(&BS_POSES[0], w, &h0, &v0);
            syn_world_to_angles(&BS_POSES[1], w, &h1, &v1);
            syn_write_bs(s, SYN_SLOT_BS0, SYN_POLY_BS0,
                         CAL_BS0_A0, CAL_BS0_B0, CAL_BS0_A1, CAL_BS0_B1, h0, v0);
            syn_write_bs(s, SYN_SLOT_BS1, SYN_POLY_BS1,
                         CAL_BS1_A0, CAL_BS1_B0, CAL_BS1_A1, CAL_BS1_B1, h1, v1);
        }
    }
}

#else  /* real hardware capture */

static void core1_entry(void)
{
    db_lh2_init(&g_lh2[0], 0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&g_lh2[1], 1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);
    db_lh2_init(&g_lh2[2], 2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&g_lh2[3], 3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);
    g_capture_ready = true;
    while (true) {
        for (int s = 0; s < NUM_SENSORS; s++)
            db_lh2_process_location(&g_lh2[s]);
    }
}

#endif /* SYNTHETIC_CAPTURE */

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/* Position variance [m²] — GDOP model grounded in Taffanel et al. ICRA 2021
 * (LH2 crossing-beam, Table III / Fig. 6).
 * σ_base = 2 cm (paper: ~1 cm flight avg, inflated for conservatism).
 * GDOP factors × {∞, 5, 3, 1.5, 1} for 0–4 active sensors.
 * 4 sensors: DLT has 16 eqs / 3 unknowns → GDOP ≈ 1.
 * 1 sensor:  DLT has  4 eqs / 3 unknowns → no redundancy → GDOP × 5. */
static const float POS_VAR_TABLE[5] = {
    0.09f,    /* 0 sensors — σ = 30 cm (no data) */
    0.0100f,  /* 1 sensor  — σ = 10 cm (GDOP × 5) */
    0.0036f,  /* 2 sensors — σ =  6 cm (GDOP × 3) */
    0.0009f,  /* 3 sensors — σ =  3 cm (GDOP × 1.5) */
    0.0004f,  /* 4 sensors — σ =  2 cm (GDOP × 1, baseline) */
};

static inline float _rad2deg(float rad)
{
    return rad * (180.0f / 3.14159265358979323846f);
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

#ifdef SYNTHETIC_CAPTURE
    printf("=== LH2 Crossing-Beams 3D Solver (dual-core, SYNTHETIC) ===\n");
#else
    printf("=== LH2 Crossing-Beams 3D Solver (dual-core) ===\n");
#endif

    /* ② Fetch BS poses from FC via MAVLink param protocol.
     *    Blocks until all 25 LH2_BS* params are received.
     *    The FC must be running lh2_bs_params.lua. */
    printf("Waiting for BS poses from FC (lh2_bs_params.lua)...\n");
    {
        uint64_t last_hb   = 0;
        uint64_t last_diag = 0;
        while (!mavlink_bs_poses_ready()) {
            uint64_t now = to_us_since_boot(get_absolute_time());
            if (now - last_hb > 1000000ULL) {
                mavlink_send_heartbeat();
                mavlink_request_ekf_stream();
                last_hb = now;
            }
            if (now - last_diag > 3000000ULL) {
                printf("  lh2=%lu/25  named_seen=%lu  rx_bytes=%lu\n",
                       (unsigned long)mavlink_lh2_params_received(),
                       (unsigned long)mavlink_param_val_seen(),
                       (unsigned long)mavlink_rx_bytes());
                last_diag = now;
            }
            mavlink_rx_update();
            sleep_ms(1);
        }
    }
    mavlink_get_bs_poses(BS_POSES);
    for (int i = 0; i < NUM_BS; i++) {
        printf("BS%d origin=(%.3f, %.3f, %.3f)  R[0]=(%.2f,%.2f,%.2f)\n", i,
               (double)BS_POSES[i].origin[0],
               (double)BS_POSES[i].origin[1],
               (double)BS_POSES[i].origin[2],
               (double)BS_POSES[i].R[0][0],
               (double)BS_POSES[i].R[0][1],
               (double)BS_POSES[i].R[0][2]);
    }

    printf("Legend: A,<sensor>,<bs>,h,v [deg]; P,<sensor>,x,y,z [m]; C,<n>,cx,cy,cz [m]\n");

    /* ③ Compute-side init */
    angle_decoder_init(g_angles, CAL);

    /* ④ Launch capture core and wait */
    multicore_launch_core1(core1_entry);
    while (!g_capture_ready) { tight_loop_contents(); }
    printf("Capture core ready.\n");

    /* ⑤ Compute loop */
    uint64_t last_print_us = 0;
    float    last_cx = 0.0f, last_cy = 0.0f, last_cz = 0.0f;
    int      last_na = 0;   /* number of fresh sensors in last solve */

    while (true) {
        uint64_t now_us = to_us_since_boot(get_absolute_time());

        mavlink_rx_update();

        angle_decoder_update(g_lh2, g_angles, CAL, now_us);

        /* Solve + cache centroid whenever fresh data exists */
        {
            lh2_point3d_t pts[NUM_SENSORS];
            int n = solve3d_calib_run(BS_POSES, g_angles, now_us, pts);

            if (n > 0) {
                float sx = 0.0f, sy = 0.0f, sz = 0.0f;
                int   na = 0;
                float acc_x[NUM_SENSORS]={0}, acc_y[NUM_SENSORS]={0}, acc_z[NUM_SENSORS]={0};
                int   cnt[NUM_SENSORS]={0};
                for (int i = 0; i < n; i++) {
                    int s = pts[i].sensor_id;
                    if (s >= 0 && s < NUM_SENSORS) {
                        acc_x[s] += pts[i].xyz[0];
                        acc_y[s] += pts[i].xyz[1];
                        acc_z[s] += pts[i].xyz[2];
                        cnt[s]++;
                    }
                }
                for (int s = 0; s < NUM_SENSORS; s++) {
                    if (!cnt[s]) continue;
                    sx += acc_x[s] / cnt[s];
                    sy += acc_y[s] / cnt[s];
                    sz += acc_z[s] / cnt[s];
                    na++;
                }
                if (na > 0) {
                    last_cx = sx / na;
                    last_cy = sy / na;
                    last_cz = sz / na;
                    last_na = na;
                }
            }
        }

        /* 10 Hz: odometry + EKF home-set + diagnostics */
        if (now_us - last_print_us < PRINT_INTERVAL_US) continue;
        last_print_us = now_us;

        /* Refresh poses from the Lua-script stream so UI changes take effect. */
        mavlink_get_bs_poses(BS_POSES);

        if (last_cx != 0.0f || last_cy != 0.0f || last_cz != 0.0f) {
            int     idx     = last_na < 4 ? last_na : 4;
            float   pos_var = POS_VAR_TABLE[idx];
            uint8_t quality = (uint8_t)(last_na * 25);
            mavlink_send_odometry(mavlink_timesync_corrected_us(now_us),
                                  last_cx, last_cy, -last_cz,
                                  pos_var, quality);
        }

        if (!g_home_set && mavlink_is_ekf_healthy()) {
            mavlink_send_do_set_home();
            g_home_set = true;
            printf("EKF healthy — home set\n");
        }

        for (int s = 0; s < NUM_SENSORS; s++) {
            if (!angle_decoder_is_fresh(g_angles, s, now_us)) continue;
            float h0 = _rad2deg(g_angles[s][0].ema_horiz);
            float v0 = _rad2deg(g_angles[s][0].ema_vert);
            float h1 = _rad2deg(g_angles[s][1].ema_horiz);
            float v1 = _rad2deg(g_angles[s][1].ema_vert);
            printf("A,%d,0,%.2f,%.2f\n", s, (double)h0, (double)v0);
            printf("A,%d,1,%.2f,%.2f\n", s, (double)h1, (double)v1);
            printf("ANG S%d | BS0 h=%+7.2f v=%+7.2f deg | BS1 h=%+7.2f v=%+7.2f deg\n",
                   s, (double)h0, (double)v0, (double)h1, (double)v1);
        }

        if (last_cx != 0.0f || last_cy != 0.0f || last_cz != 0.0f) {
            printf("C,%.4f,%.4f,%.4f\n",
                   (double)last_cx, (double)last_cy, (double)last_cz);
        }
    }

    return 0;
}
