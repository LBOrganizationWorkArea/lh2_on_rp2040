/**
 * @file   main_test.c
 * @brief  solve3d math self-test — no hardware required
 *
 * Feeds a synthetic virtual scene through angles_to_pixels + solve3d_run,
 * then prints 3D points as CSV over USB serial.
 *
 * Virtual scene (identical values used in validate_solve3d.py):
 *   4 sensors at corners of a 5 cm square (coplanar, flat Z=0 in body frame)
 *   8 body poses sweeping through the scene  (32 samples total → fills ring buffer)
 *   BS0 at world origin (0,0,0) → +Z
 *   BS1 at             (1,0,0) → +Z   (1 m baseline)
 *
 * No PIO, no DMA, no angle_decoder, no multicore.
 * Only cv/cv.c and solve3d/solve3d.c are compiled.
 *
 * Companion scripts (run on PC after capturing USB serial):
 *   validate_solve3d.py  — same math in Python/OpenCV → stdout CSV
 *   compare_results.py   — diff the two CSVs, print PASS/FAIL
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"

#include "solve3d/solve3d.h"

// ---------------------------------------------------------------------------
// Virtual scene definition
// (Keep in sync with BODY_POS / SENSOR_BODY in validate_solve3d.py)
// ---------------------------------------------------------------------------

/** 8 body positions in world frame [metres].
 *  Y and Z vary across poses to keep 2D image correspondences off a single line. */
static const float BODY_POS[8][3] = {
    /*  X       Y       Z   */
    {-0.15f,  0.00f, 1.90f},
    {-0.10f,  0.04f, 1.95f},
    {-0.05f,  0.00f, 2.00f},
    { 0.00f, -0.04f, 2.05f},
    { 0.05f,  0.00f, 2.10f},
    { 0.10f,  0.04f, 2.05f},
    { 0.15f,  0.00f, 2.00f},
    { 0.20f, -0.04f, 1.95f},
};

/** Sensor positions in body frame [metres].  Z_body = 0 for all (flat 5×5 cm square).
 *
 *   S3 (0,5) ─── S2 (5,5)
 *    │               │
 *   S0 (0,0) ─── S1 (5,0)   [cm]
 */
static const float SENSOR_BODY[4][2] = {
    {0.000f, 0.000f},   /* S0: bottom-left  */
    {0.050f, 0.000f},   /* S1: bottom-right */
    {0.050f, 0.050f},   /* S2: top-right    */
    {0.000f, 0.050f},   /* S3: top-left     */
};

#define N_POSES    8
#define N_SENSORS  4
#define N_SAMPLES  (N_POSES * N_SENSORS)   /* 32 — fills the ring buffer exactly */

// ---------------------------------------------------------------------------
// Print helper
// ---------------------------------------------------------------------------

static void print_results(int n, const lh2_point3d_t *pts)
{
    printf("=== LH2 SOLVE3D SELF-TEST ===\n");
    printf("Samples pushed: %d\n", N_SAMPLES);
    printf("Points returned: %d\n\n", n);
    printf("i,sensor_id,x,y,z\n");
    for (int i = 0; i < n; i++) {
        printf("%d,%u,%.6f,%.6f,%.6f\n",
               i,
               (unsigned)pts[i].sensor_id,
               (double)pts[i].xyz[0],
               (double)pts[i].xyz[1],
               (double)pts[i].xyz[2]);
    }
    printf("=== END (reprinting in 5 s) ===\n\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(void)
{
    stdio_init_all();
    sleep_ms(3000);   /* wait for USB host to enumerate */

    /* ---- Build the 32-sample set ---------------------------------------- */

    solve3d_ctx_t ctx;
    solve3d_init(&ctx);

    for (int k = 0; k < N_POSES; k++) {
        for (int s = 0; s < N_SENSORS; s++) {

            /* World position of sensor s at body pose k.
             * Body orientation is identity (no rotation) for simplicity. */
            float wx = BODY_POS[k][0] + SENSOR_BODY[s][0];
            float wy = BODY_POS[k][1] + SENSOR_BODY[s][1];
            float wz = BODY_POS[k][2];   /* sensors are flat: Z_body=0 */

            /* BS0 at (0,0,0) → +Z: camera frame == world frame */
            float az_a = atan2f(wx, wz);
            float el_a = atan2f(wy, sqrtf(wx * wx + wz * wz));

            /* BS1 at (1,0,0) → +Z: translate sensor position by (-1,0,0) */
            float bx   = wx - 1.0f;
            float az_b = atan2f(bx, wz);
            float el_b = atan2f(wy, sqrtf(bx * bx + wz * wz));

            lh2_sample_t sample;
            angles_to_pixels(az_a, el_a, sample.px_a);
            angles_to_pixels(az_b, el_b, sample.px_b);
            sample.sensor_id = (uint8_t)s;
            solve3d_push_sample(&ctx, &sample);
        }
    }

    /* ---- Run the solver -------------------------------------------------- */

    lh2_point3d_t pts3d[SOLVE3D_MAX_SAMPLES];
    int n = solve3d_run(&ctx, pts3d);

    if (n == 0) {
        printf("=== LH2 SOLVE3D SELF-TEST ===\n");
        printf("FAIL: solve3d_run returned 0\n");
        printf("      (degenerate scene or too few samples)\n");
        /* Halt so the failure is visible */
        while (1) tight_loop_contents();
    }

    /* ---- Loop, reprinting every 5 s so late USB connections don't miss it */

    while (1) {
        print_results(n, pts3d);
        sleep_ms(5000);
    }

    return 0;
}
