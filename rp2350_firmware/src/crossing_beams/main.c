/**
 * @file   main.c
 * @brief  MAVLink VPE square-path test — no LH2 hardware, no solver.
 *
 * Sends VISION_POSITION_ESTIMATE at 25 Hz over UART0 → Pixhawk 6C TELEM2.
 * Position traces a 1 m × 1 m square at z = 2 m (sent as z = −2 m NED).
 * Linear interpolation, 8 s per edge, full lap = 32 s.
 *
 *   seg 0:  (0,0) ──►  (1,0)
 *   seg 1:  (1,0) ──►  (1,1)
 *   seg 2:  (1,1) ──►  (0,1)
 *   seg 3:  (0,1) ──►  (0,0)
 *
 * Coordinate convention:
 *   World frame  : x right, y forward, z up  (ENU)
 *   MAVLink/NED  : x north, y east,    z down
 *   We send (bx, by, -bz) — only z sign changes for a horizontal square.
 *
 * USB serial prints DBG lines for verification (optional, no PC required).
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "mavlink/mavlink.h"

/* ---- Square path --------------------------------------------------------- */

/** Four corners: [seg][x, y, z_world].  z = 2 m = 2 m above origin. */
static const float CORNERS[4][3] = {
    {0.00f, 0.00f, 2.00f},
    {1.00f, 0.00f, 2.00f},
    {1.00f, 1.00f, 2.00f},
    {0.00f, 1.00f, 2.00f},
};

/** Duration of each edge [µs] = 8 s. */
#define SEG_US   8000000ULL

/** Loop period [µs] = 40 ms → 25 Hz. */
#define LOOP_US  40000ULL

/* ---- main ---------------------------------------------------------------- */

int main(void)
{
    stdio_init_all();
    mavlink_init();   /* UART0 GPIO 0/1 @ 115200 → Pixhawk 6C TELEM2 */

    sleep_ms(3000);   /* let USB enumerate before printing */

    printf("=== MAVLink VPE Square Test ===\n");
    printf("1 m x 1 m square, z = 2 m, 8 s/edge, 25 Hz VPE on UART0\n\n");

    uint64_t seg_start = time_us_64();
    uint64_t next_tick = seg_start;
    int      seg       = 0;

    printf("SEG 0  (0.00,0.00) -> (1.00,0.00)\n");

    while (1) {
        /* ── Absolute-deadline 25 Hz tick ─────────────────────────────────── */
        uint64_t now = time_us_64();
        if (now < next_tick)
            sleep_us(next_tick - now);
        now       = time_us_64();
        next_tick += LOOP_US;

        /* ── Advance segment every 8 s ────────────────────────────────────── */
        if ((now - seg_start) >= SEG_US) {
            seg_start += SEG_US;
            seg        = (seg + 1) % 4;
            int nxt    = (seg + 1) % 4;
            printf("SEG %d  (%.2f,%.2f) -> (%.2f,%.2f)\n", seg,
                   (double)CORNERS[seg][0], (double)CORNERS[seg][1],
                   (double)CORNERS[nxt][0], (double)CORNERS[nxt][1]);
        }

        /* ── Interpolate exact position ───────────────────────────────────── */
        float t   = (float)(now - seg_start) / (float)SEG_US;
        int   nxt = (seg + 1) % 4;
        float bx  = CORNERS[seg][0] + t * (CORNERS[nxt][0] - CORNERS[seg][0]);
        float by  = CORNERS[seg][1] + t * (CORNERS[nxt][1] - CORNERS[seg][1]);
        float bz  = 2.00f;   /* constant height */

        /*
         * NED conversion: z_NED = -z_world
         * x, y are already in the right orientation for this test.
         */
        float ned_x =  bx;
        float ned_y =  by;
        float ned_z = -bz;   /* -2.0 m = 2 m above ground in NED */

        /* ── Send VPE ─────────────────────────────────────────────────────── */
        mavlink_send_vpe(now, ned_x, ned_y, ned_z);

        /* ── USB debug ────────────────────────────────────────────────────── */
        printf("DBG,%d,%.3f,%.3f,%.3f\n",
               seg, (double)ned_x, (double)ned_y, (double)ned_z);
    }
}
