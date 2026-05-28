/**
 * @file  mavlink.c
 * @brief Minimal MAVLink v2 encoder — ODOMETRY (msg #331).
 *
 * Frame layout (245 bytes total):
 *
 *   [0]      STX           0xFD
 *   [1]      len           233  (payload length)
 *   [2]      incompat      0
 *   [3]      compat        0
 *   [4]      seq           auto-increment per call, wraps at 255
 *   [5]      sysid         1
 *   [6]      compid        191  (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
 *   [7]      msgid[0]      75   (331 & 0xFF)
 *   [8]      msgid[1]      1    (331 >> 8)
 *   [9]      msgid[2]      0
 *
 * Payload wire layout (struct offsets, relative to frame[10]):
 *   [0..7]     time_usec          uint64_t
 *   [8..11]    x                  float  [m]
 *   [12..15]   y                  float  [m]
 *   [16..19]   z                  float  0.0
 *   [20..35]   q[4]               float  {1,0,0,0} unit quaternion
 *   [36..39]   vx                 float  0.0
 *   [40..43]   vy                 float  0.0
 *   [44..47]   vz                 float  0.0
 *   [48..51]   rollspeed          float  0.0
 *   [52..55]   pitchspeed         float  0.0
 *   [56..59]   yawspeed           float  0.0
 *   [60..143]  pose_covariance    float[21]  all 0.0
 *   [144..227] velocity_covariance float[21] all 0.0
 *   [228]      frame_id           uint8  1  (MAV_FRAME_LOCAL_NED)
 *   [229]      child_frame_id     uint8  8  (MAV_FRAME_BODY_NED)
 *   [230]      reset_counter      uint8  0
 *   [231]      estimator_type     uint8  0
 *   [232]      quality            int8   0
 *
 *   [243..244] CRC16/MCRF4XX over [1]..[242], then CRC_EXTRA=91
 */

#include "mavlink.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

/* ---- Configuration -------------------------------------------------------- */

#define MAV_UART         uart0
#define MAV_BAUD         115200u
#define MAV_TX_PIN       0u
#define MAV_RX_PIN       1u

#define MAV_STX          0xFDu
#define MAV_SYSID        1u
#define MAV_COMPID       191u    /* MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY */
#define MAV_MSGID        331u    /* ODOMETRY */

#define MAV_PAYLOAD_LEN  233u
#define FRAME_LEN        245u    /* 10 header + 233 payload + 2 CRC */
#define MAV_CRC_EXTRA    91u     /* CRC_EXTRA for ODOMETRY from MAVLink common.xml */

/* ---- State ---------------------------------------------------------------- */

static uint8_t s_seq = 0u;

/* ---- CRC16/MCRF4XX -------------------------------------------------------- */

static uint16_t crc_accumulate(uint8_t b, uint16_t crc)
{
    uint8_t tmp = b ^ (uint8_t)(crc & 0xFFu);
    tmp ^= (uint8_t)(tmp << 4u);
    return (uint16_t)((crc >> 8u)
                    ^ ((uint16_t)tmp << 8u)
                    ^ ((uint16_t)tmp << 3u)
                    ^ ((uint16_t)(tmp >> 4u)));
}

/* ---- Helpers -------------------------------------------------------------- */

static void write_u64_le(uint8_t *dst, uint64_t v)
{
    for (int i = 0; i < 8; i++, v >>= 8u)
        dst[i] = (uint8_t)(v & 0xFFu);
}

/* ---- Public API ----------------------------------------------------------- */

void mavlink_init(void)
{
    uart_init(MAV_UART, MAV_BAUD);
    gpio_set_function(MAV_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MAV_RX_PIN, GPIO_FUNC_UART);
}

void mavlink_send_odometry(uint64_t usec, float x, float y)
{
    uint8_t frame[FRAME_LEN];
    memset(frame, 0, sizeof(frame));

    /* ---- Header ---------------------------------------------------------- */
    frame[0] = MAV_STX;
    frame[1] = MAV_PAYLOAD_LEN;
    frame[2] = 0u;
    frame[3] = 0u;
    frame[4] = s_seq++;
    frame[5] = MAV_SYSID;
    frame[6] = MAV_COMPID;
    frame[7] = (uint8_t)( MAV_MSGID        & 0xFFu);
    frame[8] = (uint8_t)((MAV_MSGID >>  8u) & 0xFFu);
    frame[9] = (uint8_t)((MAV_MSGID >> 16u) & 0xFFu);

    /* ---- Payload --------------------------------------------------------- */
    write_u64_le(&frame[10],      usec);   /* [0..7]  time_usec */
    memcpy      (&frame[18], &x,  4u);     /* [8..11] x         */
    memcpy      (&frame[22], &y,  4u);     /* [12..15] y        */
    /* z stays 0.0 (memset) */

    /* unit quaternion {w=1, x=0, y=0, z=0} — null rotation */
    static const float q_unit[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    memcpy(&frame[30], q_unit, 16u);       /* [20..35] q[4]     */

    /* velocities, angular rates, covariances stay 0.0 (memset) */

    frame[10 + 228] = 1u;    /* frame_id       = MAV_FRAME_LOCAL_NED */
    frame[10 + 229] = 8u;    /* child_frame_id = MAV_FRAME_BODY_NED  */
    frame[10 + 232] = 100u;  /* quality        = 100% (best)         */

    /* ---- CRC16/MCRF4XX over bytes [1]..[242], then CRC_EXTRA ------------- */
    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= (int)(9u + MAV_PAYLOAD_LEN); i++)
        crc = crc_accumulate(frame[i], crc);
    crc = crc_accumulate(MAV_CRC_EXTRA, crc);

    frame[243] = (uint8_t)(crc & 0xFFu);
    frame[244] = (uint8_t)(crc >> 8u);

    /* ---- Send over UART0 ------------------------------------------------- */
    uart_write_blocking(MAV_UART, frame, FRAME_LEN);
}
