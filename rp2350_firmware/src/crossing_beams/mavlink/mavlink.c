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
 * Payload wire layout (offsets relative to frame[10]):
 *   [0..7]     time_usec           uint64
 *   [8..11]    x                   float  [m]
 *   [12..15]   y                   float  [m]
 *   [16..19]   z                   float  [m, NED down]
 *   [20..35]   q[4]                float  {1,0,0,0} unit quaternion
 *   [36..59]   vx/vy/vz + rates    float  NaN / 0.0
 *   [60..143]  pose_covariance     float[21]  diagonal set, rest 0.0
 *   [144..227] velocity_covariance float[21]  NaN (not used by AP)
 *   [228]      frame_id            uint8  20 (MAV_FRAME_LOCAL_FRD)
 *   [229]      child_frame_id      uint8  12 (MAV_FRAME_BODY_FRD)
 *   [230]      reset_counter       uint8  0
 *   [231]      estimator_type      uint8  3  (MAV_ESTIMATOR_TYPE_VIO)
 *   [232]      quality             int8   100
 *
 *   [243..244] CRC16/MCRF4XX over [1]..[242], CRC_EXTRA=91
 *
 * pose_covariance is the upper triangle of a 6×6 matrix (position + attitude).
 * ArduPilot derives posErr from indices [0],[6],[11] (x,y,z variances).
 * We set those to POS_VAR (0.01 m² ≈ 10 cm σ); all other elements are 0.
 * velocity_covariance is unused by AP — NaN is fine there.
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
#define MAV_COMPID       191u
#define MAV_MSGID        331u

#define MAV_PAYLOAD_LEN  233u
#define FRAME_LEN        245u
#define MAV_CRC_EXTRA    91u

/** Position variance sent in pose_covariance diagonal [m²]. ~10 cm σ. */
#define POS_VAR  0.01f

/* ---- Constants ------------------------------------------------------------ */

static const uint8_t k_nan[4] = { 0x00u, 0x00u, 0xC0u, 0x7Fu };

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

static void write_f32(uint8_t *dst, float v)
{
    memcpy(dst, &v, 4u);
}

/* ---- Public API ----------------------------------------------------------- */

void mavlink_init(void)
{
    uart_init(MAV_UART, MAV_BAUD);
    gpio_set_function(MAV_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MAV_RX_PIN, GPIO_FUNC_UART);
}

void mavlink_send_odometry(uint64_t usec, float x, float y, float z)
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
    write_u64_le(&frame[10], usec);
    write_f32(&frame[18], x);               /* x */
    write_f32(&frame[22], y);               /* y */
    write_f32(&frame[26], z);               /* z */

    static const float q_unit[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    memcpy(&frame[30], q_unit, 16u);        /* q[4] */

    /* vx/vy/vz = NaN (position-only; AP handles this gracefully) */
    memcpy(&frame[46], k_nan, 4u);
    memcpy(&frame[50], k_nan, 4u);
    memcpy(&frame[54], k_nan, 4u);
    /* rollspeed/pitchspeed/yawspeed stay 0.0 (memset) */

    /* pose_covariance[21]: upper triangle of 6×6 (pos xyz + att rpy).
     * AP uses indices [0],[6],[11] for x,y,z position variance to compute
     * posErr. All other elements stay 0. */
    write_f32(&frame[70],  POS_VAR);        /* cov[0]  = x variance  */
    write_f32(&frame[94],  POS_VAR);        /* cov[6]  = y variance  */
    write_f32(&frame[114], POS_VAR);        /* cov[11] = z variance  */

    /* velocity_covariance[21]: unused by AP — NaN throughout */
    for (int i = 0; i < 21; i++)
        memcpy(&frame[154 + i * 4], k_nan, 4u);

    frame[238] = 20u;   /* frame_id       = MAV_FRAME_LOCAL_FRD  */
    frame[239] = 12u;   /* child_frame_id = MAV_FRAME_BODY_FRD   */
    frame[240] = 0u;    /* reset_counter  */
    frame[241] = 3u;    /* estimator_type = MAV_ESTIMATOR_TYPE_VIO */
    frame[242] = 100;   /* quality        */

    /* ---- CRC ------------------------------------------------------------- */
    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= (int)(9u + MAV_PAYLOAD_LEN); i++)
        crc = crc_accumulate(frame[i], crc);
    crc = crc_accumulate(MAV_CRC_EXTRA, crc);

    frame[243] = (uint8_t)(crc & 0xFFu);
    frame[244] = (uint8_t)(crc >> 8u);

    uart_write_blocking(MAV_UART, frame, FRAME_LEN);
}
