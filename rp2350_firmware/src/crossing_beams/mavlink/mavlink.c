/**
 * @file  mavlink.c
 * @brief Minimal MAVLink v2 encoder — VISION_POSITION_ESTIMATE (msg #102).
 *
 * Frame layout (45 bytes total):
 *
 *   [0]      STX           0xFD
 *   [1]      len           33  (payload length)
 *   [2]      incompat      0
 *   [3]      compat        0
 *   [4]      seq           auto-increment per call, wraps at 255
 *   [5]      sysid         1
 *   [6]      compid        191  (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
 *   [7]      msgid[0]      102
 *   [8]      msgid[1]      0
 *   [9]      msgid[2]      0
 *   [10..17] usec          uint64_t, little-endian
 *   [18..21] x             float,    little-endian
 *   [22..25] y             float,    little-endian
 *   [26..29] z             float,    little-endian
 *   [30..33] roll          NaN       (0x7FC00000 LE)
 *   [34..37] pitch         NaN       (0x7FC00000 LE)
 *   [38..41] yaw           NaN       (0x7FC00000 LE)
 *   [42]     reset_counter 0
 *   [43..44] CRC           CRC16/MCRF4XX over [1]..[42], then CRC_EXTRA=158
 *
 * CRC algorithm: CRC16/MCRF4XX, poly 0x1021, seed 0xFFFF, no reflection.
 * CRC_EXTRA for msg 102: 158 (derived from the MAVLink message definition).
 *
 * All multi-byte fields are little-endian.  RP2350 is a Cortex-M33 in
 * little-endian mode, so memcpy from float/uint64_t gives the correct layout.
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
#define MAV_MSGID        102u

/** Payload length: usec(8) + x,y,z,roll,pitch,yaw(6×4=24) + reset_counter(1). */
#define MAV_PAYLOAD_LEN  33u

/** CRC_EXTRA for VISION_POSITION_ESTIMATE, from the MAVLink message definition. */
#define MAV_CRC_EXTRA    158u

/** Total frame size: 10 header + 33 payload + 2 CRC. */
#define FRAME_LEN        45u

/* ---- Constants ------------------------------------------------------------ */

/**
 * IEEE 754 single-precision quiet NaN, little-endian: 0x7FC00000.
 * Tells the flight controller this field is not provided / do not fuse.
 */
static const uint8_t k_nan[4] = { 0x00u, 0x00u, 0xC0u, 0x7Fu };

/* ---- State ---------------------------------------------------------------- */

static uint8_t s_seq = 0u;

/* ---- CRC16/MCRF4XX -------------------------------------------------------- */

/**
 * Accumulate one byte into a running CRC16/MCRF4XX value.
 * Poly 0x1021, seed 0xFFFF, no bit-reflection.
 */
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

/** Write a uint64_t into 8 bytes, little-endian. */
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

void mavlink_send_vpe(uint64_t usec, float x, float y, float z)
{
    uint8_t frame[FRAME_LEN];

    /* ---- Header ---------------------------------------------------------- */
    frame[0] = MAV_STX;
    frame[1] = MAV_PAYLOAD_LEN;
    frame[2] = 0u;                                       /* incompat_flags */
    frame[3] = 0u;                                       /* compat_flags   */
    frame[4] = s_seq++;
    frame[5] = MAV_SYSID;
    frame[6] = MAV_COMPID;
    frame[7] = (uint8_t)( MAV_MSGID        & 0xFFu);    /* msgid byte 0   */
    frame[8] = (uint8_t)((MAV_MSGID >>  8u) & 0xFFu);   /* msgid byte 1   */
    frame[9] = (uint8_t)((MAV_MSGID >> 16u) & 0xFFu);   /* msgid byte 2   */

    /* ---- Payload --------------------------------------------------------- */
    write_u64_le(&frame[10], usec);  /* [10..17] timestamp */

    /* memcpy preserves the exact IEEE 754 bit pattern.
     * RP2350 is little-endian, matching MAVLink wire format. */
    memcpy(&frame[18], &x, 4u);      /* [18..21] x        */
    memcpy(&frame[22], &y, 4u);      /* [22..25] y        */
    memcpy(&frame[26], &z, 4u);      /* [26..29] z        */
    memcpy(&frame[30], k_nan, 4u);   /* [30..33] roll=NaN */
    memcpy(&frame[34], k_nan, 4u);   /* [34..37] pitch=NaN*/
    memcpy(&frame[38], k_nan, 4u);   /* [38..41] yaw=NaN  */
    frame[42] = 0u;                  /* [42]     reset_counter */

    /* ---- CRC16/MCRF4XX over bytes [1]..[42], then CRC_EXTRA ------------- */
    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= 42; i++)
        crc = crc_accumulate(frame[i], crc);
    crc = crc_accumulate(MAV_CRC_EXTRA, crc);

    frame[43] = (uint8_t)(crc & 0xFFu);   /* CRC low  byte */
    frame[44] = (uint8_t)(crc >> 8u);     /* CRC high byte */

    /* ---- Send over UART0 ------------------------------------------------- */
    uart_write_blocking(MAV_UART, frame, FRAME_LEN);
}
