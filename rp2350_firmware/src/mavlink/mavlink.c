/**
 * @file  mavlink.c
 * @brief Minimal MAVLink v2 codec — ODOMETRY TX (msg #331) + EKF_STATUS_REPORT RX (msg #193).
 *
 * TX frame layout (245 bytes total, ODOMETRY):
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
 *   [231]      estimator_type      uint8  0  (MAV_ESTIMATOR_TYPE_UNKNOWN)
 *   [232]      quality             int8   100
 *
 *   [243..244] CRC16/MCRF4XX over [1]..[242], CRC_EXTRA=91
 *
 * pose_covariance is the upper triangle of a 6×6 matrix (position + attitude).
 * ArduPilot derives posErr from indices [0],[6],[11] (x,y,z variances).
 * We set those to POS_VAR (0.01 m² ≈ 10 cm σ); all other elements are 0.
 * velocity_covariance is unused by AP — NaN is fine there.
 *
 * RX: parses incoming MAVLink v2 frames from the FC.  Only EKF_STATUS_REPORT
 * (msg #193) is acted upon — its flags field at payload[20..21] is checked to
 * determine whether the EKF has a healthy position solution.
 *
 * EKF_STATUS_REPORT payload wire layout (22 bytes, no extension):
 *   [0..3]   velocity_variance    float
 *   [4..7]   pos_horiz_variance   float
 *   [8..11]  pos_vert_variance    float
 *   [12..15] compass_variance     float
 *   [16..19] terrain_alt_variance float
 *   [20..21] flags                uint16  ← what we check
 *   CRC_EXTRA = 71
 *
 * COMMAND_LONG (msg #76, CRC_EXTRA = 152) is used to:
 *   • send MAV_CMD_DO_SET_HOME (179) with param1=1 (use current position)
 *   • send MAV_CMD_SET_MESSAGE_INTERVAL (511) to request EKF_STATUS_REPORT @ 1 Hz
 */

#include "mavlink.h"

#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

/* ---- Configuration -------------------------------------------------------- */

#define MAV_UART         uart0
#define MAV_BAUD         921600u   /* SERIALx_BAUD = 921 on the flight controller */
#define MAV_TX_PIN       0u
#define MAV_RX_PIN       1u

#define MAV_STX          0xFDu
#define MAV_SYSID        1u
#define MAV_COMPID       191u
#define MAV_MSGID        331u

#define MAV_PAYLOAD_LEN  233u
#define FRAME_LEN        245u
#define MAV_CRC_EXTRA    91u

#define MSGID_TUNNEL     385u
#define CRC_EXTRA_TUNNEL 147u
#define TUNNEL_TYPE_LH2  0x4C48u
#define TUNNEL_DATA_LEN  95u   /* LH2 TUNNEL v3, see mavlink_send_lh2_angles() */
#define LH2_ANGLE_LSB    16384.0f  /* int16 angle units per radian: 1/16384 rad, range +-2 rad */
#define TUNNEL_FRAME_LEN (10u + 133u + 2u)
#define MAV_GCS_SYSID    255u
#define MAV_GCS_COMPID   190u

/** Position variance sent in pose_covariance diagonal [m²]. ~10 cm σ. */
#define POS_VAR  0.01f

/* ---- RX constants --------------------------------------------------------- */

#define MSGID_EKF_STATUS   193u
#define CRC_EXTRA_EKF      71u
#define MSGID_CMD_LONG     76u
#define CRC_EXTRA_CMD      152u
#define MAV_CMD_SET_HOME   179u
#define MAV_CMD_MSG_INTV   511u

/* EKF_STATUS_REPORT flags we require for "healthy" */
#define EKF_NEED_FLAGS   (0x0001u | 0x0008u)  /* ATTITUDE | POS_HORIZ_REL */
#define EKF_CONST_POS    0x0080u               /* constant-pos mode = not healthy */

/* RX payload buffer — sized for EKF_STATUS_REPORT (22 bytes) */
#define MAX_RX_BUF  32u

/* ---- TIMESYNC (msg #111) ------------------------------------------------- */

#define MSGID_TIMESYNC       111u
#define CRC_EXTRA_TIMESYNC   34u
#define TIMESYNC_PL_LEN      16u          /* tc1(int64) + ts1(int64) */
#define TIMESYNC_FRAME_LEN   28u          /* 10 hdr + 16 payload + 2 CRC */
#define TIMESYNC_RTT_MAX_NS  50000000LL   /* discard samples with RTT > 50 ms */
#define TIMESYNC_ALPHA       0.05f        /* EMA gain — τ ≈ 2 s at 10 Hz */

/* ---- Constants ------------------------------------------------------------ */

static const uint8_t k_nan[4] = { 0x00u, 0x00u, 0xC0u, 0x7Fu };

/* ---- State ---------------------------------------------------------------- */

static uint8_t s_seq = 0u;
static uint16_t s_lh2_seq = 0u;

/* ---- RX parser state ------------------------------------------------------ */

/* Sub-states for the byte-by-byte MAVLink v2 frame parser */
enum {
    RXS_IDLE = 0,
    RXS_LEN, RXS_INCOMPAT, RXS_COMPAT, RXS_SEQ,
    RXS_SYSID, RXS_COMPID,
    RXS_MSGID0, RXS_MSGID1, RXS_MSGID2,
    RXS_PAYLOAD, RXS_CRC0, RXS_CRC1,
};

static struct {
    uint8_t  state;
    uint8_t  len;
    uint8_t  idx;
    bool     discard;      /* payload too long for buf — skip content, still parse */
    uint32_t msgid;
    uint16_t crc;          /* running CRC over bytes [1..9+len] */
    uint8_t  crc_lo;       /* saved first CRC byte */
    uint8_t  buf[MAX_RX_BUF];
} s_rx;

static bool  s_ekf_healthy        = false;
static float s_timesync_offset_ns = 0.0f;  /* EMA of (local_ns − fc_ns) */
static bool  s_timesync_valid     = false;

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

static void write_u16_le(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)(v >> 8u);
}

static void write_i64_le(uint8_t *dst, int64_t v)
{
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; i++, u >>= 8u)
        dst[i] = (uint8_t)(u & 0xFFu);
}

static int64_t read_i64_le(const uint8_t *p)
{
    uint64_t u = 0;
    for (int i = 7; i >= 0; i--)
        u = (u << 8u) | p[i];
    return (int64_t)u;
}

/* ---- RX parser ------------------------------------------------------------ */

static void _dispatch_ekf(uint8_t crc_hi)
{
    if (s_rx.msgid != MSGID_EKF_STATUS || s_rx.discard || s_rx.len < 22u) return;

    uint16_t received = (uint16_t)s_rx.crc_lo | ((uint16_t)crc_hi << 8u);
    uint16_t computed = crc_accumulate(CRC_EXTRA_EKF, s_rx.crc);
    if (computed != received) return;

    uint16_t flags = (uint16_t)s_rx.buf[20] | ((uint16_t)s_rx.buf[21] << 8u);
    s_ekf_healthy = ((flags & EKF_NEED_FLAGS) == EKF_NEED_FLAGS)
                 && !(flags & EKF_CONST_POS);
}

/* ---- TIMESYNC TX/RX ------------------------------------------------------- */

static void _send_timesync(int64_t tc1, int64_t ts1)
{
    uint8_t f[TIMESYNC_FRAME_LEN];
    memset(f, 0, sizeof(f));
    f[0] = MAV_STX;
    f[1] = TIMESYNC_PL_LEN;
    f[4] = s_seq++;
    f[5] = MAV_SYSID;
    f[6] = MAV_COMPID;
    f[7] = (uint8_t)MSGID_TIMESYNC;
    write_i64_le(&f[10], tc1);
    write_i64_le(&f[18], ts1);
    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= 9 + (int)TIMESYNC_PL_LEN; i++)
        crc = crc_accumulate(f[i], crc);
    crc = crc_accumulate(CRC_EXTRA_TIMESYNC, crc);
    f[26] = (uint8_t)(crc & 0xFFu);
    f[27] = (uint8_t)(crc >> 8u);
    uart_write_blocking(MAV_UART, f, TIMESYNC_FRAME_LEN);
}

static void _dispatch_timesync(uint8_t crc_hi)
{
    if (s_rx.msgid != MSGID_TIMESYNC || s_rx.discard || s_rx.len < TIMESYNC_PL_LEN) return;

    uint16_t received = (uint16_t)s_rx.crc_lo | ((uint16_t)crc_hi << 8u);
    uint16_t computed = crc_accumulate(CRC_EXTRA_TIMESYNC, s_rx.crc);
    if (computed != received) return;

    int64_t tc1    = read_i64_le(&s_rx.buf[0]);
    int64_t ts1    = read_i64_le(&s_rx.buf[8]);
    int64_t now_ns = (int64_t)(time_us_64() * 1000ULL);

    if (tc1 == 0) {
        /* FC broadcasting its time — calibrate offset, then echo */
        int64_t offset_ns = now_ns - ts1;
        if (!s_timesync_valid) {
            s_timesync_offset_ns = (float)offset_ns;
            s_timesync_valid     = true;
        } else {
            s_timesync_offset_ns += TIMESYNC_ALPHA * ((float)offset_ns - s_timesync_offset_ns);
        }
        _send_timesync(now_ns, ts1);
        return;
    }

    /* FC responded to our earlier request: compute RTT and update offset EMA */
    int64_t rtt_ns = now_ns - ts1;
    if (rtt_ns <= 0 || rtt_ns > TIMESYNC_RTT_MAX_NS) return;   /* stale or outlier */

    /* offset = (local_midpoint) − (fc_time_at_midpoint)
     *        = (ts1 + now_ns)/2 − tc1 */
    int64_t offset_ns = (ts1 + now_ns - 2LL * tc1) / 2LL;

    if (!s_timesync_valid) {
        s_timesync_offset_ns = (float)offset_ns;
        s_timesync_valid     = true;
    } else {
        s_timesync_offset_ns += TIMESYNC_ALPHA * ((float)offset_ns - s_timesync_offset_ns);
    }
}

static void _rx_feed(uint8_t b)
{
    switch (s_rx.state) {
    case RXS_IDLE:
        if (b == MAV_STX) s_rx.state = RXS_LEN;
        break;
    case RXS_LEN:
        s_rx.len = b; s_rx.idx = 0u;
        s_rx.discard = (b > MAX_RX_BUF);
        s_rx.crc = crc_accumulate(b, 0xFFFFu);
        s_rx.state = RXS_INCOMPAT;
        break;
    case RXS_INCOMPAT:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_COMPAT;
        break;
    case RXS_COMPAT:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_SEQ;
        break;
    case RXS_SEQ:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_SYSID;
        break;
    case RXS_SYSID:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_COMPID;
        break;
    case RXS_COMPID:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_MSGID0;
        break;
    case RXS_MSGID0:
        s_rx.msgid = b; s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_MSGID1;
        break;
    case RXS_MSGID1:
        s_rx.msgid |= (uint32_t)b << 8u; s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_MSGID2;
        break;
    case RXS_MSGID2:
        s_rx.msgid |= (uint32_t)b << 16u; s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = (s_rx.len == 0u) ? RXS_CRC0 : RXS_PAYLOAD;
        break;
    case RXS_PAYLOAD:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        if (!s_rx.discard) s_rx.buf[s_rx.idx] = b;
        if (++s_rx.idx >= s_rx.len) s_rx.state = RXS_CRC0;
        break;
    case RXS_CRC0:
        s_rx.crc_lo = b; s_rx.state = RXS_CRC1;
        break;
    case RXS_CRC1:
        _dispatch_ekf(b);
        _dispatch_timesync(b);
        s_rx.state = RXS_IDLE;
        break;
    }
}

/* ---- COMMAND_LONG helper -------------------------------------------------- */

/* Sends COMMAND_LONG (msg #76, 33-byte payload, CRC_EXTRA=152).
 * Only param1 and param2 are non-zero; all others are 0. */
static void _send_cmd_long(uint16_t cmd, float p1, float p2)
{
    enum { PL = 33, TOTAL = 45 };   /* 10 header + 33 payload + 2 CRC */
    uint8_t f[TOTAL];
    memset(f, 0, sizeof(f));

    f[0] = MAV_STX;
    f[1] = (uint8_t)PL;
    f[4] = s_seq++;
    f[5] = MAV_SYSID;
    f[6] = MAV_COMPID;
    f[7] = (uint8_t)(MSGID_CMD_LONG & 0xFFu);  /* 76 */

    write_f32(&f[10], p1);                      /* param1 */
    write_f32(&f[14], p2);                      /* param2 */

    f[38] = (uint8_t)(cmd & 0xFFu);             /* command lo */
    f[39] = (uint8_t)(cmd >>  8u);              /* command hi */
    f[40] = 1u;                                 /* target_system  */
    f[41] = 1u;                                 /* target_component */

    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= 9 + PL; i++) crc = crc_accumulate(f[i], crc);
    crc = crc_accumulate(CRC_EXTRA_CMD, crc);
    f[43] = (uint8_t)(crc & 0xFFu);
    f[44] = (uint8_t)(crc >>  8u);

    uart_write_blocking(MAV_UART, f, TOTAL);
}

/* ---- RX ring: the UART interrupt empties the 32-byte hardware FIFO into it, so nothing is lost
 * while the main loop is busy (a blocking TX burst, USB prints); mavlink_rx_update() parses it. */

#define RX_RING_SIZE 1024u                    /* power of two */
static volatile uint8_t  s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0u;      /* written by the interrupt */
static volatile uint16_t s_rx_tail = 0u;      /* written by mavlink_rx_update() */
static volatile uint32_t s_rx_dropped = 0u;   /* bytes lost because the ring was full */

static void _uart_rx_irq(void)
{
    while (uart_is_readable(MAV_UART)) {
        const uint8_t c = (uint8_t)uart_getc(MAV_UART);
        const uint16_t next = (uint16_t)((s_rx_head + 1u) & (RX_RING_SIZE - 1u));
        if (next == s_rx_tail) { s_rx_dropped++; continue; }
        s_rx_ring[s_rx_head] = c;
        s_rx_head = next;
    }
}

/* ---- Public API ----------------------------------------------------------- */

uint32_t mavlink_init(void)
{
    const uint32_t baud = uart_init(MAV_UART, MAV_BAUD);
    gpio_set_function(MAV_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MAV_RX_PIN, GPIO_FUNC_UART);
    /* RX by interrupt, on the core that calls this (core 0, like the parser). */
    irq_set_exclusive_handler(UART0_IRQ, _uart_rx_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irqs_enabled(MAV_UART, true, false);
    return baud;
}

uint32_t mavlink_rx_dropped(void)
{
    return s_rx_dropped;
}

void mavlink_send_odometry(uint64_t usec, float x, float y, float z, float yaw, float yaw_var)
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

    /* q[4] = w, x, y, z: yaw only, about the down axis (roll/pitch come from the FC's IMU) */
    write_f32(&frame[30], cosf(0.5f * yaw));
    write_f32(&frame[34], 0.0f);
    write_f32(&frame[38], 0.0f);
    write_f32(&frame[42], sinf(0.5f * yaw));

    /* vx/vy/vz = NaN (position-only; AP handles this gracefully) */
    memcpy(&frame[46], k_nan, 4u);
    memcpy(&frame[50], k_nan, 4u);
    memcpy(&frame[54], k_nan, 4u);
    /* rollspeed/pitchspeed/yawspeed stay 0.0 (memset) */

    /* pose_covariance[21]: upper triangle of 6×6 (pos xyz + att rpy).
     * AP uses the diagonal: [0],[6],[11] for the position error, [15],[18],[20]
     * for the attitude error (yaw, with EK3_SRC1_YAW = ExtNav). Others stay 0. */
    write_f32(&frame[70],  POS_VAR);        /* cov[0]  = x variance  */
    write_f32(&frame[94],  POS_VAR);        /* cov[6]  = y variance  */
    write_f32(&frame[114], POS_VAR);        /* cov[11] = z variance  */
    write_f32(&frame[130], yaw_var);        /* cov[15] = roll variance  */
    write_f32(&frame[142], yaw_var);        /* cov[18] = pitch variance */
    write_f32(&frame[150], yaw_var);        /* cov[20] = yaw variance   */

    /* velocity_covariance[21]: unused by AP — NaN throughout */
    for (int i = 0; i < 21; i++)
        memcpy(&frame[154 + i * 4], k_nan, 4u);

    frame[238] = 20u;   /* frame_id       = MAV_FRAME_LOCAL_FRD  */
    frame[239] = 12u;   /* child_frame_id = MAV_FRAME_BODY_FRD   */
    frame[240] = 0u;    /* reset_counter  */
    frame[241] = 0u;    /* estimator_type = MAV_ESTIMATOR_TYPE_UNKNOWN */
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

/*
 * LH2 TUNNEL v3 payload (95 bytes, little-endian):
 *   "LH", version 3, sequence u16, timestamp_us u64, valid_mask u16,
 *   then for index = sensor * NUM_BS + bs (4 sensors x 4 stations):
 *     angles int16[16][2]  horizontal, vertical in 1/16384 rad (range +-2 rad, ~0.06 mrad steps)
 *     ages   uint8[16]     age of that pair [ms], 255 = 255 ms or older / not valid
 * v2 (two stations, float angles, u16 ages) did not fit 4 stations in TUNNEL's 128 bytes;
 * utils/calibration/capture_lh2.py parses both.
 */
_Static_assert(20 + NUM_SENSORS * NUM_BS * 4 + NUM_SENSORS * NUM_BS <= 5 + 128 - 1,
               "LH2 TUNNEL v3 payload does not fit in the 128-byte TUNNEL payload");
_Static_assert(NUM_SENSORS * NUM_BS <= 16, "valid_mask is 16 bits");

static int16_t _angle_to_i16(float rad)
{
    float v = rad * LH2_ANGLE_LSB;
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32767.0f) v = -32767.0f;
    return (int16_t)lroundf(v);
}

/** Send one TUNNEL (payload type LH2) whose payload data is @p data[0 .. len). */
static void _send_lh2_tunnel(const uint8_t *data, uint8_t len)
{
    uint8_t frame[TUNNEL_FRAME_LEN];
    memset(frame, 0, sizeof(frame));
    frame[0] = MAV_STX;
    frame[1] = 133u;
    frame[4] = s_seq++;
    frame[5] = MAV_SYSID;
    frame[6] = MAV_COMPID;
    frame[7] = (uint8_t)(MSGID_TUNNEL & 0xFFu);
    frame[8] = (uint8_t)((MSGID_TUNNEL >> 8u) & 0xFFu);

    /* TUNNEL wire order: payload_type, target_system, target_component,
     * payload_length, then payload[128]. */
    uint8_t *p = &frame[10];
    write_u16_le(&p[0], TUNNEL_TYPE_LH2);
    p[2] = MAV_GCS_SYSID;
    p[3] = MAV_GCS_COMPID;
    p[4] = len;
    memcpy(&p[5], data, len);
    p[132] = 0xA5u;  /* Keep MAVLink 2 from trimming the unused TUNNEL tail. */

    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= 9 + 133; i++)
        crc = crc_accumulate(frame[i], crc);
    crc = crc_accumulate(CRC_EXTRA_TUNNEL, crc);
    frame[143] = (uint8_t)(crc & 0xFFu);
    frame[144] = (uint8_t)(crc >> 8u);
    uart_write_blocking(MAV_UART, frame, 145u);
}

void mavlink_send_lh2_angles(uint64_t usec,
                             const float angles[NUM_SENSORS][NUM_BS][2],
                             const uint8_t angle_age_ms[NUM_SENSORS][NUM_BS],
                             uint16_t valid_mask)
{
    uint8_t d[TUNNEL_DATA_LEN];
    memset(d, 0, sizeof(d));
    d[0] = 'L';
    d[1] = 'H';
    d[2] = 3u;
    write_u16_le(&d[3], s_lh2_seq++);
    write_u64_le(&d[5], usec);
    write_u16_le(&d[13], valid_mask);
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        for (int bs = 0; bs < NUM_BS; bs++) {
            int index = sensor * NUM_BS + bs;
            write_u16_le(&d[15 + index * 4], (uint16_t)_angle_to_i16(angles[sensor][bs][0]));
            write_u16_le(&d[17 + index * 4], (uint16_t)_angle_to_i16(angles[sensor][bs][1]));
            d[79 + index] = angle_age_ms[sensor][bs];
        }
    }
    _send_lh2_tunnel(d, TUNNEL_DATA_LEN);
}

/*
 * LH2 OOTX status, payload "LO" v2 (13 + payload_len bytes, little-endian):
 *   "LO", version 2, station index u8, flags u8 (bit 0: payload valid), frames_ok u16,
 *   bits u16 and missed turns u16 (saturating), coverage u8 (% of the frame's bits known),
 *   payload_len u8, then the latest CRC-valid OOTX frame
 *   (struct ootxDataFrame_s, as utils/calibration/lh2_ootx.py parse_payload() reads it).
 */
void mavlink_send_lh2_ootx(int bs, const ootx_station_t *station)
{
    uint8_t d[13 + OOTX_MAX_PAYLOAD];
    uint8_t len = station->valid ? station->payload_len : 0u;
    d[0] = 'L';
    d[1] = 'O';
    d[2] = 2u;
    d[3] = (uint8_t)bs;
    d[4] = station->valid ? 1u : 0u;
    write_u16_le(&d[5], station->frames_ok);
    write_u16_le(&d[7], (uint16_t)(station->bits > 0xFFFFu ? 0xFFFFu : station->bits));
    write_u16_le(&d[9], (uint16_t)(station->missed > 0xFFFFu ? 0xFFFFu : station->missed));
    d[11] = station->coverage;
    d[12] = len;
    memcpy(&d[13], station->payload, len);
    _send_lh2_tunnel(d, (uint8_t)(13 + len));
}

void mavlink_rx_update(void)
{
    while (s_rx_tail != s_rx_head) {
        const uint8_t c = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) & (RX_RING_SIZE - 1u));
        _rx_feed(c);
    }
}

bool mavlink_is_ekf_healthy(void)
{
    return s_ekf_healthy;
}

void mavlink_send_do_set_home(void)
{
    /* param1=1 → use current vehicle position as home */
    _send_cmd_long(MAV_CMD_SET_HOME, 1.0f, 0.0f);
}

void mavlink_request_ekf_stream(void)
{
    /* Ask the FC to stream EKF_STATUS_REPORT (id 193) at 1 Hz (1 000 000 µs) */
    _send_cmd_long(MAV_CMD_MSG_INTV, 193.0f, 1000000.0f);
}


uint64_t mavlink_timesync_corrected_us(uint64_t local_us)
{
    if (!s_timesync_valid) return local_us;
    /* fc_time = local_time − offset  (offset = local_ns − fc_ns) */
    int64_t corrected = (int64_t)local_us - (int64_t)(s_timesync_offset_ns / 1000.0f);
    return corrected > 0 ? (uint64_t)corrected : 0u;
}
