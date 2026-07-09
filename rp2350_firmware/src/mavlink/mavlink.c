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

#include <string.h>
#include <stdbool.h>

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

/* ---- RX constants --------------------------------------------------------- */

#define MSGID_EKF_STATUS   193u
#define CRC_EXTRA_EKF      71u
#define MSGID_CMD_LONG     76u
#define CRC_EXTRA_CMD      152u
#define MAV_CMD_SET_HOME   179u
#define MAV_CMD_MSG_INTV   511u

/* NAMED_VALUE_FLOAT (msg #251): pushed by lh2_bs_params.lua every 1 s */
#define MSGID_NAMED_FLOAT     251u
#define CRC_EXTRA_NAMED_FLOAT 170u
/* Wire layout (18 bytes): time_boot_ms(4) + value(4) + name(10) */
#define NAMED_FLOAT_VAL_OFF   4u
#define NAMED_FLOAT_NAME_OFF  8u
#define N_LH2_PARAMS          25u   /* NUMBS + 2×(3 origin + 9 R) */

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

/* ---- RX parser state ------------------------------------------------------ */

/* Sub-states for the byte-by-byte MAVLink v1/v2 frame parser */
enum {
    RXS_IDLE = 0,
    /* MAVLink v2 states */
    RXS_LEN, RXS_INCOMPAT, RXS_COMPAT, RXS_SEQ,
    RXS_SYSID, RXS_COMPID,
    RXS_MSGID0, RXS_MSGID1, RXS_MSGID2,
    RXS_PAYLOAD, RXS_CRC0, RXS_CRC1,
    /* MAVLink v1 extra states (no incompat/compat, 1-byte msgid) */
    RXS_V1_SEQ, RXS_V1_SYSID, RXS_V1_COMPID, RXS_V1_MSGID,
};

static struct {
    uint8_t  state;
    uint8_t  len;
    uint8_t  idx;
    bool     discard;      /* payload too long for buf — skip content, still parse */
    bool     is_v1;        /* true when parsing a MAVLink v1 frame (STX=0xFE) */
    uint32_t msgid;
    uint16_t crc;          /* running CRC over header+payload bytes */
    uint8_t  crc_lo;       /* saved first CRC byte */
    uint8_t  buf[MAX_RX_BUF];
} s_rx;

static bool  s_ekf_healthy        = false;
static float s_timesync_offset_ns = 0.0f;  /* EMA of (local_ns − fc_ns) */
static bool  s_timesync_valid     = false;

/* ---- BS pose param state ------------------------------------------------- */

static float    s_bs_origin[NUM_BS][3];
static float    s_bs_R[NUM_BS][3][3];
static uint32_t s_received_mask  = 0u;  /* bit i set when param i received */
static bool     s_poses_ready    = false;
static uint32_t s_param_val_seen = 0u;  /* total PARAM_VALUE frames with valid CRC */
static uint32_t s_rx_bytes       = 0u;  /* total raw bytes received on UART */

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

/* ---- LH2 NAMED_VALUE_FLOAT parser ---------------------------------------- */

/*
 * Bit layout for s_received_mask (25 bits total):
 *   bit  0        : NUMBS
 *   bits 1–3      : BS0 X, Y, Z
 *   bits 4–12     : BS0 R[0][0]..R[2][2]  (row-major: 4+r*3+c)
 *   bits 13–15    : BS1 X, Y, Z
 *   bits 16–24    : BS1 R[0][0]..R[2][2]
 */
static void _parse_named_float(const char *name, float value)
{
    int bit = -1;

    if (strcmp(name, "NUMBS") == 0) {
        bit = 0;
    } else if (name[0]=='B' && name[1]=='S' &&
               (name[2]=='0' || name[2]=='1')) {
        int         i    = name[2] - '0';
        const char *f    = name + 3;
        int         base = (i == 0) ? 1 : 13;

        if (f[0]=='X' && f[1]=='\0') {
            s_bs_origin[i][0] = value; bit = base + 0;
        } else if (f[0]=='Y' && f[1]=='\0') {
            s_bs_origin[i][1] = value; bit = base + 1;
        } else if (f[0]=='Z' && f[1]=='\0') {
            s_bs_origin[i][2] = value; bit = base + 2;
        } else if (f[0]=='R' && f[1]>='0' && f[1]<='2' &&
                                 f[2]>='0' && f[2]<='2' && f[3]=='\0') {
            int r = f[1]-'0', c = f[2]-'0';
            s_bs_R[i][r][c] = value; bit = base + 3 + r*3 + c;
        }
    }

    if (bit < 0 || (unsigned)bit >= N_LH2_PARAMS) return;
    s_received_mask |= (1u << bit);
    if (s_received_mask == (1u << N_LH2_PARAMS) - 1u)
        s_poses_ready = true;
}

static void _dispatch_named_float(uint8_t crc_hi)
{
    if (s_rx.msgid != MSGID_NAMED_FLOAT || s_rx.discard) return;
    /* MAVLink v2 truncates trailing zero bytes: "NUMBS" → LEN=13, not 18.
     * Need at least: time(4) + value(4) + 1 name byte = 9. */
    if (s_rx.len < (NAMED_FLOAT_VAL_OFF + 4u + 1u)) return;

    uint16_t received = (uint16_t)s_rx.crc_lo | ((uint16_t)crc_hi << 8u);
    uint16_t computed = crc_accumulate(CRC_EXTRA_NAMED_FLOAT, s_rx.crc);
    if (computed != received) return;

    s_param_val_seen++;

    float value;
    memcpy(&value, &s_rx.buf[NAMED_FLOAT_VAL_OFF], 4u);

    /* Copy available name bytes; zero-fill the rest (handles v2 truncation). */
    char name[11];
    uint8_t avail = (s_rx.len > NAMED_FLOAT_NAME_OFF)
                  ? (uint8_t)(s_rx.len - NAMED_FLOAT_NAME_OFF) : 0u;
    if (avail > 10u) avail = 10u;
    memcpy(name, &s_rx.buf[NAMED_FLOAT_NAME_OFF], avail);
    memset(name + avail, '\0', (size_t)(11u - avail));

    _parse_named_float(name, value);
}

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
        if (b == MAV_STX)  { s_rx.is_v1 = false; s_rx.state = RXS_LEN; }
        else if (b == 0xFEu) { s_rx.is_v1 = true;  s_rx.state = RXS_LEN; }
        break;
    case RXS_LEN:
        s_rx.len = b; s_rx.idx = 0u;
        s_rx.discard = (b > MAX_RX_BUF);
        s_rx.crc = crc_accumulate(b, 0xFFFFu);
        s_rx.state = s_rx.is_v1 ? RXS_V1_SEQ : RXS_INCOMPAT;
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
        _dispatch_named_float(b);
        s_rx.state = RXS_IDLE;
        break;

    /* MAVLink v1: seq, sysid, compid, msgid (1 byte) — same CRC accumulation */
    case RXS_V1_SEQ:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_V1_SYSID;
        break;
    case RXS_V1_SYSID:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_V1_COMPID;
        break;
    case RXS_V1_COMPID:
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = RXS_V1_MSGID;
        break;
    case RXS_V1_MSGID:
        s_rx.msgid = b;
        s_rx.crc = crc_accumulate(b, s_rx.crc);
        s_rx.state = (s_rx.len == 0u) ? RXS_CRC0 : RXS_PAYLOAD;
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

/* ---- Public API ----------------------------------------------------------- */

void mavlink_send_heartbeat(void)
{
    /* HEARTBEAT (msg #0, CRC_EXTRA=50), 9-byte payload:
     *   custom_mode  uint32  0
     *   type         uint8   18 (MAV_TYPE_ONBOARD_CONTROLLER)
     *   autopilot    uint8   8  (MAV_AUTOPILOT_INVALID)
     *   base_mode    uint8   0
     *   system_status uint8  4  (MAV_STATE_ACTIVE)
     *   mavlink_version uint8 3
     */
    enum { PL = 9, TOTAL = 21 };
    uint8_t f[TOTAL];
    memset(f, 0, sizeof(f));
    f[0] = MAV_STX;
    f[1] = (uint8_t)PL;
    f[4] = s_seq++;
    f[5] = MAV_SYSID;
    f[6] = MAV_COMPID;
    /* msgid = 0 → bytes 7,8,9 stay 0 */
    /* payload: custom_mode(4)=0, type(1)=18, autopilot(1)=8, base_mode(1)=0,
     *          system_status(1)=4, mavlink_version(1)=3 */
    f[14] = 18u;   /* type           */
    f[15] = 8u;    /* autopilot      */
    f[17] = 4u;    /* system_status  */
    f[18] = 3u;    /* mavlink_version */
    uint16_t crc = 0xFFFFu;
    for (int i = 1; i <= 9 + PL; i++) crc = crc_accumulate(f[i], crc);
    crc = crc_accumulate(50u, crc);   /* CRC_EXTRA for HEARTBEAT */
    f[19] = (uint8_t)(crc & 0xFFu);
    f[20] = (uint8_t)(crc >> 8u);
    uart_write_blocking(MAV_UART, f, TOTAL);
}

void mavlink_init(void)
{
    uart_init(MAV_UART, MAV_BAUD);
    gpio_set_function(MAV_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MAV_RX_PIN, GPIO_FUNC_UART);
}

void mavlink_send_odometry(uint64_t usec, float x, float y, float z,
                           const float q[4], float pos_var, float yaw_var)
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

    /* Quaternion [w, x, y, z] from caller (yaw-only or identity) */
    memcpy(&frame[30], q, 16u);             /* q[4] */

    /* vx/vy/vz = NaN (position-only; AP handles this gracefully) */
    memcpy(&frame[46], k_nan, 4u);
    memcpy(&frame[50], k_nan, 4u);
    memcpy(&frame[54], k_nan, 4u);
    /* rollspeed/pitchspeed/yawspeed stay 0.0 (memset) */

    /* pose_covariance[21]: upper triangle of 6×6 (pos xyz + att rpy).
     * AP uses indices [0],[6],[11] for x,y,z position variance (posErr),
     * and index [20] for yaw variance.
     * Diagonal mapping: (r,r) → index r*(13-r)/2 for a 6×6 upper triangle.
     *   cov[0]  (0,0) = x var   → frame[70]
     *   cov[6]  (1,1) = y var   → frame[94]
     *   cov[11] (2,2) = z var   → frame[114]
     *   cov[20] (5,5) = yaw var → frame[150]
     */
    write_f32(&frame[70],  pos_var);        /* cov[0]  = x variance  */
    write_f32(&frame[94],  pos_var);        /* cov[6]  = y variance  */
    write_f32(&frame[114], pos_var);        /* cov[11] = z variance  */
    write_f32(&frame[150], yaw_var);        /* cov[20] = yaw variance */

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

void mavlink_rx_update(void)
{
    while (uart_is_readable(MAV_UART)) {
        s_rx_bytes++;
        _rx_feed(uart_getc(MAV_UART));
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

bool mavlink_bs_poses_ready(void)
{
    return s_poses_ready;
}

uint32_t mavlink_param_val_seen(void)
{
    return s_param_val_seen;
}

uint32_t mavlink_lh2_params_received(void)
{
    return (uint32_t)__builtin_popcount(s_received_mask);
}

uint32_t mavlink_rx_bytes(void)
{
    return s_rx_bytes;
}

void mavlink_get_bs_poses(lh2_bs_pose_t poses[NUM_BS])
{
    for (int i = 0; i < NUM_BS; i++) {
        memcpy(poses[i].origin, s_bs_origin[i], 3u * sizeof(float));
        memcpy(poses[i].R,      s_bs_R[i],      9u * sizeof(float));
    }
}
