/**
 * @file   ootx.c
 * @brief  On-board OOTX decoding — see ootx.h.
 */

#include "ootx.h"

#include <math.h>
#include <string.h>

#define LFSR_CLOCK_MHZ   6.0      ///< LFSR counts per microsecond
#define TURN_TOLERANCE   0.3      ///< hits within 30 % of a turn belong to the same turn
#define PREAMBLE_ZEROS   17
/* The sensors' sweeps reach core 0 out of order: in a real log a third of them arrive one to
 * three rotor turns after newer ones. Turns stay open in a small window and are voted only
 * once they are TURN_LAG turns older than the newest one seen. */
#define TURN_LAG         5
#define OPEN_TURNS       12

/** Rotor period [us] per physical station (DotBots LH_PERIODS / 48 MHz). */
static const double PERIOD_US[16] = {
    959000 / 48.0, 957000 / 48.0, 953000 / 48.0, 949000 / 48.0, 947000 / 48.0, 943000 / 48.0,
    941000 / 48.0, 939000 / 48.0, 937000 / 48.0, 929000 / 48.0, 919000 / 48.0, 911000 / 48.0,
    907000 / 48.0, 901000 / 48.0, 893000 / 48.0, 887000 / 48.0,
};

typedef enum { SEARCH, LENGTH, DATA } frame_state_t;

typedef struct {
    double   start_us;     ///< turn start (first hit of the turn)
    uint16_t votes[2];     ///< hits with polynomial parity 0 / 1
} turn_t;

typedef struct {
    /* rotor turns still collecting votes, oldest first */
    turn_t   open[OPEN_TURNS];
    int      n_open;
    bool     have_closed;
    double   closed_start_us;  ///< start of the last turn turned into a bit
    /* frame decoder */
    frame_state_t state;
    uint32_t zeros;
    uint16_t word;
    uint8_t  word_bits;      ///< bits of `word` received (16 data + 1 stuffing)
    uint16_t length;
    uint16_t words_needed;
    uint16_t words_done;
    uint8_t  buf[OOTX_MAX_PAYLOAD + 1 + 4];
    /* output */
    ootx_station_t out;
    bool     updated;
} station_t;

static station_t s_st[NUM_BS];

// ---------------------------------------------------------------------------
// CRC32 (zlib / IEEE 802.3, reflected 0xEDB88320)
// ---------------------------------------------------------------------------

static uint32_t _crc32(const uint8_t *data, int len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

// ---------------------------------------------------------------------------
// Frame decoder: one bit at a time
// ---------------------------------------------------------------------------

static void _restart(station_t *st)
{
    st->state = SEARCH;
    st->zeros = 0;
}

static void _frame_complete(station_t *st)
{
    const int data_bytes = st->words_needed * 2 - 4;
    const uint8_t *crc_bytes = &st->buf[data_bytes];
    uint32_t crc = (uint32_t)crc_bytes[0] | ((uint32_t)crc_bytes[1] << 8)
                 | ((uint32_t)crc_bytes[2] << 16) | ((uint32_t)crc_bytes[3] << 24);
    if (_crc32(st->buf, st->length) == crc) {
        bool changed = !st->out.valid || st->out.payload_len != st->length
                       || memcmp(st->out.payload, st->buf, st->length) != 0;
        memcpy(st->out.payload, st->buf, st->length);
        st->out.payload_len = (uint8_t)st->length;
        st->out.valid = true;
        st->out.frames_ok++;
        if (changed) st->updated = true;
    }
    _restart(st);
}

static void _push_bit(station_t *st, int bit)
{
    st->out.bits++;
    if (st->state == SEARCH) {
        if (bit == 1 && st->zeros >= PREAMBLE_ZEROS) {
            st->state = LENGTH;
            st->word = 0;
            st->word_bits = 0;
        }
        st->zeros = bit ? 0 : st->zeros + 1;
        return;
    }
    if (st->word_bits < 16) {  /* data bits, most significant first */
        st->word = (uint16_t)((st->word << 1) | (bit & 1));
        st->word_bits++;
        return;
    }
    /* 17th bit: stuffing, must be 1 */
    st->word_bits = 0;
    if (bit != 1) { _restart(st); return; }
    if (st->state == LENGTH) {
        st->length = (uint16_t)(((st->word & 0xFFu) << 8) | (st->word >> 8));  /* byte-swapped */
        if (st->length > OOTX_MAX_PAYLOAD) { _restart(st); return; }
        st->words_needed = (uint16_t)((st->length + 1) / 2 + 2);  /* data words, then CRC0, CRC1 */
        st->words_done = 0;
        st->state = DATA;
    } else {
        st->buf[st->words_done * 2]     = (uint8_t)(st->word >> 8);   /* received byte order */
        st->buf[st->words_done * 2 + 1] = (uint8_t)(st->word & 0xFFu);
        if (++st->words_done == st->words_needed) _frame_complete(st);
    }
    st->word = 0;
}

// ---------------------------------------------------------------------------
// Rotor turns -> bits
// ---------------------------------------------------------------------------

/** Vote the oldest open turn into a bit (a missing or tied turn restarts the frame search). */
static void _close_oldest(station_t *st, double period)
{
    turn_t turn = st->open[0];
    st->n_open--;
    memmove(&st->open[0], &st->open[1], (size_t)st->n_open * sizeof(turn_t));
    if (st->have_closed && lround((turn.start_us - st->closed_start_us) / period) != 1) _restart(st);
    st->have_closed = true;
    st->closed_start_us = turn.start_us;
    if (turn.votes[0] == turn.votes[1]) _restart(st);
    else _push_bit(st, turn.votes[1] > turn.votes[0] ? 1 : 0);
}

void ootx_add_hit(int bs, uint8_t poly, uint32_t lfsr, uint64_t t_us)
{
    if (bs < 0 || bs >= NUM_BS || (poly >> 1) >= 16) return;
    station_t *st = &s_st[bs];
    const double period = PERIOD_US[poly >> 1];
    const double start = (double)t_us - (double)lfsr / LFSR_CLOCK_MHZ;

    if (st->have_closed && start < st->closed_start_us + TURN_TOLERANCE * period) return;  /* too late */
    for (int i = 0; i < st->n_open; i++) {           /* a turn already open: one more vote */
        if (fabs(start - st->open[i].start_us) < TURN_TOLERANCE * period) {
            st->open[i].votes[poly & 1u]++;
            return;
        }
    }
    if (st->n_open == OPEN_TURNS) _close_oldest(st, period);
    int at = st->n_open;                             /* insert in time order */
    while (at > 0 && st->open[at - 1].start_us > start) at--;
    memmove(&st->open[at + 1], &st->open[at], (size_t)(st->n_open - at) * sizeof(turn_t));
    st->open[at] = (turn_t){ .start_us = start, .votes = { 0, 0 } };
    st->open[at].votes[poly & 1u] = 1;
    st->n_open++;
    /* close the turns that no late hit can reach any more */
    const double newest = st->open[st->n_open - 1].start_us;
    while (st->n_open > 0 && st->open[0].start_us < newest - TURN_LAG * period) _close_oldest(st, period);
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

const ootx_station_t *ootx_station(int bs)
{
    return (bs >= 0 && bs < NUM_BS) ? &s_st[bs].out : 0;
}

/** IEEE 754 half -> float. */
static float _half(const uint8_t *p)
{
    uint16_t h = (uint16_t)(p[0] | (p[1] << 8));
    uint32_t sign = (uint32_t)(h & 0x8000u) << 16, exp = (h >> 10) & 0x1Fu, man = h & 0x3FFu, bits;
    if (exp == 0) {
        if (man == 0) bits = sign;
        else {  /* subnormal: normalise */
            exp = 127 - 15 + 1;
            while ((man & 0x400u) == 0) { man <<= 1; exp--; }
            bits = sign | (exp << 23) | ((man & 0x3FFu) << 13);
        }
    } else if (exp == 31) {
        bits = sign | 0x7F800000u | (man << 13);
    } else {
        bits = sign | ((exp - 15 + 127) << 23) | (man << 13);
    }
    float f;
    memcpy(&f, &bits, sizeof f);
    return f;
}

bool ootx_take_update(int bs, lh2_plane_cal_t planes[2], uint32_t *uid)
{
    if (bs < 0 || bs >= NUM_BS || !s_st[bs].updated) return false;
    station_t *st = &s_st[bs];
    st->updated = false;
    const uint8_t *p = st->out.payload;
    if (st->out.payload_len < 33) return false;   /* too short for the fields used below */
    /* struct ootxDataFrame_s offsets (see lh2_ootx.py FRAME_FIELDS) */
    *uid = (uint32_t)p[2] | ((uint32_t)p[3] << 8) | ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24);
    planes[0] = (lh2_plane_cal_t){ .phase = _half(&p[6]),  .tilt = _half(&p[10]),
                                   .gibmag = _half(&p[27]), .gibphase = _half(&p[23]) };
    planes[1] = (lh2_plane_cal_t){ .phase = _half(&p[8]),  .tilt = _half(&p[12]),
                                   .gibmag = _half(&p[29]), .gibphase = _half(&p[25]) };
    return true;
}
