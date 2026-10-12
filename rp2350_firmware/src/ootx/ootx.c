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
/* A frame takes ~440 rotor turns and with four stations in view many turns get no sweep at
 * all, so frames are rarely received whole. A station repeats the same frame back to back
 * (17 zeros, 1, length, data, CRC), always 43 bytes on LH2: the stream is periodic with
 * CYCLE_BITS turns. Each turn's bit is voted into slot (turn mod CYCLE_BITS) of a ring, a
 * missing turn is just a slot not voted this time, and the frame is read from the ring,
 * starting after its preamble, once every bit is known and the CRC matches. */
#define FRAME_WORDS      ((OOTX_MAX_PAYLOAD + 1) / 2 + 2)          ///< data words + 2 CRC words
#define CYCLE_BITS       (PREAMBLE_ZEROS + 1 + 17 * (1 + FRAME_WORDS))  ///< 443 turns
#define MAX_GAP_TURNS    1000     ///< a longer silence (20 s) may have lost the turn count
#define VOTE_LIMIT       4        ///< saturation: a changed frame takes over after a few cycles
#define RESET_CYCLES     8        ///< complete but CRC still bad after this many: start over

/** Rotor period [us] per physical station (DotBots LH_PERIODS / 48 MHz). */
static const double PERIOD_US[16] = {
    959000 / 48.0, 957000 / 48.0, 953000 / 48.0, 949000 / 48.0, 947000 / 48.0, 943000 / 48.0,
    941000 / 48.0, 939000 / 48.0, 937000 / 48.0, 929000 / 48.0, 919000 / 48.0, 911000 / 48.0,
    907000 / 48.0, 901000 / 48.0, 893000 / 48.0, 887000 / 48.0,
};

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
    /* bit votes per slot of the cycle: > 0 means 1, < 0 means 0, 0 unknown */
    int8_t   vote[CYCLE_BITS];
    uint16_t slot;             ///< slot of the last turn
    uint16_t cycle_turns;      ///< turns since the last decode attempt
    uint8_t  tries;            ///< complete cycles whose CRC failed since the last reset
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
// Ring of voted bits -> frame
// ---------------------------------------------------------------------------

static int8_t _bit(const station_t *st, int i)
{
    return st->vote[((i % CYCLE_BITS) + CYCLE_BITS) % CYCLE_BITS];
}

/** 16 data bits starting at ring index @p at, most significant first; false while one is unknown. */
static bool _word(const station_t *st, int at, uint16_t *word)
{
    uint16_t v = 0;
    for (int b = 0; b < 16; b++) {
        int8_t x = _bit(st, at + b);
        if (x == 0) return false;
        v = (uint16_t)((v << 1) | (x > 0));
    }
    *word = v;
    return true;
}

static void _start_over(station_t *st)
{
    memset(st->vote, 0, sizeof st->vote);
    st->tries = 0;
}

/** Once per cycle: find the preamble in the ring and read the frame after it. */
static void _try_decode(station_t *st)
{
    int known = 0;
    for (int i = 0; i < CYCLE_BITS; i++) known += st->vote[i] != 0;
    st->out.coverage = (uint8_t)(known * 100 / CYCLE_BITS);

    int start = -1;                                  /* index of the preamble's final 1 */
    for (int p = 0; p < CYCLE_BITS && start < 0; p++) {
        if (st->vote[p] <= 0) continue;
        int z = 1;
        while (z <= PREAMBLE_ZEROS && _bit(st, p - z) < 0) z++;
        if (z > PREAMBLE_ZEROS) start = p;
    }
    if (start < 0) return;                           /* preamble not complete yet */

    uint16_t w;
    if (!_word(st, start + 1, &w)) return;
    const uint16_t length = (uint16_t)(((w & 0xFFu) << 8) | (w >> 8));  /* byte-swapped */
    if (length == 0 || length > OOTX_MAX_PAYLOAD) {
        if (++st->tries >= RESET_CYCLES) _start_over(st);
        return;
    }
    uint8_t buf[OOTX_MAX_PAYLOAD + 1 + 4];
    const int words = (length + 1) / 2 + 2;          /* data words, then CRC0, CRC1 */
    for (int k = 0; k < words; k++) {
        if (!_word(st, start + 1 + 17 * (1 + k), &w)) return;  /* holes left: next cycle */
        buf[2 * k]     = (uint8_t)(w >> 8);          /* received byte order */
        buf[2 * k + 1] = (uint8_t)(w & 0xFFu);
    }
    const uint8_t *c = &buf[words * 2 - 4];
    uint32_t crc = (uint32_t)c[0] | ((uint32_t)c[1] << 8) | ((uint32_t)c[2] << 16) | ((uint32_t)c[3] << 24);
    if (_crc32(buf, length) != crc) {
        if (++st->tries >= RESET_CYCLES) _start_over(st);
        return;
    }
    st->tries = 0;
    bool changed = !st->out.valid || st->out.payload_len != length
                   || memcmp(st->out.payload, buf, length) != 0;
    memcpy(st->out.payload, buf, length);
    st->out.payload_len = (uint8_t)length;
    st->out.valid = true;
    st->out.frames_ok++;
    if (changed) st->updated = true;
}

/** One rotor turn's bit: 0, 1, or -1 when the turn had no sweep or a tied vote. */
static void _push_bit(station_t *st, int bit)
{
    st->slot = (uint16_t)((st->slot + 1) % CYCLE_BITS);
    if (bit < 0) {
        st->out.missed++;
    } else {
        st->out.bits++;
        int8_t *v = &st->vote[st->slot];
        if (bit && *v < VOTE_LIMIT) (*v)++;
        else if (!bit && *v > -VOTE_LIMIT) (*v)--;
        if (*v == 0) *v = bit ? 1 : -1;              /* a tie keeps the newest bit, not a hole */
    }
    if (++st->cycle_turns >= CYCLE_BITS) {
        st->cycle_turns = 0;
        _try_decode(st);
    }
}

// ---------------------------------------------------------------------------
// Rotor turns -> bits
// ---------------------------------------------------------------------------

/** Vote the oldest open turn into a bit; the turns skipped since the previous one are holes. */
static void _close_oldest(station_t *st, double period)
{
    turn_t turn = st->open[0];
    st->n_open--;
    memmove(&st->open[0], &st->open[1], (size_t)st->n_open * sizeof(turn_t));
    long step = st->have_closed ? lround((turn.start_us - st->closed_start_us) / period) : 1;
    if (step < 1) return;                            /* not a new turn: timing glitch, ignore */
    st->have_closed = true;
    st->closed_start_us = turn.start_us;
    if (step > MAX_GAP_TURNS) {                      /* the ring's alignment is no longer sure */
        _start_over(st);
        st->out.missed += (uint32_t)(step - 1);
    } else {
        for (long k = 1; k < step; k++) _push_bit(st, -1);
    }
    _push_bit(st, turn.votes[0] == turn.votes[1] ? -1 : turn.votes[1] > turn.votes[0]);
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
