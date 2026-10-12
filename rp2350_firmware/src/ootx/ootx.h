/**
 * @file   ootx.h
 * @brief  On-board decoding of the base stations' factory calibration (OOTX).
 *
 * Every LH2 base station broadcasts its factory calibration one bit per rotor turn: the
 * bit is which polynomial of its pair carried the sweeps of that turn. Like the Crazyflie
 * (pulse_processor_v2.c + ootx_decoder.c), the drone decodes it by itself: no USB cable,
 * no firmware rebuild when a station is added or replaced. main.c applies each decoded
 * station's correction at run time and forwards the frame to the ground station.
 *
 * Same algorithm as utils/calibration/lh2_ootx.py (which decodes logged "O," lines):
 *   rotor turn start = capture time - lfsr / 6 MHz; the hits of one turn vote the bit;
 *   frame = 17 zeros + 1, 16-bit length (byte-swapped), 16-bit words each followed by a
 *   stuffing 1, CRC32 of the payload. The station repeats the frame back to back, so each
 *   turn is voted into slot (turn mod 443) of a ring and missing turns fill in on later
 *   repetitions: decoding works with many sweeps lost (four stations in view).
 */

#ifndef OOTX_H
#define OOTX_H

#include <stdbool.h>
#include <stdint.h>

#include "../angle_decoder/angle_decoder.h"   /* NUM_BS, lh2_plane_cal_t */

#define OOTX_MAX_PAYLOAD 43   ///< bytes (LH2 frame: 41)

/** What one station's OOTX stream has produced so far. */
typedef struct {
    uint32_t bits;                          ///< OOTX bits received (rotor turns)
    uint32_t missed;                        ///< rotor turns without a usable bit (holes)
    uint16_t frames_ok;                     ///< frames with a valid CRC
    uint8_t  coverage;                      ///< % of the cycle's bits known so far
    bool     valid;                         ///< payload holds a decoded frame
    uint8_t  payload_len;
    uint8_t  payload[OOTX_MAX_PAYLOAD];     ///< latest valid frame (struct ootxDataFrame_s, little-endian)
} ootx_station_t;

/**
 * @brief  Feed one decoded sweep (call for every sweep of a known station).
 * @param bs        base-station index [0, NUM_BS)
 * @param poly      LFSR polynomial of the sweep (its parity is the OOTX bit)
 * @param lfsr      LFSR position of the sweep (6 MHz counts since the rotor turn started)
 * @param t_us      capture time of the sweep [us]
 */
void ootx_add_hit(int bs, uint8_t poly, uint32_t lfsr, uint64_t t_us);

/** Decoder output for station @p bs (NULL if out of range). */
const ootx_station_t *ootx_station(int bs);

/**
 * @brief  True once per newly decoded frame whose content changed: fills the two light
 *         planes' correction {phase, tilt, gibmag, gibphase} [rad] and the station uid.
 */
bool ootx_take_update(int bs, lh2_plane_cal_t planes[2], uint32_t *uid);

#endif /* OOTX_H */
