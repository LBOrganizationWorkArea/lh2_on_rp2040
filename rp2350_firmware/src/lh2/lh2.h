#ifndef __LH2_H_
#define __LH2_H_

/**
 * @defgroup    bsp_lh2 LightHouse 2 support
 * @ingroup     bsp
 * @brief       Control the LH2 sensor
 *
 * @{
 * @file
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022-present
 * @}
 */

#include <stdint.h>
#include <stdbool.h>

// Definition needed for test builds, who don't have access to the full pico-SDK,
// and thus, have no idea what an absolute_time_t is.
#ifndef _PICO_TYPES_H
typedef uint64_t absolute_time_t;
#endif

//=========================== defines ==========================================

#define LH2_BASESTATION_COUNT          16                         ///< Number of supported concurrent basestations
#define LH2_POLYNOMIAL_COUNT           LH2_BASESTATION_COUNT * 2  ///< Number of supported LFSR polynomials, two per basestation
#define LH2_SWEEP_COUNT                2                          ///< Number of laser sweeps per basestations rotation

/// LH2 data ready buffer state
typedef enum {
    DB_LH2_NO_NEW_DATA,               ///< The data occupying this spot of the buffer has already been sent.
    DB_LH2_RAW_DATA_AVAILABLE,        ///< The data occupying this spot of the buffer is new and ready to send.
    DB_LH2_PROCESSED_DATA_AVAILABLE,  ///< The data occupying this spot of the buffer is new and ready to send.
} db_lh2_data_ready_state_t;

/// LH2 raw data
typedef struct __attribute__((packed)) {
    uint64_t bits_sweep;           ///< bits sweep is the result of the demodulation, sweep_N indicates which SPI transfer those bits are associated with
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    int8_t   bit_offset;           ///< bit_offset indicates an offset between the start of the packet, as indicated by envelope dropping, and the 17-bit sequence that is verified to be in a known LFSR sequence
} db_lh2_raw_data_t;

/// LH2 raw data location
typedef struct __attribute__((packed)) {
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    uint32_t lfsr_location;        ///< LFSR location is the position in a given polynomial's LFSR that the decoded data is, initialize to error state
} db_lh2_location_t;

/// LH2 instance (one row per laser sweep, and one column per basestation)
typedef struct {
    db_lh2_raw_data_t         raw_data[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];    ///< raw data decoded from the lighthouse
    db_lh2_location_t         locations[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];   ///< buffer holding the computed locations
    absolute_time_t           timestamps[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];  ///< timestamp of when the raw data was received
    db_lh2_data_ready_state_t data_ready[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];  ///< Is the data in the buffer ready to send over radio, or has it already been sent ?
    uint8_t                  *spi_ring_buffer_count_ptr;                           ///< pointer to the SPI rung buffer packet count, so the user application can read how many spi captures are waiting to be processed.
    uint8_t                   sensor;                                              ///< Which TS4231 sensor is associated with this data structure (valid values [0-3]).
} db_lh2_t;

/// Receive counters of one sensor since boot (diagnostics: light that arrives vs. light that is decoded)
typedef struct {
    uint32_t captures;   ///< light pulses captured (envelope fell, 16 us sampled)
    uint32_t overflows;  ///< captures lost because core 1 did not empty the ring buffer in time
    uint32_t no_poly;    ///< captures that matched no polynomial of a known station (noise, too few clean bits)
    uint32_t lfsr_fail;  ///< polynomial found but the LFSR position search failed
    uint32_t decoded;    ///< sweeps decoded (polynomial + LFSR position)
    uint32_t corrected;  ///< ... of which accepted with 1-2 bit errors in the check bits
} db_lh2_stats_t;

//=========================== public ===========================================

/**
 * @brief Initialize LH2
 *
 * @param[in]   lh2     pointer to the lh2 instance
 * @param[in]   sensor  which TS4231 sensor is associated with this data structure (valid values [0-3])
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void db_lh2_init(db_lh2_t *lh2, uint8_t sensor, const uint8_t gpio_d, const uint8_t gpio_e);

/**
 * @brief Compute the location based on raw data coming from the lighthouse
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_process_location(db_lh2_t *lh2);

/**
 * @brief Only look for these polynomials (bit n = polynomial n; default: all 32).
 *        Fewer candidates = less work per capture and fewer chances for noise to match.
 *
 * @param[in]   mask    bit mask of the polynomials to search
 */
void db_lh2_set_polynomial_mask(uint32_t mask);

/**
 * @brief Copy the receive counters of a sensor (see db_lh2_stats_t)
 */
void db_lh2_get_stats(uint8_t sensor, db_lh2_stats_t *out);

/**
 * @brief Configuration word the TS4231 answered at init (0x392B = configured as asked)
 */
uint16_t db_lh2_config_readback(uint8_t sensor);

/**
 * @brief Start the LH2 frame acquisition
 *
 */
void db_lh2_start(void);

/**
 * @brief Stop the LH2 frame acquisition
 *
 */
void db_lh2_stop(void);

#endif /* __LH2_H_ */
