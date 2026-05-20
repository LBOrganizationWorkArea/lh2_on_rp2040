/**
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief LH2 TS4231 example for RP2040, sending data through USB serial.
 */
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "lh2/lh2.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 10
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)
#define LH2_1_DATA_PIN 12
#define LH2_1_ENV_PIN  (LH2_1_DATA_PIN + 1)
#define LH2_2_DATA_PIN 18
#define LH2_2_ENV_PIN  (LH2_2_DATA_PIN + 1)
#define LH2_3_DATA_PIN 20
#define LH2_3_ENV_PIN  (LH2_3_DATA_PIN + 1)

#define LH2_SENSOR_COUNT 4
#define LH2_POLY_SLOT_COUNT 2
#define LH2_MIN_BLOCK_SENSORS 2
#define LH2_MIN_PAIR_SENSORS 2
#define LH2_DEBUG_PERMISSIVE_PAIRING 1
#define LH2_CANDIDATES_PER_SENSOR 8
#define LH2_PREVIOUS_BLOCK_CANDIDATES 12
#if LH2_DEBUG_PERMISSIVE_PAIRING
#define LH2_BLOCK_WINDOW_US 30000
#define LH2_BLOCK_TIMESTAMP0_TOLERANCE_TICKS 30000
#define LH2_PAIR_OFFSET_TOLERANCE_TICKS 60000
#define LH2_PAIR_MAX_AGE_US 400000
#else
#define LH2_BLOCK_WINDOW_US 12000
#define LH2_BLOCK_TIMESTAMP0_TOLERANCE_TICKS 12000
#define LH2_PAIR_OFFSET_TOLERANCE_TICKS 60000
#define LH2_PAIR_MAX_AGE_US 250000
#endif
#define LH2_TIMESTAMP_TICKS_PER_US 24
#define LH2_OFFSET_TICKS_PER_LFSR 4
#define LH2_PAIR_TIMESTAMP0_TOLERANCE_TICKS 30000

#define LH2_OUTPUT_LEGACY_FRAMES 0
#define LH2_OUTPUT_EXTENDED_FRAMES 1
#define LH2_OUTPUT_BLOCKS 0
#define LH2_OUTPUT_PAIRS 1
#define LH2_OUTPUT_HEARTBEAT 1
#define LH2_OUTPUT_BOOT_BANNER 0
#define HEARTBEAT_INTERVAL_US 1000000u
#define FIRMWARE_TAG "lh2p-v10-recover"
#define LH2_PAIR_OUTPUT_MIN_INTERVAL_US 25000u
#define LH2_PAIRING_STALE_RESET_US 3000000u

//=========================== variables ========================================

db_lh2_t        _lh2_0;
db_lh2_t        _lh2_1;
db_lh2_t        _lh2_2;
db_lh2_t _lh2_3;
bool     clk_conf_OK;

uint8_t sensor_0 = 0;
uint8_t sensor_1 = 1;
uint8_t sensor_2 = 2;
uint8_t sensor_3 = 3;

typedef struct {
    bool     valid;
    uint64_t time_us;
    uint32_t ts_24;
    uint32_t offset_24;
    uint32_t lfsr_location;
    uint32_t lfsr_bits;
    uint8_t  sensor;
    uint8_t  sweep;
    uint8_t  bs;
    uint8_t  polynomial;
    int8_t   bit_offset;
} lh2_frame_t;

typedef struct {
    bool        active;
    uint64_t    start_time_us;
    uint32_t    id;
    uint8_t     bs;
    uint8_t     sweep;
    uint8_t     polynomial;
    uint8_t     count;
    uint8_t     candidate_count[LH2_SENSOR_COUNT];
    lh2_frame_t candidates[LH2_SENSOR_COUNT][LH2_CANDIDATES_PER_SENSOR];
} lh2_block_builder_t;

typedef struct {
    bool     valid;
    uint64_t time_us;
    uint32_t id;
    uint8_t  bs;
    uint8_t  sweep;
    uint8_t  polynomial;
    uint8_t  base_sensor;
    uint8_t  sensor_mask;
    uint8_t  sensor_count;
    uint32_t timestamp0_24;
    uint32_t offsets_24[LH2_SENSOR_COUNT];
    uint32_t lfsr_locations[LH2_SENSOR_COUNT];
} lh2_block_t;

lh2_block_builder_t block_builders[LH2_BASESTATION_COUNT][LH2_POLY_SLOT_COUNT] = { 0 };
lh2_block_t         previous_blocks[LH2_BASESTATION_COUNT][LH2_POLY_SLOT_COUNT][LH2_PREVIOUS_BLOCK_CANDIDATES] = { 0 };
uint32_t            next_block_id = 1;
uint32_t            telemetry_frame_count = 0;
uint32_t            telemetry_pair_count = 0;
uint64_t            telemetry_last_frame_us = 0;
uint64_t            telemetry_last_pair_us = 0;
uint32_t            telemetry_poly_reject_count = 0;
uint32_t            telemetry_builder_timeout_count = 0;
uint32_t            telemetry_block_attempt_count = 0;
uint32_t            telemetry_block_count = 0;
uint32_t            telemetry_block_reject_count = 0;
uint32_t            telemetry_pair_candidate_count = 0;
uint32_t            telemetry_pair_offset_reject_count = 0;
uint32_t            telemetry_pair_age_reject_count = 0;
uint32_t            telemetry_pair_timestamp_reject_count = 0;
uint64_t            last_lh2p_output_us[LH2_BASESTATION_COUNT] = { 0 };

//=========================== prototypes =======================================

void core1_entry(void);

//=========================== helpers ==========================================

static uint32_t timestamp_us_to_24mhz(uint64_t time_us) {
    return (uint32_t)((time_us * LH2_TIMESTAMP_TICKS_PER_US) & 0x00FFFFFFu);
}

static const uint32_t cycle_periods_24mhz[LH2_BASESTATION_COUNT] = {
    959000u / 2u, 957000u / 2u, 953000u / 2u, 949000u / 2u,
    947000u / 2u, 943000u / 2u, 941000u / 2u, 939000u / 2u,
    937000u / 2u, 929000u / 2u, 919000u / 2u, 911000u / 2u,
    907000u / 2u, 901000u / 2u, 893000u / 2u, 887000u / 2u,
};

static uint32_t ts_diff_24(uint32_t first, uint32_t second) {
    return (first - second) & 0x00FFFFFFu;
}

static uint32_t ts_abs_diff_24(uint32_t first, uint32_t second) {
    uint32_t diff = ts_diff_24(first, second);
    if (diff > 0x00800000u) {
        diff = 0x01000000u - diff;
    }
    return diff;
}

static uint32_t frame_timestamp0_24(const lh2_frame_t *frame) {
    return ts_diff_24(frame->ts_24, frame->offset_24);
}

static uint32_t period_abs_diff(uint32_t first, uint32_t second, uint32_t period) {
    uint32_t a = first % period;
    uint32_t b = second % period;
    uint32_t diff = (a > b) ? (a - b) : (b - a);
    uint32_t wrapped = period - diff;
    return diff < wrapped ? diff : wrapped;
}

static uint8_t count_bits_u8(uint8_t value) {
    uint8_t count = 0;
    while (value != 0u) {
        count += value & 1u;
        value >>= 1u;
    }
    return count;
}

static uint32_t frame_score_against_builder(const lh2_block_builder_t *builder,
                                            uint8_t sensor,
                                            const lh2_frame_t *frame) {
    uint32_t frame_timestamp0 = frame_timestamp0_24(frame);
    uint32_t period = cycle_periods_24mhz[frame->bs];
    uint32_t score = 0;

    for (uint8_t other_sensor = 0; other_sensor < LH2_SENSOR_COUNT; other_sensor++) {
        if (other_sensor == sensor || builder->candidate_count[other_sensor] == 0) {
            continue;
        }

        uint32_t best_diff = UINT32_MAX;
        for (uint8_t candidate = 0; candidate < builder->candidate_count[other_sensor]; candidate++) {
            uint32_t other_timestamp0 = frame_timestamp0_24(&builder->candidates[other_sensor][candidate]);
            uint32_t diff = period_abs_diff(frame_timestamp0, other_timestamp0, period);
            if (diff < best_diff) {
                best_diff = diff;
            }
        }
        score += best_diff;
    }

    return score;
}

static bool block_offsets_consistent(const lh2_block_t *first, const lh2_block_t *second) {
    uint8_t common_mask = first->sensor_mask & second->sensor_mask;
    if (count_bits_u8(common_mask) < LH2_MIN_PAIR_SENSORS) {
        return false;
    }

    uint32_t period = cycle_periods_24mhz[first->bs];
    for (uint8_t sensor = 0; sensor < LH2_SENSOR_COUNT; sensor++) {
        if ((common_mask & (1u << sensor)) == 0u) {
            continue;
        }
        uint32_t diff = period_abs_diff(first->offsets_24[sensor], second->offsets_24[sensor], period);
        if (diff > LH2_PAIR_OFFSET_TOLERANCE_TICKS) {
            return false;
        }
    }

    return true;
}

static uint32_t block_offset_score(const lh2_block_t *first, const lh2_block_t *second) {
    uint8_t common_mask = first->sensor_mask & second->sensor_mask;
    uint32_t period = cycle_periods_24mhz[first->bs];
    uint32_t score = 0;
    for (uint8_t sensor = 0; sensor < LH2_SENSOR_COUNT; sensor++) {
        if ((common_mask & (1u << sensor)) == 0u) {
            continue;
        }
        score += period_abs_diff(first->offsets_24[sensor], second->offsets_24[sensor], period);
    }

    return score;
}

static uint64_t block_age_us(const lh2_block_t *first, const lh2_block_t *second) {
    return first->time_us > second->time_us ? first->time_us - second->time_us : second->time_us - first->time_us;
}

static void serial_send_lh2_line(uint64_t time_us,
                                 uint8_t sensor_id,
                                 uint8_t sweep,
                                 uint8_t bs,
                                 uint8_t polynomial,
                                 uint32_t lfsr_location) {
    printf("LH2,%llu,%u,%u,%u,%u,%lu\r\n",
           (unsigned long long)time_us,
           sensor_id,
           sweep,
           bs,
           polynomial,
           (unsigned long)lfsr_location);
}

static void serial_send_lh2_raw_line(const lh2_frame_t *frame) {
    serial_send_lh2_line(frame->time_us,
                         frame->sensor,
                         frame->polynomial & 1u,
                         frame->bs,
                         frame->polynomial,
                         frame->lfsr_location);
}

static void serial_send_lh2_extended_line(const lh2_frame_t *frame) {
    printf("LH2R,%llu,%lu,%u,%u,%u,%u,%ld,%lu,%lu,%lu,%lu\r\n",
           (unsigned long long)frame->time_us,
           (unsigned long)frame->ts_24,
           frame->sensor,
           frame->sweep,
           frame->bs,
           frame->polynomial,
           (long)frame->bit_offset,
           (unsigned long)frame->lfsr_bits,
           (unsigned long)frame->lfsr_location,
           (unsigned long)frame->offset_24,
           (unsigned long)ts_diff_24(frame->ts_24, frame->offset_24));
}

static void serial_send_lh2_block_line(const lh2_block_t *block) {
    printf("LH2B,%lu,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
           (unsigned long)block->id,
           block->bs,
           block->sweep,
           block->polynomial,
           block->base_sensor,
           block->sensor_mask,
           block->sensor_count,
           (unsigned long)block->timestamp0_24,
           (unsigned long)block->offsets_24[0],
           (unsigned long)block->offsets_24[1],
           (unsigned long)block->offsets_24[2],
           (unsigned long)block->offsets_24[3],
           (unsigned long)block->lfsr_locations[0],
           (unsigned long)block->lfsr_locations[1],
           (unsigned long)block->lfsr_locations[2],
           (unsigned long)block->lfsr_locations[3]);
}

static void serial_send_lh2_pair_line(const lh2_block_t *previous, const lh2_block_t *latest, uint32_t timestamp0_delta) {
    uint8_t common_mask = previous->sensor_mask & latest->sensor_mask;
    uint32_t previous_offsets[LH2_SENSOR_COUNT] = { 0 };
    uint32_t latest_offsets[LH2_SENSOR_COUNT] = { 0 };

    for (uint8_t sensor = 0; sensor < LH2_SENSOR_COUNT; sensor++) {
        if ((common_mask & (1u << sensor)) != 0u) {
            previous_offsets[sensor] = previous->offsets_24[sensor];
            latest_offsets[sensor] = latest->offsets_24[sensor];
        }
    }

    printf("LH2P;%u;%u;%u;%u;%u;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu\r\n",
           latest->bs,
           previous->sweep,
           latest->sweep,
           previous->polynomial,
           latest->polynomial,
           (unsigned long)previous->id,
           (unsigned long)latest->id,
           (unsigned long)timestamp0_delta,
           (unsigned long)previous_offsets[0],
           (unsigned long)latest_offsets[0],
           (unsigned long)previous_offsets[1],
           (unsigned long)latest_offsets[1],
           (unsigned long)previous_offsets[2],
           (unsigned long)latest_offsets[2],
           (unsigned long)previous_offsets[3],
           (unsigned long)latest_offsets[3]);
}

#if LH2_OUTPUT_HEARTBEAT
static void serial_send_heartbeat(uint32_t seq0, uint32_t seq1, uint32_t seq2, uint32_t seq3) {
    uint64_t now_us = to_us_since_boot(get_absolute_time());
    uint64_t age_frame_ms = telemetry_last_frame_us == 0 ? 0 : (now_us - telemetry_last_frame_us) / 1000u;
    uint64_t age_pair_ms = telemetry_last_pair_us == 0 ? 0 : (now_us - telemetry_last_pair_us) / 1000u;

    printf("HB;%llu;%lu;%lu;%llu;%llu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu\r\n",
           (unsigned long long)(now_us / 1000u),
           (unsigned long)telemetry_frame_count,
           (unsigned long)telemetry_pair_count,
           (unsigned long long)age_frame_ms,
           (unsigned long long)age_pair_ms,
           (unsigned long)seq0,
           (unsigned long)seq1,
           (unsigned long)seq2,
           (unsigned long)seq3,
           (unsigned long)telemetry_block_count,
           (unsigned long)telemetry_block_attempt_count,
           (unsigned long)telemetry_block_reject_count,
           (unsigned long)telemetry_builder_timeout_count,
           (unsigned long)telemetry_pair_candidate_count,
           (unsigned long)telemetry_pair_offset_reject_count,
           (unsigned long)telemetry_pair_age_reject_count,
           (unsigned long)telemetry_pair_timestamp_reject_count,
           (unsigned long)telemetry_poly_reject_count);
}
#endif

static bool copy_latest_frame_if_new(db_lh2_t *lh2, uint32_t *last_seq, lh2_frame_t *out) {
    uint32_t seq_before = lh2->latest_frame_seq;
    if (seq_before == *last_seq || !lh2->latest_frame.valid) {
        return false;
    }

    db_lh2_decoded_frame_t frame = lh2->latest_frame;
    uint32_t seq_after = lh2->latest_frame_seq;
    if (seq_before != seq_after) {
        return false;
    }
    *last_seq = seq_after;

    out->valid         = true;
    out->time_us       = to_us_since_boot(frame.timestamp);
    out->ts_24         = timestamp_us_to_24mhz(out->time_us);
    out->offset_24     = (frame.lfsr_location * LH2_OFFSET_TICKS_PER_LFSR) & 0x00FFFFFFu;
    out->lfsr_location = frame.lfsr_location;
    out->lfsr_bits     = frame.lfsr_bits;
    out->sensor        = frame.sensor;
    out->sweep         = frame.sweep;
    out->bs            = frame.basestation;
    out->polynomial    = frame.selected_polynomial;
    out->bit_offset    = frame.bit_offset;

    return true;
}

static bool build_complete_block(lh2_block_builder_t *builder, lh2_block_t *out) {
    if (!builder->active || builder->count < LH2_MIN_BLOCK_SENSORS) {
        return false;
    }

    const lh2_frame_t *selected[LH2_SENSOR_COUNT] = { 0 };
    uint32_t best_score = UINT32_MAX;
    uint32_t best_max_diff = UINT32_MAX;
    uint8_t best_indexes[LH2_SENSOR_COUNT] = { 0 };
    uint32_t period = cycle_periods_24mhz[builder->bs];
    uint8_t sensor_mask = 0;

    for (uint8_t sensor = 0; sensor < LH2_SENSOR_COUNT; sensor++) {
        if (builder->candidate_count[sensor] > 0) {
            sensor_mask |= 1u << sensor;
        }
    }

    uint8_t max_i0 = builder->candidate_count[0] == 0 ? 1 : builder->candidate_count[0];
    uint8_t max_i1 = builder->candidate_count[1] == 0 ? 1 : builder->candidate_count[1];
    uint8_t max_i2 = builder->candidate_count[2] == 0 ? 1 : builder->candidate_count[2];
    uint8_t max_i3 = builder->candidate_count[3] == 0 ? 1 : builder->candidate_count[3];

    for (uint8_t i0 = 0; i0 < max_i0; i0++) {
        for (uint8_t i1 = 0; i1 < max_i1; i1++) {
            for (uint8_t i2 = 0; i2 < max_i2; i2++) {
                for (uint8_t i3 = 0; i3 < max_i3; i3++) {
                    const lh2_frame_t *combo[LH2_SENSOR_COUNT] = {
                        builder->candidate_count[0] == 0 ? NULL : &builder->candidates[0][i0],
                        builder->candidate_count[1] == 0 ? NULL : &builder->candidates[1][i1],
                        builder->candidate_count[2] == 0 ? NULL : &builder->candidates[2][i2],
                        builder->candidate_count[3] == 0 ? NULL : &builder->candidates[3][i3],
                    };
                    uint32_t score = 0;
                    uint32_t max_diff = 0;
                    for (uint8_t a = 0; a < LH2_SENSOR_COUNT; a++) {
                        if (combo[a] == NULL) {
                            continue;
                        }
                        for (uint8_t b = a + 1; b < LH2_SENSOR_COUNT; b++) {
                            if (combo[b] == NULL) {
                                continue;
                            }
                            uint32_t diff = period_abs_diff(frame_timestamp0_24(combo[a]), frame_timestamp0_24(combo[b]), period);
                            score += diff;
                            if (diff > max_diff) {
                                max_diff = diff;
                            }
                        }
                    }

                    if (max_diff < best_max_diff || (max_diff == best_max_diff && score < best_score)) {
                        best_max_diff = max_diff;
                        best_score = score;
                        best_indexes[0] = i0;
                        best_indexes[1] = i1;
                        best_indexes[2] = i2;
                        best_indexes[3] = i3;
                    }
                }
            }
        }
    }

    if (best_max_diff > LH2_BLOCK_TIMESTAMP0_TOLERANCE_TICKS) {
        return false;
    }

    for (uint8_t sensor = 0; sensor < LH2_SENSOR_COUNT; sensor++) {
        if (builder->candidate_count[sensor] > 0) {
            selected[sensor] = &builder->candidates[sensor][best_indexes[sensor]];
        }
    }

    uint8_t base_sensor = 0;
    while (base_sensor < LH2_SENSOR_COUNT && selected[base_sensor] == NULL) {
        base_sensor++;
    }
    if (base_sensor >= LH2_SENSOR_COUNT) {
        return false;
    }
    const lh2_frame_t *base = selected[base_sensor];

    memset(out, 0, sizeof(*out));
    out->valid         = true;
    out->time_us       = base->time_us;
    out->id            = builder->id;
    out->bs            = builder->bs;
    out->sweep         = builder->sweep;
    out->polynomial    = builder->polynomial;
    out->base_sensor   = base_sensor;
    out->sensor_mask   = sensor_mask;
    out->sensor_count  = count_bits_u8(sensor_mask);
    out->timestamp0_24 = ts_diff_24(base->ts_24, base->offset_24);

    for (uint8_t sensor = 0; sensor < LH2_SENSOR_COUNT; sensor++) {
        const lh2_frame_t *frame = selected[sensor];
        if (frame == NULL) {
            out->offsets_24[sensor] = 0;
            out->lfsr_locations[sensor] = 0;
            continue;
        }
        uint32_t timestamp_delta = ts_diff_24(base->ts_24, frame->ts_24);
        out->offsets_24[sensor] = ts_diff_24(base->offset_24, timestamp_delta);
        out->lfsr_locations[sensor] = frame->lfsr_location;
    }

    return true;
}

static void reset_block_builder(lh2_block_builder_t *builder) {
    memset(builder, 0, sizeof(*builder));
}

static void reset_pairing_state(void) {
    memset(block_builders, 0, sizeof(block_builders));
    memset(previous_blocks, 0, sizeof(previous_blocks));
    memset(last_lh2p_output_us, 0, sizeof(last_lh2p_output_us));
}

static void remember_previous_block(const lh2_block_t *block) {
    uint8_t poly_slot = block->polynomial & 1u;
    lh2_block_t *candidates = previous_blocks[block->bs][poly_slot];
    int replace_index = -1;

    for (uint8_t index = 0; index < LH2_PREVIOUS_BLOCK_CANDIDATES; index++) {
        if (candidates[index].valid && block_age_us(&candidates[index], block) > LH2_PAIR_MAX_AGE_US) {
            candidates[index].valid = false;
        }
        if (!candidates[index].valid) {
            candidates[index] = *block;
            return;
        }
        if (block_offsets_consistent(&candidates[index], block)) {
            replace_index = index;
            break;
        }
    }

    if (replace_index < 0) {
        replace_index = 0;
        for (uint8_t index = 1; index < LH2_PREVIOUS_BLOCK_CANDIDATES; index++) {
            if (candidates[index].id < candidates[replace_index].id) {
                replace_index = index;
            }
        }
    }

    candidates[replace_index] = *block;
}

static void add_frame_candidate(lh2_block_builder_t *builder, const lh2_frame_t *frame) {
    uint8_t sensor = frame->sensor;
    uint8_t candidate_count = builder->candidate_count[sensor];

    if (candidate_count == 0) {
        builder->count++;
    }

    if (candidate_count < LH2_CANDIDATES_PER_SENSOR) {
        builder->candidates[sensor][candidate_count] = *frame;
        builder->candidate_count[sensor]++;
        return;
    }

    uint32_t new_score = frame_score_against_builder(builder, sensor, frame);
    uint32_t worst_score = 0;
    uint8_t worst_candidate = 0;
    for (uint8_t candidate = 0; candidate < LH2_CANDIDATES_PER_SENSOR; candidate++) {
        uint32_t score = frame_score_against_builder(builder, sensor, &builder->candidates[sensor][candidate]);
        if (score >= worst_score) {
            worst_score = score;
            worst_candidate = candidate;
        }
    }

    if (new_score < worst_score) {
        builder->candidates[sensor][worst_candidate] = *frame;
    }
}

static void process_completed_block(lh2_block_t *block) {
#if LH2_OUTPUT_BLOCKS
    serial_send_lh2_block_line(block);
#endif

    uint8_t poly_slot = block->polynomial & 1u;
    uint8_t other_slot = poly_slot ^ 1u;
    lh2_block_t *best_other = NULL;
    uint32_t best_timestamp0_delta = UINT32_MAX;
    uint32_t best_score = UINT32_MAX;

    for (uint8_t index = 0; index < LH2_PREVIOUS_BLOCK_CANDIDATES; index++) {
        lh2_block_t *other = &previous_blocks[block->bs][other_slot][index];
        if (!other->valid) {
            continue;
        }
        telemetry_pair_candidate_count++;

        if (block_age_us(other, block) > LH2_PAIR_MAX_AGE_US) {
            telemetry_pair_age_reject_count++;
            other->valid = false;
            continue;
        }

        if (!block_offsets_consistent(other, block)) {
            telemetry_pair_offset_reject_count++;
            continue;
        }

        uint32_t timestamp0_delta = period_abs_diff(other->timestamp0_24,
                                                    block->timestamp0_24,
                                                    cycle_periods_24mhz[block->bs]);
        if (timestamp0_delta > LH2_PAIR_TIMESTAMP0_TOLERANCE_TICKS) {
            telemetry_pair_timestamp_reject_count++;
            continue;
        }

        uint32_t score = timestamp0_delta + block_offset_score(other, block);
        if (score < best_score) {
            best_score = score;
            best_timestamp0_delta = timestamp0_delta;
            best_other = other;
        }
    }

    if (best_other != NULL) {
#if LH2_OUTPUT_PAIRS
        if (block->time_us - last_lh2p_output_us[block->bs] >= LH2_PAIR_OUTPUT_MIN_INTERVAL_US) {
            if ((best_other->polynomial & 1u) == 0u) {
                serial_send_lh2_pair_line(best_other, block, best_timestamp0_delta);
            } else {
                serial_send_lh2_pair_line(block, best_other, best_timestamp0_delta);
            }
            last_lh2p_output_us[block->bs] = block->time_us;
        }
#endif
        telemetry_pair_count++;
        telemetry_last_pair_us = block->time_us;
    }

    remember_previous_block(block);
}

static void process_frame_for_blocks(const lh2_frame_t *frame) {
    if (frame->bs >= LH2_BASESTATION_COUNT || frame->sensor >= LH2_SENSOR_COUNT) {
        return;
    }

    if ((frame->polynomial >> 1u) != frame->bs) {
        telemetry_poly_reject_count++;
        return;
    }

    uint8_t poly_slot = frame->polynomial & 1u;
    lh2_block_builder_t *builder = &block_builders[frame->bs][poly_slot];
    if (builder->active && frame->time_us - builder->start_time_us > LH2_BLOCK_WINDOW_US) {
        telemetry_builder_timeout_count++;
        if (builder->count >= LH2_MIN_BLOCK_SENSORS) {
            lh2_block_t block;
            telemetry_block_attempt_count++;
            if (build_complete_block(builder, &block)) {
                telemetry_block_count++;
                process_completed_block(&block);
            } else {
                telemetry_block_reject_count++;
            }
        }
        reset_block_builder(builder);
    }

    if (!builder->active) {
        builder->active = true;
        builder->start_time_us = frame->time_us;
        builder->id = next_block_id++;
        builder->bs = frame->bs;
        builder->sweep = poly_slot;
        builder->polynomial = frame->polynomial;
    }

    if (builder->polynomial != frame->polynomial) {
        reset_block_builder(builder);
        builder->active = true;
        builder->start_time_us = frame->time_us;
        builder->id = next_block_id++;
        builder->bs = frame->bs;
        builder->sweep = poly_slot;
        builder->polynomial = frame->polynomial;
    }

    add_frame_candidate(builder, frame);

    if (builder->count == LH2_SENSOR_COUNT) {
        lh2_block_t block;
        telemetry_block_attempt_count++;
        if (build_complete_block(builder, &block)) {
            telemetry_block_count++;
            process_completed_block(&block);
        } else {
            telemetry_block_reject_count++;
        }
        reset_block_builder(builder);
    }
}

//=========================== main core #0 =====================================

int main(void) {
    clk_conf_OK = set_sys_clock_khz(128000, true);

    stdio_init_all();
    sleep_ms(3000);
    watchdog_enable(5000, 1);
    watchdog_update();
#if LH2_OUTPUT_BOOT_BANNER
    printf("Start code;%s;block_window_us=%u;prev_blocks=%u;candidates=%u;out_min_us=%u;stale_reset_us=%u\r\n",
           FIRMWARE_TAG,
           LH2_BLOCK_WINDOW_US,
           LH2_PREVIOUS_BLOCK_CANDIDATES,
           LH2_CANDIDATES_PER_SENSOR,
           LH2_PAIR_OUTPUT_MIN_INTERVAL_US,
           LH2_PAIRING_STALE_RESET_US);
#endif

    db_lh2_init(&_lh2_0, sensor_0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&_lh2_1, sensor_1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);

    multicore_launch_core1(core1_entry);

    uint32_t last_frame_seq[LH2_SENSOR_COUNT] = { 0 };
#if LH2_OUTPUT_HEARTBEAT
    uint64_t last_heartbeat_us = 0;
#endif

    while (true) {
        watchdog_update();
        db_lh2_process_location(&_lh2_0);
        db_lh2_process_location(&_lh2_1);

        db_lh2_t *sensors[LH2_SENSOR_COUNT] = {&_lh2_0, &_lh2_1, &_lh2_2, &_lh2_3};
        for (uint8_t sensor_id = 0; sensor_id < LH2_SENSOR_COUNT; sensor_id++) {
            lh2_frame_t frame;
            if (copy_latest_frame_if_new(sensors[sensor_id], &last_frame_seq[sensor_id], &frame)) {
                telemetry_frame_count++;
                telemetry_last_frame_us = frame.time_us;
#if LH2_OUTPUT_LEGACY_FRAMES
                serial_send_lh2_raw_line(&frame);
#endif
#if LH2_OUTPUT_EXTENDED_FRAMES
                serial_send_lh2_extended_line(&frame);
#endif
                process_frame_for_blocks(&frame);
            }
        }

        uint64_t now_us = to_us_since_boot(get_absolute_time());
        if (telemetry_last_frame_us != 0 &&
            telemetry_last_pair_us != 0 &&
            now_us - telemetry_last_frame_us < HEARTBEAT_INTERVAL_US &&
            now_us - telemetry_last_pair_us > LH2_PAIRING_STALE_RESET_US) {
            reset_pairing_state();
            telemetry_last_pair_us = now_us;
        }

#if LH2_OUTPUT_HEARTBEAT
        if (now_us - last_heartbeat_us >= HEARTBEAT_INTERVAL_US) {
            serial_send_heartbeat(last_frame_seq[0], last_frame_seq[1], last_frame_seq[2], last_frame_seq[3]);
            last_heartbeat_us = now_us;
        }
#endif

        tight_loop_contents();
    }
}

//=========================== main core #1 =====================================

void core1_entry(void) {
    db_lh2_init(&_lh2_2, sensor_2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&_lh2_3, sensor_3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);

    while (true) {
        db_lh2_process_location(&_lh2_2);
        db_lh2_process_location(&_lh2_3);
    }
}
