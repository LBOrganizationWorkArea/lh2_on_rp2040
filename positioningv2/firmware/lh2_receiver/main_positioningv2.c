/**
 * @file
 * @brief positioningv2 firmware main file.
 *
 * Reads 4 TS4231 Lighthouse sensors with RP2040/RP2350 and prints decoded
 * Lighthouse v2 data over USB serial.
 *
 * Output format:
 * LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location
 */

#include "hardware/pio.h"
#include "pico/time.h"
#include "lh2/lh2.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 10
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)

#define LH2_1_DATA_PIN 12
#define LH2_1_ENV_PIN  (LH2_1_DATA_PIN + 1)

#define LH2_2_DATA_PIN 18
#define LH2_2_ENV_PIN  (LH2_2_DATA_PIN + 1)

#define LH2_3_DATA_PIN 20
#define LH2_3_ENV_PIN  (LH2_3_DATA_PIN + 1)

// Send serial data every 100 ms
#define TIMER_DELAY_US 100000

//=========================== variables ========================================

db_lh2_t _lh2_0;
db_lh2_t _lh2_1;
db_lh2_t _lh2_2;
db_lh2_t _lh2_3;

absolute_time_t timer_0;
bool clk_conf_OK;

uint8_t sensor_0 = 0;
uint8_t sensor_1 = 1;
uint8_t sensor_2 = 2;
uint8_t sensor_3 = 3;

//=========================== prototypes =======================================

void core1_entry(void);
void print_sensor_csv(uint8_t sensor_id, db_lh2_t *lh2);

//=========================== main core #0 =====================================

int main(void) {
    // Configure clock to 128 MHz
    clk_conf_OK = set_sys_clock_khz(128000, true);

    // Init USB serial
    stdio_init_all();
    sleep_ms(3000);

    printf("Start positioningv2 firmware\n");
    printf("LH2,time_us,sensor,sweep,basestation,polynomial,lfsr_location\n");

    // Init sensors on core 0
    db_lh2_init(&_lh2_0, sensor_0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&_lh2_1, sensor_1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);

    // Launch second core for sensors 2 and 3
    multicore_launch_core1(core1_entry);

    timer_0 = get_absolute_time();

    while (true) {
        // These functions must run continuously
        db_lh2_process_location(&_lh2_0);
        db_lh2_process_location(&_lh2_1);

        if (absolute_time_diff_us(timer_0, get_absolute_time()) > TIMER_DELAY_US) {
            // Print all valid decoded values for all 4 sensors
            print_sensor_csv(0, &_lh2_0);
            print_sensor_csv(1, &_lh2_1);
            print_sensor_csv(2, &_lh2_2);
            print_sensor_csv(3, &_lh2_3);

            timer_0 = get_absolute_time();
        }
    }

    return 0;
}

//=========================== core #1 ==========================================

void core1_entry(void) {
    db_lh2_init(&_lh2_2, sensor_2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&_lh2_3, sensor_3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);

    while (true) {
        db_lh2_process_location(&_lh2_2);
        db_lh2_process_location(&_lh2_3);
    }
}

//=========================== serial output ====================================

void print_sensor_csv(uint8_t sensor_id, db_lh2_t *lh2) {
    uint64_t time_us = to_us_since_boot(get_absolute_time());

    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
        for (uint8_t basestation = 0; basestation < LH2_BASESTATION_COUNT; basestation++) {

            uint8_t polynomial = lh2->locations[sweep][basestation].selected_polynomial;
            uint32_t lfsr_location = lh2->locations[sweep][basestation].lfsr_location;

            // Ignore invalid/uninitialized data
            if (polynomial == 255 || lfsr_location == 0xFFFFFFFF) {
                continue;
            }

            printf("LH2,%llu,%u,%u,%u,%u,%lu\n",
                   time_us,
                   sensor_id,
                   sweep,
                   basestation,
                   polynomial,
                   (unsigned long)lfsr_location);
        }
    }
}