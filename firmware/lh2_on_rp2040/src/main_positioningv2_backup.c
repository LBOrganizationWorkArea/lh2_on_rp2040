/**
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the lighthouse v2 chip (TS4231) using the RP2040 microcontroller.
 *
 * Load this program on your board. with a TS4231 connected to pins 15 (Data) and 16 (Envelope).
 *
 * @date 2024
 *
 * @copyright Inria, 2024
 *
 */
#include "hardware/pio.h"
#include "pico/time.h"
#include "lh2/lh2.h"
#include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
#include <stdio.h>
#include <stdlib.h>

#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 10                     // 
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)   // The Envelope pin will be (Data pin + 1)
#define LH2_1_DATA_PIN 12                     // 
#define LH2_1_ENV_PIN  (LH2_1_DATA_PIN + 1)   // The Envelope pin will be (Data pin + 1)
#define LH2_2_DATA_PIN 18                     // 
#define LH2_2_ENV_PIN  (LH2_2_DATA_PIN + 1)   // The Envelope pin will be (Data pin + 1)
#define LH2_3_DATA_PIN 20                     // 
#define LH2_3_ENV_PIN  (LH2_3_DATA_PIN + 1)   // The Envelope pin will be (Data pin + 1)
#define TIMER_DELAY_US 100000                 // How often the LH2 updates are sent through the serial (in microseconds)



//=========================== variables ========================================

// is nedeed so the variable is accesible to both cores [1]
db_lh2_t        _lh2_0;
db_lh2_t        _lh2_1;
db_lh2_t        _lh2_2;
db_lh2_t        _lh2_3;
absolute_time_t timer_0;
bool            clk_conf_OK;

uint8_t sensor_0 = 0;
uint8_t sensor_1 = 1;
uint8_t sensor_2 = 2;
uint8_t sensor_3 = 3;

//=========================== prototypes ========================================

void core1_entry();

//=========================== main core #0 =============================================

int main() {
    // configure the clock for 128MHz
    clk_conf_OK = set_sys_clock_khz(128000, true);

    // init the USB UART
    stdio_init_all();
    sleep_ms(3000);
    printf("Start code\n");

    // set-up the on-board LED for the W version of the Raspberry Pi Pico
    // cyw43_arch_init();
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // LH2 config, before starting the second core
    db_lh2_init(&_lh2_0, sensor_0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&_lh2_1, sensor_1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);

    // Launch the second core
    multicore_launch_core1(core1_entry);

    // Start timer 
    timer_0 = get_absolute_time();

    while (true) {

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2_0);
        db_lh2_process_location(&_lh2_1);

        if (absolute_time_diff_us(timer_0, get_absolute_time()) > TIMER_DELAY_US) {

            uint64_t time_us = to_us_since_boot(get_absolute_time());

            db_lh2_t *sensors[4] = {&_lh2_0, &_lh2_1, &_lh2_2, &_lh2_3};

            for (uint8_t sensor_id = 0; sensor_id < 4; sensor_id++) {
                db_lh2_t *lh2 = sensors[sensor_id];

                for (uint8_t bs = 0; bs < LH2_BASESTATION_COUNT; bs++) {
                    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {

                        uint8_t polynomial = lh2->locations[sweep][bs].selected_polynomial;
                        uint32_t lfsr_location = lh2->locations[sweep][bs].lfsr_location;

                        if (polynomial == 255 || lfsr_location == 0xFFFFFFFF) {
                            continue;
                        }

                        printf("LH2,%llu,%u,%u,%u,%u,%lu\n",
                            time_us,
                            sensor_id,
                            sweep,
                            bs,
                            polynomial,
                            (unsigned long)lfsr_location);
                    }
                }
            }

            fflush(stdout);
            timer_0 = get_absolute_time();
        }
    }
}

//=========================== main core #0 =============================================

void core1_entry() {

    db_lh2_init(&_lh2_2, sensor_2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&_lh2_3, sensor_3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);

    while (true) {
        db_lh2_process_location(&_lh2_2);
        db_lh2_process_location(&_lh2_3);
    }
}

//=========================== references ========================================