/**
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief LH2 TS4231 example for RP2040, sending data through hardware UART pins.
 *
 * UART wiring:
 * - Pico GP0 / UART0 TX -> RX of the external device
 * - Pico GP1 / UART0 RX -> TX of the external device, optional if you only send
 * - GND -> GND
 */
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "lh2/lh2.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 10
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)
#define LH2_1_DATA_PIN 12
#define LH2_1_ENV_PIN  (LH2_1_DATA_PIN + 1)
#define LH2_2_DATA_PIN 18
#define LH2_2_ENV_PIN  (LH2_2_DATA_PIN + 1)
#define LH2_3_DATA_PIN 20
#define LH2_3_ENV_PIN  (LH2_3_DATA_PIN + 1)

#define TIMER_DELAY_US 100000

#define UART_ID       uart0
#define UART_BAUDRATE 115200
#define UART_TX_PIN   0
#define UART_RX_PIN   1

//=========================== variables ========================================

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

//=========================== prototypes =======================================

void core1_entry(void);

//=========================== helpers ==========================================

static void uart_pins_init(void) {
    uart_init(UART_ID, UART_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

static void uart_send_lh2_line(uint64_t time_us,
                               uint8_t sensor_id,
                               uint8_t sweep,
                               uint8_t bs,
                               uint8_t polynomial,
                               uint32_t lfsr_location) {
    char line[96];
    int  len = snprintf(line,
                       sizeof(line),
                       "LH2,%llu,%u,%u,%u,%u,%lu\r\n",
                       time_us,
                       sensor_id,
                       sweep,
                       bs,
                       polynomial,
                       (unsigned long)lfsr_location);

    if (len > 0) {
        uart_write_blocking(UART_ID, (const uint8_t *)line, (size_t)len);
    }
}

//=========================== main core #0 =====================================

int main(void) {
    clk_conf_OK = set_sys_clock_khz(128000, true);

    stdio_init_all();
    uart_pins_init();

    sleep_ms(3000);
    uart_puts(UART_ID, "Start code\r\n");

    db_lh2_init(&_lh2_0, sensor_0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&_lh2_1, sensor_1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);

    multicore_launch_core1(core1_entry);

    timer_0 = get_absolute_time();

    while (true) {
        db_lh2_process_location(&_lh2_0);
        db_lh2_process_location(&_lh2_1);

        if (absolute_time_diff_us(timer_0, get_absolute_time()) > TIMER_DELAY_US) {
            uint64_t  time_us = to_us_since_boot(get_absolute_time());
            db_lh2_t *sensors[4] = {&_lh2_0, &_lh2_1, &_lh2_2, &_lh2_3};

            for (uint8_t sensor_id = 0; sensor_id < 4; sensor_id++) {
                db_lh2_t *lh2 = sensors[sensor_id];

                for (uint8_t bs = 0; bs < LH2_BASESTATION_COUNT; bs++) {
                    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
                        uint8_t  polynomial = lh2->locations[sweep][bs].selected_polynomial;
                        uint32_t lfsr_location = lh2->locations[sweep][bs].lfsr_location;

                        if (polynomial == 255 || lfsr_location == 0xFFFFFFFF) {
                            continue;
                        }

                        uart_send_lh2_line(time_us,
                                           sensor_id,
                                           sweep,
                                           bs,
                                           polynomial,
                                           lfsr_location);
                    }
                }
            }

            timer_0 = get_absolute_time();
        }
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
