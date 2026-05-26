/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 encoder — VISION_POSITION_ESTIMATE only (msg #102).
 *
 * Sends raw 45-byte frames over UART0 (GPIO 0 TX / GPIO 1 RX, 115200 baud)
 * to a Pixhawk 6C on TELEM2.
 *
 * Roll, pitch, and yaw are always encoded as NaN (MAVLink "field not
 * provided") so the flight controller fuses only the position and derives
 * attitude from its own compass.
 *
 * No heap allocation, no external MAVLink library, no global buffers.
 * Everything lives on the stack inside mavlink_send_vpe().
 *
 * Usage:
 *   mavlink_init();                          // once at startup
 *   mavlink_send_vpe(now_us, cx, cy, cz);   // call at 25 Hz
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>

/**
 * @brief Initialise UART0 at 115200 baud on GPIO 0 (TX) and GPIO 1 (RX).
 *        Must be called once before mavlink_send_vpe().
 */
void mavlink_init(void);

/**
 * @brief Encode and send one VISION_POSITION_ESTIMATE frame over UART0.
 *
 * Frame: MAVLink v2, 45 bytes total
 *   (10 B header + 33 B payload + 2 B CRC16/MCRF4XX)
 *
 * Payload fields:
 *   usec         — timestamp [µs since boot]
 *   x, y, z      — position [metres]
 *   roll/pitch/yaw — NaN (not provided)
 *   reset_counter — 0
 *
 * @param usec  timestamp in microseconds since boot
 * @param x     position X in metres
 * @param y     position Y in metres
 * @param z     position Z in metres
 */
void mavlink_send_vpe(uint64_t usec, float x, float y, float z);

#endif /* MAVLINK_H */
