/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 encoder — ODOMETRY (msg #331).
 *
 * Sends raw 245-byte frames over UART0 (GPIO 0 TX / GPIO 1 RX, 115200 baud).
 *
 * Only x and y are populated. All other fields (z, attitude, velocities,
 * covariances) are zero. Quaternion is set to the unit rotation {1,0,0,0}.
 *
 * Usage:
 *   mavlink_init();                          // once at startup
 *   mavlink_send_odometry(now_us, cx, cy);   // call at ~10 Hz
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>

void mavlink_init(void);

/**
 * @brief Encode and send one ODOMETRY frame over UART0.
 *
 * @param usec  timestamp in microseconds since boot
 * @param x     position X in metres
 * @param y     position Y in metres
 */
void mavlink_send_odometry(uint64_t usec, float x, float y);

#endif /* MAVLINK_H */
