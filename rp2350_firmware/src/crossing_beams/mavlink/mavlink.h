/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 encoder — ODOMETRY (msg #331).
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>

void mavlink_init(void);
void mavlink_send_odometry(uint64_t usec, float x, float y, float z);

#endif /* MAVLINK_H */
