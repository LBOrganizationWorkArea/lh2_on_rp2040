/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 codec — ODOMETRY TX (msg #331) + EKF_STATUS_REPORT RX (msg #193).
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>
#include <stdbool.h>

void mavlink_init(void);
void mavlink_send_odometry(uint64_t usec, float x, float y, float z);

/* RX — call every main-loop iteration to drain UART FIFO */
void mavlink_rx_update(void);

/* Returns true once EKF_STATUS_REPORT from the FC shows healthy flags */
bool mavlink_is_ekf_healthy(void);

/* Send COMMAND_LONG MAV_CMD_DO_SET_HOME (param1=1 = use current position) */
void mavlink_send_do_set_home(void);

/* Ask FC to stream EKF_STATUS_REPORT at 1 Hz via SET_MESSAGE_INTERVAL */
void mavlink_request_ekf_stream(void);

/* Return local_us corrected to FC timebase [µs]; unmodified until first sync. */
uint64_t mavlink_timesync_corrected_us(uint64_t local_us);

#endif /* MAVLINK_H */
