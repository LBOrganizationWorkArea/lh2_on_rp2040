/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 codec — ODOMETRY TX (msg #331) + EKF_STATUS_REPORT RX (msg #193).
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>
#include <stdbool.h>

void mavlink_init(void);
void mavlink_send_heartbeat(void);
/* q = quaternion [w,x,y,z]; pos_var [m²]; yaw_var [rad²] (9.87=unknown);
 * quality = 0–100 (0/25/50/75/100 for 0–4 active sensors). */
void mavlink_send_odometry(uint64_t usec, float x, float y, float z,
                           const float q[4], float pos_var, float yaw_var,
                           uint8_t quality);

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

/* ---- BS pose receive (pushed by lh2_bs_params.lua via NAMED_VALUE_FLOAT) - */

/* Returns true once all 25 BS pose values have been received. */
bool mavlink_bs_poses_ready(void);

/* Total PARAM_VALUE frames received with valid CRC (debug). */
uint32_t mavlink_param_val_seen(void);

/* How many of the 25 LH2_BS* params have been received so far (debug). */
uint32_t mavlink_lh2_params_received(void);

/* Total raw bytes received on UART RX (debug). */
uint32_t mavlink_rx_bytes(void);

/* Copy the received BS poses into caller-supplied array[NUM_BS].
 * Each element has .origin[3] and .R[3][3] matching lh2_bs_pose_t layout.
 * Only call after mavlink_bs_poses_ready() returns true. */
#include "../solve3d/solve3d.h"   /* lh2_bs_pose_t */
void mavlink_get_bs_poses(lh2_bs_pose_t poses[NUM_BS]);

#endif /* MAVLINK_H */
