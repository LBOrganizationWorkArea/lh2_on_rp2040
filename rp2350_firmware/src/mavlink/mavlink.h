/**
 * @file  mavlink.h
 * @brief Minimal MAVLink v2 codec — ODOMETRY TX (msg #331) + EKF_STATUS_REPORT RX (msg #193).
 */

#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>
#include <stdbool.h>

#include "../angle_decoder/angle_decoder.h"  /* NUM_SENSORS, NUM_BS */
#include "../ootx/ootx.h"                    /* ootx_station_t */

/* UART0 at MAV_BAUD with interrupt-driven RX; returns the baud rate actually set. */
uint32_t mavlink_init(void);
/* ODOMETRY in MAV_FRAME_LOCAL_FRD: position [m] and yaw [rad, clockwise about down] with its
 * variance [rad^2] (roll/pitch are left to the flight controller's IMU). */
void mavlink_send_odometry(uint64_t usec, float x, float y, float z, float yaw, float yaw_var);

/* Send one complete, instantaneous LH2 angle snapshot in MAVLink TUNNEL (format v3, see mavlink.c).
 * valid_mask bit (sensor * NUM_BS + bs) set when that pair is fresh; ages in ms, 255 = not valid. */
void mavlink_send_lh2_angles(uint64_t usec,
                             const float angles[NUM_SENSORS][NUM_BS][2],
                             const uint8_t angle_age_ms[NUM_SENSORS][NUM_BS],
                             uint16_t valid_mask);

/* Send one station's on-board OOTX decoding state (and its latest valid factory-calibration
 * frame) in MAVLink TUNNEL, payload "LO" v2 (see mavlink.c). */
void mavlink_send_lh2_ootx(int bs, const ootx_station_t *station);

/* RX — call every main-loop iteration to parse what the UART interrupt has buffered */
void mavlink_rx_update(void);
/* Bytes lost because the RX buffer was full (should stay 0) */
uint32_t mavlink_rx_dropped(void);

/* Returns true once EKF_STATUS_REPORT from the FC shows healthy flags */
bool mavlink_is_ekf_healthy(void);

/* Send COMMAND_LONG MAV_CMD_DO_SET_HOME (param1=1 = use current position) */
void mavlink_send_do_set_home(void);

/* Ask FC to stream EKF_STATUS_REPORT at 1 Hz via SET_MESSAGE_INTERVAL */
void mavlink_request_ekf_stream(void);

/* Return local_us corrected to FC timebase [µs]; unmodified until first sync. */
uint64_t mavlink_timesync_corrected_us(uint64_t local_us);

/* Skybrush SHOW mode without a GPS (call every loop, rate-limited inside): GPS_INPUT at 5 Hz with
 * no fix and a made-up GPS time (the show only starts if the GPS week is non-zero, its countdown
 * runs on the drone's own clock) and, while `origin` is true, SET_GPS_GLOBAL_ORIGIN every 5 s.
 * GPS_INPUT is used only with GPS_TYPE = 14 on the flight controller. */
void mavlink_send_show_time_and_origin(uint64_t now_us, bool origin);

#endif /* MAVLINK_H */
