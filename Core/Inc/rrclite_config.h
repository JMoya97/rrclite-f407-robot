#ifndef RRCLITE_CONFIG_H
#define RRCLITE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file rrclite_config.h
 * @brief Build-time feature and protocol defaults for the RRCLite firmware.
 */

/* Feature retention flags (1 = keep module compiled in) */
#define RRC_KEEP_BATTERY           1
#define RRC_KEEP_MOTORS            1
#define RRC_KEEP_STEERING_SERVO    1
#define RRC_KEEP_LEDS              1
#define RRC_KEEP_BUZZER            1
#define RRC_KEEP_BUTTONS           1
#define RRC_KEEP_ENCODERS          1
#define RRC_KEEP_IMU_PRIMARY       1
#define RRC_KEEP_IMU_OPTIONAL      1

/* Serial transport defaults */
#define RRC_DEFAULT_BAUD_BPS       1000000U
#define RRC_HEARTBEAT_MS           500U

/* Stream behaviour */
#define RRC_STREAM_ACK_DEFAULT     0U

#ifdef __cplusplus
}
#endif

#endif /* RRCLITE_CONFIG_H */
