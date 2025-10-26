#pragma once

typedef enum {
    RRCV2_FUNC_SYS = 0x00,
    RRCV2_FUNC_MOTOR = 0x10,
    RRCV2_FUNC_STEER = 0x11,
    RRCV2_FUNC_LED = 0x12,
    RRCV2_FUNC_BUZZ = 0x13,
    RRCV2_FUNC_BUTTON = 0x20,
    RRCV2_FUNC_ENC = 0x21,
    RRCV2_FUNC_BATT = 0x22,
    RRCV2_FUNC_IMU = 0x23,
} rrc_func_v2_t;

/*
 * Subcommand conventions (target mapping):
 *   SET          = 0x01
 *   GET          = 0x02
 *   ACK          = 0x03
 *   STREAM_CTRL  = 0x10
 *   STREAM_FRAME = 0x11
 *   STREAM_ACK   = 0x12
 *   ONE_SHOT     = 0x20
 *   SYS specials: 0xE0 (Echo), 0xF0 (Version), 0xF1 (Capabilities),
 *                 0xC0 (Baud Set), 0xC1 (Baud Get), 0xB0 (Failsafe),
 *                 0xEE (Error), 0xEF (Recovered)
 */
#define RRCV2_SUB_SET             0x01U
#define RRCV2_SUB_GET             0x02U
#define RRCV2_SUB_ACK             0x03U
#define RRCV2_SUB_STREAM_CTRL     0x10U
#define RRCV2_SUB_STREAM_FRAME    0x11U
#define RRCV2_SUB_STREAM_ACK      0x12U
#define RRCV2_SUB_ONE_SHOT        0x20U
#define RRCV2_SUB_IMU_SET_PRESET  0x31U
#define RRCV2_SUB_IMU_SET_BIASES  0x32U
