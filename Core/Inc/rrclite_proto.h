#ifndef RRCLITE_PROTO_H
#define RRCLITE_PROTO_H

#include <stdint.h>

#define RRC_PROTO_VERSION_MAJOR 2
#define RRC_PROTO_VERSION_MINOR 0
#define RRC_PROTO_COMPAT_LEGACY 1

typedef enum {
    RRCV2_FUNC_SYS    = 0x00U,
    RRCV2_FUNC_MOTOR  = 0x10U,
    RRCV2_FUNC_STEER  = 0x11U,
    RRCV2_FUNC_LED    = 0x12U,
    RRCV2_FUNC_BUZZ   = 0x13U,
    RRCV2_FUNC_BUTTON = 0x20U,
    RRCV2_FUNC_ENC    = 0x21U,
    RRCV2_FUNC_BATT   = 0x22U,
    RRCV2_FUNC_IMU    = 0x23U,
} rrc_func_v2_t;

typedef enum {
    RRCV2_SUB_SET          = 0x01U,
    RRCV2_SUB_GET          = 0x02U,
    RRCV2_SUB_ACK          = 0x03U,
    RRCV2_SUB_STREAM_CTRL  = 0x10U,
    RRCV2_SUB_STREAM_FRAME = 0x11U,
    RRCV2_SUB_STREAM_ACK   = 0x12U,
    RRCV2_SUB_ONE_SHOT     = 0x20U,
} rrc_proto_v2_sub_t;

/* IMU configuration extensions reserved under the v2 map. */
#define RRCV2_SUB_IMU_SET_PRESET  0x31U
#define RRCV2_SUB_IMU_SET_BIASES  0x32U

/* SYS retains existing specialised sub-IDs (B0, C0, C1, E0, EE, EF, F0, F1). */

#endif /* RRCLITE_PROTO_H */
