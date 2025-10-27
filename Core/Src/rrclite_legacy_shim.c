#include "rrclite_proto.h"

#include "rrclite_packets.h"

#if RRC_PROTO_COMPAT_LEGACY

typedef struct {
    uint8_t old_func;
    uint8_t old_sub;
    uint8_t new_func;
    uint8_t new_sub;
} rrc_legacy_map_entry_t;

/* clang-format off */
static const rrc_legacy_map_entry_t g_rrc_legacy_map[] = {
    { RRC_FUNC_SYS,   RRC_SYS_BATTERY_ONESHOT,       RRC_FUNC_BATT,   RRCV2_SUB_ONE_SHOT },
    { RRC_FUNC_SYS,   RRC_SYS_BATTERY_STREAM_CTRL,   RRC_FUNC_BATT,   RRCV2_SUB_STREAM_CTRL },
    { RRC_FUNC_SYS,   RRC_SYS_MOTOR_FAILSAFE_SET,    RRC_FUNC_SYS,    RRC_SYS_MOTOR_FAILSAFE_SET },
    { RRC_FUNC_SYS,   RRC_SYS_HEALTH_PERIOD_SET,     RRC_FUNC_SYS,    RRC_SYS_HEALTH_PERIOD_SET },
    { RRC_FUNC_SYS,   RRC_SYS_UART_BAUD_SET,         RRC_FUNC_SYS,    RRC_SYS_UART_BAUD_SET },
    { RRC_FUNC_SYS,   RRC_SYS_UART_BAUD_GET,         RRC_FUNC_SYS,    RRC_SYS_UART_BAUD_GET },
    { RRC_FUNC_SYS,   RRC_SYS_PING_ECHO,             RRC_FUNC_SYS,    RRC_SYS_PING_ECHO },
    { RRC_FUNC_SYS,   RRC_SYS_VERSION,               RRC_FUNC_SYS,    RRC_SYS_VERSION },
    { RRC_FUNC_SYS,   RRC_SYS_CAPABILITIES,          RRC_FUNC_SYS,    RRC_SYS_CAPABILITIES },

    { RRC_FUNC_MOTOR_LEGACY, RRC_MOTOR_PWM_SET,             RRC_FUNC_MOTOR,  RRCV2_SUB_SET },

    { RRC_FUNC_MOTOR_LEGACY, RRC_MOTOR_ENCODER_ONESHOT_LEGACY,     RRC_FUNC_ENC,    RRCV2_SUB_ONE_SHOT },
    { RRC_FUNC_MOTOR_LEGACY, RRC_MOTOR_ENCODER_STREAM_CTRL_LEGACY, RRC_FUNC_ENC,    RRCV2_SUB_STREAM_CTRL },

    { RRC_FUNC_IO_LEGACY, RRC_IO_LED_SET_LEGACY,            RRC_FUNC_LED,    RRCV2_SUB_SET },
    { RRC_FUNC_IO_LEGACY, RRC_IO_BUZZER_SET_LEGACY,         RRC_FUNC_BUZZ,   RRCV2_SUB_SET },
    { RRC_FUNC_IO_LEGACY, RRC_IO_BUTTON_ONESHOT_LEGACY,     RRC_FUNC_BUTTON, RRCV2_SUB_ONE_SHOT },
    { RRC_FUNC_IO_LEGACY, RRC_IO_BUTTON_STREAM_CTRL_LEGACY, RRC_FUNC_BUTTON, RRCV2_SUB_STREAM_CTRL },

    { RRC_FUNC_IMU_LEGACY, RRC_IMU_ONESHOT,               RRC_FUNC_IMU,    RRCV2_SUB_ONE_SHOT },
    { RRC_FUNC_IMU_LEGACY, RRC_IMU_STREAM_CTRL,           RRC_FUNC_IMU,    RRCV2_SUB_STREAM_CTRL },
    { RRC_FUNC_IMU_LEGACY, RRC_IMU_SET_PRIMARY,           RRC_FUNC_IMU,    RRCV2_SUB_SET },
    { RRC_FUNC_IMU_LEGACY, RRC_IMU_SET_PRESET,            RRC_FUNC_IMU,    RRCV2_SUB_IMU_SET_PRESET },
    { RRC_FUNC_IMU_LEGACY, RRC_IMU_SET_BIASES,            RRC_FUNC_IMU,    RRCV2_SUB_IMU_SET_BIASES },
    { RRC_FUNC_IMU_LEGACY, RRC_IMU_WHOAMI_STATUS,         RRC_FUNC_IMU,    RRCV2_SUB_GET },
};
/* clang-format on */

#endif /* RRC_PROTO_COMPAT_LEGACY */

void rrc_legacy_translate_rx(uint8_t *pfunc, uint8_t *psub)
{
#if RRC_PROTO_COMPAT_LEGACY
    if ((pfunc == NULL) || (psub == NULL)) {
        return;
    }

    for (size_t i = 0U; i < (sizeof(g_rrc_legacy_map) / sizeof(g_rrc_legacy_map[0])); ++i) {
        if ((g_rrc_legacy_map[i].old_func == *pfunc) &&
            (g_rrc_legacy_map[i].old_sub == *psub)) {
            *pfunc = g_rrc_legacy_map[i].new_func;
            *psub  = g_rrc_legacy_map[i].new_sub;
            return;
        }
    }
#else
    (void)pfunc;
    (void)psub;
#endif
}
