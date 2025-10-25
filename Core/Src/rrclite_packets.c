#include "rrclite_packets.h"

#include "stm32f4xx_hal.h"

#include <string.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define RRC_SYS_CAPABILITIES_FLAGS_DEFAULT \
    (RRC_SYS_CAP_TXID_ACKS | RRC_SYS_CAP_SEQ_STREAMS | \
     RRC_SYS_CAP_STREAM_ACK_OPT | RRC_SYS_CAP_DUAL_IMU | \
     RRC_SYS_CAP_BAUD_1M | RRC_SYS_CAP_FAILSAFE)

#define RRC_SYS_CAPABILITIES_MAX_BAUD   1000000U
#define RRC_SYS_CAPABILITIES_MAX_IMU_HZ 200U
#define RRC_SYS_CAPABILITIES_MAX_ENC_HZ 1000U

typedef struct {
    uint8_t sub;
    uint16_t max_payload;
} rrc_sub_entry_t;

typedef struct {
    uint8_t func;
    const rrc_sub_entry_t *subs;
    size_t sub_count;
} rrc_func_entry_t;

#define STREAM_PAYLOAD_MAX (RRC_MAX_PAYLOAD_LEN - 1U)

static const rrc_sub_entry_t g_sys_subs[] = {
    {RRC_SYS_BATTERY_ONESHOT, sizeof(uint16_t)},
    {RRC_SYS_BATTERY_STREAM_CTRL, sizeof(rrc_sys_battery_stream_ack_t)},
    {RRC_SYS_MOTOR_FAILSAFE_SET, sizeof(rrc_sys_motor_failsafe_ack_t)},
    {RRC_SYS_HEALTH_PERIOD_SET, sizeof(rrc_sys_period_ack_t)},
    {RRC_SYS_UART_BAUD_SET, sizeof(rrc_sys_uart_baud_ack_t)},
    {RRC_SYS_UART_BAUD_GET, sizeof(uint32_t)},
    {RRC_SYS_PING_ECHO, STREAM_PAYLOAD_MAX},
    {RRC_SYS_ERROR_EVENT, sizeof(rrc_sys_error_report_t)},
    {RRC_SYS_RECOVERED_EVENT, sizeof(rrc_sys_recovered_report_t)},
    {RRC_SYS_VERSION, sizeof(rrc_sys_version_resp_t)},
    {RRC_SYS_CAPABILITIES, sizeof(rrc_sys_capabilities_resp_t)},
};

static const rrc_sub_entry_t g_motor_subs[] = {
    {RRC_MOTOR_PWM_ACK_SINGLE, sizeof(rrc_motor_pwm_ack_t)},
    {RRC_MOTOR_PWM_ACK_MULTI, STREAM_PAYLOAD_MAX},
    {RRC_MOTOR_ENCODER_ONESHOT, sizeof(rrc_encoder_stream_frame_t)},
    {RRC_MOTOR_ENCODER_STREAM_CTRL, sizeof(rrc_encoder_stream_frame_t)},
    {RRC_MOTOR_ENCODER_STREAM_ACK, sizeof(rrc_encoder_frame_ack_t)},
};

static const rrc_sub_entry_t g_io_subs[] = {
    {RRC_IO_LED_SET, sizeof(rrc_io_led_ack_t)},
    {RRC_IO_BUZZER_SET, sizeof(rrc_io_buzzer_ack_t)},
    {RRC_IO_BUTTON_ONESHOT, sizeof(uint8_t)},
    {RRC_IO_BUTTON_STREAM_CTRL, sizeof(rrc_button_stream_ack_t)},
    {RRC_IO_BUTTON_STREAM_ACK, sizeof(rrc_button_frame_ack_t)},
};

static const rrc_sub_entry_t g_imu_subs[] = {
    {RRC_IMU_ONESHOT, STREAM_PAYLOAD_MAX},
    {RRC_IMU_STREAM_CTRL, sizeof(rrc_imu_stream_ack_t)},
    {RRC_IMU_STREAM_ACK, sizeof(rrc_imu_frame_ack_t)},
    {RRC_IMU_SET_PRIMARY, sizeof(rrc_imu_primary_ack_t)},
    {RRC_IMU_SET_PRESET, sizeof(rrc_imu_preset_ack_t)},
    {RRC_IMU_SET_BIASES, sizeof(rrc_imu_bias_ack_t)},
    {RRC_IMU_WHOAMI_STATUS, 3U},
};

static const rrc_func_entry_t g_func_table[] = {
    {RRC_FUNC_SYS, g_sys_subs, ARRAY_SIZE(g_sys_subs)},
    {RRC_FUNC_MOTOR, g_motor_subs, ARRAY_SIZE(g_motor_subs)},
    {RRC_FUNC_IO, g_io_subs, ARRAY_SIZE(g_io_subs)},
    {RRC_FUNC_IMU, g_imu_subs, ARRAY_SIZE(g_imu_subs)},
};

#undef STREAM_PAYLOAD_MAX

static const rrc_func_entry_t *rrc_lookup_func_entry(uint8_t func)
{
    for (size_t i = 0; i < ARRAY_SIZE(g_func_table); ++i) {
        if (g_func_table[i].func == func) {
            return &g_func_table[i];
        }
    }

    return NULL;
}

static const rrc_sub_entry_t *rrc_lookup_sub_entry(uint8_t func, uint8_t sub)
{
    const rrc_func_entry_t *func_entry = rrc_lookup_func_entry(func);
    if (func_entry == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < func_entry->sub_count; ++i) {
        if (func_entry->subs[i].sub == sub) {
            return &func_entry->subs[i];
        }
    }

    return NULL;
}

static bool rrc_handle_sys_version(const void *payload, size_t len)
{
    (void)payload;
    (void)len;

    const rrc_sys_version_resp_t resp = {
        .major = RRC_PROTO_VERSION_MAJOR,
        .minor = RRC_PROTO_VERSION_MINOR,
        .patch_le = RRC_PROTO_VERSION_PATCH,
    };

    return rrc_send_ack(RRC_FUNC_SYS, RRC_SYS_VERSION, &resp, sizeof(resp),
                        RRC_TXID_NONE);
}

static bool rrc_handle_sys_capabilities(const void *payload, size_t len)
{
    (void)payload;
    (void)len;

    const rrc_sys_capabilities_resp_t resp = {
        .proto_major = RRC_PROTO_VERSION_MAJOR,
        .proto_minor = RRC_PROTO_VERSION_MINOR,
        .rsvd0 = {0U, 0U},
        .caps0_le = RRC_SYS_CAPABILITIES_FLAGS_DEFAULT,
        .max_baud_le = RRC_SYS_CAPABILITIES_MAX_BAUD,
        .max_imu_hz_le = RRC_SYS_CAPABILITIES_MAX_IMU_HZ,
        .max_enc_hz_le = RRC_SYS_CAPABILITIES_MAX_ENC_HZ,
    };

    return rrc_send_ack(RRC_FUNC_SYS, RRC_SYS_CAPABILITIES,
                        &resp, sizeof(resp), RRC_TXID_NONE);
}

static bool rrc_dispatch_sys(uint8_t sub, const void *payload, size_t len)
{
    switch (sub) {
    case RRC_SYS_VERSION:
        return rrc_handle_sys_version(payload, len);
    case RRC_SYS_CAPABILITIES:
        return rrc_handle_sys_capabilities(payload, len);
    default:
        break;
    }

    return false;
}

#ifndef RRC_UART_DEFAULT_APPLY_DELAY_MS
#define RRC_UART_DEFAULT_APPLY_DELAY_MS 100U
#endif

__attribute__((weak)) bool rrc_transport_send(uint8_t func, uint8_t sub,
                                              const void *payload, size_t len)
{
    (void)func;
    (void)sub;
    (void)payload;
    (void)len;
    return false;
}

bool rrc_func_is_supported(uint8_t func)
{
    return rrc_lookup_func_entry(func) != NULL;
}

bool rrc_sub_is_supported(uint8_t func, uint8_t sub)
{
    return rrc_lookup_sub_entry(func, sub) != NULL;
}

bool rrc_payload_len_valid(size_t len)
{
    return len < (size_t)RRC_MAX_PAYLOAD_LEN;
}

uint16_t rrc_payload_max_for(uint8_t func, uint8_t sub)
{
    const rrc_sub_entry_t *sub_entry = rrc_lookup_sub_entry(func, sub);
    if (sub_entry == NULL) {
        return 0U;
    }

    return sub_entry->max_payload;
}

bool rrc_payload_len_valid_for(uint8_t func, uint8_t sub, size_t len)
{
    if (!rrc_payload_len_valid(len)) {
        return false;
    }

    const uint16_t max_payload = rrc_payload_max_for(func, sub);
    if (max_payload == 0U) {
        return false;
    }

    return len <= (size_t)max_payload;
}

bool rrc_dispatch_command(uint8_t func, uint8_t sub,
                          const void *payload, size_t len)
{
    switch (func) {
    case RRC_FUNC_SYS:
        return rrc_dispatch_sys(sub, payload, len);
    default:
        break;
    }

    return false;
}

bool rrc_send_ack(uint8_t func, uint8_t sub, const void *payload, size_t len,
                  uint8_t txid)
{
    if (txid == RRC_TXID_NONE) {
        return rrc_transport_send(func, sub, payload, len);
    }

    uint8_t scratch[RRC_MAX_PAYLOAD_LEN];
    const void *out_payload = payload;
    size_t out_len = len;

    if (len == 0U) {
        scratch[0] = txid;
        out_payload = scratch;
        out_len = 1U;
    } else {
        if (len > sizeof(scratch)) {
            return false;
        }

        if (payload == NULL) {
            return false;
        }

        memcpy(scratch, payload, len);
        scratch[0] = txid;
        out_payload = scratch;
    }

    return rrc_transport_send(func, sub, out_payload, out_len);
}

bool rrc_send_err(uint8_t origin_func, uint8_t origin_sub,
                  rrc_error_code_t err_code, uint8_t detail,
                  uint32_t t_ms, uint8_t txid_or_0)
{
    rrc_sys_error_report_t evt = {
        .origin_func = origin_func,
        .origin_sub = origin_sub,
        .err_code = (uint8_t)err_code,
        .detail = detail,
        .t_ms_le = t_ms,
        .txid = txid_or_0,
    };

    return rrc_transport_send(RRC_FUNC_SYS, RRC_SYS_ERROR_EVENT,
                              &evt, sizeof(evt));
}

bool rrc_send_recovered(uint8_t origin_func, uint8_t origin_sub,
                        rrc_error_code_t prev_err_code, uint32_t t_ms)
{
    rrc_sys_recovered_report_t evt = {
        .origin_func = origin_func,
        .origin_sub = origin_sub,
        .prev_err_code = (uint8_t)prev_err_code,
        .t_ms_le = t_ms,
    };

    return rrc_transport_send(RRC_FUNC_SYS, RRC_SYS_RECOVERED_EVENT,
                              &evt, sizeof(evt));
}

bool rrc_uart_baud_is_supported(uint32_t baud)
{
    return (baud == 115200U) || (baud == 1000000U);
}

uint16_t rrc_uart_baud_apply_delay_ms(uint32_t baud)
{
    (void)baud;
    return RRC_UART_DEFAULT_APPLY_DELAY_MS;
}
