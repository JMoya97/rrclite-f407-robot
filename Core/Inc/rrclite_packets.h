#ifndef RRCLITE_PACKETS_H
#define RRCLITE_PACKETS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    RRC_FUNC_SYS                = 0x00U,
    RRC_FUNC_MOTOR              = 0x03U,
    RRC_FUNC_IO                 = 0x04U,
    RRC_FUNC_IMU                = 0x07U,
} rrc_func_t;

/* Common protocol helpers */
#define RRC_TXID_NONE               0xFFU
#define RRC_MAX_PAYLOAD_LEN         255U

/* -------------------------------------------------------------------------- */
/* SYS (0x00) sub-commands                                                    */

typedef enum {
    RRC_SYS_BATTERY_ONESHOT        = 0xA0,
    RRC_SYS_BATTERY_STREAM_CTRL    = 0xA1,
    RRC_SYS_MOTOR_FAILSAFE_SET     = 0xB0,
    RRC_SYS_HEALTH_PERIOD_SET      = 0xB1,
    RRC_SYS_UART_BAUD_SET          = 0xC0,
    RRC_SYS_UART_BAUD_GET          = 0xC1,
    RRC_SYS_PING_ECHO              = 0xE0,
    RRC_SYS_ERROR_EVENT            = 0xEE,
    RRC_SYS_RECOVERED_EVENT        = 0xEF,
    RRC_SYS_VERSION                = 0xF0,
    RRC_SYS_CAPABILITIES           = 0xF1,
} rrc_sys_sub_t;

#define RRC_PROTO_VERSION_MAJOR      0x02U
#define RRC_PROTO_VERSION_MINOR      0x00U
#define RRC_PROTO_VERSION_PATCH      0x0000U

typedef enum {
    RRC_SYS_CAP_TXID_ACKS          = (1U << 0),
    RRC_SYS_CAP_SEQ_STREAMS        = (1U << 1),
    RRC_SYS_CAP_STREAM_ACK_OPT     = (1U << 2),
    RRC_SYS_CAP_DUAL_IMU           = (1U << 3),
    RRC_SYS_CAP_BAUD_1M            = (1U << 4),
    RRC_SYS_CAP_FAILSAFE           = (1U << 5),
} rrc_sys_capability_flags_t;

typedef struct __attribute__((packed)) {
    uint8_t major;
    uint8_t minor;
    uint16_t patch_le;
} rrc_sys_version_resp_t;

typedef struct __attribute__((packed)) {
    uint8_t proto_major;
    uint8_t proto_minor;
    uint8_t rsvd0[2];
    uint32_t caps0_le;
    uint32_t max_baud_le;
    uint16_t max_imu_hz_le;
    uint16_t max_enc_hz_le;
} rrc_sys_capabilities_resp_t;

typedef enum {
    RRC_SYS_ERR_INVALID_ARG        = 0x01,
    RRC_SYS_ERR_BUSY               = 0x02,
    RRC_SYS_ERR_TIMEOUT            = 0x03,
    RRC_SYS_ERR_IO_FAIL            = 0x04,
    RRC_SYS_ERR_NOT_READY          = 0x05,
    RRC_SYS_ERR_NO_DEVICE          = 0x06,
    RRC_SYS_ERR_CRC_FAIL           = 0x07,
    RRC_SYS_ERR_UNSUPPORTED        = 0x08
} rrc_error_code_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t enable;
    uint16_t period_ms_le;
} rrc_sys_battery_stream_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint16_t timeout_ms_le;
} rrc_sys_motor_failsafe_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint16_t period_ms_le;
} rrc_sys_period_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint32_t baud_le;
    uint16_t apply_after_ms_le;
} rrc_sys_uart_baud_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t origin_func;
    uint8_t origin_sub;
    uint8_t err_code;
    uint8_t detail;
    uint32_t t_ms_le;
    uint8_t txid;
} rrc_sys_error_report_t;

typedef struct __attribute__((packed)) {
    uint8_t origin_func;
    uint8_t origin_sub;
    uint8_t prev_err_code;
    uint32_t t_ms_le;
} rrc_sys_recovered_report_t;

/* -------------------------------------------------------------------------- */
/* MOTOR (0x03) sub-commands                                                  */

typedef enum {
    RRC_MOTOR_PWM_SET              = 0x10,
    RRC_MOTOR_PWM_ACK_SINGLE       = 0x18,
    RRC_MOTOR_PWM_ACK_MULTI        = 0x19,
    RRC_MOTOR_ENCODER_ONESHOT      = 0x90,
    RRC_MOTOR_ENCODER_STREAM_CTRL  = 0x91,
    RRC_MOTOR_ENCODER_STREAM_ACK   = 0x99
} rrc_motor_sub_t;

#define RRC_MOTOR_ENCODER_STREAM_FRAME RRC_MOTOR_ENCODER_STREAM_CTRL

typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    int16_t pwm;
    uint8_t txid;
} rrc_motor_pwm_set_req_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t motor_id;
    int16_t pwm_target;
    int16_t pwm_applied;
} rrc_motor_pwm_ack_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint16_t c1;
    uint16_t c2;
    uint16_t c3;
    uint16_t c4;
} rrc_encoder_stream_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t enable;
    uint16_t period_ms_le;
} rrc_encoder_stream_ack_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
} rrc_encoder_frame_ack_t;

/* -------------------------------------------------------------------------- */
/* IO (0x04) sub-commands                                                     */

typedef enum {
    RRC_IO_LED_SET                 = 0x20,
    RRC_IO_BUZZER_SET              = 0x21,
    RRC_IO_BUTTON_ONESHOT          = 0x22,
    RRC_IO_BUTTON_STREAM_CTRL      = 0x23,
    RRC_IO_BUTTON_STREAM_ACK       = 0x29
} rrc_io_sub_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t mode;
} rrc_io_led_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint16_t freq_hz;
    uint8_t duty_pct;
    uint16_t duration_ms;
} rrc_io_buzzer_ack_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint8_t mask;
} rrc_button_stream_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t enable;
    uint16_t period_ms_le;
} rrc_button_stream_ack_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
} rrc_button_frame_ack_t;

/* -------------------------------------------------------------------------- */
/* IMU (0x07) sub-commands                                                    */

typedef enum {
    RRC_IMU_ONESHOT                = 0xA0,
    RRC_IMU_STREAM_CTRL            = 0xA1,
    RRC_IMU_STREAM_ACK             = 0xA9,
    RRC_IMU_SET_PRIMARY            = 0xA2,
    RRC_IMU_SET_PRESET             = 0xA3,
    RRC_IMU_SET_BIASES             = 0xA4,
    RRC_IMU_WHOAMI_STATUS          = 0xA5
} rrc_imu_sub_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t sources_mask;
    uint16_t period_ms_le;
    uint8_t ack_each_frame;
} rrc_imu_stream_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t source_id;
} rrc_imu_primary_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t source_id;
    uint8_t preset;
} rrc_imu_preset_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t txid;
    uint8_t source_id;
} rrc_imu_bias_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t source_id;
    uint16_t seq;
    uint32_t t_ms;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    float temp_c;
} rrc_imu_stream_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t source_id;
    uint16_t seq;
} rrc_imu_frame_ack_t;

/* -------------------------------------------------------------------------- */
/* Helper API                                                                 */

bool rrc_send_ack(uint8_t func, uint8_t sub, const void *payload, size_t len,
                  uint8_t txid);
bool rrc_send_err(uint8_t origin_func, uint8_t origin_sub,
                  rrc_error_code_t err_code, uint8_t detail,
                  uint32_t t_ms, uint8_t txid_or_0);
bool rrc_send_recovered(uint8_t origin_func, uint8_t origin_sub,
                        rrc_error_code_t prev_err_code, uint32_t t_ms);

bool rrc_func_is_supported(uint8_t func);
bool rrc_sub_is_supported(uint8_t func, uint8_t sub);
bool rrc_payload_len_valid(size_t len);
bool rrc_payload_len_valid_for(uint8_t func, uint8_t sub, size_t len);
uint16_t rrc_payload_max_for(uint8_t func, uint8_t sub);

bool rrc_dispatch_command(uint8_t func, uint8_t sub,
                          const void *payload, size_t len);

bool rrc_uart_baud_is_supported(uint32_t baud);
uint16_t rrc_uart_baud_apply_delay_ms(uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* RRCLITE_PACKETS_H */
