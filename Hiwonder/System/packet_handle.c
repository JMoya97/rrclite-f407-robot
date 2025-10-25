#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"
#include "serial_servo.h"
#include "packet_reports.h"
#include "motors_param.h"
#include "tim.h"
#include "cmsis_os2.h"
#include "rrclite_config.h"
#include "rrclite_packets.h"
#include "rrc_backoff.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern uint16_t encoders_set_stream(uint8_t enable, uint16_t period_ms);
extern void encoders_read_once_and_report(uint8_t sub);
extern uint16_t imu_set_stream(uint8_t sources_mask, uint16_t period_ms,
                               uint8_t ack_each_frame, uint8_t *applied_mask,
                               uint8_t *applied_ack_each_frame);
extern void imu_emit_oneshot(uint8_t sources_mask);
extern void imu_emit_whoami(uint8_t source_id);
extern volatile int motors_pwm_current[2];
extern uint16_t battery_set_stream(uint8_t enable, uint16_t period_ms);
extern uint16_t battery_latest_millivolts_le(void);
extern uint16_t buttons_set_stream(uint8_t enable, uint16_t period_ms);
extern uint8_t buttons_read_mask(void);

volatile uint16_t rrc_motor_failsafe_timeout_ms;
volatile uint32_t rrc_motor_last_cmd_ms;
volatile uint16_t rrc_heartbeat_period_ms = RRC_HEARTBEAT_MS;

static bool motor_pwm_fault_active;
static rrc_error_code_t motor_pwm_last_error;
static rrc_backoff_t g_motor_backoff;
static uint8_t g_motor_err_active;
static uint32_t g_motor_retry_due_ms;

typedef struct {
    float bax;
    float bay;
    float baz;
    float bgx;
    float bgy;
    float bgz;
    float bmx;
    float bmy;
    float bmz;
} rrc_imu_bias_store_t;

/* IMU configuration shadow (RAM only, reset on boot). */
static uint8_t g_imu_primary;
static rrc_imu_bias_store_t g_imu_bias[2];
static uint8_t g_imu_preset[2];

typedef struct {
    rrc_backoff_t backoff;
    uint8_t init;
    uint8_t err_active;
    uint32_t retry_due_ms;
    uint8_t detail;
    uint16_t extra0;
    uint16_t extra1;
} rrc_io_recovery_state_t;

static rrc_io_recovery_state_t g_led_recovery;
static rrc_io_recovery_state_t g_buzzer_recovery;
static rrc_io_recovery_state_t g_steering_recovery;

static void rrc_uart_apply_with_delay(uint32_t baud, uint16_t delay_ms,
                                      uint8_t txid)
{
    if (delay_ms == 0U) {
        delay_ms = rrc_uart_baud_apply_delay_ms(baud);
    }

    if (delay_ms != 0U) {
        osDelay(delay_ms);
    }

    if (!rrc_uart_runtime_reconfigure(baud)) {
        const uint32_t now = HAL_GetTick();
        const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
        (void)rrc_send_err(RRC_FUNC_SYS, RRC_SYS_UART_BAUD_SET,
                           RRC_SYS_ERR_IO_FAIL, 0U, now, err_txid);
    }
}

static bool motor_pwm_try_reinit(void)
{
    /* Stub hook for quick reinitialisation after an apply failure. */
    return true;
}

static void motor_backoff_ensure_init(void)
{
    static uint8_t backoff_init_done;
    if (backoff_init_done == 0U) {
        rrc_backoff_init(&g_motor_backoff, 50U, 3.0f, 1000U);
        backoff_init_done = 1U;
    }
}

void rrc_motor_recovery_tick(uint32_t now_ms)
{
    motor_backoff_ensure_init();

    if (g_motor_err_active == 0U) {
        return;
    }

    if ((int32_t)(now_ms - g_motor_retry_due_ms) < 0) {
        return;
    }

    if (motor_pwm_try_reinit()) {
        g_motor_err_active = 0U;
        rrc_backoff_reset(&g_motor_backoff);
        (void)rrc_send_recovered(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET,
                                 RRC_SYS_ERR_IO_FAIL, now_ms);
    } else {
        const uint32_t delay_ms = rrc_backoff_next(&g_motor_backoff);
        g_motor_retry_due_ms = now_ms + delay_ms;
    }
}

static bool motor_pwm_apply(uint8_t id, int pwm)
{
    extern void motor_set_target_pwm(uint8_t id, int cmd);

    if (id >= 2U) {
        return false;
    }

    motor_set_target_pwm(id, pwm);
    return true;
}

static void rrc_io_recovery_schedule(rrc_io_recovery_state_t *state,
                                     uint8_t func, uint8_t sub,
                                     uint8_t detail,
                                     uint16_t extra0,
                                     uint16_t extra1)
{
    if (state == NULL) {
        return;
    }

    if (state->init == 0U) {
        rrc_backoff_init(&state->backoff, 50U, 3.0f, 1000U);
        state->init = 1U;
    }

    state->detail = detail;
    state->extra0 = extra0;
    state->extra1 = extra1;

    const uint32_t now = HAL_GetTick();

    if (state->err_active == 0U) {
        state->err_active = 1U;
        (void)rrc_send_err(func, sub, RRC_SYS_ERR_IO_FAIL, detail, now, 0U);
        rrc_backoff_reset(&state->backoff);
    }

    const uint32_t delay_ms = rrc_backoff_next(&state->backoff);
    state->retry_due_ms = now + delay_ms;
}

static void rrc_io_recovery_clear(rrc_io_recovery_state_t *state,
                                  uint8_t func, uint8_t sub)
{
    if ((state == NULL) || (state->err_active == 0U)) {
        return;
    }

    state->err_active = 0U;
    rrc_backoff_reset(&state->backoff);
    state->retry_due_ms = 0U;

    const uint32_t now = HAL_GetTick();
    (void)rrc_send_recovered(func, sub, RRC_SYS_ERR_IO_FAIL, now);
}

static bool rrc_led_try_reinit(const rrc_io_recovery_state_t *state)
{
    if (state == NULL) {
        return false;
    }

    const uint8_t led_index = state->detail;
    if (led_index >= LED_NUM) {
        return false;
    }

    LEDObjectTypeDef *led = leds[led_index];
    if (led == NULL) {
        return false;
    }

    return led_flash(led, 0U, 0U, 0U) == 0;
}

static bool rrc_buzzer_try_reinit(const rrc_io_recovery_state_t *state)
{
    (void)state;
    return buzzer_off(buzzers[0]) == 0;
}

static bool rrc_steering_try_reinit(const rrc_io_recovery_state_t *state)
{
    if (state == NULL) {
        return false;
    }

    uint8_t servo_id = state->detail;

    return serial_servo_set_position(&serial_servo_controller, servo_id,
                                     (int)state->extra0, state->extra1) == 0;
}

static void rrc_io_recovery_tick_one(rrc_io_recovery_state_t *state,
                                     uint8_t func, uint8_t sub,
                                     bool (*try_reinit)(const rrc_io_recovery_state_t *),
                                     uint32_t now_ms)
{
    if ((state == NULL) || (state->err_active == 0U)) {
        return;
    }

    if ((int32_t)(now_ms - state->retry_due_ms) < 0) {
        return;
    }

    if ((try_reinit != NULL) && try_reinit(state)) {
        state->err_active = 0U;
        rrc_backoff_reset(&state->backoff);
        state->retry_due_ms = 0U;
        (void)rrc_send_recovered(func, sub, RRC_SYS_ERR_IO_FAIL, now_ms);
    } else {
        const uint32_t delay = rrc_backoff_next(&state->backoff);
        state->retry_due_ms = now_ms + delay;
    }
}

void rrc_io_recovery_tick(uint32_t now_ms)
{
    rrc_io_recovery_tick_one(&g_led_recovery, RRC_FUNC_IO, RRC_IO_LED_SET,
                             rrc_led_try_reinit, now_ms);
    rrc_io_recovery_tick_one(&g_buzzer_recovery, RRC_FUNC_IO, RRC_IO_BUZZER_SET,
                             rrc_buzzer_try_reinit, now_ms);
    rrc_io_recovery_tick_one(&g_steering_recovery, PACKET_FUNC_BUS_SERVO, 0x01U,
                             rrc_steering_try_reinit, now_ms);
}

#pragma pack(1)
typedef struct {
    uint8_t cmd;
    uint8_t motor_num;
    struct {
        uint8_t motor_id;
        float speed;
    } element[];
} MotorMutilCtrlCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_id;
    float speed;
} MotorSingalCtrlCommandTypeDef;

/* ---- RAW PWM passthrough payloads ---- */
typedef struct {
    uint8_t  cmd;       /* 0x10 */
    uint8_t  motor_id;  /* 0..3 */
    int16_t  pwm;       /* -1000..1000 */
} MotorPWMSingleCommandTypeDef;

typedef struct {
    uint8_t  cmd;        /* 0x11 */
    uint8_t  motor_num;  /* N elements */
    struct {
        uint8_t motor_id;
        int16_t pwm;
    } element[];
} MotorPWMMultiCommandTypeDef;

typedef struct { uint8_t cmd; } EncoderReadCommandTypeDef; /* 0x90 */
typedef struct {
    uint8_t  cmd;        /* 0x91 */
    uint8_t  enable;     /* 0=stop,1=start */
    uint16_t period_ms;  /* >=5 */
} EncoderStreamCtrlCommandTypeDef;

typedef struct { uint8_t cmd; } IMUReadOnceCmd;
typedef struct {
	uint8_t cmd;
	uint8_t enable;
	uint16_t period_ms;
} IMUStreamCtrlCmd;

typedef struct {
    uint8_t cmd;
    uint8_t motor_id;
} MotorSingalStopCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_mask;
} MotorMultiStopCommandTypeDef;

/* Serial bus servo */
typedef struct {
    uint8_t cmd;
    uint8_t servo_id;
    uint8_t args[];
} SerialServoCommandTypeDef;

/* Serial bus servo */
typedef struct {
    uint8_t cmd;
    uint8_t servo_num;
	uint8_t args[];
} SerialServoMultiCommandTypeDef;

/* Serial bus servo */
typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_num;
    struct {
        uint8_t servo_id;
        uint16_t position;
    } elements[];
} SerialServoSetPositionCommandTypeDef;

/* PWM servo */
typedef struct {
    uint8_t cmd;
    uint8_t servo_id;
    uint8_t args[];
} PWM_ServoCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_id;
    uint16_t pulse;
} PWM_ServoSetPositionCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_num;
    struct {
        uint8_t servo_id;
        uint16_t pulse;
    } elements[];
} PWMServoSetMultiPositionCommandTypeDef;

/* LED */
typedef struct {
    uint8_t led_id;
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} LedCommandTypeDef;

typedef struct {
    uint16_t freq;
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} BuzzerCommandTypeDef;


typedef struct {
	uint8_t sub_cmd;
	uint8_t length;
	uint8_t data[];
} OLEDCommandTypeDef;

// Motor type switch
typedef struct {
    uint8_t func;
    uint8_t type;
} MotorTypeCtlTypeDef; 

// Voltage alarm configuration
typedef struct {
    uint8_t cmd;
    uint16_t limit;
} BatteryWarnTypeDef;

// RGB LED structure
typedef struct {
    uint8_t id;
    uint8_t data[];
} RGBCtlTypeDef;

#pragma pack()

#if ENABLE_OLED
static void packet_oled_handle(struct PacketRawFrame *frame)
{	
	extern osMutexId_t oled_mutexHandle;
	extern char oled_l1[];
	extern char oled_l2[];
	OLEDCommandTypeDef *cmd = (OLEDCommandTypeDef*)frame->data_and_checksum;
	osMutexAcquire(oled_mutexHandle, osWaitForever);
	switch(cmd->sub_cmd) {
		case 0x01: { /* Set SSID */
			memcpy(oled_l1, cmd->data, cmd->length);
			oled_l1[cmd->length] = '\0';
			break;
		}
		case 0x02:{ /* Set IP address */
			memcpy(oled_l2, cmd->data, cmd->length);
			oled_l2[cmd->length] = '\0';
			break;
		}
		default: {
			break;
		}
	}
	osMutexRelease(oled_mutexHandle);
}
#endif

static void packet_led_handle(struct PacketRawFrame *frame)
{
    const uint8_t *payload = frame->data_and_checksum;
    const size_t payload_len = frame->data_length;

    if (payload_len == 0U) {
        return;
    }

    if (payload[0] == RRC_IO_LED_SET) {
        if (payload_len <= 1U) {
            return;
        }

        const uint8_t *body = &payload[1];
        size_t body_len = payload_len - 1U;
        uint8_t txid = RRC_TXID_NONE;

        const size_t legacy_simple_len = sizeof(LedCommandTypeDef);
        if (body_len == legacy_simple_len ||
            body_len == (legacy_simple_len + 1U)) {
            if (body_len == (legacy_simple_len + 1U)) {
                txid = body[legacy_simple_len];
                body_len = legacy_simple_len;
            }

            const LedCommandTypeDef *cmd =
                (const LedCommandTypeDef *)body;
            if (cmd->led_id == 0U) {
                const uint32_t now = HAL_GetTick();
                (void)rrc_send_err(RRC_FUNC_IO, RRC_IO_LED_SET,
                                   RRC_SYS_ERR_INVALID_ARG, 0U, now, 0U);
                return;
            }

            const uint8_t led_index = (uint8_t)(cmd->led_id - 1U);
            if (led_index >= 2U) {
                const uint32_t now = HAL_GetTick();
                const uint8_t err_txid =
                    (txid == RRC_TXID_NONE) ? 0U : txid;
                (void)rrc_send_err(RRC_FUNC_IO, RRC_IO_LED_SET,
                                   RRC_SYS_ERR_INVALID_ARG, 0U, now, err_txid);
                return;
            }

            const int apply_rc =
                led_flash(leds[led_index], cmd->on_time, cmd->off_time,
                          cmd->repeat);
            if (apply_rc != 0) {
                rrc_io_recovery_schedule(&g_led_recovery, RRC_FUNC_IO,
                                         RRC_IO_LED_SET, led_index, 0U, 0U);
                return;
            }

            rrc_io_recovery_clear(&g_led_recovery, RRC_FUNC_IO,
                                  RRC_IO_LED_SET);

            const uint8_t mode = (cmd->on_time > 0U) ? 1U : 0U;
            const rrc_io_led_ack_t ack = {
                .txid = txid,
                .mode = mode,
            };

            (void)rrc_send_ack(RRC_FUNC_IO, RRC_IO_LED_SET, &ack, sizeof(ack),
                                txid);
            return;
        }

        if (body_len >= 2U) {
            const uint8_t rgb_id = body[0];
            size_t rgb_data_len = body_len - 1U;
            const uint8_t *rgb_data = &body[1];
            size_t expected_len = 0U;

            if (rgb_id == 0U) {
                expected_len = 6U;
            } else if (rgb_id == 1U || rgb_id == 2U) {
                expected_len = 3U;
            }

            if (expected_len != 0U) {
                if (rgb_data_len == (expected_len + 1U)) {
                    txid = rgb_data[expected_len];
                    rgb_data_len = expected_len;
                }

                if (rgb_data_len == expected_len) {
                    if (rgb_id == 0U) {
                        set_rgb_color((uint8_t *)rgb_data);
                    } else {
                        set_id_rgb_color((uint8_t)(rgb_id - 1U),
                                         (uint8_t *)rgb_data);
                    }

                    const rrc_io_led_ack_t ack = {
                        .txid = txid,
                        .mode = 2U,
                    };

                    rrc_io_recovery_clear(&g_led_recovery, RRC_FUNC_IO,
                                          RRC_IO_LED_SET);
                    (void)rrc_send_ack(RRC_FUNC_IO, RRC_IO_LED_SET, &ack,
                                        sizeof(ack), txid);
                    return;
                }
            }
        }

        const uint32_t now = HAL_GetTick();
        (void)rrc_send_err(RRC_FUNC_IO, RRC_IO_LED_SET,
                           RRC_SYS_ERR_INVALID_ARG, 0U, now, 0U);
        return;
    }

    LedCommandTypeDef *cmd = (LedCommandTypeDef*)payload;
    uint8_t led_id = cmd->led_id - 1U;
    if (led_id < 2U) { /* IDs start from 1 */
        led_flash(leds[led_id], cmd->on_time, cmd->off_time, cmd->repeat);
    }
}

static void packet_buzzer_handle(struct PacketRawFrame *frame)
{
    const uint8_t *payload = frame->data_and_checksum;
    const size_t payload_len = frame->data_length;

    if (payload_len == 0U) {
        return;
    }

    if (payload[0] == RRC_IO_BUZZER_SET) {
        if (payload_len <= 1U) {
            return;
        }

        const uint8_t *body = &payload[1];
        size_t body_len = payload_len - 1U;
        uint8_t txid = RRC_TXID_NONE;

        const size_t new_len = 5U; /* freq(2) + duty(1) + duration(2) */
        if (body_len == new_len || body_len == (new_len + 1U)) {
            if (body_len == (new_len + 1U)) {
                txid = body[new_len];
                body_len = new_len;
            }

            const uint16_t freq_hz = (uint16_t)body[0] |
                                      ((uint16_t)body[1] << 8);
            uint8_t duty_pct = body[2];
            const uint16_t duration_ms = (uint16_t)body[3] |
                                          ((uint16_t)body[4] << 8);

            if (duty_pct > 100U) {
                duty_pct = 100U;
            }

            int apply_rc;
            if (freq_hz == 0U || duty_pct == 0U || duration_ms == 0U) {
                apply_rc = buzzer_off(buzzers[0]);
            } else {
                const uint32_t on_time =
                    ((uint32_t)duration_ms * (uint32_t)duty_pct) / 100U;
                const uint32_t off_time = duration_ms - on_time;
                apply_rc = buzzer_didi(buzzers[0], freq_hz, on_time, off_time,
                                        1U);
            }

            if (apply_rc != 0) {
                rrc_io_recovery_schedule(&g_buzzer_recovery, RRC_FUNC_IO,
                                         RRC_IO_BUZZER_SET, 0U, freq_hz,
                                         duration_ms);
                return;
            }

            rrc_io_recovery_clear(&g_buzzer_recovery, RRC_FUNC_IO,
                                  RRC_IO_BUZZER_SET);

            const rrc_io_buzzer_ack_t ack = {
                .txid = txid,
                .freq_hz = freq_hz,
                .duty_pct = duty_pct,
                .duration_ms = duration_ms,
            };

            (void)rrc_send_ack(RRC_FUNC_IO, RRC_IO_BUZZER_SET, &ack,
                                sizeof(ack), txid);
            return;
        }

        const size_t legacy_len = sizeof(BuzzerCommandTypeDef);
        if (body_len == legacy_len || body_len == (legacy_len + 1U)) {
            if (body_len == (legacy_len + 1U)) {
                txid = body[legacy_len];
                body_len = legacy_len;
            }

            const BuzzerCommandTypeDef *cmd =
                (const BuzzerCommandTypeDef *)body;
            const int apply_rc =
                buzzer_didi(buzzers[0], cmd->freq, cmd->on_time,
                            cmd->off_time, cmd->repeat);

            if (apply_rc != 0) {
                rrc_io_recovery_schedule(&g_buzzer_recovery, RRC_FUNC_IO,
                                         RRC_IO_BUZZER_SET, 0U, cmd->freq,
                                         cmd->on_time);
                return;
            }

            rrc_io_recovery_clear(&g_buzzer_recovery, RRC_FUNC_IO,
                                  RRC_IO_BUZZER_SET);

            uint8_t duty_pct = 0U;
            const uint32_t total =
                (uint32_t)cmd->on_time + (uint32_t)cmd->off_time;
            if (total > 0U) {
                uint32_t duty_calc =
                    ((uint32_t)cmd->on_time * 100U) / total;
                if (duty_calc > 100U) {
                    duty_calc = 100U;
                }
                duty_pct = (uint8_t)duty_calc;
            }

            const rrc_io_buzzer_ack_t ack = {
                .txid = txid,
                .freq_hz = cmd->freq,
                .duty_pct = duty_pct,
                .duration_ms = cmd->on_time,
            };

            (void)rrc_send_ack(RRC_FUNC_IO, RRC_IO_BUZZER_SET, &ack,
                                sizeof(ack), txid);
            return;
        }

        const uint32_t now = HAL_GetTick();
        const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
        (void)rrc_send_err(RRC_FUNC_IO, RRC_IO_BUZZER_SET,
                           RRC_SYS_ERR_INVALID_ARG, 0U, now, err_txid);
        return;
    }

    BuzzerCommandTypeDef *cmd = (BuzzerCommandTypeDef*)payload;
    buzzer_didi(buzzers[0], cmd->freq, cmd->on_time, cmd->off_time,
                cmd->repeat);
}


static void packet_serial_servo_report_init(PacketReportSerialServoTypeDef * report, uint8_t servo_id, uint8_t cmd, int success)
{
    report->servo_id = servo_id;
    report->sub_command = cmd;
    report->success = (uint8_t)((int8_t)success);
}

static void packet_serial_servo_handle(struct PacketRawFrame *frame)
{
    PacketReportSerialServoTypeDef report;
    switch(frame->data_and_checksum[0]) {
        case 0x01: { /* Servo control */
            SerialServoSetPositionCommandTypeDef *cmd=(SerialServoSetPositionCommandTypeDef *)frame->data_and_checksum;
            bool all_ok = true;
            for(int i = 0; i < cmd->servo_num; i++) {
                const uint8_t servo_id = cmd->elements[i].servo_id;
                const uint16_t position = cmd->elements[i].position;
                const uint16_t duration = cmd->duration;
                if (serial_servo_set_position(&serial_servo_controller,
                                              servo_id, (int)position,
                                              duration) != 0) {
                    rrc_io_recovery_schedule(&g_steering_recovery,
                                             PACKET_FUNC_BUS_SERVO, 0x01U,
                                             servo_id, position, duration);
                    all_ok = false;
                    break;
                }
            }

            if (all_ok) {
                rrc_io_recovery_clear(&g_steering_recovery,
                                      PACKET_FUNC_BUS_SERVO, 0x01U);
            }
            break;
        }
        case 0x03: { /* Stop servo */
            SerialServoMultiCommandTypeDef *cmd = (SerialServoMultiCommandTypeDef *)frame->data_and_checksum;
			for(int i = 0; i < cmd->servo_num; i++) {
				serial_servo_stop(&serial_servo_controller, cmd->args[i]);
			}
            break;
        }
        case 0x05: { /* Read position */
            int16_t position = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd,  
					                                  serial_servo_read_position(&serial_servo_controller, 
					                                                             cmd->servo_id, &position));
            memcpy(report.args, &position, 2);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 5);
            break;
        }
        case 0x07: { /* Read input voltage */
            uint16_t vin = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_vin(&serial_servo_controller, cmd->servo_id, &vin));
            memcpy(report.args, &vin, 2);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 5);
            break;
        }
        case 0x09: { /* Read temperature */
            uint8_t temp = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd,  serial_servo_read_temp(&serial_servo_controller, cmd->servo_id, &temp));
            report.args[0] = temp;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }
        case 0x0B: { /* Disable torque */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_load_unload(&serial_servo_controller, cmd->servo_id, 0);
            break;
        }
        case 0x0C: { /* Enable torque */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_load_unload(&serial_servo_controller, cmd->servo_id, 1);
            break;
        }
		    case 0x0D: { /* Read torque state */
            uint8_t load_unload;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_load_unload(&serial_servo_controller, cmd->servo_id, &load_unload));
            report.args[0] = load_unload;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
		}			
        case 0x10: { /* Write ID */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_id(&serial_servo_controller, cmd->servo_id, cmd->args[0]);
            break;
        }
        case 0x12: { /* Read ID */
            uint8_t servo_id;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_id(&serial_servo_controller, cmd->servo_id, &servo_id));
            report.args[0] = servo_id;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }
        case 0x20: { /* Adjust offset */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_deviation(&serial_servo_controller, cmd->servo_id, cmd->args[0]);
            break;
        }
        case 0x22: { /* Read offset */
            int8_t dev = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_deviation(&serial_servo_controller, cmd->servo_id, &dev));
            report.args[0] = (uint8_t)dev;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }
        case 0x24: { /* Save offset */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_save_deviation(&serial_servo_controller, cmd->servo_id);
            break;
        }
        case 0x30: { /* Configure position limits */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_angle_limit(&serial_servo_controller, cmd->servo_id, *((uint16_t*)(&cmd->args[0])), *((uint16_t*)(&cmd->args[2])));
            break;
        }
        case 0x32: { /* Read position limits */
            uint16_t limit[2] = {0};
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_angle_limit(&serial_servo_controller, cmd->servo_id, limit));
            memcpy(&report.args, limit, 4);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 7);
            break;
        }
        case 0x34: { /* Configure voltage limits */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_vin_limit(&serial_servo_controller, cmd->servo_id, *((uint16_t*)(&cmd->args[0])), *((uint16_t*)(&cmd->args[2])));
            break;
        }
        case 0x36: { /* Read voltage limits */
            uint16_t limit[2] = {0};
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_vin_limit(&serial_servo_controller, cmd->servo_id, limit));
            memcpy(&report.args, limit, 4);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 7);
            break;
        }
        case 0x38: { /* Configure temperature limits */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_temp_limit(&serial_servo_controller, cmd->servo_id, cmd->args[0]);
            break;
        }
        case 0x3A: { /* Read temperature limits */
            uint8_t limit = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_temp_limit(&serial_servo_controller, cmd->servo_id, &limit));
            report.args[0] = limit;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }

        default:
            break;
    }
}

static void packet_pwm_servo_handle(struct PacketRawFrame *frame)
{
    const uint8_t sub = frame->data_and_checksum[0];

    if (sub == RRC_IO_BUTTON_ONESHOT) {
        const uint8_t mask = buttons_read_mask();
        (void)rrc_transport_send(RRC_FUNC_IO, RRC_IO_BUTTON_ONESHOT,
                                 &mask, sizeof(mask));
        return;
    }

    if (sub == RRC_IO_BUTTON_STREAM_CTRL) {
        const uint8_t *payload = frame->data_and_checksum;
        const size_t payload_len = frame->data_length;
        if (payload_len < 4U) {
            return;
        }

        uint8_t txid = RRC_TXID_NONE;
        if (payload_len == 5U) {
            txid = payload[4];
        } else if (payload_len != 4U) {
            return;
        }

        const uint8_t requested_enable = payload[1];
        const uint16_t requested_period =
            (uint16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
        const uint16_t applied_period =
            buttons_set_stream(requested_enable, requested_period);

        rrc_button_stream_ack_t ack = {
            .txid = txid,
            .enable = (uint8_t)(requested_enable ? 1U : 0U),
            .period_ms_le = applied_period,
        };

        (void)rrc_send_ack(RRC_FUNC_IO, RRC_IO_BUTTON_STREAM_CTRL,
                            &ack, sizeof(ack), txid);
        return;
    }

    switch(frame->data_and_checksum[0]) {
        case 0x01: {    // Control multiple servos
            PWMServoSetMultiPositionCommandTypeDef *cmd = (PWMServoSetMultiPositionCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < cmd->servo_num; ++i) {
                if(cmd->elements[i].servo_id <= 4) {
                    pwm_servo_set_position( pwm_servos[cmd->elements[i].servo_id - 1], cmd->elements[i].pulse, cmd->duration);
                }
            }
            break;
        }
        case 0x03: {    // Control a single servo
            PWM_ServoSetPositionCommandTypeDef *cmd = (PWM_ServoSetPositionCommandTypeDef *)frame->data_and_checksum;
            // Host indexes servos starting at ID 1
            if(cmd->servo_id <= 4) {
                pwm_servo_set_position( pwm_servos[cmd->servo_id - 1], cmd->pulse, cmd->duration );
            }
            break;
        }
        case 0x05: { // Read current servo position
            PWM_ServoCommandTypeDef *cmd = (PWM_ServoCommandTypeDef*)frame->data_and_checksum;
            if(cmd->servo_id <= 4) {
                uint16_t pulse = pwm_servos[cmd->servo_id - 1]->current_duty;
                PacketReportPWMServoTypeDef report;
                report.servo_id = cmd->servo_id;
                report.sub_command = cmd->cmd;
                memcpy(report.args, &pulse, 2);
                packet_transmit(&packet_controller, PACKET_FUNC_PWM_SERVO, &report, 4);
            }
            break;
        }
        case 0x07: { // Set servo offset
            PWM_ServoCommandTypeDef *cmd = (PWM_ServoCommandTypeDef*)frame->data_and_checksum;
            if(cmd->servo_id <= 4) {
                pwm_servo_set_offset(pwm_servos[cmd->servo_id - 1], ((int)((int8_t)cmd->args[0])));
            }
            break;
        }
        case 0x09: { // Read servo offset
            PWM_ServoCommandTypeDef *cmd = (PWM_ServoCommandTypeDef*)frame->data_and_checksum;
            if(cmd->servo_id <= 4) {
                int offset = pwm_servos[cmd->servo_id - 1]->offset;
                PacketReportPWMServoTypeDef report;
                report.servo_id = cmd->servo_id;
                report.sub_command = cmd->cmd;
                report.args[0] = (uint8_t)((int8_t)offset);
                packet_transmit(&packet_controller, PACKET_FUNC_PWM_SERVO, &report, 3);
            }
            break;
        }
        default:
            break;
    }
}

static void packet_motor_handle(struct PacketRawFrame *frame)
{
    extern void motor_set_target_pwm(uint8_t id, int cmd);
    extern EncoderMotorObjectTypeDef *motors[2];   // only used by other cases here

    switch (frame->data_and_checksum[0]) {

        /* ---------------- PWM (open-loop) ---------------- */

        case 0x10: { /* single motor PWM set */
            const uint8_t *payload = frame->data_and_checksum;
            const size_t payload_len = frame->data_length;
            if (payload_len < 4U) {
                break;
            }

            uint8_t txid = RRC_TXID_NONE;
            if (payload_len == 5U) {
                txid = payload[4];
            } else if (payload_len != 4U) {
                const uint32_t now = HAL_GetTick();
                const uint8_t err_txid = (payload_len > 4U) ? payload[4] : 0U;
                if (!motor_pwm_fault_active) {
                    motor_pwm_fault_active = true;
                    motor_pwm_last_error = RRC_SYS_ERR_INVALID_ARG;
                    (void)rrc_send_err(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET,
                                       motor_pwm_last_error,
                                       (uint8_t)payload_len, now, err_txid);
                }
                break;
            }

            const uint8_t raw_id = payload[1];
            int pwm = (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));

            if (pwm > 1000) {
                pwm = 1000;
            } else if (pwm < -1000) {
                pwm = -1000;
            }

            const bool applied = motor_pwm_apply(raw_id & 0x03U, pwm);
            const uint32_t now = HAL_GetTick();

            if (!applied) {
                const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
                motor_backoff_ensure_init();
                motor_pwm_last_error = RRC_SYS_ERR_IO_FAIL;
                if (g_motor_err_active == 0U) {
                    g_motor_err_active = 1U;
                    (void)rrc_send_err(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET,
                                       RRC_SYS_ERR_IO_FAIL, raw_id,
                                       now, err_txid);
                    rrc_backoff_reset(&g_motor_backoff);
                    const uint32_t delay_ms = rrc_backoff_next(&g_motor_backoff);
                    g_motor_retry_due_ms = now + delay_ms;
                }
                break;
            }

            if (motor_pwm_fault_active) {
                motor_pwm_fault_active = false;
                if (motor_pwm_last_error != RRC_SYS_ERR_IO_FAIL) {
                    (void)rrc_send_recovered(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET,
                                             motor_pwm_last_error, now);
                }
            }

            const uint8_t id = (uint8_t)(raw_id & 0x03U);

            int16_t applied_pwm = 0;
            if (id < 2U) {
                applied_pwm = (int16_t)motors_pwm_current[id];
            }

            rrc_motor_last_cmd_ms = now;

            rrc_motor_pwm_ack_t ack = {
                .txid = txid,
                .motor_id = id,
                .pwm_target = (int16_t)pwm,
                .pwm_applied = applied_pwm,
            };

            (void)rrc_send_ack(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_ACK_SINGLE,
                                &ack, sizeof(ack), txid);
            break;
        }

        case 0x11: { /* multi motor PWM set */
            MotorPWMMultiCommandTypeDef *cmd = (void*)frame->data_and_checksum;

            /* apply targets */
            uint8_t n = cmd->motor_num;
            if (n > 2) n = 2;   /* we only have M1/M2 right now */

            /* build ACK */
            PacketReportMotorPwmAck_Multi rep;
            rep.sub   = 0x19;
            rep.t_ms  = HAL_GetTick();
            rep.count = n;

            for (uint8_t i = 0; i < n; ++i) {
                uint8_t id = (uint8_t)(cmd->element[i].motor_id & 0x03);
                int pwm = (int)cmd->element[i].pwm;
                if (pwm > 1000) pwm = 1000; else if (pwm < -1000) pwm = -1000;
                motor_set_target_pwm(id, pwm);

                rep.item[i].motor_id    = id;
                rep.item[i].pwm_target  = (int16_t)pwm;
                rep.item[i].pwm_applied = (int16_t)motors_pwm_current[id];
            }
            packet_transmit(&packet_controller, PACKET_FUNC_MOTOR, &rep, sizeof(rep));
            break;
        }

        /* ---------------- Manufacturer velocity/stop/type (keep as-is) ---------------- */

        case 0: {  /* single motor velocity mode (manufacturer) */
            MotorSingalCtrlCommandTypeDef *mscc = (MotorSingalCtrlCommandTypeDef *)frame->data_and_checksum;
            motors[mscc->motor_id]->pid_controller.set_point = mscc->speed;
            break;
        }
        case 1: {  /* multi motor velocity mode (manufacturer) */
            MotorMutilCtrlCommandTypeDef *mmcc = (MotorMutilCtrlCommandTypeDef *)frame->data_and_checksum;
            for (int i = 0; i < mmcc->motor_num; ++i) {
                motors[mmcc->element[i].motor_id]->pid_controller.set_point = mmcc->element[i].speed;
            }
            break;
        }
        case 2: {  /* single motor stop (manufacturer) */
            MotorSingalStopCommandTypeDef *mssc = (MotorSingalStopCommandTypeDef *)frame->data_and_checksum;
            motors[mssc->motor_id]->pid_controller.set_point = 0;
            break;
        }
        case 3: {  /* multi motor stop (manufacturer) */
            MotorMultiStopCommandTypeDef *mmsc = (MotorMultiStopCommandTypeDef *)frame->data_and_checksum;
            for (int i = 0; i < 4; ++i) {
                if (mmsc->motor_mask & (0x01 << i)) {
                    motors[i]->pid_controller.set_point = 0;
                }
            }
            break;
        }
        case 5: {  /* motor type switch (manufacturer) */
            MotorTypeCtlTypeDef *mmsc = (MotorTypeCtlTypeDef *)frame->data_and_checksum;
            MotorTypeEnum type = MOTOR_TYPE_JGA27;
            if      (mmsc->type == MOTOR_TYPE_JGB520) type = MOTOR_TYPE_JGB520;
            else if (mmsc->type == MOTOR_TYPE_JGB37)  type = MOTOR_TYPE_JGB37;
            else if (mmsc->type == MOTOR_TYPE_JGA27)  type = MOTOR_TYPE_JGA27;
            else if (mmsc->type == MOTOR_TYPE_JGB528) type = MOTOR_TYPE_JGB528;
            else                                      type = MOTOR_TYPE_JGB520;
            for (int i = 0; i < 4; ++i) {
                set_motor_type(motors[i], type);
            }
            break;
        }

        default:
            break;
    }
}

static void packet_imu_handle(struct PacketRawFrame *frame)
{
    const uint8_t *payload = frame->data_and_checksum;
    const size_t payload_len = frame->data_length;

    if (payload_len == 0U) {
        return;
    }

    const uint8_t sub = payload[0];

    switch (sub) {
    case RRC_IMU_ONESHOT: {
        if (payload_len >= 2U) {
            const uint8_t sources_mask = payload[1];
            imu_emit_oneshot(sources_mask);
        }
        break;
    }
    case RRC_IMU_STREAM_CTRL: {
        if (payload_len < 5U) {
            break;
        }

        uint8_t txid = RRC_TXID_NONE;
        if (payload_len == 6U) {
            txid = payload[5];
        } else if (payload_len != 5U) {
            break;
        }

        const uint8_t requested_mask = payload[1];
        const uint16_t requested_period =
            (uint16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
        const uint8_t requested_ack = payload[4];

        uint8_t applied_mask = 0U;
        uint8_t applied_ack = 0U;
        const uint16_t applied_period =
            imu_set_stream(requested_mask, requested_period, requested_ack,
                           &applied_mask, &applied_ack);

        rrc_imu_stream_ack_t ack = {
            .txid = txid,
            .sources_mask = applied_mask,
            .period_ms_le = applied_period,
            .ack_each_frame = applied_ack,
        };

        (void)rrc_send_ack(RRC_FUNC_IMU, RRC_IMU_STREAM_CTRL,
                            &ack, sizeof(ack), txid);
        break;
    }
    case RRC_IMU_SET_PRIMARY: {
        if (payload_len < 2U) {
            break;
        }

        uint8_t txid = RRC_TXID_NONE;
        if (payload_len == 3U) {
            txid = payload[2];
        } else if (payload_len != 2U) {
            break;
        }

        const uint8_t source_id = payload[1];
        if (source_id > 1U) {
            const uint32_t now = HAL_GetTick();
            const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
            (void)rrc_send_err(RRC_FUNC_IMU, RRC_IMU_SET_PRIMARY,
                               RRC_SYS_ERR_INVALID_ARG, 0U, now, err_txid);
            break;
        }

        g_imu_primary = source_id;

        const rrc_imu_primary_ack_t ack = {
            .txid = txid,
            .source_id = source_id,
        };

        (void)rrc_send_ack(RRC_FUNC_IMU, RRC_IMU_SET_PRIMARY,
                            &ack, sizeof(ack), txid);
        break;
    }
    case RRC_IMU_SET_PRESET: {
        if (payload_len < 3U) {
            break;
        }

        uint8_t txid = RRC_TXID_NONE;
        if (payload_len == 4U) {
            txid = payload[3];
        } else if (payload_len != 3U) {
            break;
        }

        const uint8_t source_id = payload[1];
        if (source_id > 1U) {
            const uint32_t now = HAL_GetTick();
            const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
            (void)rrc_send_err(RRC_FUNC_IMU, RRC_IMU_SET_PRESET,
                               RRC_SYS_ERR_INVALID_ARG, 0U, now, err_txid);
            break;
        }

        uint8_t applied_preset = payload[2];
        if (applied_preset > 2U) {
            applied_preset = 2U;
        }

        g_imu_preset[source_id] = applied_preset;

        const rrc_imu_preset_ack_t ack = {
            .txid = txid,
            .source_id = source_id,
            .preset = applied_preset,
        };

        (void)rrc_send_ack(RRC_FUNC_IMU, RRC_IMU_SET_PRESET,
                            &ack, sizeof(ack), txid);
        break;
    }
    case RRC_IMU_SET_BIASES: {
        const size_t min_len =
            1U + 1U + sizeof(rrc_imu_bias_store_t); /* sub + source + bias */
        if (payload_len < min_len) {
            break;
        }

        uint8_t txid = RRC_TXID_NONE;
        if (payload_len == (min_len + 1U)) {
            txid = payload[min_len];
        } else if (payload_len != min_len) {
            break;
        }

        const uint8_t source_id = payload[1];
        if (source_id > 1U) {
            const uint32_t now = HAL_GetTick();
            const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
            (void)rrc_send_err(RRC_FUNC_IMU, RRC_IMU_SET_BIASES,
                               RRC_SYS_ERR_INVALID_ARG, 0U, now, err_txid);
            break;
        }

        const size_t bias_offset = 2U; /* sub + source */
        rrc_imu_bias_store_t biases;
        memcpy(&biases, &payload[bias_offset], sizeof(biases));
        g_imu_bias[source_id] = biases;

        const rrc_imu_bias_ack_t ack = {
            .txid = txid,
            .source_id = source_id,
        };

        (void)rrc_send_ack(RRC_FUNC_IMU, RRC_IMU_SET_BIASES,
                            &ack, sizeof(ack), txid);
        break;
    }
    case RRC_IMU_WHOAMI_STATUS: {
        uint8_t source_id = 0U;
        if (payload_len >= 2U) {
            source_id = payload[1];
        }

        imu_emit_whoami(source_id);
        break;
    }
    default:
        break;
    }
}

static void packet_battery_limit_handle(struct PacketRawFrame *frame)
{
    const uint8_t *payload = frame->data_and_checksum;
    const size_t payload_len = frame->data_length;

    if (payload_len == 0U) {
        return;
    }

    switch (payload[0]) {
        case RRC_SYS_BATTERY_ONESHOT: {
            const uint16_t mv = battery_latest_millivolts_le();
            (void)rrc_transport_send(RRC_FUNC_SYS, RRC_SYS_BATTERY_ONESHOT,
                                     &mv, sizeof(mv));
            break;
        }
        case RRC_SYS_BATTERY_STREAM_CTRL: {
            if (payload_len < 4U) {
                break;
            }

            uint8_t txid = RRC_TXID_NONE;
            if (payload_len == 5U) {
                txid = payload[4];
            } else if (payload_len != 4U) {
                break;
            }

            const uint8_t requested_enable = payload[1];
            const uint16_t requested_period =
                (uint16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
            const uint16_t applied_period =
                battery_set_stream(requested_enable, requested_period);

            rrc_sys_battery_stream_ack_t ack = {
                .txid = txid,
                .enable = (uint8_t)(requested_enable ? 1U : 0U),
                .period_ms_le = applied_period,
            };

            (void)rrc_send_ack(RRC_FUNC_SYS, RRC_SYS_BATTERY_STREAM_CTRL,
                                &ack, sizeof(ack), txid);
            break;
        }
        case RRC_SYS_MOTOR_FAILSAFE_SET: {
            if (payload_len < 3U) {
                break;
            }

            uint8_t txid = RRC_TXID_NONE;
            if (payload_len == 4U) {
                txid = payload[3];
            } else if (payload_len != 3U) {
                break;
            }

            const uint16_t timeout_ms =
                (uint16_t)((uint16_t)payload[1] | ((uint16_t)payload[2] << 8));

            rrc_motor_failsafe_timeout_ms = timeout_ms;
            rrc_motor_last_cmd_ms = HAL_GetTick();

            const rrc_sys_motor_failsafe_ack_t ack = {
                .txid = txid,
                .timeout_ms_le = timeout_ms,
            };

            (void)rrc_send_ack(RRC_FUNC_SYS, RRC_SYS_MOTOR_FAILSAFE_SET,
                                &ack, sizeof(ack), txid);
            break;
        }
        case RRC_SYS_HEALTH_PERIOD_SET: {
            if (payload_len < 3U) {
                break;
            }

            uint8_t txid = RRC_TXID_NONE;
            if (payload_len == 4U) {
                txid = payload[3];
            } else if (payload_len != 3U) {
                break;
            }

            const uint16_t period_ms =
                (uint16_t)((uint16_t)payload[1] | ((uint16_t)payload[2] << 8));

            rrc_heartbeat_period_ms = period_ms;

            const rrc_sys_period_ack_t ack = {
                .txid = txid,
                .period_ms_le = period_ms,
            };

            (void)rrc_send_ack(RRC_FUNC_SYS, RRC_SYS_HEALTH_PERIOD_SET,
                                &ack, sizeof(ack), txid);
            break;
        }
        case RRC_SYS_UART_BAUD_SET: {
            if (payload_len < 7U) {
                break;
            }

            uint8_t txid = RRC_TXID_NONE;
            if (payload_len == 8U) {
                txid = payload[7];
            } else if (payload_len != 7U) {
                break;
            }

            const uint32_t baud = (uint32_t)payload[1] |
                                   ((uint32_t)payload[2] << 8) |
                                   ((uint32_t)payload[3] << 16) |
                                   ((uint32_t)payload[4] << 24);
            uint16_t apply_after_ms =
                (uint16_t)((uint16_t)payload[5] | ((uint16_t)payload[6] << 8));

            if (!rrc_uart_baud_is_supported(baud)) {
                const uint8_t err_txid = (txid == RRC_TXID_NONE) ? 0U : txid;
                (void)rrc_send_err(RRC_FUNC_SYS, RRC_SYS_UART_BAUD_SET,
                                   RRC_SYS_ERR_UNSUPPORTED, 0U, HAL_GetTick(),
                                   err_txid);
                break;
            }

            if (apply_after_ms == 0U) {
                apply_after_ms = rrc_uart_baud_apply_delay_ms(baud);
            }

            const rrc_sys_uart_baud_ack_t ack = {
                .txid = txid,
                .baud_le = baud,
                .apply_after_ms_le = apply_after_ms,
            };

            if (!rrc_send_ack(RRC_FUNC_SYS, RRC_SYS_UART_BAUD_SET,
                              &ack, sizeof(ack), txid)) {
                break;
            }

            rrc_uart_apply_with_delay(baud, apply_after_ms, txid);
            break;
        }
        case RRC_SYS_UART_BAUD_GET: {
            const uint32_t baud_le = rrc_uart_current_baud();
            (void)rrc_transport_send(RRC_FUNC_SYS, RRC_SYS_UART_BAUD_GET,
                                     &baud_le, sizeof(baud_le));
            break;
        }
        case 1: {
            if (payload_len >= sizeof(BatteryWarnTypeDef)) {
                const BatteryWarnTypeDef *cmd = (const BatteryWarnTypeDef*)payload;
                change_battery_limit(cmd->limit);
            }
            break;
        }
        default:
            break;
    }
}

static void packet_RGB_Ctl_handle(struct PacketRawFrame *frame)
{
    RGBCtlTypeDef *cmd = (RGBCtlTypeDef*)frame->data_and_checksum;
    switch(cmd->id) {
        case 0: {
//            for(int i = 0 ; i < Pixel_S1_NUM ; i++)
//            {
//                set_id_rgb_color(i , &cmd->data[i*3]);
//            }
            set_rgb_color(cmd->data);
        }break;
        
        case 1: 
        case 2: {
            set_id_rgb_color((cmd->id-1) , cmd->data);
        }break;
        
        default:
            break;
    }
}

static void packet_encoder_handle(struct PacketRawFrame *frame)
{
    switch (frame->data_and_checksum[0]) {
        case 0x90: { /* one-shot now */
            encoders_read_once_and_report(0x90);
            break;
        }
        case 0x91: { /* stream control */
            const uint8_t *payload = frame->data_and_checksum;
            const size_t payload_len = frame->data_length;
            if (payload_len < 4U) {
                break;
            }

            uint8_t txid = RRC_TXID_NONE;
            if (payload_len == 5U) {
                txid = payload[4];
            } else if (payload_len != 4U) {
                break;
            }

            const uint8_t requested_enable = payload[1];
            const uint16_t requested_period =
                (uint16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
            const uint16_t applied_period =
                encoders_set_stream(requested_enable, requested_period);

            rrc_encoder_stream_ack_t ack = {
                .txid = txid,
                .enable = (uint8_t)(requested_enable ? 1U : 0U),
                .period_ms_le = applied_period,
            };

            (void)rrc_send_ack(RRC_FUNC_MOTOR, RRC_MOTOR_ENCODER_STREAM_CTRL,
                                &ack, sizeof(ack), txid);
            break;
        }
        default: break;
    }
}

void packet_handle_init(void)
{
    packet_register_callback(&packet_controller, PACKET_FUNC_LED, packet_led_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_BUZZER, packet_buzzer_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_MOTOR, packet_motor_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_BUS_SERVO, packet_serial_servo_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_PWM_SERVO, packet_pwm_servo_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_IMU, packet_imu_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_SYS, packet_battery_limit_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_RGB, packet_RGB_Ctl_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_ENCODER, packet_encoder_handle);
#if ENABLE_OLED
    packet_register_callback(&packet_controller, PACKET_FUNC_OLED, packet_oled_handle);
#endif

    rrc_backoff_init(&g_motor_backoff, 50U, 3.0f, 1000U);
    g_motor_err_active = 0U;
    g_motor_retry_due_ms = 0U;
}
