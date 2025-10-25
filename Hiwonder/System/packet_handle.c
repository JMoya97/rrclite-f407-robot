#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"
#include "serial_servo.h"
#include "packet_reports.h"
#include "motors_param.h"
#include "tim.h"
#include "cmsis_os2.h"
#include "rrclite_packets.h"

#include <stdbool.h>

extern uint16_t encoders_set_stream(uint8_t enable, uint16_t period_ms);
extern void encoders_read_once_and_report(uint8_t sub);
extern void imu_set_stream(uint8_t enable, uint16_t period_ms);
extern void imu_read_once_and_report(uint8_t sub);
extern volatile int motors_pwm_current[2];

static bool motor_pwm_fault_active;
static rrc_error_code_t motor_pwm_last_error;

static bool motor_pwm_try_reinit(void)
{
    /* Stub hook for quick reinitialisation after an apply failure. */
    return true;
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
    LedCommandTypeDef *cmd = (LedCommandTypeDef*)frame->data_and_checksum;
    uint8_t led_id = cmd->led_id - 1;
    if(led_id < 2) { /* IDs start from 1 */
        led_flash(leds[led_id], cmd->on_time, cmd->off_time, cmd->repeat);
    }
}

static void packet_buzzer_handle(struct PacketRawFrame *frame)
{
    BuzzerCommandTypeDef *cmd = (BuzzerCommandTypeDef*)frame->data_and_checksum;
    buzzer_didi(buzzers[0], cmd->freq, cmd->on_time, cmd->off_time, cmd->repeat);
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
            for(int i = 0; i < cmd->servo_num; i++) {
                serial_servo_set_position(&serial_servo_controller, cmd->elements[i].servo_id,
                            							cmd->elements[i].position, cmd->duration);
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
                (void)motor_pwm_try_reinit();
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
                if (!motor_pwm_fault_active) {
                    motor_pwm_fault_active = true;
                    motor_pwm_last_error = RRC_SYS_ERR_INVALID_ARG;
                    (void)rrc_send_err(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET,
                                       motor_pwm_last_error, raw_id,
                                       now, err_txid);
                }

                (void)motor_pwm_try_reinit();
                break;
            }

            if (motor_pwm_fault_active) {
                motor_pwm_fault_active = false;
                (void)rrc_send_recovered(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET,
                                         motor_pwm_last_error, now);
            }

            const uint8_t id = (uint8_t)(raw_id & 0x03U);

            int16_t applied_pwm = 0;
            if (id < 2U) {
                applied_pwm = (int16_t)motors_pwm_current[id];
            }

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
    switch (frame->data_and_checksum[0]) {
        case 0xA0: { /* one-shot now */
            imu_read_once_and_report(0xA0);
            break;
        }
        case 0xA1: { /* stream control */
            const IMUStreamCtrlCmd* c = (const IMUStreamCtrlCmd*)frame->data_and_checksum;
            uint16_t p = c->period_ms; if (p < 5) p = 5;
            imu_set_stream(c->enable, p);
            break;
        }
        default: break;
    }
}

static void packet_battery_limit_handle(struct PacketRawFrame *frame)
{
    BatteryWarnTypeDef *cmd = (BatteryWarnTypeDef*)frame->data_and_checksum;
    switch(frame->data_and_checksum[0]) {
        case 1: {
            change_battery_limit(cmd->limit);
        }break;
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
}
