#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"
#include "serial_servo.h"

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

typedef struct {
    uint8_t cmd;
    uint8_t motor_id;
} MotorSingalStopCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_mask;
} MotorMultiStopCommandTypeDef;

/* 串口舵机 */
typedef struct {
    uint8_t cmd;
    uint8_t servo_id;
} SerialServoCommandTypeDef;

/* 串口舵机 */
typedef struct {
    SerialServoCommandTypeDef base;
    uint16_t duration;
    struct {
        uint8_t servo_id;
        uint16_t position;
    } elements[];
} SerialServoSetPositionCommandTypeDef;

/* PWM 舵机 */
typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_id;
    uint16_t pulse;
} PWM_ServoSetPositionCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint16_t duration;
    struct {
        uint8_t servo_id;
        uint16_t pulse;
    } elements;
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

#pragma pack()

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
void packet_led_handle(struct PacketRawFrame *frame)
{
    LedCommandTypeDef *cmd = (LedCommandTypeDef*)frame->data_and_checksum;
    uint8_t led_id = cmd->led_id - 1;
    if(led_id < 1) { /* ID 都是从 1 开始 */
        led_flash(leds[led_id], cmd->on_time, cmd->off_time, cmd->repeat);
    }
}

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
void packet_buzzer_handle(struct PacketRawFrame *frame)
{
    BuzzerCommandTypeDef *cmd = (BuzzerCommandTypeDef*)frame->data_and_checksum;
    buzzer_didi(buzzers[0], cmd->freq, cmd->on_time, cmd->off_time, cmd->repeat);
}


void packet_serial_servo_handle(struct PacketRawFrame *frame)
{
    switch(frame->data_and_checksum[0]) {
        case 0x01: {
			SerialServoSetPositionCommandTypeDef *cmd = (SerialServoSetPositionCommandTypeDef *)frame->data_and_checksum;
			for(int i = 0; i < cmd->base.servo_id; i++) {
				serial_servo_set_position(cmd->elements[i].servo_id, cmd->elements[i].position, cmd->duration);
			} 
			break;
		}
		default:
			break;
    }
}
/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/

void packet_motor_handle(struct PacketRawFrame *frame)
{

    switch(frame->data_and_checksum[0]) {
        case 0: {
            MotorSingalCtrlCommandTypeDef *mscc = (MotorSingalCtrlCommandTypeDef *)frame->data_and_checksum;
            motors[mscc->motor_id]->pid_controller.set_point = mscc->speed;
            break;
        }
        case 1: {
            MotorMutilCtrlCommandTypeDef *mmcc = NULL;
            mmcc = (MotorMutilCtrlCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < mmcc->motor_num; ++i) {
                motors[mmcc->element[i].motor_id]->pid_controller.set_point = mmcc->element[i].speed;
            }
            break;
        }
        case 2:  {
            MotorSingalStopCommandTypeDef *mssc = (MotorSingalStopCommandTypeDef *)frame->data_and_checksum;
            motors[mssc->motor_id]->pid_controller.set_point = 0;
            break;
        }
        case 3: {
            MotorMultiStopCommandTypeDef *mmsc = (MotorMultiStopCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < 4; ++i) {
                if(mmsc->motor_mask & (0x01 << i)) {
                    motors[i]->pid_controller.set_point = 0;
                }
            }
            break;
        }
        default:
            break;
    }
}


void packet_handle_init(void)
{
    packet_register_callback(&packet_controller, PACKET_FUNC_LED, packet_led_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_BUZZER, packet_buzzer_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_MOTOR, packet_motor_handle);
	packet_register_callback(&packet_controller, PACKET_FUNC_BUS_SERVO, packet_serial_servo_handle);
}

