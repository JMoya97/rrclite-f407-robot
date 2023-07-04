/**
 * @file app.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 主应用逻辑
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "cmsis_os2.h"
#include "led.h"
#include "lwmem_porting.h"
#include "global.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "global.h"
#include "adc.h"

void buzzers_init(void);
void buttons_init(void);
void leds_init(void);


void motors_init(void);
void pwm_servos_init(void);
void sbus_init(void);

void global_init(void)
{
    lwmem_assignmem(lwmem_regions); /* 动态内存初始化 */
}

void button_event_callback(ButtonObjectTypeDef *button,  ButtonEventIDEnum event)
{
    if(button == buttons[0]) { /* 按键1的事件  */
        if(event == BUTTON_EVENT_CLICK) {
            buzzer_didi(buzzers[0], 2000, 50, 100, 1);
        }
        if(event == BUTTON_EVENT_DOUBLE_CLICK) {
            buzzer_didi(buzzers[0], 2000, 50, 50, 2);
        }
    }
}

void battery_check_timer_callback(void *argument) {
	HAL_ADC_Start(&hadc1);
}


void app_task_entry(void *argument)
{
    extern osTimerId_t led_timerHandle;
    extern osTimerId_t buzzer_timerHandle;
    extern osTimerId_t button_timerHandle;
	extern osTimerId_t battery_check_timerHandle;
    extern osMessageQueueId_t moving_ctrl_queueHandle;
    extern osMessageQueueId_t bluetooth_tx_queueHandle;

    motors_init();
    leds_init();
    buzzers_init();
    buttons_init();
    button_register_callback(buttons[0], button_event_callback);
    button_register_callback(buttons[1], button_event_callback);

    osTimerStart(led_timerHandle, LED_TASK_PERIOD);
    osTimerStart(buzzer_timerHandle, BUZZER_TASK_PERIOD);
    osTimerStart(button_timerHandle, BUTTON_TASK_PERIOD);
	//osTimerStart(battery_check_timerHandle, 100);
    char msg = '\0';
    uint8_t msg_prio;
    osMessageQueueReset(moving_ctrl_queueHandle);
    for(;;) {
        
        //printf("msg: %c\r\n", msg);
		if(osMessageQueueGet(moving_ctrl_queueHandle, &msg, &msg_prio, osWaitForever) != osOK) {
			continue;
		}
        switch(msg) {
			case 'M': {
				for(int i = 0; i < 4; ++i) {
					motors[i]->pid_controller.kp -= 1;
				}
				//printf("s:%f, e:%f\r\n", motors[1]->pid_controller.set_point, motors[1]->pid_controller.err);
				break;
			}
			case 'J': {
				for(int i = 0; i < 4; ++i) {
					motors[i]->pid_controller.kp += 1;
				}
				//printf("s:%f, e:%f\r\n", motors[1]->pid_controller.set_point, motors[1]->pid_controller.err);
				break;
			}
            case 'O': {
                encoder_set_speed(motors[0], 0);
                encoder_set_speed(motors[1], 0);
			    encoder_set_speed(motors[2], 0);
                encoder_set_speed(motors[3], 0);
                break;
            }
            case 'A': {
                encoder_set_speed(motors[0], 0.5);
                encoder_set_speed(motors[1], 0.5);
                encoder_set_speed(motors[2], 0.5);
                encoder_set_speed(motors[3], 0.5);
                break;
            }
            case 'E': {
                encoder_set_speed(motors[0], -1);
                encoder_set_speed(motors[1], -1);
                encoder_set_speed(motors[2], -1);
                encoder_set_speed(motors[3], -1);
                break;
            }
            default:
                break;
        }
    }
}
