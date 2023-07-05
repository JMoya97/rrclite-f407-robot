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
void chassis_init(void);


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

void jetauto_control(char msg);
void jettank_control(char msg);
void ti4wd_control(char msg);
void tankblack_control(char msg);

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
    osTimerStart(battery_check_timerHandle, BATTERY_TASK_PERIOD);
    HAL_ADC_Start(&hadc1);

    char msg = '\0';
    uint8_t msg_prio;
    osMessageQueueReset(moving_ctrl_queueHandle);

    chassis_init();
    set_chassis_type(CHASSIS_TYPE_TI4WD);

    for(;;) {

        if(osMessageQueueGet(moving_ctrl_queueHandle, &msg, &msg_prio, 100) != osOK) {
            chassis->stop(chassis);
            continue;
        }
        printf("msg: %c\r\n", msg);
        switch(chassis->chassis_type) {
            case CHASSIS_TYPE_TI4WD:
                ti4wd_control(msg);
                break;
            case CHASSIS_TYPE_JETAUTO:
                jetauto_control(msg);
                break;
            case CHASSIS_TYPE_JETTANK:
                jettank_control(msg);
				break;
			case CHASSIS_TYPE_TANKBLACK:
				tankblack_control(msg);
                break;
            default:
                break;
        }

    }
}


void tankblack_control(char msg)
{
    static float speed = 300.0f;
    switch(msg) {
        case 'J':
            speed += 50;
            speed = speed > 460 ? 460 : speed;
            break;
        case 'N':
            speed -= 50;
            speed = speed < 50 ? 50 : speed;
            break;

        case 'I': {
            chassis->stop(chassis);
            break;
        }
        case 'A': {
            chassis->set_velocity(chassis, speed, 0, 0);
            break;
        }
        case 'B': {
            chassis->set_velocity_radius(chassis, speed, 300, false);
            break;
        }
        case 'C': {
            chassis->set_velocity_radius(chassis, speed, 150, true);
            break;
        }
        case 'D': {
            chassis->set_velocity_radius(chassis, -speed, 300, false);
            break;
        }
        case 'E': {
            chassis->set_velocity(chassis, -speed, 0, 0);
            break;
        }
        case 'F': {
            chassis->set_velocity_radius(chassis, -speed, -300, false);
            break;
        }
        case 'G': {
            chassis->set_velocity_radius(chassis, speed, -150, true);
            break;
        }
        case 'H': {
            chassis->set_velocity_radius(chassis, speed, -300, false);
            break;
        }
        default:
            break;
    }
}

void ti4wd_control(char msg)
{
    static float speed = 500.0f;
    switch(msg) {
        case 'J':
            speed += 100;
            speed = speed > 800 ? 800 : speed;
            break;
        case 'N':
            speed -= 100;
            speed = speed < 100 ? 100 : speed;
            break;

        case 'I': {
            chassis->stop(chassis);
            break;
        }
        case 'A': {
            chassis->set_velocity(chassis, speed, 0, 0);
            break;
        }
        case 'B': {
            chassis->set_velocity_radius(chassis, speed, 250, false);
            break;
        }
        case 'C': {
            chassis->set_velocity_radius(chassis, speed, 100, false);
            break;
        }
        case 'D': {
            chassis->set_velocity_radius(chassis, -speed, 250, false);
            break;
        }
        case 'E': {
            chassis->set_velocity(chassis, -speed, 0, 0);
            break;
        }
        case 'F': {
            chassis->set_velocity_radius(chassis, -speed, -250, false);
            break;
        }
        case 'G': {
            chassis->set_velocity_radius(chassis, speed, -100, false);
            break;
        }
        case 'H': {
            chassis->set_velocity_radius(chassis, speed, -250, false);
            break;
        }
        default:
            break;
    }
}

void jettank_control(char msg)
{
    static float speed = 140.0f;
    switch(msg) {
        case 'J':
            speed += 20;
            speed = speed > 180 ? 180 : speed;
            break;
        case 'N':
            speed -= 20;
            speed = speed < 50 ? 50 : speed;
            break;

        case 'I': {
            chassis->stop(chassis);
            break;
        }
        case 'A': {
            chassis->set_velocity(chassis, speed, 0, 0);
            break;
        }
        case 'B': {
            chassis->set_velocity_radius(chassis, speed, 200, false);
            break;
        }
        case 'C': {
            chassis->set_velocity_radius(chassis, speed, 200, true);
            break;
        }
        case 'D': {
            chassis->set_velocity_radius(chassis, -speed, 200, false);
            break;
        }
        case 'E': {
            chassis->set_velocity(chassis, -speed, 0, 0);
            break;
        }
        case 'F': {
            chassis->set_velocity_radius(chassis, -speed, -200, false);
            break;
        }
        case 'G': {
            chassis->set_velocity_radius(chassis, speed, -200, true);
            break;
        }
        case 'H': {
            chassis->set_velocity_radius(chassis, speed, -200, false);
            break;
        }
        default:
            break;
    }
}

void jetauto_control(char msg)
{
    static float speed = 300.0f;
    switch(msg) {
        case 'J':
            speed += 50;
            speed = speed > 450 ? 450 : speed;
            break;
        case 'N':
            speed -= 50;
            speed = speed < 50 ? 50 : speed;
            break;
        case 'L':
            chassis->set_velocity(chassis, 0, speed, 0);
            break;
        case 'P':
            chassis->set_velocity(chassis, 0, -speed, 0);
            break;
        case 'I': {
            chassis->stop(chassis);
            break;
        }
        case 'A': {
            chassis->set_velocity(chassis, speed, 0, 0);
            break;
        }
        case 'B': {
            chassis->set_velocity_radius(chassis, speed, 500, false);
            break;
        }
        case 'C': {
            chassis->set_velocity_radius(chassis, speed, 400, true);
            break;
        }
        case 'D': {
            chassis->set_velocity_radius(chassis, -speed, 500, false);
            break;
        }
        case 'E': {
            chassis->set_velocity(chassis, -speed, 0, 0);
            break;
        }
        case 'F': {
            chassis->set_velocity_radius(chassis, -speed, -500, false);
            break;
        }
        case 'G': {
            chassis->set_velocity_radius(chassis, speed, -400, true);
            break;
        }
        case 'H': {
            chassis->set_velocity_radius(chassis, speed, -500, false);
            break;
        }
        default:
            break;
    }
}
	