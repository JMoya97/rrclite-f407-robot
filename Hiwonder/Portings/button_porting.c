#include "button.h"
#include "global_conf.h"
#include "packet_reports.h"
#include "cmsis_os2.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "lwmem_porting.h"

ButtonObjectTypeDef* buttons[2];
static uint32_t button_read_pin(ButtonObjectTypeDef *self); /* 读取按键IO状态 */

void buttons_init(void)
{
	for(int i = 0; i < 2; ++i) {
		buttons[i] = LWMEM_CCM_MALLOC(sizeof(ButtonObjectTypeDef));
		button_object_init(buttons[i]);
		buttons[i]->id = i + 1;
	    buttons[i]->read_pin = button_read_pin;
		buttons[i]->combin_th = 300; 
		buttons[i]->lp_th = 1500;  
		buttons[i]->repeat_th = 400;
	}
}

static uint32_t button_read_pin(ButtonObjectTypeDef *self)
{
    switch(self->id) {
        case 1:
            return ((uint32_t)HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)) ^ 1;
        case 2:
            return ((uint32_t)HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)) ^ 1;
        default:
            return 0;
    }
}

void button_timer_callback(void *argument)
{
	button_task_handler(buttons[0], BUTTON_TASK_PERIOD);
	button_task_handler(buttons[1], BUTTON_TASK_PERIOD);
}
