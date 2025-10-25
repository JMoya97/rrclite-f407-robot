#include "button.h"
#include "global_conf.h"
#include "packet_reports.h"
#include "cmsis_os2.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "lwmem_porting.h"
#include "rrclite_packets.h"

ButtonObjectTypeDef* buttons[2];
static uint32_t button_read_pin(ButtonObjectTypeDef *self); /* Read button GPIO */

static volatile uint8_t buttons_stream_enabled;
static uint16_t buttons_stream_period_ms;
static uint16_t buttons_stream_elapsed_ms;
static uint16_t buttons_seq;

static uint8_t buttons_current_mask(void)
{
    uint8_t mask = 0U;

    for (uint8_t i = 0; i < 2; ++i) {
        if (buttons[i] != NULL) {
            if (button_read_pin(buttons[i])) {
                mask |= (uint8_t)(1U << i);
            }
        }
    }

    return mask;
}

uint16_t buttons_set_stream(uint8_t enable, uint16_t period_ms)
{
    if (enable) {
        if (period_ms < BUTTON_TASK_PERIOD) {
            period_ms = BUTTON_TASK_PERIOD;
        }

        buttons_stream_enabled = 1U;
        buttons_stream_period_ms = period_ms;
        buttons_stream_elapsed_ms = 0U;
        buttons_seq = 0U;

        return period_ms;
    }

    buttons_stream_enabled = 0U;
    buttons_stream_period_ms = 0U;
    buttons_stream_elapsed_ms = 0U;
    buttons_seq = 0U;

    return 0U;
}

uint8_t buttons_read_mask(void)
{
    return buttons_current_mask();
}

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

        if (buttons_stream_enabled) {
            uint16_t elapsed = buttons_stream_elapsed_ms + BUTTON_TASK_PERIOD;
            if (elapsed >= buttons_stream_period_ms) {
                elapsed = 0U;

                const rrc_button_stream_frame_t frame = {
                    .seq = buttons_seq++,
                    .mask = buttons_current_mask(),
                };

                (void)rrc_transport_send(RRC_FUNC_IO,
                                         RRC_IO_BUTTON_STREAM_CTRL,
                                         &frame, sizeof(frame));
            }

            buttons_stream_elapsed_ms = elapsed;
        }
}
