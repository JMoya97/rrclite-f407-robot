#include "led.h"

void led_task_handler(LEDObjectTypeDef *self, uint32_t period)
{
    /* Try to fetch a new control block from the queue; reset the state machine if one is available */
    if(self->get_ctrl_block(self, &self->ctrl_structure) == 0) {
        self->stage = LED_STAGE_START_NEW_CYCLE;
    }
    /* State machine processing */
    switch(self->stage) {
        case LED_STAGE_START_NEW_CYCLE: {
            if(self->ctrl_structure.ticks_on > 0) {
                self->set_pin(self, 1);
                if(self->ctrl_structure.ticks_off > 0) { /* Non-zero off-time means blinking; otherwise keep it on */
                    self->ticks_count = 0;
                    self->stage = LED_STAGE_WATTING_OFF; /* Wait for the on-time to finish */
                }else{
                    self->stage = LED_STAGE_IDLE; /* Steady on, return to idle */
                }
            } else { /* Zero on-time means keep the LED off */
                self->set_pin(self, 0);
                self->stage = LED_STAGE_IDLE; /* Steady off, return to idle */
            }
            break;
        }
        case LED_STAGE_WATTING_OFF: {
            self->ticks_count += period;
            if(self->ticks_count >= self->ctrl_structure.ticks_on) { /* LED on-time finished */
                self->set_pin(self, 0);
                self->stage = LED_STAGE_WATTING_PERIOD_END;
            }
            break;
        }
        case LED_STAGE_WATTING_PERIOD_END: { /* Wait for the full period to end */
            self->ticks_count += period;
            if(self->ticks_count >= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on)) {
                self->ticks_count -= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on);
                if(self->ctrl_structure.repeat == 1) { /* Last repetition completes the control cycle */
                    self->set_pin(self, 0);
                    self->stage = LED_STAGE_IDLE;  /* All repetitions done, return to idle */
                } else {
                    self->set_pin(self, 1);
                    self->ctrl_structure.repeat = self->ctrl_structure.repeat == 0 ? 0 : self->ctrl_structure.repeat - 1;
                    self->stage = LED_STAGE_WATTING_OFF;
                }
            }
            break;
        }
        case LED_STAGE_IDLE: {
            break;
        }
        default:
            break;
    }
}

int led_on(LEDObjectTypeDef *self)
{
    LEDCtrlTypeDef ctrl_structure = {
        .ticks_on = 1,
        .ticks_off = 0,
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int led_off(LEDObjectTypeDef *self)
{
    LEDCtrlTypeDef ctrl_structure = {
        .ticks_on = 0,
        .ticks_off = 0,
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int led_flash(LEDObjectTypeDef *self, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat)
{
    LEDCtrlTypeDef ctrl_structure = {
        .ticks_on = ticks_on,
        .ticks_off = ticks_off,
        .repeat = repeat,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}


void led_object_init(LEDObjectTypeDef *self)
{
    self->stage = LED_STAGE_IDLE;
}


