#include "buzzer.h"

void buzzer_task_handler(BuzzerObjectTypeDef *self, uint32_t period)
{
    /* Try to fetch a new control block from the queue; reset the state machine if one is available */
    if(self->get_ctrl_block(self, &self->ctrl_structure) == 0) {
        self->stage = BUZZER_STAGE_START_NEW_CYCLE;
    }
    /* State machine processing */
    switch(self->stage) {
        case BUZZER_STAGE_START_NEW_CYCLE: {
            if(self->ctrl_structure.ticks_on > 0 && self->ctrl_structure.freq > 0) {
                self->set_pwm(self, self->ctrl_structure.freq); /* Drive the buzzer */
                if(self->ctrl_structure.ticks_off > 0) {/* Non-zero quiet time means intermittent beeps; otherwise hold the tone */
                    self->ticks_count = 0;
                    self->stage = BUZZER_STAGE_WATTING_OFF; /* Wait for the audible interval to finish */
                }else{
                    self->stage = BUZZER_STAGE_IDLE; /* Steady tone, return to idle */
                }
            } else { /* Zero duration means silence */
                self->set_pwm(self, 0);
                self->stage = BUZZER_STAGE_IDLE;  /* Steady silence, return to idle */
            }
            break;
        }
        case BUZZER_STAGE_WATTING_OFF: {
            self->ticks_count += period;
            if(self->ticks_count >= self->ctrl_structure.ticks_on) { /* Audible interval finished */
                self->set_pwm(self, 0);
                self->stage = BUZZER_STAGE_WATTING_PERIOD_END;
            }
            break;
        }
        case BUZZER_STAGE_WATTING_PERIOD_END: { /* Wait for the full period to end */
            self->ticks_count += period;
            if(self->ticks_count >= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on)) {
                self->ticks_count -= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on);
                if(self->ctrl_structure.repeat == 1) { /* Final repetition completes the control task */
                    self->set_pwm(self, 0);
                    self->stage = BUZZER_STAGE_IDLE;
                } else {
                    self->set_pwm(self, self->ctrl_structure.freq);
                    self->ctrl_structure.repeat = self->ctrl_structure.repeat == 0 ? 0 : self->ctrl_structure.repeat - 1;
                    self->stage = BUZZER_STAGE_WATTING_OFF;
                }
            }
            break;
        }
        case BUZZER_STAGE_IDLE: {
            break;
        }
        default:
            break;
    }
}

int buzzer_on( BuzzerObjectTypeDef *self, uint32_t freq)
{
    BuzzerCtrlTypeDef ctrl_structure = {
        .freq = freq,
        .ticks_on = 1,  
        .ticks_off = 0, /* Zero quiet time creates a continuous tone */
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int buzzer_off(BuzzerObjectTypeDef *self)
{
    BuzzerCtrlTypeDef ctrl_structure = {
        .freq = 0,
        .ticks_on = 0, /* Zero audible time keeps the buzzer silent */
        .ticks_off = 0,
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int buzzer_didi(BuzzerObjectTypeDef *self, uint32_t freq, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat)
{
    BuzzerCtrlTypeDef ctrl_structure = {
        .freq = freq,
        .ticks_on = ticks_on,
        .ticks_off = ticks_off,
        .repeat = repeat,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

void buzzer_object_init(BuzzerObjectTypeDef *self)
{
    self->stage = BUZZER_STAGE_IDLE;
}

