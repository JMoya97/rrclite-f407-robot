#include "button.h"

void button_task_handler(ButtonObjectTypeDef *self, uint32_t period)
{
    self->ticks_count += period;

    uint32_t pin = self->read_pin(self);
    if(pin != self->last_pin_raw)  { /* Two consecutive readings differ: debounce not yet stable */
        self->last_pin_raw = pin;
        return;
    }
	
	/* No state change: state machine remains in place */
    if(self->last_pin_filtered == self->last_pin_raw && self->stage != BUTTON_STAGE_PRESS && self->stage != BUTTON_STAGE_LONGPRESS) { 
        return;
    }

    self->last_pin_filtered = self->last_pin_raw; /* Store the debounced state */
    switch(self->stage) {
        case BUTTON_STAGE_NORMAL: {
            if(self->last_pin_filtered) {
                self->event_callback(self, BUTTON_EVENT_PRESSED); /* Fire press event */
                if(self->ticks_count < self->combin_th && self->combin_counter > 0) { /* Multi-click only counts if already tracking */
                    self->combin_counter += 1;
                    if(self->combin_counter == 2) {  /* Double-click callback */
                        self->event_callback(self, BUTTON_EVENT_DOUBLE_CLICK);
                    }
                    if(self->combin_counter == 3) {  /* Triple-click callback */
                        self->event_callback(self, BUTTON_EVENT_TRIPLE_CLICK);
                    }
                }
                self->ticks_count = 0;
                self->stage = BUTTON_STAGE_PRESS;
            } else {
                if(self->ticks_count > self->combin_th && self->combin_counter != 0) {
                    self->combin_counter = 0;
                    self->ticks_count = 0;
                }
            }
            break;
		}
        case BUTTON_STAGE_PRESS: {
            if(self->last_pin_filtered) {
                if(self->ticks_count > self->lp_th) { /* Long-press threshold reached */
                    self->event_callback(self, BUTTON_EVENT_LONGPRESS); /* Trigger long-press event */
                    self->ticks_count = 0;
                    self->stage = BUTTON_STAGE_LONGPRESS; /* Enter long-press state */
                }
            } else { /* Button released */
                self->event_callback(self, BUTTON_EVENT_RELEASE_FROM_SP); /* Trigger short-press release */
                self->event_callback(self, BUTTON_EVENT_CLICK);  /* Trigger click event */
                self->combin_counter = self->combin_counter == 0 ? 1 : self->combin_counter; /* Multi-click only counts if already tracking */
                self->stage = BUTTON_STAGE_NORMAL;
            }
            break;
		}
        case BUTTON_STAGE_LONGPRESS: {
            if(self->last_pin_filtered) {
                if(self->ticks_count > self->repeat_th)  {
                    self->event_callback(self, BUTTON_EVENT_LONGPRESS_REPEAT); /* Trigger long-press repeat */
                    self->ticks_count = 0; /* Restart repeat timer */
                }
            } else { /* Button released */
                self->event_callback(self, BUTTON_EVENT_RELEASE_FROM_LP);  /* Trigger long-press release */
                self->combin_counter = 0;                /* Long-press cancels multi-click tracking */
                self->ticks_count = self->combin_th + 1; /* Force multi-click timing to expire */
                self->stage = BUTTON_STAGE_NORMAL;
            }
            break;
		}
    }
}

void button_register_callback(ButtonObjectTypeDef *self, ButtonEventCallbackFuncTypeDef callback)
{
    if(NULL == callback) {
        return;
    }
    self->event_callback = callback;
}

void button_default_event_callback(ButtonObjectTypeDef *self,  ButtonEventIDEnum event)
{
}

void button_object_init(ButtonObjectTypeDef *self)
{
    self->stage = BUTTON_STAGE_NORMAL;
    self->last_pin_raw = 0;
	self->last_pin_filtered = 0;
    self->combin_counter = 0;
    self->ticks_count = 0;

    /* config */
    self->combin_th = 400; 
    self->lp_th = 2000;  
    self->repeat_th = 500;
    self->event_callback = button_default_event_callback;
}

