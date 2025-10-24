/// Hardware-agnostic button detection implementation
#include "global.h"
#include "key.h"

static void default_callback(KeyObjectTypeDef *key, KeyEventEnum event)
{
//    const char events[8][64] = {
//        "KEY_EVENT_PRESSED",
//        "KEY_EVENT_LONGPRESS",
//        "KEY_EVENT_LONGPRESS_REPEAT",
//        "KEY_EVENT_RELEASE_FROM_LP",
//        "KEY_EVENT_RELEASE_FROM_SP",
//        "KEY_EVENT_CLICK",
//        "KEY_EVENT_DOUBLE_CLICK",
//        "KEY_EVENT_TRIPLE_CLICK",
//    };
//    printf("event: %s\r\n", (char*)&events[event]);
}

void key_refresh(KeyObjectTypeDef *self)
{
    uint8_t level = self->read_pin();
    uint32_t current_tick = self->get_ticks();
    if(level == self->last_level) { // Two identical raw readings imply a stable button state
        switch(self->state) {
        case KEY_STATE_NORMAL:
            if(level == self->level_press) {
                /* Trigger button press event */
                if(NULL != self->event_callback) {
                    self->event_callback(self, KEY_EVENT_PRESSED);
                }
                if((current_tick - self->stamp) < self->combin_th) {
                    self->combin_counter += 1;
                    if(NULL != self->event_callback) {
                        if(self->combin_counter == 1) {
                            self->event_callback(self, KEY_EVENT_DOUBLE_CLICK);
                        }
                        if(self->combin_counter == 2) {
                            self->event_callback(self, KEY_EVENT_TRIPLE_CLICK);
                        }
                    }
                }
                self->stamp = current_tick; /* Record the time of this press */
                self->state = KEY_STATE_PRESS;
            } else {
                if((current_tick - self->stamp) > self->combin_th && self->combin_counter != 0) {
                    self->combin_counter = 0;
                }
            }
            break;
        case KEY_STATE_PRESS:
            if(level != self->level_press) { // Button released
                /* Trigger short-press release event */
                if(NULL != self->event_callback) {
                    self->event_callback(self, KEY_EVENT_RELEASE_FROM_SP);
                    self->event_callback(self, KEY_EVENT_CLICK);
                }
                self->state = KEY_STATE_NORMAL;
            } else {
                /* Long-press threshold reached: emit event and enter long-press state */
                if((current_tick - self->stamp) > self->lp_th) {
                    self->event_callback(self, KEY_EVENT_LONGPRESS);
                    self->state = KEY_STATE_LONGPRESS;
                    self->combin_counter = 0;
                    self->stamp = current_tick; /* Record the first long-press trigger time */
                }
            }
            break;
        case KEY_STATE_LONGPRESS:
            if(level != self->level_press) { // Button released
                /* Trigger long-press release event */
                if(NULL != self->event_callback) {
                    self->event_callback(self, KEY_EVENT_RELEASE_FROM_LP);
                    self->stamp = 0;
                }
                self->state = KEY_STATE_NORMAL;
            } else {
                if((current_tick - self->stamp) > self->repeat_th)  {
                    self->event_callback(self, KEY_EVENT_LONGPRESS_REPEAT);
                    self->stamp = current_tick; /* Record the retrigger time */
                }
            }
            break;
        default:
            while(1) {
                printf("KEY REFRESH ERROR!!!\r\n");
            }
        }
    }
    self->last_level = level;
}


void key_obj_init(KeyObjectTypeDef *self)
{
    self->state = KEY_STATE_NORMAL;
    self->last_level = !self->level_press;
    self->refresh = key_refresh;
    self->event_callback = default_callback;
}

