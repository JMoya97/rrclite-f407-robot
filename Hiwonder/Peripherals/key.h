/// Hardware-agnostic button detection implementation


#ifndef __KEY_H_
#define __KEY_H_

#include <stdio.h>
#include <stdint.h>
#include <object.h>

typedef enum {
	KEY_STATE_NORMAL,
	KEY_STATE_PRESS,
	KEY_STATE_LONGPRESS,
	KEY_STATE_REPEAT,
} KeyStateEnum;

typedef enum {
	KEY_EVENT_PRESSED = 0x01,
	KEY_EVENT_LONGPRESS = 0x02,
	KEY_EVENT_LONGPRESS_REPEAT = 0x04,
	KEY_EVENT_RELEASE_FROM_LP = 0x08,
	KEY_EVENT_RELEASE_FROM_SP = 0x10,
	KEY_EVENT_CLICK = 0x20,
	KEY_EVENT_DOUBLE_CLICK = 0x40,
	KEY_EVENT_TRIPLE_CLICK = 0x80,
}KeyEventEnum;

typedef struct{
	ObjectTypeDef base;
	int key_id;
	KeyEventEnum event;
}KeyEventObjectTypeDef;

typedef struct _KeyObjectTypeDef KeyObjectTypeDef;
struct _KeyObjectTypeDef {
        KeyStateEnum state;  // Current button state machine status

        uint8_t last_level; // Last raw GPIO level read
        uint32_t stamp; //
        int combin_counter;

        /* config */
        int combin_th; /**< Maximum interval between combined presses (ms) */
        int lp_th;     /**< Long-press threshold (ms) */
        int repeat_th; /**< Long-press retrigger interval (ms) */

        // Internally exposed interface
        void (*refresh)(KeyObjectTypeDef *self); // Refresh button state
        uint32_t (*get_ticks)(void); // Get current millisecond timestamp

        // Porting interface
        char *name;  // Button name
        uint8_t level_press; // GPIO level when the button is pressed
        uint8_t (*read_pin)(void); // Read button level
        void (*event_callback)(KeyObjectTypeDef *key, KeyEventEnum event);
};


void key_obj_init(KeyObjectTypeDef *self);
#endif

