#ifndef __BUTTON_H_
#define __BUTTON_H_

#include <stdio.h>
#include <stdint.h>

typedef enum {
    BUTTON_STAGE_NORMAL,
    BUTTON_STAGE_PRESS,
    BUTTON_STAGE_LONGPRESS,
} ButtonStageEnum;

typedef enum {
    BUTTON_EVENT_PRESSED = 0x01,           /**< @brief Button pressed */
    BUTTON_EVENT_LONGPRESS = 0x02,         /**< @brief Button long-press */
    BUTTON_EVENT_LONGPRESS_REPEAT = 0x04,  /**< @brief Button long-press retrigger */
    BUTTON_EVENT_RELEASE_FROM_LP = 0x08,   /**< @brief Release after a long-press */
    BUTTON_EVENT_RELEASE_FROM_SP = 0x10,   /**< @brief Release after a short-press */
    BUTTON_EVENT_CLICK = 0x20,             /**< @brief Button click */
    BUTTON_EVENT_DOUBLE_CLICK = 0x40,      /**< @brief Button double-click */
    BUTTON_EVENT_TRIPLE_CLICK = 0x80,      /**< @brief Button triple-click */
} ButtonEventIDEnum;

typedef struct ButtonObject ButtonObjectTypeDef;

typedef struct {
    ButtonObjectTypeDef *button; /**< @brief Pointer to the button that triggered the event */
    ButtonEventIDEnum event; /**< @brief Button event identifier */
} ButtonEventObjectTypeDef;

typedef void (*ButtonEventCallbackFuncTypeDef)(ButtonObjectTypeDef *self,  ButtonEventIDEnum event);

struct ButtonObject {
	uint32_t id;

    ButtonStageEnum stage;  /**< @brief Current state of the button scan state machine */
    uint32_t last_pin_raw; /**< @brief Last raw GPIO state */
        uint32_t last_pin_filtered; /**< @brief Last debounced GPIO state */
    uint32_t combin_counter; /**< @brief Multi-click counter */
        uint32_t ticks_count; /**< @brief Millisecond timer */

    /* config */
    uint32_t combin_th; /**< @brief Maximum interval between clicks (ms) */
    uint32_t lp_th;     /**< @brief Long-press threshold (ms) */
    uint32_t repeat_th; /**< @brief Long-press retrigger interval (ms) */
    ButtonEventCallbackFuncTypeDef event_callback; /**< @brief Event callback */

    uint32_t (*read_pin)(ButtonObjectTypeDef *self);

};

void button_object_init(ButtonObjectTypeDef *self);

void button_task_handler(ButtonObjectTypeDef *self, uint32_t period);

void button_register_callback(ButtonObjectTypeDef *self, ButtonEventCallbackFuncTypeDef callback);

void button_default_event_callback(ButtonObjectTypeDef *self,  ButtonEventIDEnum event);

#endif

