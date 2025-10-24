#ifndef __LED_H_
#define __LED_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


#define LED_NUM  3

typedef enum {
    LED_STAGE_START_NEW_CYCLE,
    LED_STAGE_WATTING_OFF,
    LED_STAGE_WATTING_PERIOD_END,
    LED_STAGE_IDLE,
} LEDStageEnum;

typedef struct {
    uint32_t ticks_on;  /**< @brief LED on-time per cycle (ms) */
    uint32_t ticks_off; /**< @brief LED off-time per cycle (ms) */
    uint16_t repeat;    /**< @brief Number of blink repetitions */
} LEDCtrlTypeDef;

typedef struct LEDObjectObject LEDObjectTypeDef;
struct LEDObjectObject {
	uint32_t id;
	
    LEDStageEnum stage;   /**< @brief Current state of the LED control state machine */
    uint32_t ticks_count;    /**< @brief Millisecond accumulator for state transitions */
    LEDCtrlTypeDef ctrl_structure; /**< @brief LED control parameters */

    int (*get_ctrl_block)(LEDObjectTypeDef *self, LEDCtrlTypeDef *p);

    int (*put_ctrl_block)(LEDObjectTypeDef *self, LEDCtrlTypeDef *p);

    void (*set_pin)(LEDObjectTypeDef *self, uint32_t new_state);
};

void led_object_init(LEDObjectTypeDef *self);

void led_task_handler(LEDObjectTypeDef *self, uint32_t period);

int led_on(LEDObjectTypeDef *self);

int led_off(LEDObjectTypeDef *self);

int led_flash(LEDObjectTypeDef *self, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat);

#endif

