#ifndef __BUZZER_H_
#define __BUZZER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BUZZER_STAGE_START_NEW_CYCLE,
    BUZZER_STAGE_WATTING_OFF,
    BUZZER_STAGE_WATTING_PERIOD_END,
    BUZZER_STAGE_IDLE,
} BuzzerStageEnum;

typedef struct {
    uint16_t freq;      /**< @brief Buzzer frequency */
    uint32_t ticks_on;  /**< @brief Duration the buzzer is active within one period (ms) */
    uint32_t ticks_off; /**< @brief Duration the buzzer is silent within one period (ms) */
    uint16_t repeat;    /**< @brief Number of times the buzzer repeats the beep */
} BuzzerCtrlTypeDef;

typedef struct BuzzerObject BuzzerObjectTypeDef;
struct BuzzerObject {
	uint32_t id;

    BuzzerStageEnum stage;   /**< @brief Current state of the buzzer control state machine */
    uint32_t ticks_count;    /**< @brief Millisecond counter used to advance the state machine */
    BuzzerCtrlTypeDef ctrl_structure; /**< @brief Buzzer control parameters */

    int (*get_ctrl_block)(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);

    int (*put_ctrl_block)(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);

    void (*set_pwm)(BuzzerObjectTypeDef *self, uint32_t freq);
};

void buzzer_object_init(BuzzerObjectTypeDef *self);

void buzzer_task_handler(BuzzerObjectTypeDef *self, uint32_t period);

int buzzer_on(BuzzerObjectTypeDef *self, uint32_t freq);

int buzzer_off(BuzzerObjectTypeDef *self);

int buzzer_didi(BuzzerObjectTypeDef *self, uint32_t freq, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat);

#endif
