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
    uint16_t freq;      /**< @brief 蜂鸣器频率 */
    uint32_t ticks_on;  /**< @brief 周期内蜂鸣器响起时长，毫秒 */
    uint32_t ticks_off; /**< @brief 周期内蜂鸣器静音时长，毫秒 */
    uint16_t repeat;    /**< @brief 蜂鸣器"嘀"响重复次数 */
} BuzzerCtrlTypeDef;

typedef struct BuzzerObject BuzzerObjectTypeDef;
struct BuzzerObject {
	uint32_t id;

    BuzzerStageEnum stage;   /**< @brief 蜂鸣器控制状态机的当前状态 */
    uint32_t ticks_count;    /**< @brief 毫秒计数，用于累计毫秒数切换状态机状态 */
    BuzzerCtrlTypeDef ctrl_structure; /**< @brief 蜂鸣器控制参数 */

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
