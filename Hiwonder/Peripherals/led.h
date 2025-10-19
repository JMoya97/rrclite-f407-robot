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
    uint32_t ticks_on;  /**< @brief 周期内LED亮起时长，毫秒 */
    uint32_t ticks_off; /**< @brief 周期内LED熄灭时长，毫秒 */
    uint16_t repeat;    /**< @brief LED闪烁重复次数 */
} LEDCtrlTypeDef;

typedef struct LEDObjectObject LEDObjectTypeDef;
struct LEDObjectObject {
	uint32_t id;
	
    LEDStageEnum stage;   /**< @brief LED控制状态机的当前状态 */
    uint32_t ticks_count;    /**< @brief 毫秒计数，用于累计毫秒数切换状态机状态 */
    LEDCtrlTypeDef ctrl_structure; /**< @brief LED控制参数 */

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

