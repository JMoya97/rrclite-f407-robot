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
    BUTTON_EVENT_PRESSED = 0x01,           /**< @brief 按钮被按下 */
    BUTTON_EVENT_LONGPRESS = 0x02,         /**< @brief 按钮被长按 */
    BUTTON_EVENT_LONGPRESS_REPEAT = 0x04,  /**< @brief 按钮长按重触发 */
    BUTTON_EVENT_RELEASE_FROM_LP = 0x08,   /**< @brief 按钮从长按中松开 */
    BUTTON_EVENT_RELEASE_FROM_SP = 0x10,   /**< @brief 按钮从短按中松开 */
    BUTTON_EVENT_CLICK = 0x20,             /**< @brief 按钮被点击 */
    BUTTON_EVENT_DOUBLE_CLICK = 0x40,      /**< @brief 按钮被双击 */
    BUTTON_EVENT_TRIPLE_CLICK = 0x80,      /**< @brief 按钮被三连击 */
} ButtonEventIDEnum;

typedef struct ButtonObject ButtonObjectTypeDef;

typedef struct {
    ButtonObjectTypeDef *button; /**< @brief 触发事件的按钮对象指针 */
    ButtonEventIDEnum event; /**< @brief 按钮事件ID */
} ButtonEventObjectTypeDef;

typedef void (*ButtonEventCallbackFuncTypeDef)(ButtonObjectTypeDef *self,  ButtonEventIDEnum event);

struct ButtonObject {
	uint32_t id;

    ButtonStageEnum stage;  /**< @brief 按键扫描状态机当前状态 */
    uint32_t last_pin_raw; /**< @brief 上次读到的原始IO口状态 */
	uint32_t last_pin_filtered; /**< @brief 上次经过消抖的IO口状态 */
    uint32_t combin_counter; /**< @brief 连击计数 */
	uint32_t ticks_count; /**< @brief 毫秒计时变量 */

    /* config */
    uint32_t combin_th; /**< @brief 连按最大间隔 毫秒*/
    uint32_t lp_th;     /**< @brief 长按阈值 毫秒 */
    uint32_t repeat_th; /**< @brief 长按重触发间隔 毫秒 */
    ButtonEventCallbackFuncTypeDef event_callback; /**< @brief 事件回调函数指针 */

    uint32_t (*read_pin)(ButtonObjectTypeDef *self);

};

void button_object_init(ButtonObjectTypeDef *self);

void button_task_handler(ButtonObjectTypeDef *self, uint32_t period);

void button_register_callback(ButtonObjectTypeDef *self, ButtonEventCallbackFuncTypeDef callback);

void button_default_event_callback(ButtonObjectTypeDef *self,  ButtonEventIDEnum event);

#endif

