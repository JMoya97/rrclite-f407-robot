#ifndef __PWM_SERVO_H_
#define __PWM_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct PWMServoObject  PWMServoObjectTypeDef;

struct PWMServoObject {
    int id;     /**< @brief 舵机ID */
    int offset; /**< @brief 舵机偏差 */
    int target_duty;  /**< @brief 目标脉宽 */
    int current_duty; /**< @brief 当前脉宽 */
    int duty_raw;     /**< @brief 机器脉宽，最终写入定时器的脉宽，包含了offset， == current_duty + offset */

    /* 速度控制需要的变量 */
    uint32_t duration;    /**< @brief 舵机从当前角度运动到指定角度的时间，也就是控制速度 单位:ms */
    float duty_inc; /**< @brief 每次位置更新的脉宽增量 */
    int  inc_times; /**< @brief 需要递增的次数 */
    bool is_running;  /**< @brief 舵机是否在转动过程中 */
    bool duty_changed; /**< @brief 脉宽是否被改变 */

    /* 外部提供，硬件抽象接口 */
    void (*write_pin)(uint32_t new_state); /* IO口电平设置 */
};

void pwm_servo_object_init(PWMServoObjectTypeDef *object);

void pwm_servo_duty_compare(PWMServoObjectTypeDef *self);

void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration);

void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset);

#endif

