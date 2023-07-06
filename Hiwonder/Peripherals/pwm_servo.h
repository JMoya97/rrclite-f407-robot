/**
 * @file pwm_servo.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief st7735显示屏驱动
 * @version 0.1
 * @date 2023-05-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __PWM_SERVO_H_
#define __PWM_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct PWMServoObject  PWMServoObjectTypeDef;
struct PWMServoObject {
    int offset; // 舵机偏差
    int target_duty;  // 目标脉宽
    int current_duty; // 当前脉宽
    int duty_raw;     // 机器脉宽，最终写入定时器的脉宽，包含了offset， == current_duty + offset

    /* 速度控制需要的变量 */
    uint32_t duration;    // 舵机从当前角度运动到指定角度的时间，也就是控制速度 单位:ms
    float duty_inc; // 每次位置更新的脉宽增量
    int  inc_times; // 需要递增的次数
    bool is_running;  // 舵机是否在转动过程中
    bool duty_changed; // 脉宽是否被改变

    PWMServoObjectTypeDef* next; // 形成链表方便遍历

    /* 内部接口 */
    void (*refresh)(PWMServoObjectTypeDef *self); /* 每50ms调用1次以刷新舵机的位置使舵机转动 */
    void (*set_position)(PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration); //设置舵机位置
    void (*set_offset)(PWMServoObjectTypeDef *self, int offset); // 设置舵机偏差

    /* 外部提供，硬件抽象接口 */
    void (*write_pin)(uint32_t new_state); /* IO口电平设置 */
};


void pwm_servo_object_init(PWMServoObjectTypeDef *object);
void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration);
void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset);

#endif

