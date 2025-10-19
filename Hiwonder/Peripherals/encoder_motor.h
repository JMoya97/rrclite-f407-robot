#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

struct EncoderMotorObject{
    int64_t counter;        /**< @brief 总计数值, 64bit 认为不会溢出 */
    int64_t overflow_num;   /**< @brief 溢出计数 */
    int32_t ticks_overflow; /**< @brief 计数溢出值 */
    float tps;              /**< @brief ticks per second 计数器频率 */
    float rps;              /**< @brief revolutions per second 输出轴转速 转每秒 */
    int current_pulse;      /**< @brief 当前输出的PWM值, 有符号对应正反转 */
    PID_ControllerTypeDef pid_controller; /**< @brief PID 控制器 */

    /** porting 可移植硬件接口 **/
    int32_t ticks_per_circle; /**< @brief 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
	float rps_limit;  /**< @brief 电机地转速极限，一般会取比100% PWM占空比时地转速稍小的值以确保PID控制器对速度的控制 */

    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);
};

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);

void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t new_counter);

void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);

void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif

