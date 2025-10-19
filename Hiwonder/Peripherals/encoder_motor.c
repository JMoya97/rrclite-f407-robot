#include "encoder_motor.h"
#include "gpio.h"
#include "global_conf.h"
#include "motors_param.h"

void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t counter)
{
    counter = counter + self->overflow_num * self->ticks_overflow; /* 总的计数值, 60000 根据实际设置的定时器溢出值 */
    int delta_count = counter - self->counter;
    self->counter = counter; /* 存储新的计数值 */
    self->tps = (float)delta_count / period * 0.9f + self->tps * 0.1f; /* 计算脉冲频率 */
    self->rps = self->tps / self->ticks_per_circle; /* 计算转速 单位rps, 转每秒 */
}

void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period)
{
#if ENABLE_MOTOR_PID_LOOP
    float pulse = 0;
    pid_controller_update(&self->pid_controller, self->rps, period);   /* 更新 PID控制器 */
        pulse = self->current_pulse + self->pid_controller.output; /* 计算新的 PWM 值 */

        /* 对输出的 PWM 值进行限幅, 限幅根据定时器的设置确定，本示例定时器设置的占空比 0-100 对应 0-1000 */
        pulse = pulse > 1000 ?  1000 : pulse;
        pulse = pulse < -1000 ? -1000 : pulse;

    self->set_pulse(self, pulse > -250 && pulse < 250 ? 0 : pulse); /* 设置新的PWM值且限制 PWM 的最小值, PWM过小电机只会发出嗡嗡声而不动 */
    self->current_pulse = pulse; /* 记录新的 PWM 值 */
#else
    (void)self; (void)period; /* raw-PWM mode: do nothing */
#endif
}

void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps)
{
    rps = rps > self->rps_limit ? self->rps_limit : (rps < -self->rps_limit ? -self->rps_limit : rps); /* 对速度进行限幅 */
    self->pid_controller.set_point = rps; /* 设置 PID 控制器目标 */
}

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self)
{
    self->counter = 0;
    self->overflow_num = 0;
    self->tps = 0;
    self->rps = 0;
    self->current_pulse = 0;
    self->ticks_overflow = 0;
    self->ticks_per_circle = MOTOR_JGA27_TICKS_PER_CIRCLE; /* 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
    pid_controller_init(&self->pid_controller, 0, 0, 0);
}

