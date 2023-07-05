#include "global.h"
#include "encoder_motor.h"
#include <string.h>

void encoder_set_speed(EncoderMotorObjectTypeDef *self, float rps)
{
	rps = rps > self->rps_limit ? self->rps_limit : (rps < -self->rps_limit ? -self->rps_limit : rps);
    self->pid_controller.set_point = rps;
}

void encoder_speed_control(EncoderMotorObjectTypeDef *self, float period)
{
    float pulse = 0;
    if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin) == RESET) {
        pid_update(&self->pid_controller, self->rps, period);
        pulse = self->current_pulse + self->pid_controller.output;
        pulse = pulse > 1000 ?  1000 : pulse;
        pulse = pulse < -1000 ? -1000 : pulse;
        // LOG_DEBUG("pid_output:%f\r\n", pulse);
    } else {
        pulse = 0;
    }
    self->set_pulse(self, pulse > -250 && pulse < 250 ? 0 : pulse);
    self->current_pulse = pulse;
}

// 定时更新速度计算
// period : 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t count)
{
    int32_t delta_count = count - self->last_count;
    self->tps = (float)delta_count / period;// * 0.9f + self->tps * 0.1f; // 计算脉冲频率
    self->last_count = count; // 存储当前计数器数值
    self->rps = self->tps / self->ticks_per_circle; // 计算转速 单位rps

    //LOG_DEBUG("COUNT:%d\r\n", delta_count);
    //LOG_DEBUG("TPS:%f\r\n", self->tps);
    //LOG_DEBUG("speed:%f\r\n\r\n", self->rps);
}

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self)
{
    memset(self, 0, sizeof(EncoderMotorObjectTypeDef));
    self->last_count = 0; // 上次计数值
    self->total_count = 0; // 总计数值
    self->overflow_num = 0; // 溢出计数
    self->last_tps = 0;
    self->tps = 0; // 转速
    self->rps = 0; // 转速
    self->direct = 0; // 旋转方向
    memset(&self->pid_controller, 0, sizeof(pid_controller_t));
}

