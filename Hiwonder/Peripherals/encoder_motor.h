#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;
struct EncoderMotorObject {
	int64_t last_count; // 上次更新速度时的计数器数值
	int32_t new_overflow_num;
	uint32_t new_count;
	int64_t total_count; // 总计数值
	int32_t overflow_num; // 溢出计数
	int32_t ticks_per_circle; // 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写
	float last_tps;
	float tps; // ticks per second 计数器频率
	float rps; // revolutions per second 输出轴转速 转每秒
	float current_pulse;
	uint8_t direct; // 旋转方向
	pid_controller_t pid_controller;
	float rps_limit;
	// portting 接口函数
	void (*set_pulse)(EncoderMotorObjectTypeDef *self, int speed); // 设置电机速度  -1000 ~ 1000
};

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t count);
void encoder_speed_control(EncoderMotorObjectTypeDef *self, float period);

/**
*/
void encoder_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif

