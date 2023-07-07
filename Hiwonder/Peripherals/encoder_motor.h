/**
 * @file encoder_motor.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机控制头文件
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

/** 
 * @brief 编码器电机对象结构体
*/
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

/**
@breif 编码器电机对象初始化
@param self 编码器电机对象指针
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);

/**
 * @brief 编码器电机速度测量更新
 * @detials 
 * @param self 编码器电机对象
 * @param period 本次调用与上次调用的时间间隔，单位为秒
 * @param count 编码器当前计数值
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t count);

/**
 * @drief 
 * @detials 实现了编码器电机的pid计算，控制，pwm输出控制更新， 需要定时执行
 * @param period
*/
void encoder_speed_control(EncoderMotorObjectTypeDef *self, float period);

/**
 * @brief 编码器电机速度控制
 * @detials 用户实际调用的控制接口
 * @param self 编码器电机对象
 * @param rps 电机目标转速， 单位 转每秒 r/s 
 * @retval None.
*/
void encoder_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif

