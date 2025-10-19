#include "global.h"
#include "tim.h"
#include "lwmem_porting.h"
#include "motors_param.h"
#include "global_conf.h"

#ifndef MOTOR_POLARITY_0
#define MOTOR_POLARITY_0 (+1)
#endif
#ifndef MOTOR_POLARITY_1
#define MOTOR_POLARITY_1 (+1)
#endif

EncoderMotorObjectTypeDef *motors[2];
volatile int motors_pwm_target[2]  = {0,0};
volatile int motors_pwm_current[2] = {0,0};

void set_motor_param(EncoderMotorObjectTypeDef *motor, int32_t tpc, float rps_limit, float kp, float ki, float kd);
void set_motor_type(EncoderMotorObjectTypeDef *motor, MotorTypeEnum type);
void motors_init(void);
void motor_set_target_pwm(uint8_t id, int cmd);
static inline uint16_t duty_from_cmd(int cmd, TIM_HandleTypeDef* htim);

static void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int PWM);
static void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int PWM);

void set_motor_param(EncoderMotorObjectTypeDef *motor, int32_t tpc, float rps_limit, float kp, float ki, float kd)
{
    motor->ticks_per_circle = tpc;
    motor->rps_limit = rps_limit;
    motor->pid_controller.kp = kp;
    motor->pid_controller.ki = ki;
    motor->pid_controller.kd = kd;
}

void set_motor_type(EncoderMotorObjectTypeDef *motor, MotorTypeEnum type)
{
	switch(type) {
		case MOTOR_TYPE_JGB520:
			set_motor_param(motor, MOTOR_JGB520_TICKS_PER_CIRCLE, MOTOR_JGB520_RPS_LIMIT, MOTOR_JGB520_PID_KP, MOTOR_JGB520_PID_KI, MOTOR_JGB520_PID_KD);
			break;
		case MOTOR_TYPE_JGB37:
			set_motor_param(motor, MOTOR_JGB37_TICKS_PER_CIRCLE, MOTOR_JGB37_RPS_LIMIT, MOTOR_JGB37_PID_KP, MOTOR_JGB37_PID_KI, MOTOR_JGB37_PID_KD);
			break;
		case MOTOR_TYPE_JGA27:
			set_motor_param(motor, MOTOR_JGA27_TICKS_PER_CIRCLE, MOTOR_JGA27_RPS_LIMIT, MOTOR_JGA27_PID_KP, MOTOR_JGA27_PID_KI, MOTOR_JGA27_PID_KD);
			break;
		case MOTOR_TYPE_JGB528:
			set_motor_param(motor, MOTOR_JGB528_TICKS_PER_CIRCLE, MOTOR_JGB528_RPS_LIMIT, MOTOR_JGB528_PID_KP, MOTOR_JGB528_PID_KI, MOTOR_JGB528_PID_KD);
			break;
		default:
			break;
	}
}

void motors_init(void)
{
    for(int i = 0; i < 2; ++i) {
        motors[i] = LWMEM_CCM_MALLOC(sizeof( EncoderMotorObjectTypeDef));
        encoder_motor_object_init(motors[i]);
		motors[i]->ticks_overflow = MOTOR_JGA27_TICKS_PER_CIRCLE;
        motors[i]->ticks_per_circle = MOTOR_JGA27_TICKS_PER_CIRCLE;
        motors[i]->rps_limit = MOTOR_JGA27_RPS_LIMIT;
        motors[i]->pid_controller.set_point = 0.0f;
        motors[i]->pid_controller.kp = MOTOR_JGA27_PID_KP;
        motors[i]->pid_controller.ki = MOTOR_JGA27_PID_KI;
        motors[i]->pid_controller.kd = MOTOR_JGA27_PID_KD;
    }

    /* Motor 1 timer */
    motors[0]->set_pulse = motor1_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Motor 2 timer */
    motors[1]->set_pulse = motor2_set_pulse;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* Speed update timer */
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim7);

    /* Force all PWM outputs to 0 to ensure drivers latch a known state */
    for (int i = 0; i < 2; ++i) {
        if (motors[i] && motors[i]->set_pulse) motors[i]->set_pulse(motors[i], 0);
    }
}

void motor_set_target_pwm(uint8_t id, int cmd) {
	if (id >= 2) return;
	if (cmd >  1000) cmd =  1000;
    if (cmd < -1000) cmd = -1000;
    motors_pwm_target[id & 0x03] = cmd;
}

static inline uint16_t duty_from_cmd(int cmd, TIM_HandleTypeDef* htim)
{
    if (cmd >  1000) cmd =  1000;
    if (cmd < -1000) cmd = -1000;
    uint32_t arr  = __HAL_TIM_GET_AUTORELOAD(htim) + 1;
    uint32_t mag  = (uint32_t)(cmd < 0 ? -cmd : cmd);
    uint32_t duty = (mag * arr) / 1000u;
    if (duty >= arr) duty = arr - 1;
    return (uint16_t)duty;
}

static void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int PWM)
{
#if (MOTOR_POLARITY_0) < 0
    PWM = -PWM;
#endif
    uint16_t d = duty_from_cmd(PWM, &htim1);

    if (PWM > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, d);
    }
    else if (PWM < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

static void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int PWM)
{
#if (MOTOR_POLARITY_1) < 0
    PWM = -PWM;
#endif
    uint16_t d = duty_from_cmd(PWM, &htim1);

    if (PWM > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
    else if (PWM < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}
