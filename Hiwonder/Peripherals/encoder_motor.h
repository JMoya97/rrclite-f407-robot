#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

struct EncoderMotorObject{
    int64_t counter;        /**< @brief Total count (64-bit to avoid overflow) */
    int64_t overflow_num;   /**< @brief Number of timer overflows */
    int32_t ticks_overflow; /**< @brief Counts per overflow interval */
    float tps;              /**< @brief Counter frequency (ticks per second) */
    float rps;              /**< @brief Output shaft speed in revolutions per second */
    int current_pulse;      /**< @brief Current PWM output; sign indicates direction */
    PID_ControllerTypeDef pid_controller; /**< @brief PID controller state */

    /** Porting hooks for hardware-specific behavior **/
    int32_t ticks_per_circle; /**< @brief Counts produced per output-shaft revolution */
        float rps_limit;  /**< @brief Maximum permitted speed, typically slightly below the 100% PWM value */

    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);
};

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);

void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t new_counter);

void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);

void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif

