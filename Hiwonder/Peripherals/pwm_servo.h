#ifndef __PWM_SERVO_H_
#define __PWM_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct PWMServoObject  PWMServoObjectTypeDef;

struct PWMServoObject {
    int id;     /**< @brief Servo ID */
    int offset; /**< @brief Servo offset */
    int target_duty;  /**< @brief Target pulse width */
    int current_duty; /**< @brief Current pulse width */
    int duty_raw;     /**< @brief Raw pulse width written to the timer, includes the offset (current_duty + offset) */

    /* Variables used for speed control */
    uint32_t duration;    /**< @brief Time for the servo to reach the target position (ms) */
    float duty_inc; /**< @brief Pulse width increment per update */
    int  inc_times; /**< @brief Number of required increments */
    bool is_running;  /**< @brief Whether the servo is currently moving */
    bool duty_changed; /**< @brief Whether the pulse width has changed */

    /* Hardware abstraction hooks provided externally */
    void (*write_pin)(uint32_t new_state); /* Configure the output level */
};

void pwm_servo_object_init(PWMServoObjectTypeDef *object);

void pwm_servo_duty_compare(PWMServoObjectTypeDef *self);

void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration);

void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset);

#endif

