#include "pwm_servo.h"
#include <string.h>

void pwm_servo_duty_compare(PWMServoObjectTypeDef *self)   // Pulse-width comparison and speed control
{
    // Recalculate servo parameters based on the new target
    if(self->duty_changed) {
        self->duty_changed = false;
        self->inc_times = self->duration / 20; // Compute the number of increments
        if(self->target_duty > self->current_duty) { /* Compute the total position delta */
            self->duty_inc = (float)(-(self->target_duty - self->current_duty));
        } else {
            self->duty_inc = (float)(self->current_duty - self->target_duty);
        }
        self->duty_inc /= (float)self->inc_times; /* Compute the positional increment per control period */
        self->is_running = true;  // Servo begins to move
    }
		
		// Continue stepping the servo toward the target
    if(self->is_running) {
        --self->inc_times;
        if(self->inc_times == 0) {
            self->current_duty = self->target_duty;   // Final increment snaps to the target to ensure accuracy
            self->is_running = false; // Target reached; stop the servo
        } else {
            self->current_duty = self->target_duty + (int)(self->duty_inc * self->inc_times);
        }
    }
    self->duty_raw = self->current_duty + self->offset; // Apply configured offset to the command
}

void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration)
{
    duration = duration < 20 ? 20 : (duration > 30000 ? 30000 : duration); /* Clamp minimum and maximum motion duration */
	duty = duty > 2500 ? 2500 : (duty < 500 ? 500 : duty); /* Clamp pulse width range */
    self->target_duty = duty;
    self->duration = duration;
    self->duty_changed = true; /* Flag that the target changed so pwm_servo_duty_compare recalculates */
}

void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset)
{
    offset = offset < -100 ? -100 : (offset > 100 ? 100 : offset); /* Clamp offset range; different servos vary but ±100 is a safe default */
    self->offset = offset;
}

void pwm_servo_object_init(PWMServoObjectTypeDef *obj)
{
    memset(obj, 0, sizeof(PWMServoObjectTypeDef));
    obj->current_duty = 1500; /* Default position */
    obj->duty_raw = 1500;     /* Default pulse width */
}
