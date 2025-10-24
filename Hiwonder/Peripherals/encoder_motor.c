#include "encoder_motor.h"
#include "gpio.h"
#include "global_conf.h"
#include "motors_param.h"

void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t counter)
{
    counter = counter + self->overflow_num * self->ticks_overflow; /* Total count; 60000 depends on the configured timer overflow */
    int delta_count = counter - self->counter;
    self->counter = counter; /* Store the new count */
    self->tps = (float)delta_count / period * 0.9f + self->tps * 0.1f; /* Compute pulses per second */
    self->rps = self->tps / self->ticks_per_circle; /* Convert to revolutions per second */
}

void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period)
{
#if ENABLE_MOTOR_PID_LOOP
    float pulse = 0;
    pid_controller_update(&self->pid_controller, self->rps, period);   /* Update the PID controller */
        pulse = self->current_pulse + self->pid_controller.output; /* Compute the new PWM value */

        /* Clamp the PWM output based on the timer configuration; here 0-100% maps to 0-1000 */
        pulse = pulse > 1000 ?  1000 : pulse;
        pulse = pulse < -1000 ? -1000 : pulse;

    self->set_pulse(self, pulse > -250 && pulse < 250 ? 0 : pulse); /* Apply the new PWM value and avoid too-small duty cycles */
    self->current_pulse = pulse; /* Record the updated PWM value */
#else
    (void)self; (void)period; /* raw-PWM mode: do nothing */
#endif
}

void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps)
{
    rps = rps > self->rps_limit ? self->rps_limit : (rps < -self->rps_limit ? -self->rps_limit : rps); /* Clamp the target speed */
    self->pid_controller.set_point = rps; /* Set the PID target */
}

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self)
{
    self->counter = 0;
    self->overflow_num = 0;
    self->tps = 0;
    self->rps = 0;
    self->current_pulse = 0;
    self->ticks_overflow = 0;
    self->ticks_per_circle = MOTOR_JGA27_TICKS_PER_CIRCLE; /* Counts produced per output-shaft revolution */
    pid_controller_init(&self->pid_controller, 0, 0, 0);
}

