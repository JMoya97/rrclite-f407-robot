#ifndef __MOTORS_PARAM_H
#define __MOTORS_PARAM_H
#include "encoder_motor.h"

typedef enum {
	MOTOR_TYPE_JGB520,
	MOTOR_TYPE_JGB37,
	MOTOR_TYPE_JGA27,
	MOTOR_TYPE_JGB528,
}MotorTypeEnum;

/* The motor shaft generates 11 pulses per revolution. The counter increments on both rising and falling edges of phases A and B,
 * so the count is four times the pulse count. With a 90.0:1 reduction ratio the output shaft produces 11.0 * 4.0 * 90.0 = 3960 counts per turn.
 */
#define MOTOR_JGB520_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_JGB520_PID_KP  63.0f
#define MOTOR_JGB520_PID_KI  2.6f
#define MOTOR_JGB520_PID_KD  2.4f
#define MOTOR_JGB520_RPS_LIMIT 1.5f

/* The motor shaft generates 11 pulses per revolution. Counting both edges of phases A and B results in four counts per pulse.
 * With a 45.0:1 reduction ratio the output shaft produces 11.0 * 4.0 * 45.0 = 1980 counts per turn.
 */
#define MOTOR_JGB37_TICKS_PER_CIRCLE 1980.0f
#define MOTOR_JGB37_PID_KP  40.0f
#define MOTOR_JGB37_PID_KI  2.0f
#define MOTOR_JGB37_PID_KD  2.0f
#define MOTOR_JGB37_RPS_LIMIT 3.0f

/* The motor shaft generates 13 pulses per revolution. Counting both edges of phases A and B yields four counts per pulse.
 * With a 20.0:1 reduction ratio the output shaft produces 13.0 * 4.0 * 20.0 = 1040 counts per turn.
 */
#define MOTOR_JGA27_TICKS_PER_CIRCLE 1040.0f
#define MOTOR_JGA27_PID_KP  -36.0f
#define MOTOR_JGA27_PID_KI  -1.0f
#define MOTOR_JGA27_PID_KD  -1.0f
#define MOTOR_JGA27_RPS_LIMIT 6.0f 

/* The motor shaft generates 11 pulses per revolution, and counting both edges of phases A and B yields four counts per pulse.
 * With a 131.0:1 reduction ratio the output shaft produces 11.0 * 4.0 * 131.0 = 5764 counts per turn.
 */
#define MOTOR_JGB528_TICKS_PER_CIRCLE 5764.0f
#define MOTOR_JGB528_PID_KP  300.0f
#define MOTOR_JGB528_PID_KI  2.0f
#define MOTOR_JGB528_PID_KD  12.0f
#define MOTOR_JGB528_RPS_LIMIT 1.1f 

/* The motor shaft generates 11 pulses per revolution. Counting both edges of phases A and B yields four counts per pulse.
 * With a 90.0:1 reduction ratio the output shaft produces 11.0 * 4.0 * 90.0 = 3960 counts per turn.
 */
#define MOTOR_DEFAULT_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_DEFAULT_PID_KP  63.0f
#define MOTOR_DEFAULT_PID_KI  2.6f
#define MOTOR_DEFAULT_PID_KD  2.4f
#define MOTOR_DEFAULT_RPS_LIMIT 1.35f

void set_motor_type(EncoderMotorObjectTypeDef *motor, MotorTypeEnum type);

#endif

