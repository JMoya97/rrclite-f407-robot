#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct {
	float set_point;
	float kp;
	float ki;
	float kd;
	
	float previous_0_err;
	float previous_1_err;

	float output;
}pid_controller_t;

bool pid_update(pid_controller_t *pid, float actual, float time_delta);



#endif
