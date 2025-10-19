#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
    MSTATE_IDLE = 0,
    MSTATE_RUN,
    MSTATE_HOLD,
    MSTATE_REVERSING_COAST
} motor_state_t;

typedef struct {
    int16_t cmd_in;        // last requested [-1000..1000]
    int16_t target;        // clamped target after logic
    int16_t applied;       // what we actually apply (after ramps/deadband)
    float   rps;           // estimated speed (+forward)
    int8_t  dir;           // sign(applied) { -1,0,+1 }
    motor_state_t state;

    // tuning
    int16_t deadband;      // ~60..100
    int16_t hold_pwm;      // ~150 (gentle hold)
    int16_t ramp_up;       // per 10ms tick, e.g. 30
    int16_t ramp_down;     // per 10ms tick, e.g. 60
    int16_t ramp_brake;    // when reversing, e.g. 150
    float   reverse_gate_rps; // allow dir flip when |rps| < 0.30
} motor_fsm_t;

void motor_fsm_init(motor_fsm_t* m);
void motor_fsm_update(motor_fsm_t* m, int16_t cmd_in, float rps);

#ifdef __cplusplus
}
#endif
