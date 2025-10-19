#include "motor_fsm.h"

static int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void motor_fsm_init(motor_fsm_t* m) {
    m->cmd_in = m->target = m->applied = 0;
    m->rps = 0.0f; m->dir = 0; m->state = MSTATE_IDLE;
    m->deadband = 100; m->hold_pwm = 150;
    m->ramp_up = 30; m->ramp_down = 60; m->ramp_brake = 150;
    m->reverse_gate_rps = 0.30f;
}

void motor_fsm_update(motor_fsm_t* m, int16_t cmd_in, float rps) {
    m->cmd_in = clamp16(cmd_in, -1000, 1000);
    m->rps = rps;

    // Decide desired state
    if (m->cmd_in == 0) {
        // choose HOLD if we are still moving noticeably, else IDLE
        if ( (m->rps > 0.02f) || (m->rps < -0.02f) ) m->state = MSTATE_HOLD;
        else m->state = MSTATE_IDLE;
    } else {
        int8_t new_dir = (m->cmd_in > 0) ? +1 : -1;
        if (m->dir != 0 && new_dir != m->dir && ( (m->rps > 0.0f ? m->rps : -m->rps) > m->reverse_gate_rps ) ) {
            m->state = MSTATE_REVERSING_COAST;
        } else {
            m->state = MSTATE_RUN;
        }
    }

    // Compute target per state
    switch (m->state) {
    case MSTATE_IDLE:
        m->target = 0;
        break;
    case MSTATE_HOLD:
        // gentle hold torque towards zero speed
        if (m->rps > 0.02f)       m->target = -m->hold_pwm;
        else if (m->rps < -0.02f) m->target =  m->hold_pwm;
        else                      m->target = 0;
        break;
    case MSTATE_REVERSING_COAST:
        // brake down fast toward zero
        if (m->applied > 0) m->target = (int16_t)(m->applied - m->ramp_brake);
        else                m->target = (int16_t)(m->applied + m->ramp_brake);
        break;
    case MSTATE_RUN:
    default:
        m->target = m->cmd_in;
        break;
    }

    // Ramp applied towards target with asymmetric slopes
    int16_t diff = (int16_t)(m->target - m->applied);
    int16_t step = 0;
    if (diff > 0) {
        step = (m->state == MSTATE_REVERSING_COAST) ? m->ramp_brake : m->ramp_up;
        if (diff < step) step = diff;
        m->applied = (int16_t)(m->applied + step);
    } else if (diff < 0) {
        step = (m->state == MSTATE_REVERSING_COAST) ? m->ramp_brake : m->ramp_down;
        if (-diff < step) step = (int16_t)(-diff);
        m->applied = (int16_t)(m->applied - step);
    }

    // Deadband: ensure nonzero commands exceed threshold; zero stays zero
    if (m->applied != 0) {
        if (m->applied > 0 && m->applied <  m->deadband) m->applied = m->deadband;
        if (m->applied < 0 && m->applied > -m->deadband) m->applied = (int16_t)(-m->deadband);
    }

    // Update direction sign
    m->dir = (m->applied > 0) ? +1 : (m->applied < 0 ? -1 : 0);
}
