# Motor FSM plan

## States
- IDLE, RUN, HOLD, REVERSING_COAST

## Inputs
- New PWM command ([-1000..1000]), current rps, direction sign, timers

## Outputs / rules
- Clamp target; ramp up/down; brake-down when reversing
- Deadband (~60100) + coast/hold behavior
- Reverse-gate until |rps| < ~0.30
- ACK: 0x18 (single) / 0x19 (multi) with {motor_id, pwm_target, pwm_applied}

## TODO
- Transition table
- Timing: run FSM in motor task (not ISR)
- Unit-ish asserts in a lightweight sim (optional)
