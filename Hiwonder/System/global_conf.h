#ifndef __GLOBAL_CONF_H
#define __GLOBAL_CONF_H

#include "motors_param.h"

/* Feature toggles */
#define ENABLE_IMU                   1   /* Enable IMU task */
#define ENABLE_LVGL                  0   /* Enable LVGL task */
#define ENABLE_OLED                  0   /* Enable OLED display */
#define ENABLE_BATTERY_LOW_ALARM     1   /* Enable low-voltage alarm */

/* Motor ramp & hold (safe defaults) */
#define ENABLE_MOTOR_PID_LOOP        0

#define MOTOR_POLARITY_0 (+1)
#define MOTOR_POLARITY_1 (+1)
#define MOTOR_POLARITY_2 (+1)
#define MOTOR_POLARITY_3 (-1)

/* Slew per 10 ms (TIM7 tick) */
#define PWM_SLEW_UP_PER_TICK        30
#define PWM_SLEW_DOWN_PER_TICK      60

/* Gentle coast/hold when target = 0 */
#ifndef PWM_COAST_HOLD_PWM
#define PWM_COAST_HOLD_PWM          80
#endif
#ifndef HOLD_RPS_DEADBAND
#define HOLD_RPS_DEADBAND           0.05f   /* |rps| below this is considered stopped */
#endif

/* Reverse gate settings */
#ifndef REVERSE_GATE_RPS_THRESH
#define REVERSE_GATE_RPS_THRESH     0.30f   /* must be slower than this to allow sign flip */
#endif
#ifndef REVERSE_GATE_MIN_PWM
#define REVERSE_GATE_MIN_PWM        120     /* minimum magnitude when starting opposite dir */
#endif
#ifndef NORMAL_IDLE_DEADBAND
#define NORMAL_IDLE_DEADBAND        100     /* near-zero applied PWM considered 'idle' */
#endif

/* Legacy hold constants (kept for compatibility) */
#define HOLD_ENABLE                 1
#define HOLD_KP_TICK                2
#define HOLD_KD_TICK                0
#define HOLD_STATIC_PWM             60
#define HOLD_PWM_MAX                300
#define HOLD_DEADBAND_TICKS         1

/* Board I/O levels */
#define KEY1_PUSHED_LEVEL           0
#define KEY2_PUSHED_LEVEL           0
#define LED_SYS_LEVEL_ON            0

/* Task periods (ms) */
#define LED_TASK_PERIOD             30u  /* LED status refresh interval */
#define BUZZER_TASK_PERIOD          30u  /* Buzzer status refresh interval */
#define BUTTON_TASK_PERIOD          30u  /* Onboard button scan interval */
#define BATTERY_TASK_PERIOD         50u  /* Battery level check interval */

#define MOTOR_BRINGUP_TEST          0

#endif /* __GLOBAL_CONF_H */
