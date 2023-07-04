#ifndef __GLOBAL_CONF_H
#define __GLOBAL_CONF_H


#define KEY1_PUSHED_LEVEL 0
#define KEY2_PUSHED_LEVEL 0
#define LED_SYS_LEVEL_ON  0

#define LED_TASK_PERIOD    30u /* LED状态刷新间隔 */
#define BUZZER_TASK_PERIOD 30u /* 蜂鸣器状态刷新间隔 */
#define BUTTON_TASK_PERIOD 30u /* 板载按键扫描间隔 */


#define JGB520          1
#define HE37_52012      1
#define JGB37           2
#define JGB37_3865_520  2
#define JGA27           3
#define JGA27_310R_74   3
#define JGB528          4
#define JGB528_R131_8V  4


#ifndef MOTOR_TYPE
#define MOTOR_TYPE JGB520
#endif


#if MOTOR_TYPE==JGB520
/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为90.0。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_PID_KP  63.0f
#define MOTOR_PID_KI  2.6f
#define MOTOR_PID_KD  2.4f

#elif MOTOR_TYPE==JGB37
/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为45.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 45.0 = 1980
 */
#define MOTOR_TICKS_PER_CIRCLE 1980.0f
#define MOTOR_PID_KP  40.0f
#define MOTOR_PID_KI  2.0f
#define MOTOR_PID_KD  2.0f

#elif MOTOR_TYPE==JGA27
/* 电机轴每转产生13个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为20.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 13.0 * 4.0 * 20.0 = 1040
 */
#define MOTOR_TICKS_PER_CIRCLE 1040.0f
#define MOTOR_PID_KP  -36.0f
#define MOTOR_PID_KI  -1.0f
#define MOTOR_PID_KD  -1.0f

#elif MOTOR_TYPE==JGB528
/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为131.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 131.0 = 5764
 */
#define MOTOR_TICKS_PER_CIRCLE 5764.0f
#define MOTOR_PID_KP  300.0f
#define MOTOR_PID_KI  2.0f
#define MOTOR_PID_KD  12.0f

#else
#define MOTOR_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_PID_KP  350.0f
#define MOTOR_PID_KI  0.5f
#define MOTOR_PID_KD  45.0f

#endif

#endif

