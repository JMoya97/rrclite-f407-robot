#ifndef __GLOBAL_CONF_H
#define __GLOBAL_CONF_H


#define LITHIUM_ION_3S 1
#define LITHIUM_ION_2S 2


#ifndef BATTERY_TYPE
#define BATTERY_TYPE LITHIUM_ION_2S
#endif


#if BATTERY_TYPE==LITHIUM_ION_3S
#define LOW_BATTERY_ALARM_THRESHOLD 9500
#elif BATTERY_TYPE==LITHIUM_ION_2S
#define LOW_BATTERY_ALARM_THRESHOLD 6300
#else
#define LOW_BATTERY_ALARM_THRESHOLD 0
#endif



#define KEY1_PUSHED_LEVEL 0
#define KEY2_PUSHED_LEVEL 0
#define LED_SYS_LEVEL_ON  0

#define LED_TASK_PERIOD     30u /* LED状态刷新间隔 */
#define BUZZER_TASK_PERIOD  30u /* 蜂鸣器状态刷新间隔 */
#define BUTTON_TASK_PERIOD  30u /* 板载按键扫描间隔 */
#define BATTERY_TASK_PERIOD 50u /* 电池电量检测间隔 */

#endif

