#include "adc.h"
#include "global.h"
#include "global_conf.h"
#include "buzzer.h"

float battery_volt = 0.0f; /* 电池电压全局变量, 单位 v */
static uint16_t adc_value[2];


void battery_check_timer_callback(void *argument)
{
    if(adc_value[0] != 0) { /* 内部参考电压不能为0, 否则无法计算 */
        float vdda = 3300.0f * ((float)(*((__IO uint16_t*)(0x1FFF7A2A)))) / ((float)adc_value[0]);
        float volt = vdda / 4095.0f * ((float)adc_value[1]) * 11.0f ; /* 100k + 10k 电阻分压， 实际电压是测量电压的11倍 */
        battery_volt = battery_volt == 0 ? volt : battery_volt * 0.95f + volt * 0.05f;
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 2);
	
#if ENABLE_BATTERY_LOW_ALARM
	static int count = 0;
    if(battery_volt < BATTERY_LOW_ALARM_THRESHOLD && battery_volt > 4900) {
        count++;
    } else {
        count = 0;
    }
    if(count > (int)(10 * 1000 / BATTERY_TASK_PERIOD)) { /* 每 10s 触发一次警报声 */
        buzzer_didi(buzzers[0], 2100, 800, 200, 5);
        count = 0;
    }
#endif
}
