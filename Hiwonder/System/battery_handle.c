#include "adc.h"
#include "global.h"
#include "global_conf.h"
#include "buzzer.h"

void battery_check_timer_callback(void *argument)
{
    static float last_volt = 0;
	static int count = 0;
    int adc = HAL_ADC_GetValue(&hadc1);
	
    last_volt = last_volt * 0.6f + (((float)adc / 4095.0f * 3300) / (10.0f / 110.0f)) * 0.4f;
	printf("volt:%d, %f\r\n",adc, last_volt);
	if(last_volt < LOW_BATTERY_ALARM_THRESHOLD && last_volt > 5500) {
		count++;
	}else{
		count = 0;
	}
	if(count > (int)(1000 / BATTERY_TASK_PERIOD * 8)) {
		buzzer_didi(buzzers[0], 2000, 1000, 500, 4);
		count = 0;
	}
}
