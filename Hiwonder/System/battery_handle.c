#include "adc.h"
#include "global.h"
#include "global_conf.h"
#include "buzzer.h"
#include "packet_reports.h"
#include "packet.h"
#include "rgb_spi.h"


/* Battery LED thresholds (mV) for 2S LiPo - adjust as needed */
#define VBAT_LED_75 7900
#define VBAT_LED_50 7600
#define VBAT_LED_25 7400
/* Hysteresis to avoid flicker (mV) */
#define VBAT_LED_HYST 50


float battery_volt = 0.0f; /* 电池电压全局变量, 单位 v */
static uint16_t battery_min_limit = 6300; /* 低压报警值 */

static uint16_t adc_value[2];

/* 0: RED, 1: ORANGE, 2: YELLOW, 3: GREEN */
static uint8_t battery_led_band = 255;


extern osMessageQueueId_t lvgl_event_queueHandle;


static void battery_led_apply(uint8_t band)
{
    /* WS2812 expects GRB bytes per LED */
    uint8_t rgb[6] = {0};
    switch(band) {
        case 3: /* GREEN */
            rgb[0]=0x30; rgb[1]=0x00; rgb[2]=0x00;
            rgb[3]=0x30; rgb[4]=0x00; rgb[5]=0x00;
            break;
        case 2: /* YELLOW */
            rgb[0]=0x20; rgb[1]=0x20; rgb[2]=0x00;
            rgb[3]=0x20; rgb[4]=0x20; rgb[5]=0x00;
            break;
        case 1: /* ORANGE */
            rgb[0]=0x10; rgb[1]=0x30; rgb[2]=0x00;
            rgb[3]=0x10; rgb[4]=0x30; rgb[5]=0x00;
            break;
        case 0: default: /* RED */
            rgb[0]=0x00; rgb[1]=0x30; rgb[2]=0x00;
            rgb[3]=0x00; rgb[4]=0x30; rgb[5]=0x00;
            break;
    }
    set_id_rgb_color(1 , rgb);
}
void battery_check_timer_callback(void *argument)
{
    if(adc_value[0] != 0 && adc_value[0] != 4095) { /* 内部参考电压不能为0, 否则无法计算 */
        //float vdda = 3300.0f * ((float)(*((__IO uint16_t*)(0x1FFF7A2A)))) / ((float)adc_value[0]);
        //float volt = vdda / 4095.0f * ((float)adc_value[1]) * 11.0f ; /* 100k + 10k 电阻分压， 实际电压是测量电压的11倍 */
		float volt = 1210.0f / ((float)adc_value[0]) * ((float)adc_value[1]) * 11.0f;
        volt = volt > 20000 ? 0 : volt; /* ADC读取值超过最大允许供电电压，数据错误 */
        battery_volt = battery_volt == 0 ? volt : battery_volt * 0.95f + volt * 0.05f;
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 2);
    static int battery_report_count = 0;
    battery_report_count++;
    /* BATTERY_LED_UPDATE: map voltage to color bands with hysteresis */
    int v = (int)battery_volt; /* mV */
    uint8_t next_band = battery_led_band;
    if (battery_led_band == 255) {
        /* first time classify without hysteresis */
        next_band = (v < VBAT_LED_25) ? 0 : (v < VBAT_LED_50) ? 1 : (v < VBAT_LED_75) ? 2 : 3;
    } else {
        switch (battery_led_band) {
            case 3: if (v < (VBAT_LED_75 - VBAT_LED_HYST)) next_band = 2; break;
            case 2: if (v >= (VBAT_LED_75 + VBAT_LED_HYST)) next_band = 3; else if (v < (VBAT_LED_50 - VBAT_LED_HYST)) next_band = 1; break;
            case 1: if (v >= (VBAT_LED_50 + VBAT_LED_HYST)) next_band = 2; else if (v < (VBAT_LED_25 - VBAT_LED_HYST)) next_band = 0; break;
            case 0: default: if (v >= (VBAT_LED_25 + VBAT_LED_HYST)) next_band = 1; break;
        }
    }
    if (next_band != battery_led_band) {
        battery_led_band = next_band;
        battery_led_apply(battery_led_band);
    }


    if(battery_report_count > (int)(1 * 1000 / BATTERY_TASK_PERIOD)) { /* 定时发送蓝牙电压报告 */
        battery_report_count = 0;
		PacketReportBatteryVoltageTypeDef report;
		report.sub_cmd = 0x04;
		report.voltage = (int)(battery_volt + 0.5f);
        packet_transmit(&packet_controller, PACKET_FUNC_SYS, &report, sizeof(PacketReportBatteryVoltageTypeDef));

#if ENABLE_OLED
		extern int oled_battery;
		oled_battery = (int)(battery_volt + 0.5f);
#endif
		
#if ENABLE_LVGL
        ObjectTypeDef object;
        object.structure.type_id = OBJECT_TYPE_ID_BATTERY_VOLTAGE;
        *((uint16_t*)object.structure.data) = (int)(battery_volt + 0.5f);
        osMessageQueuePut(lvgl_event_queueHandle, &object, 0, 0);
#endif
    }

#if ENABLE_BATTERY_LOW_ALARM
    static int count = 0;
    if(battery_volt < battery_min_limit && battery_volt > 4900) {
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


void change_battery_limit(uint16_t limit)
{
    battery_min_limit = limit;
}

