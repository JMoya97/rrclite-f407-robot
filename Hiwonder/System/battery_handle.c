#include "adc.h"
#include "global.h"
#include "global_conf.h"
#include "buzzer.h"
#include "packet_reports.h"
#include "packet.h"
#include "rgb_spi.h"
#include "rrclite_packets.h"

#include <limits.h>


/* Battery LED thresholds (mV) for 2S LiPo - adjust as needed */
#define VBAT_LED_75 7900
#define VBAT_LED_50 7600
#define VBAT_LED_25 7400
/* Hysteresis to avoid flicker (mV) */
#define VBAT_LED_HYST 50


float battery_volt = 0.0f; /* Global battery voltage (mV) */
static uint16_t battery_min_limit = 6300; /* Low-voltage alarm threshold */

static uint16_t adc_value[2];

/* 0: RED, 1: ORANGE, 2: YELLOW, 3: GREEN */
static uint8_t battery_led_band = 255;

static volatile uint8_t battery_stream_enabled;
static uint16_t battery_stream_period_ms;
static uint16_t battery_stream_elapsed_ms;
static uint16_t batt_seq;

extern volatile uint16_t rrc_heartbeat_period_ms;

static uint16_t battery_latest_millivolts(void)
{
    const float v = battery_volt;
    if (v <= 0.0f) {
        return 0U;
    }

    const uint32_t rounded = (uint32_t)(v + 0.5f);
    return (uint16_t)(rounded > UINT16_MAX ? UINT16_MAX : rounded);
}

uint16_t battery_set_stream(uint8_t enable, uint16_t period_ms)
{
    if (enable) {
        if (period_ms < BATTERY_TASK_PERIOD) {
            period_ms = BATTERY_TASK_PERIOD;
        }

        battery_stream_enabled = 1U;
        battery_stream_period_ms = period_ms;
        battery_stream_elapsed_ms = 0U;
        batt_seq = 0U;

        return period_ms;
    }

    battery_stream_enabled = 0U;
    battery_stream_period_ms = 0U;
    battery_stream_elapsed_ms = 0U;
    batt_seq = 0U;

    return 0U;
}

uint16_t battery_latest_millivolts_le(void)
{
    return battery_latest_millivolts();
}


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
    if(adc_value[0] != 0 && adc_value[0] != 4095) { /* Internal reference must be non-zero to calculate voltage */
        //float vdda = 3300.0f * ((float)(*((__IO uint16_t*)(0x1FFF7A2A)))) / ((float)adc_value[0]);
        //float volt = vdda / 4095.0f * ((float)adc_value[1]) * 11.0f ; /* 100k + 10k resistor divider; actual voltage is 11x the measurement */
		float volt = 1210.0f / ((float)adc_value[0]) * ((float)adc_value[1]) * 11.0f;
        volt = volt > 20000 ? 0 : volt; /* Invalid reading if ADC exceeds maximum supply voltage */
        battery_volt = battery_volt == 0 ? volt : battery_volt * 0.95f + volt * 0.05f;
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 2);
    static uint32_t battery_report_elapsed_ms = 0U;
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
    const uint16_t heartbeat_period = rrc_heartbeat_period_ms;
    if (heartbeat_period > 0U) {
        uint32_t elapsed = battery_report_elapsed_ms + BATTERY_TASK_PERIOD;
        if (elapsed >= heartbeat_period) {
            battery_report_elapsed_ms = 0U;

            PacketReportBatteryVoltageTypeDef report;
            report.sub_cmd = 0x04;
            report.voltage = (int)(battery_volt + 0.5f);
            packet_transmit(&packet_controller, PACKET_FUNC_SYS, &report,
                            sizeof(PacketReportBatteryVoltageTypeDef));

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
        } else {
            battery_report_elapsed_ms = elapsed;
        }
    } else {
        battery_report_elapsed_ms = 0U;
    }

#if ENABLE_BATTERY_LOW_ALARM
    static int count = 0;
    if(battery_volt < battery_min_limit && battery_volt > 4900) {
        count++;
    } else {
        count = 0;
    }
    if(count > (int)(10 * 1000 / BATTERY_TASK_PERIOD)) { /* Trigger alarm every 10 s */
        buzzer_didi(buzzers[0], 2100, 800, 200, 5);
        count = 0;
    }
#endif

    if (battery_stream_enabled) {
        uint16_t elapsed = battery_stream_elapsed_ms + BATTERY_TASK_PERIOD;
        if (elapsed >= battery_stream_period_ms) {
            elapsed = 0U;

            const rrc_sys_battery_stream_frame_t frame = {
                .seq = batt_seq++,
                .millivolts_le = battery_latest_millivolts(),
            };

            (void)rrc_transport_send(RRC_FUNC_SYS,
                                     RRC_SYS_BATTERY_STREAM_CTRL,
                                     &frame, sizeof(frame));
        }

        battery_stream_elapsed_ms = elapsed;
    }
}


void change_battery_limit(uint16_t limit)
{
    battery_min_limit = limit;
}

