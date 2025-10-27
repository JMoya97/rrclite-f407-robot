#include "rrclite_health.h"

#include "rrclite_config.h"
#include "global_conf.h"

#include <stdbool.h>

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include "tim.h"

#if RRC_KEEP_IMU && ENABLE_IMU
bool rrc_imu_probe_source(uint8_t source_id, uint8_t *whoami_out);
#define ICM20948_WHOAMI_EXPECTED 0xEAU
#define QMI8658_WHOAMI_EXPECTED  0x05U
#endif

#if RRC_KEEP_BATTERY
extern uint16_t battery_latest_millivolts_le(void);
#endif

volatile uint16_t g_selftest_bits;

uint16_t rrc_selftest_snapshot(void)
{
    return g_selftest_bits;
}

void rrc_run_selftest(void)
{
    uint16_t bits = 0U;

#if RRC_KEEP_IMU && ENABLE_IMU
    uint8_t whoami = 0U;
    if (rrc_imu_probe_source(0U, &whoami) &&
        (whoami == ICM20948_WHOAMI_EXPECTED)) {
        bits |= ST_OK_IMU0;
    }

    whoami = 0U;
    if (rrc_imu_probe_source(1U, &whoami) &&
        (whoami == QMI8658_WHOAMI_EXPECTED)) {
        bits |= ST_OK_IMU1;
    }
#endif

#if RRC_KEEP_ENCODERS
    const uint16_t enc1_before = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    const uint16_t enc2_before = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

    osDelay(10U);

    const uint16_t enc1_after = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    const uint16_t enc2_after = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

    if ((enc1_before != enc1_after) || (enc2_before != enc2_after)) {
        bits |= ST_OK_ENCODERS;
    }
#endif

#if RRC_KEEP_BATTERY
    const uint16_t mv = battery_latest_millivolts_le();
    if ((mv > 500U) && (mv < 30000U)) {
        bits |= ST_OK_BATTERY;
    }
#endif

    bits |= ST_OK_UART;

    g_selftest_bits = bits;
}
