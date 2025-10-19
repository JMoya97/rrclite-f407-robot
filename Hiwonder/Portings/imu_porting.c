#include "global.h"
#include "packet_reports.h"
#include "packet.h"
#include "global_conf.h"
#include "cmsis_os2.h"
#include "icm20948.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

extern osSemaphoreId_t IMU_data_readyHandle;

static volatile uint8_t imu_stream_enabled;

#if ENABLE_IMU
void imu_task_entry(void *argument)
{
    (void)argument;

    (void)icm20948_begin();

    for (;;) {
		osSemaphoreAcquire(IMU_data_readyHandle, osWaitForever);
		if (!imu_stream_enabled) continue;

		imu20948_raw_t m;
		if (icm20948_read_all(&m)) {
			PacketReportIMU_Raw_V2_TypeDef rep;
			rep.sub_cmd = 0xA1;           // stream sample
			rep.t_ms    = HAL_GetTick();
            rep.accel.x = m.ax;  rep.accel.y = m.ay;  rep.accel.z = m.az;
            rep.gyro.x  = m.gx;  rep.gyro.y  = m.gy;  rep.gyro.z  = m.gz;
            rep.mag.x   = m.mx;  rep.mag.y   = m.my;  rep.mag.z   = m.mz;
            rep.temp_c  = m.temp_c;

            packet_transmit(&packet_controller, PACKET_FUNC_IMU, &rep, sizeof(rep));
        }
    }
}

void imu_read_once_and_report(void) {
    imu20948_raw_t m;
    if (icm20948_read_all(&m)) {
        PacketReportIMU_Raw_V2_TypeDef rep;
        rep.sub_cmd = 0xA0;   // one-shot
        rep.t_ms    = HAL_GetTick();
        rep.accel.x = m.ax;  rep.accel.y = m.ay;  rep.accel.z = m.az;
        rep.gyro.x  = m.gx;  rep.gyro.y  = m.gy;  rep.gyro.z  = m.gz;
        rep.mag.x   = m.mx;  rep.mag.y   = m.my;  rep.mag.z   = m.mz;
        rep.temp_c  = m.temp_c;

        packet_transmit(&packet_controller, PACKET_FUNC_IMU, (uint8_t*)&rep, sizeof(rep));
    }
}

void imu_set_stream(uint8_t enable, uint16_t period_ms)
{
    if (period_ms < 5) period_ms = 5;

    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim6);

    if (enable) {
        uint32_t arr = (uint32_t)period_ms * 10u;   // PSC=8399 -> 10 kHz
        if (arr < 2u) arr = 2u;

        __HAL_TIM_SET_AUTORELOAD(&htim6, arr - 1u);
        __HAL_TIM_SET_COUNTER(&htim6, 0u);
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

        htim6.Instance->EGR = TIM_EGR_UG;           // <-- latch ARR now (fixes your link error)

        __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim6);              // ensure the timer runs
        imu_stream_enabled = 1;
    } else {
        imu_stream_enabled = 0;
        HAL_TIM_Base_Stop_IT(&htim6);
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
    }
}


#endif // ENABLE_IMU
