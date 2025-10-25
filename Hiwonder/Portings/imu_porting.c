#include "global.h"
#include "global_conf.h"
#include "cmsis_os2.h"
#include "icm20948.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "rrclite_packets.h"
#include "QMI8658.h"

#include <stdbool.h>

extern osSemaphoreId_t IMU_data_readyHandle;

static volatile uint8_t imu_stream_enabled;
static uint8_t imu_stream_sources_mask;
static uint8_t imu_stream_ack_each_frame;
static uint16_t imu0_seq;

static bool imu0_init_done;
static bool imu0_available;
static uint8_t imu0_whoami = 0xEAU;

static bool imu1_init_done;
static bool imu1_available;
static uint8_t imu1_whoami;

static void imu_timer_start(uint16_t period_ms);
static void imu_timer_stop(void);
static bool imu0_ensure_init(void);
static bool imu1_ensure_init(void);
static bool imu0_read_sample(rrc_imu_sample_t *out);
static bool imu1_read_sample(rrc_imu_sample_t *out);

#if ENABLE_IMU
static void imu_timer_start(uint16_t period_ms)
{
    if (period_ms < 5U) {
        period_ms = 5U;
    }

    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim6);
    HAL_TIM_Base_Stop_IT(&htim6);

    uint32_t arr = (uint32_t)period_ms * 10U;
    if (arr < 2U) {
        arr = 2U;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim6, arr - 1U);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

    htim6.Instance->EGR = TIM_EGR_UG;

    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim6);
}

static void imu_timer_stop(void)
{
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim6);
    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
}

static bool imu0_ensure_init(void)
{
    if (!imu0_init_done) {
        imu0_available = icm20948_begin();
        imu0_init_done = true;
        if (!imu0_available) {
            imu0_whoami = 0U;
        }
    }

    return imu0_available;
}

static bool imu1_ensure_init(void)
{
    if (!imu1_init_done) {
        imu1_whoami = get_id();
        if (imu1_whoami == 0x05U) {
            imu1_available = (begin() != 0U);
        } else {
            imu1_available = false;
        }
        imu1_init_done = true;
        if (!imu1_available) {
            imu1_whoami = 0U;
        }
    }

    return imu1_available;
}

static bool imu0_read_sample(rrc_imu_sample_t *out)
{
    if ((out == NULL) || !imu0_ensure_init()) {
        return false;
    }

    imu20948_raw_t raw;
    if (!icm20948_read_all(&raw)) {
        return false;
    }

    out->source_id = 0U;
    out->t_ms = HAL_GetTick();
    out->ax = raw.ax;
    out->ay = raw.ay;
    out->az = raw.az;
    out->gx = raw.gx;
    out->gy = raw.gy;
    out->gz = raw.gz;
    out->mx = raw.mx;
    out->my = raw.my;
    out->mz = raw.mz;
    out->temp_c = raw.temp_c;

    return true;
}

static bool imu1_read_sample(rrc_imu_sample_t *out)
{
    if ((out == NULL) || !imu1_ensure_init()) {
        return false;
    }

    float acc[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    read_sensor_data(acc, gyro);

    out->source_id = 1U;
    out->t_ms = HAL_GetTick();
    out->ax = acc[0];
    out->ay = acc[1];
    out->az = acc[2];
    out->gx = gyro[0];
    out->gy = gyro[1];
    out->gz = gyro[2];
    out->mx = 0.0f;
    out->my = 0.0f;
    out->mz = 0.0f;
    out->temp_c = 0.0f;

    return true;
}

void imu_task_entry(void *argument)
{
    (void)argument;

    (void)imu0_ensure_init();

    for (;;) {
        osSemaphoreAcquire(IMU_data_readyHandle, osWaitForever);
        if (!imu_stream_enabled) {
            continue;
        }

        if ((imu_stream_sources_mask & 0x01U) == 0U) {
            continue;
        }

        rrc_imu_sample_t sample;
        if (!imu0_read_sample(&sample)) {
            continue;
        }

        const rrc_imu_stream_frame_t frame = {
            .source_id = sample.source_id,
            .seq = imu0_seq++,
            .t_ms = sample.t_ms,
            .ax = sample.ax,
            .ay = sample.ay,
            .az = sample.az,
            .gx = sample.gx,
            .gy = sample.gy,
            .gz = sample.gz,
            .mx = sample.mx,
            .my = sample.my,
            .mz = sample.mz,
            .temp_c = sample.temp_c,
        };

        if (rrc_transport_send(RRC_FUNC_IMU, RRC_IMU_STREAM_FRAME,
                               &frame, sizeof(frame)) &&
            imu_stream_ack_each_frame) {
            const rrc_imu_frame_ack_t ack = {
                .source_id = frame.source_id,
                .seq = frame.seq,
            };
            (void)rrc_transport_send(RRC_FUNC_IMU, RRC_IMU_STREAM_ACK,
                                     &ack, sizeof(ack));
        }
    }
}

void imu_emit_oneshot(uint8_t sources_mask)
{
    if ((sources_mask & 0x01U) != 0U) {
        rrc_imu_sample_t sample;
        if (imu0_read_sample(&sample)) {
            (void)rrc_transport_send(RRC_FUNC_IMU, RRC_IMU_ONESHOT,
                                     &sample, sizeof(sample));
        }
    }

    if ((sources_mask & 0x02U) != 0U) {
        rrc_imu_sample_t sample;
        if (imu1_read_sample(&sample)) {
            (void)rrc_transport_send(RRC_FUNC_IMU, RRC_IMU_ONESHOT,
                                     &sample, sizeof(sample));
        }
    }
}

uint16_t imu_set_stream(uint8_t sources_mask, uint16_t period_ms,
                        uint8_t ack_each_frame, uint8_t *applied_mask,
                        uint8_t *applied_ack_each_frame)
{
    uint8_t mask = (uint8_t)(sources_mask & 0x01U);
    uint8_t ack_mode = (uint8_t)(ack_each_frame ? 1U : 0U);
    uint16_t applied_period = 0U;

    if ((mask != 0U) && !imu0_ensure_init()) {
        mask = 0U;
    }

    if (mask != 0U) {
        if (period_ms < 5U) {
            period_ms = 5U;
        }

        imu_timer_start(period_ms);

        imu_stream_enabled = 1U;
        imu_stream_sources_mask = mask;
        imu_stream_ack_each_frame = ack_mode;
        imu0_seq = 0U;
        applied_period = period_ms;
    } else {
        imu_timer_stop();
        imu_stream_enabled = 0U;
        imu_stream_sources_mask = 0U;
        imu_stream_ack_each_frame = 0U;
        imu0_seq = 0U;
    }

    if (applied_mask != NULL) {
        *applied_mask = mask;
    }

    if (applied_ack_each_frame != NULL) {
        *applied_ack_each_frame = (mask != 0U) ? ack_mode : 0U;
    }

    return applied_period;
}

void imu_emit_whoami(uint8_t source_id)
{
    rrc_imu_whoami_resp_t resp = {
        .source_id = source_id,
        .whoami = 0U,
        .status = (uint8_t)RRC_SYS_ERR_NO_DEVICE,
    };

    if (source_id == 0U) {
        if (imu0_ensure_init()) {
            resp.whoami = imu0_whoami;
            resp.status = 0U;
        }
    } else if (source_id == 1U) {
        if (imu1_ensure_init()) {
            resp.whoami = imu1_whoami;
            resp.status = 0U;
        }
    }

    (void)rrc_transport_send(RRC_FUNC_IMU, RRC_IMU_WHOAMI_STATUS,
                             &resp, sizeof(resp));
}

#endif // ENABLE_IMU
