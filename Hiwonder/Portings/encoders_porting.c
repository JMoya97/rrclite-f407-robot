#include "global.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "tim.h"

#include "packet.h"
#include "packet_reports.h"
#include "rrclite_packets.h"

#define TIM7_PERIOD_MS 10

/* latest latched counters (updated from TIM7 cb) */
static uint16_t enc_c1, enc_c2;
static uint16_t enc_seq;

/* paced stream state (local semaphore) */
static osSemaphoreId_t s_enc_stream_sem;       /* local, not global */
static volatile uint8_t  enc_stream_enabled = 0;
static volatile uint16_t enc_stream_div     = 0;   /* emit every (div * 10ms) */
static volatile uint16_t enc_stream_cnt     = 0;

/* start encoder HW (moved out of motor_porting.c) */
void encoders_init(void) {
    /* M1 / TIM5 */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim5);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    /* M2 / TIM2 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    if (!s_enc_stream_sem) {
        s_enc_stream_sem = osSemaphoreNew(1, 0, NULL);
    }
}

/* called from TIM7 IRQ: latch counts + (maybe) wake stream task */
void encoders_timer7_cb(void) {
    enc_c1 = __HAL_TIM_GET_COUNTER(&htim5); /* M1 */
    enc_c2 = __HAL_TIM_GET_COUNTER(&htim2); /* M2 */

    if (enc_stream_enabled) {
        if (++enc_stream_cnt >= enc_stream_div) {
            enc_stream_cnt = 0;
            if (s_enc_stream_sem) osSemaphoreRelease(s_enc_stream_sem);
        }
    }
}

/* host control (sub 0x91): enable stream and set divider in TIM7 ticks */
uint16_t encoders_set_stream(uint8_t enable, uint16_t period_ms) {
    enc_stream_enabled = enable ? 1U : 0U;
    if (!enc_stream_enabled) {
        enc_stream_cnt = 0U;
        enc_stream_div = 0U;
        enc_seq = 0U;
        return 0U;
    }

    uint16_t ticks = (uint16_t)((period_ms + (TIM7_PERIOD_MS / 2U)) / TIM7_PERIOD_MS);
    if (ticks < 1U) {
        ticks = 1U;
    }

    enc_stream_div = ticks;
    enc_stream_cnt = 0U;
    enc_seq = 0U;

    return (uint16_t)(ticks * TIM7_PERIOD_MS);
}

/* one-shot reply (sub 0x90), called from packet handler (task context) */
void encoders_read_once_and_report(uint8_t sub) {
    EncoderReadReportTypeDef rep;
    rep.sub  = sub;            /* 0x90 */
    rep.t_ms = HAL_GetTick();
    rep.c1   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    rep.c2   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    packet_transmit(&packet_controller, PACKET_FUNC_ENCODER, &rep, sizeof(rep));
}

/* stream task (waits on local semaphore; sends PACKET_FUNC_ENCODER, sub=0x91) */
void encoders_task_entry(void *argument) {
    (void)argument;
    for (;;) {
        osSemaphoreAcquire(s_enc_stream_sem, osWaitForever);
        if (!enc_stream_enabled) continue;

        // --- back-pressure guard (no vendor TX changes needed) ---
        extern osMessageQueueId_t packet_tx_queueHandle;
        if (osMessageQueueGetCount(packet_tx_queueHandle) >= 56) continue;

        const rrc_encoder_stream_frame_t frame = {
            .seq = enc_seq++,
            .c1 = enc_c1,
            .c2 = enc_c2,
            .c3 = 0U,
            .c4 = 0U,
        };

        (void)rrc_transport_send(RRC_FUNC_MOTOR,
                                 RRC_MOTOR_ENCODER_STREAM_CTRL,
                                 &frame, sizeof(frame));
    }
}
