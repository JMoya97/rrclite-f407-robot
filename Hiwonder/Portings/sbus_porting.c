#include <stdio.h>
#include "SBus.h"
#include "lwmem_porting.h"
#include "lwrb.h"
#include "usart.h"
#include "global.h"
#include "packet_reports.h"

#define SBUS_RX_DMA_BUFFER_SIZE 32 /* SBUS DMA 接收缓存长度 */
#define SBUS_RX_FIFO_BUFFER_SIZE 512 /* SBUS FIFO 缓存长度 */

/* SBUS1 相关变量 */
static uint8_t *subs1_rx_dma_buffers[2];
static uint32_t sbus1_rx_dma_buffer_index;
static uint8_t *sbus1_rx_fifo_buffer;
static lwrb_t *sbus1_rx_fifo;
static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);

/* SBUS 对外暴露变量 */
SBusStatusObjectTypeDef *sbus1_status;

/* SBUS1 需要的外部变量 */
extern osSemaphoreId_t sbus_data_readyHandle; /* subs数据就绪信号量Handle */
extern osSemaphoreId_t sbus_data_ready_01_Handle;
extern osEventFlagsId_t sbus_data_ready_event_Handle;

void sbus_init(void)
{
    sbus1_status = LWMEM_CCM_MALLOC(sizeof(SBusStatusObjectTypeDef));
    sbus1_status->type_id = OBJECT_TYPE_ID_GAMEPAD_STATUS;

    /* DMA 接收缓存初始化 */
    subs1_rx_dma_buffers[0] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE); /* DMA 缓存不能放在 CCMRAM 上 */
    subs1_rx_dma_buffers[1] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE);
    sbus1_rx_dma_buffer_index = 0;

    /* 接收 FIFO 初始化 */
    sbus1_rx_fifo_buffer  = LWMEM_CCM_MALLOC(SBUS_RX_FIFO_BUFFER_SIZE);
    sbus1_rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t));
    lwrb_init(sbus1_rx_fifo, sbus1_rx_fifo_buffer, SBUS_RX_FIFO_BUFFER_SIZE);

}

static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    uint32_t cur_index = sbus1_rx_dma_buffer_index; /* 取得当前DMA缓存的索引号 */
    sbus1_rx_dma_buffer_index ^= 1;
    if(length < SBUS_RX_DMA_BUFFER_SIZE) {
        HAL_UART_AbortReceive(&huart5);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], SBUS_RX_DMA_BUFFER_SIZE);
    lwrb_write(sbus1_rx_fifo, subs1_rx_dma_buffers[cur_index], length); /* 将接收到的数据写入fifo ring */
    if(lwrb_get_full(sbus1_rx_fifo) >= 25) { /* SBUS 每帧数据大小为25个字节， 每一次超过25个字节就进行一次解析 */
        osSemaphoreRelease(sbus_data_ready_01_Handle); /* 置位 SBUS 数据就绪信号量 */
    }
}

#if ENABLE_SBUS
void sbus_rx_task_entry(void *argument)
{
    extern osMessageQueueId_t lvgl_event_queueHandle;
    extern SBusStatusObjectTypeDef sbus_status_disp;

    static uint8_t buf_temp[32];

    sbus_init();

    /* 开始接收 */
start_sbus_receive:
    HAL_UART_AbortReceive(&huart5);
    HAL_UART_RegisterRxEventCallback(&huart5, sbus_dma_receive_event_callback); /* 注册接收事件回调 */
    /* 使用 ReceiveToIdle_DMA 进行接收， 该函数会在DMA缓存满时中断或在接收空闲时中断并触发接收事件回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], SBUS_RX_DMA_BUFFER_SIZE);
    /* 解析循环 */
    for(;;) {
        if(osSemaphoreAcquire(sbus_data_ready_01_Handle, 500) != osOK) { /* 等待数据就绪 */
            goto start_sbus_receive;
        }
        for(;;) {
            memset(buf_temp, 0, 25);
            if(lwrb_peek(sbus1_rx_fifo, 0, buf_temp, 25) == 25) {  /* 从缓存中取出25个字节 */
                if(sbus_decode_frame(buf_temp, sbus1_status) == 0) { /* 尝试解析直到字节不够 */
                    lwrb_skip(sbus1_rx_fifo, 25);
                    PacketReportSBusTypeDef report;
                    for(int i = 0; i < 16; ++i) {
                        report.channels[i] = sbus1_status->channels[i];
                    }
                    report.ch17 = sbus1_status->ch17;
                    report.ch18 = sbus1_status->ch18;
                    report.signal_loss = sbus1_status->signal_loss;
                    report.fail_safe = sbus1_status->fail_safe;
                    packet_transmit(&packet_controller, PACKET_FUNC_SBUS, &report, sizeof(PacketReportSBusTypeDef));
                } else {
                    lwrb_skip(sbus1_rx_fifo, 1);
                }
            } else {
                break;
            }
        }
    }
}
#endif

