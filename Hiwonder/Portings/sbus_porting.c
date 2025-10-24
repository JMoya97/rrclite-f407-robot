#include <stdio.h>
#include "SBus.h"
#include "lwmem_porting.h"
#include "lwrb.h"
#include "usart.h"
#include "global.h"
#include "packet_reports.h"

#define SBUS_RX_DMA_BUFFER_SIZE 32 /* SBUS DMA receive buffer length */
#define SBUS_RX_FIFO_BUFFER_SIZE 512 /* SBUS FIFO buffer length */

/* SBUS1 related variables */
static uint8_t *subs1_rx_dma_buffers[2];
static uint32_t sbus1_rx_dma_buffer_index;
static uint8_t *sbus1_rx_fifo_buffer;
static lwrb_t *sbus1_rx_fifo;
static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);

/* SBUS variables exposed externally */
SBusStatusObjectTypeDef *sbus1_status;

/* External variables required by SBUS1 */
extern osSemaphoreId_t sbus_data_readyHandle; /* SBUS data-ready semaphore handle */
extern osSemaphoreId_t sbus_data_ready_01_Handle;
extern osEventFlagsId_t sbus_data_ready_event_Handle;

void sbus_init(void)
{
    sbus1_status = LWMEM_CCM_MALLOC(sizeof(SBusStatusObjectTypeDef));
    sbus1_status->type_id = OBJECT_TYPE_ID_GAMEPAD_STATUS;

    /* Initialize DMA receive buffers */
    subs1_rx_dma_buffers[0] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE); /* DMA buffers cannot reside in CCM RAM */
    subs1_rx_dma_buffers[1] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE);
    sbus1_rx_dma_buffer_index = 0;

    /* Initialize the receive FIFO */
    sbus1_rx_fifo_buffer  = LWMEM_CCM_MALLOC(SBUS_RX_FIFO_BUFFER_SIZE);
    sbus1_rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t));
    lwrb_init(sbus1_rx_fifo, sbus1_rx_fifo_buffer, SBUS_RX_FIFO_BUFFER_SIZE);

}

static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    uint32_t cur_index = sbus1_rx_dma_buffer_index; /* Obtain the index of the current DMA buffer */
    sbus1_rx_dma_buffer_index ^= 1;
    if(length < SBUS_RX_DMA_BUFFER_SIZE) {
        HAL_UART_AbortReceive(&huart5);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], SBUS_RX_DMA_BUFFER_SIZE);
    lwrb_write(sbus1_rx_fifo, subs1_rx_dma_buffers[cur_index], length); /* Write the received data into the FIFO ring */
    if(lwrb_get_full(sbus1_rx_fifo) >= 25) { /* Each SBUS frame is 25 bytes; parse whenever enough bytes are collected */
        osSemaphoreRelease(sbus_data_ready_01_Handle); /* Signal that SBUS data is ready */
    }
}

#if ENABLE_SBUS
void sbus_rx_task_entry(void *argument)
{
    extern osMessageQueueId_t lvgl_event_queueHandle;
    extern SBusStatusObjectTypeDef sbus_status_disp;

    static uint8_t buf_temp[32];

    sbus_init();

    /* Start receiving */
start_sbus_receive:
    HAL_UART_AbortReceive(&huart5);
    HAL_UART_RegisterRxEventCallback(&huart5, sbus_dma_receive_event_callback); /* Register receive event callback */
    /* Receive with ReceiveToIdle_DMA: triggers when the buffer fills or reception idles and invokes the callback */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], SBUS_RX_DMA_BUFFER_SIZE);
    /* Parsing loop */
    for(;;) {
        if(osSemaphoreAcquire(sbus_data_ready_01_Handle, 500) != osOK) { /* Wait for data */
            goto start_sbus_receive;
        }
        for(;;) {
            memset(buf_temp, 0, 25);
            if(lwrb_peek(sbus1_rx_fifo, 0, buf_temp, 25) == 25) {  /* Retrieve 25 bytes from the buffer */
                if(sbus_decode_frame(buf_temp, sbus1_status) == 0) { /* Attempt to parse until bytes are insufficient */
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

