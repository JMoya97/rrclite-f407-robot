#include "global.h"
#include "lwrb.h"
#include "usart.h"
#include "packet.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include "lwmem_porting.h"
#include "packet_handle.h"
#include "rrclite_packets.h"

#define PACKET_RX_FIFO_BUFFER_SIZE 2048 /* FIFO buffer length */
#define PACKET_RX_DMA_BUFFER_SIZE 256 /* DMA buffer length */

/* Externally visible variables */
struct PacketController packet_controller; /* Packet controller instance */

/* Internal functions */
static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);
static void packet_dma_transmit_finished(UART_HandleTypeDef *huart);
static int send_packet(struct PacketController *self, struct PacketRawFrame *frame);
static void packet_uart_error_callblack(UART_HandleTypeDef *huart);

static uint32_t g_rrc_uart_current_baud = RRC_UART_BAUD_115200;

/* External dependencies for UART control */
extern osSemaphoreId_t packet_tx_idleHandle;
extern osSemaphoreId_t packet_rx_not_emptyHandle;
extern osMessageQueueId_t packet_tx_queueHandle;

bool rrc_transport_send(uint8_t func, uint8_t sub, const void *payload, size_t len)
{
    if (!rrc_func_is_supported(func)) {
        return false;
    }

    if (!rrc_sub_is_supported(func, sub)) {
        return false;
    }

    if (!rrc_payload_len_valid_for(func, sub, len)) {
        return false;
    }

    const size_t total = len + 1U;
    uint8_t frame[RRC_MAX_PAYLOAD_LEN];

    frame[0] = sub;
    if (len > 0U) {
        if (payload == NULL) {
            return false;
        }
        memcpy(&frame[1], payload, len);
    }

    return packet_transmit(&packet_controller, func, frame, total) == 0;
}

void packet_init(void)
{
    memset(&packet_controller, 0, sizeof(packet_controller));
    packet_controller.state = PACKET_CONTROLLER_STATE_STARTBYTE1;
    packet_controller.data_index = 0;

    g_rrc_uart_current_baud = huart1.Init.BaudRate;

    /* Initialize DMA receive buffers */
    static uint8_t rx_dma_buffer1[PACKET_RX_DMA_BUFFER_SIZE];
    static uint8_t rx_dma_buffer2[PACKET_RX_DMA_BUFFER_SIZE];

    packet_controller.rx_dma_buffers[0] = rx_dma_buffer1;
    packet_controller.rx_dma_buffers[1] = rx_dma_buffer2;
    packet_controller.rx_dma_buffer_size = PACKET_RX_DMA_BUFFER_SIZE;
    packet_controller.rx_dma_buffer_index = 0;

    /* Initialize receive FIFO */
    packet_controller.rx_fifo_buffer  = LWMEM_CCM_MALLOC(PACKET_RX_FIFO_BUFFER_SIZE);
    packet_controller.rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t));
    lwrb_init(packet_controller.rx_fifo, packet_controller.rx_fifo_buffer, PACKET_RX_FIFO_BUFFER_SIZE);

    /* Transmit interface */
    packet_controller.send_packet = send_packet;
}

static int send_packet(struct PacketController *self, struct PacketRawFrame *frame)
{
    return osMessageQueuePut(packet_tx_queueHandle, &frame, 0, 10);
}

static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    int cur_index = packet_controller.rx_dma_buffer_index; /* Obtain the index of the current DMA buffer */
    packet_controller.rx_dma_buffer_index ^= 1;
    HAL_UART_AbortReceive(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                 packet_controller.rx_dma_buffers[packet_controller.rx_dma_buffer_index], 
	                               PACKET_RX_DMA_BUFFER_SIZE);
    lwrb_write(packet_controller.rx_fifo, 
	             packet_controller.rx_dma_buffers[cur_index], length); /* Write received data into the FIFO ring */
    osSemaphoreRelease(packet_rx_not_emptyHandle); /* Signal that the receive buffer is non-empty */
}

void packet_start_recv(void)
{
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID, packet_uart_error_callblack);
    HAL_UART_RegisterRxEventCallback(&huart1, packet_dma_receive_event_callback); /* Register receive event callback */
    /* Receive via ReceiveToIdle_DMA: triggers on full buffer or idle and invokes the callback */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, packet_controller.rx_dma_buffers[packet_controller.rx_dma_buffer_index], PACKET_RX_DMA_BUFFER_SIZE);
    /* Start receiving */
}

uint32_t rrc_uart_current_baud(void)
{
    return g_rrc_uart_current_baud;
}

bool rrc_uart_runtime_reconfigure(uint32_t baud)
{
    if (baud == g_rrc_uart_current_baud) {
        return true;
    }

    HAL_UART_AbortTransmit(&huart1);
    HAL_UART_AbortReceive(&huart1);

    if (HAL_UART_DeInit(&huart1) != HAL_OK) {
        return false;
    }

    huart1.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        return false;
    }

    g_rrc_uart_current_baud = baud;
    packet_start_recv();
    return true;
}

static void packet_uart_error_callblack(UART_HandleTypeDef *huart)
{
    packet_start_recv();
}

void packet_rx_task_entry(void *argument)
{
    osSemaphoreAcquire(packet_rx_not_emptyHandle, 0); /* Clear default non-zero semaphore value */
    __HAL_UNLOCK(&huart1);
    packet_start_recv();
    for(;;) {
        osSemaphoreAcquire(packet_rx_not_emptyHandle, osWaitForever); /* Wait until the receive buffer is non-empty */
        packet_recv(&packet_controller);
    }
}

void packet_tx_task_entry(void *argument)
{
    for(;;) {
        osSemaphoreAcquire(packet_tx_idleHandle, osWaitForever); /* Wait for transmit idle signal */
        osStatus_t status = osMessageQueueGet(packet_tx_queueHandle, &packet_controller.tx_dma_buffer, NULL, osWaitForever); /* Fetch frame from transmit queue */
        if(osOK == status) {
            HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, packet_dma_transmit_finished); /* Register DMA completion callback */
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)packet_controller.tx_dma_buffer, packet_controller.tx_dma_buffer->data_length + 5);  /* Start DMA transmission */
        }
    }
}

static void packet_dma_transmit_finished(UART_HandleTypeDef * huart)
{
    lwmem_free(packet_controller.tx_dma_buffer);
    osStatus_t status = osMessageQueueGet(packet_tx_queueHandle, &packet_controller.tx_dma_buffer, NULL, 0); /* Fetch frame from transmit queue */
    if(osOK == status) {
        HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, packet_dma_transmit_finished); /* Register DMA completion callback */
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)packet_controller.tx_dma_buffer, packet_controller.tx_dma_buffer->data_length + 5); /* Start DMA transmission */
    } else {
        osSemaphoreRelease(packet_tx_idleHandle); /* Signal transmit idle */
    }
}

