#include "global.h"
#include "lwrb.h"
#include "usart.h"
#include "packet.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include "lwmem_porting.h"
#include "packet_handle.h"

#define PACKET_RX_FIFO_BUFFER_SIZE 2048 /* FIFO缓存长度 */
#define PACKET_RX_DMA_BUFFER_SIZE 256 /* 单个DMA缓存长度 */

/* 对外暴露的变量 */
struct PacketController packet_controller; /* 协议控制器实例 */

/* 内部函数 */
static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);
static void packet_dma_transmit_finished(UART_HandleTypeDef *huart);
static int send_packet(struct PacketController *self, struct PacketRawFrame *frame);
static void packet_uart_error_callblack(UART_HandleTypeDef *huart);

/* 串口控制依赖的外部变量 */
extern osSemaphoreId_t packet_tx_idleHandle;
extern osSemaphoreId_t packet_rx_not_emptyHandle;
extern osMessageQueueId_t packet_tx_queueHandle;

void packet_init(void)
{
    memset(&packet_controller, 0, sizeof(packet_controller));
    packet_controller.state = PACKET_CONTROLLER_STATE_STARTBYTE1;
    packet_controller.data_index = 0;

    /* DMA 接收缓存初始化 */
    static uint8_t rx_dma_buffer1[PACKET_RX_DMA_BUFFER_SIZE];
    static uint8_t rx_dma_buffer2[PACKET_RX_DMA_BUFFER_SIZE];

    packet_controller.rx_dma_buffers[0] = rx_dma_buffer1;
    packet_controller.rx_dma_buffers[1] = rx_dma_buffer2;
    packet_controller.rx_dma_buffer_size = PACKET_RX_DMA_BUFFER_SIZE;
    packet_controller.rx_dma_buffer_index = 0;

    /* 接收 FIFO 初始化 */
    packet_controller.rx_fifo_buffer  = LWMEM_CCM_MALLOC(PACKET_RX_FIFO_BUFFER_SIZE);
    packet_controller.rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t));
    lwrb_init(packet_controller.rx_fifo, packet_controller.rx_fifo_buffer, PACKET_RX_FIFO_BUFFER_SIZE);

    /* 发送接口 */
    packet_controller.send_packet = send_packet;
}

static int send_packet(struct PacketController *self, struct PacketRawFrame *frame)
{
    return osMessageQueuePut(packet_tx_queueHandle, &frame, 0, 10);
}

static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    int cur_index = packet_controller.rx_dma_buffer_index; /* 取得当前DMA缓存的索引号 */
    packet_controller.rx_dma_buffer_index ^= 1;
    HAL_UART_AbortReceive(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                 packet_controller.rx_dma_buffers[packet_controller.rx_dma_buffer_index], 
	                               PACKET_RX_DMA_BUFFER_SIZE);
    lwrb_write(packet_controller.rx_fifo, 
	             packet_controller.rx_dma_buffers[cur_index], length); /* 将接收到的数据写入fifo ring */
    osSemaphoreRelease(packet_rx_not_emptyHandle); /* 置位接收缓存非空信号 */
}

void packet_start_recv(void)
{
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID, packet_uart_error_callblack);
    HAL_UART_RegisterRxEventCallback(&huart1, packet_dma_receive_event_callback); /* 注册接收事件回调 */
    /* 使用 ReceiveToIdle_DMA 进行接收， 该函数会在DMA缓存满时中断或在接收空闲时中断并触发接收事件回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, packet_controller.rx_dma_buffers[packet_controller.rx_dma_buffer_index], PACKET_RX_DMA_BUFFER_SIZE);
    /* 开始接收 */
}

static void packet_uart_error_callblack(UART_HandleTypeDef *huart)
{
    packet_start_recv();
}

void packet_rx_task_entry(void *argument)
{
    osSemaphoreAcquire(packet_rx_not_emptyHandle, 0); /* 默认信号不为零，先清除掉 */
    __HAL_UNLOCK(&huart1);
    packet_start_recv();
    for(;;) {
        osSemaphoreAcquire(packet_rx_not_emptyHandle, osWaitForever); /* 等待接收缓存非空 */
        packet_recv(&packet_controller);
    }
}

void packet_tx_task_entry(void *argument)
{
    for(;;) {
        osSemaphoreAcquire(packet_tx_idleHandle, osWaitForever); /* 等待发送空闲信号 */
        osStatus_t status = osMessageQueueGet(packet_tx_queueHandle, &packet_controller.tx_dma_buffer, NULL, osWaitForever); /* 从发送队列中取出数据 */
        if(osOK == status) {
            HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, packet_dma_transmit_finished); /* 注册 DMA 接收完成回调 */
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)packet_controller.tx_dma_buffer, packet_controller.tx_dma_buffer->data_length + 5);  /* 触发 DMA 发送*/
        }
    }
}

static void packet_dma_transmit_finished(UART_HandleTypeDef * huart)
{
    lwmem_free(packet_controller.tx_dma_buffer);
    osStatus_t status = osMessageQueueGet(packet_tx_queueHandle, &packet_controller.tx_dma_buffer, NULL, 0); /* 从发送队列中取出数据 */
    if(osOK == status) {
        HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, packet_dma_transmit_finished); /* 注册 DMA 接收完成回调 */
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)packet_controller.tx_dma_buffer, packet_controller.tx_dma_buffer->data_length + 5); /* 触发 DMA 发送*/
    } else {
        osSemaphoreRelease(packet_tx_idleHandle); /* 置位发送空闲信号 */
    }
}

