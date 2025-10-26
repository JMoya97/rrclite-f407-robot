#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include <stdint.h>
#include <cmsis_os2.h>
#include "lwrb.h"
#include "lwmem.h"
#include "rrclite_proto.h"

#define PROTO_CONST_STARTBYTE1 0xAAu
#define PROTO_CONST_STARTBYTE2 0x55u
#define PACKET_PARSE_BUFFER_SIZE 64

#pragma pack(1)
struct PacketRawFrame {
    uint8_t start_byte1;
    uint8_t start_byte2;
    uint8_t function;
    uint8_t data_length;
    uint8_t data_and_checksum[257];
};
#pragma pack()

enum PacketControllerState {
    PACKET_CONTROLLER_STATE_STARTBYTE1, /**< Searching for frame header flag 1 */
    PACKET_CONTROLLER_STATE_STARTBYTE2, /**< Searching for frame header flag 2 */
    PACKET_CONTROLLER_STATE_FUNCTION, /**< Processing the frame function code */
    PACKET_CONTROLLER_STATE_LENGTH, /**< Processing the frame length */
    PACKET_CONTROLLER_STATE_DATA, /**< Processing the frame data */
    PACKET_CONTROLLER_STATE_CHECKSUM, /**< Processing the checksum */
};

enum PACKET_FUNCTION {
    PACKET_FUNC_SYS = RRC_FUNC_SYS,
    PACKET_FUNC_LED = RRC_FUNC_LED,
    PACKET_FUNC_BUZZER = RRC_FUNC_BUZZ,
    PACKET_FUNC_MOTOR = RRC_FUNC_MOTOR,
    PACKET_FUNC_BATT = RRC_FUNC_BATT,
    PACKET_FUNC_PWM_SERVO = 0x04,
    PACKET_FUNC_STEER = RRCV2_FUNC_STEER,
    PACKET_FUNC_KEY = RRC_FUNC_BUTTON,
    PACKET_FUNC_IMU = RRC_FUNC_IMU,
    PACKET_FUNC_ENCODER = RRC_FUNC_ENC,
    PACKET_FUNC_GAMEPAD = 0x09,
    PACKET_FUNC_SBUS = 0x0A,
    PACKET_FUNC_OLED = 0x0B,
    PACKET_FUNC_RGB = 0x0C,
    PACKET_FUNC_NONE = 0x80,
};

typedef void(*packet_handle)(struct PacketRawFrame *);

struct PacketController {
    enum PacketControllerState state;        /**< Current parser state */
    struct PacketRawFrame frame;             /**< Frame currently being processed */
    packet_handle handles[PACKET_FUNC_NONE]; /**< Registered handlers */
    int data_index;
    uint8_t *rx_dma_buffers[2]; /**< DMA buffer list */
    size_t rx_dma_buffer_size; /**< Size of each DMA buffer */
    volatile int rx_dma_buffer_index;    /**< Index of the DMA buffer currently receiving */

    uint8_t *rx_fifo_buffer;    /**< Receive FIFO buffer */
    lwrb_t *rx_fifo;            /**< Receive FIFO object */

    int (*send_packet)(struct PacketController *self, struct PacketRawFrame *frame);
    struct PacketRawFrame* tx_dma_buffer; /**< DMA buffer currently being transmitted */
};

void packet_register_callback(struct PacketController *self, enum PACKET_FUNCTION func, packet_handle p);

void packet_recv(struct PacketController *self);

int packet_transmit(struct PacketController *self, uint8_t func, void* data, size_t data_len);

#endif
