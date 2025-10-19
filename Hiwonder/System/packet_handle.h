#ifndef __PACKET_HANDLE
#define __PACKET_HANDLE

#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"

void packet_led_handle(struct PacketRawFrame *frame);

void packet_buzzer_handle(struct PacketRawFrame *frame);

void packet_motor_handle(struct PacketRawFrame *frame);

void packet_handle_init(void) ;

void imu_read_once_and_report(uint8_t sub_cmd);

void imu_set_stream(uint8_t enable, uint16_t period_ms);

void encoders_set_stream(uint8_t enable, uint16_t period_ms);

void encoders_read_once_and_report(uint8_t sub);

#endif


