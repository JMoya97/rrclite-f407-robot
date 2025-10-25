#ifndef __PACKET_HANDLE
#define __PACKET_HANDLE

#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"

#include <stdint.h>

void packet_led_handle(struct PacketRawFrame *frame);

void packet_buzzer_handle(struct PacketRawFrame *frame);

void packet_motor_handle(struct PacketRawFrame *frame);

void packet_handle_init(void) ;

void imu_emit_oneshot(uint8_t sources_mask);

uint16_t imu_set_stream(uint8_t sources_mask, uint16_t period_ms,
                        uint8_t ack_each_frame, uint8_t *applied_mask,
                        uint8_t *applied_ack_each_frame);

void imu_emit_whoami(uint8_t source_id);

uint16_t encoders_set_stream(uint8_t enable, uint16_t period_ms);

void encoders_read_once_and_report(uint8_t sub);

extern volatile uint16_t rrc_motor_failsafe_timeout_ms;
extern volatile uint32_t rrc_motor_last_cmd_ms;
extern volatile uint16_t rrc_heartbeat_period_ms;

void rrc_motor_recovery_tick(uint32_t now_ms);

#endif


