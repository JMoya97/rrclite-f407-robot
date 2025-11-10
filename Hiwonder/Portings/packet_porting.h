#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool rrc_transport_send(uint8_t func, uint8_t sub, const void *payload, size_t len);
uint8_t rrc_packet_txq_high_water(void);

