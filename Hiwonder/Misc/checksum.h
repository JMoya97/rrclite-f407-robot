#ifndef __CHECKSUM_H__
#define __CHECKSUM_H__

#include <stdio.h>
#include <stdint.h>

uint16_t checksum_sum(const uint8_t *buf, uint16_t len);

uint16_t checksum_xor(const uint8_t *buf, uint16_t len);  

uint16_t checksum_crc8(const uint8_t *buf, uint16_t len);  /* CRC8 校验 */

uint16_t checksum_crc16(const uint8_t *buf, uint16_t len); /* CRC16 校验 */

#endif
