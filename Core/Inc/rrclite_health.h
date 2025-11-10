#pragma once

#include <stdint.h>

/* Self-test bit assignments returned by SYS/0xF5 */
#define ST_OK_IMU0      (1u << 0)
#define ST_OK_IMU1      (1u << 1)
#define ST_OK_ENCODERS  (1u << 2)
#define ST_OK_BATTERY   (1u << 3)
#define ST_OK_UART      (1u << 4)
#define ST_WARN_WS2812  (1u << 5)

#pragma pack(push, 1)
typedef struct {
    uint8_t  system_state;        /* e.g. SAFE/RUN */
    uint8_t  uart_apply_pending;  /* 0/1 */
    uint16_t heartbeat_age_ms;    /* clamped */
    uint16_t failsafe_ms;
    uint8_t  motor_state;
    uint8_t  steer_state;
    uint8_t  enc_state;
    uint8_t  batt_state;
    uint8_t  btn_state;
    uint8_t  imu0_state;
    uint8_t  imu1_state;
    uint8_t  motor_err;
    uint8_t  steer_err;
    uint8_t  imu_err;
    uint8_t  io_err;
    uint16_t retry_in_ms_min;     /* min remaining across devices */
} rrc_health_t;
#pragma pack(pop)

void rrc_fill_health(rrc_health_t *out);

uint16_t rrc_get_selftest_bits(void);
void rrc_set_selftest_bits(uint16_t bits);

void rrc_selftest_run_once(void);

