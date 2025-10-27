#pragma once

#include <stdint.h>

#define ST_OK_IMU0      (1u << 0)
#define ST_OK_IMU1      (1u << 1)
#define ST_OK_ENCODERS  (1u << 2)
#define ST_OK_BATTERY   (1u << 3)
#define ST_OK_UART      (1u << 4)
#define ST_WARN_WS2812  (1u << 5)

extern volatile uint16_t g_selftest_bits;

typedef struct __attribute__((packed)) {
    uint8_t  system_state;
    uint8_t  uart_apply_pending;
    uint16_t heartbeat_age_ms;
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
    uint16_t retry_in_ms;
} rrc_health_t;

typedef struct __attribute__((packed)) {
    uint8_t  txq_depth;
    uint8_t  txq_high;
    uint16_t drops_imu;
    uint16_t drops_enc;
    uint16_t drops_batt;
    uint16_t drops_btn;
    uint16_t err_motor;
    uint16_t err_steer;
    uint16_t err_imu;
    uint16_t err_io;
} rrc_stats_snapshot_t;

void rrc_fill_health(rrc_health_t *out);
void rrc_fill_stats(rrc_stats_snapshot_t *out);

void rrc_stats_note_err_motor(void);
void rrc_stats_note_err_steer(void);
void rrc_stats_note_err_imu(void);
void rrc_stats_note_err_io(void);

void rrc_run_selftest(void);
uint16_t rrc_selftest_snapshot(void);
