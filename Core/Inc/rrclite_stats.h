#pragma once

#include <stdint.h>

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

uint8_t rrc_txq_depth(void);
uint8_t rrc_txq_high_water(void);
void    rrc_txq_note_depth(uint8_t depth);

void rrc_stats_inc_drop_imu(void);
void rrc_stats_inc_drop_enc(void);
void rrc_stats_inc_drop_batt(void);
void rrc_stats_inc_drop_btn(void);

uint16_t rrc_stats_get_drop_imu(void);
uint16_t rrc_stats_get_drop_enc(void);
uint16_t rrc_stats_get_drop_batt(void);
uint16_t rrc_stats_get_drop_btn(void);

void rrc_stats_inc_err_motor(void);
void rrc_stats_inc_err_steer(void);
void rrc_stats_inc_err_imu(void);
void rrc_stats_inc_err_io(void);

uint16_t rrc_stats_get_err_motor(void);
uint16_t rrc_stats_get_err_steer(void);
uint16_t rrc_stats_get_err_imu(void);
uint16_t rrc_stats_get_err_io(void);

void rrc_fill_stats(rrc_stats_snapshot_t *out);

