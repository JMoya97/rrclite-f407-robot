#include "rrclite_stats.h"

#include "cmsis_os2.h"

extern osMessageQueueId_t packet_tx_queueHandle;

static volatile uint8_t s_txq_high_water;

static volatile uint16_t s_drop_imu;
static volatile uint16_t s_drop_enc;
static volatile uint16_t s_drop_batt;
static volatile uint16_t s_drop_btn;

static volatile uint16_t s_err_motor;
static volatile uint16_t s_err_steer;
static volatile uint16_t s_err_imu;
static volatile uint16_t s_err_io;

static inline uint16_t clamp16(uint32_t value)
{
    return (value > 0xFFFFu) ? 0xFFFFu : (uint16_t)value;
}

uint8_t rrc_txq_depth(void)
{
    if (packet_tx_queueHandle == NULL) {
        return 0U;
    }

    return (uint8_t)osMessageQueueGetCount(packet_tx_queueHandle);
}

uint8_t rrc_txq_high_water(void)
{
    return s_txq_high_water;
}

void rrc_txq_note_depth(uint8_t depth)
{
    if (depth > s_txq_high_water) {
        s_txq_high_water = depth;
    }
}

static inline void inc_u16(volatile uint16_t *value)
{
    if (*value != 0xFFFFu) {
        ++(*value);
    }
}

void rrc_stats_inc_drop_imu(void)  { inc_u16(&s_drop_imu); }
void rrc_stats_inc_drop_enc(void)  { inc_u16(&s_drop_enc); }
void rrc_stats_inc_drop_batt(void) { inc_u16(&s_drop_batt); }
void rrc_stats_inc_drop_btn(void)  { inc_u16(&s_drop_btn); }

uint16_t rrc_stats_get_drop_imu(void)  { return s_drop_imu; }
uint16_t rrc_stats_get_drop_enc(void)  { return s_drop_enc; }
uint16_t rrc_stats_get_drop_batt(void) { return s_drop_batt; }
uint16_t rrc_stats_get_drop_btn(void)  { return s_drop_btn; }

void rrc_stats_inc_err_motor(void) { inc_u16(&s_err_motor); }
void rrc_stats_inc_err_steer(void) { inc_u16(&s_err_steer); }
void rrc_stats_inc_err_imu(void)   { inc_u16(&s_err_imu); }
void rrc_stats_inc_err_io(void)    { inc_u16(&s_err_io); }

uint16_t rrc_stats_get_err_motor(void) { return s_err_motor; }
uint16_t rrc_stats_get_err_steer(void) { return s_err_steer; }
uint16_t rrc_stats_get_err_imu(void)   { return s_err_imu; }
uint16_t rrc_stats_get_err_io(void)    { return s_err_io; }

void rrc_fill_stats(rrc_stats_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }

    out->txq_depth = rrc_txq_depth();
    out->txq_high  = rrc_txq_high_water();
    out->drops_imu  = clamp16(rrc_stats_get_drop_imu());
    out->drops_enc  = clamp16(rrc_stats_get_drop_enc());
    out->drops_batt = clamp16(rrc_stats_get_drop_batt());
    out->drops_btn  = clamp16(rrc_stats_get_drop_btn());
    out->err_motor  = clamp16(rrc_stats_get_err_motor());
    out->err_steer  = clamp16(rrc_stats_get_err_steer());
    out->err_imu    = clamp16(rrc_stats_get_err_imu());
    out->err_io     = clamp16(rrc_stats_get_err_io());
}

