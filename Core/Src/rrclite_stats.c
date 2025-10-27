#include "rrclite_health.h"
#include "rrclite_packets.h"

#include <limits.h>
#include <string.h>

static uint32_t g_err_episodes_motor;
static uint32_t g_err_episodes_steer;
static uint32_t g_err_episodes_imu;
static uint32_t g_err_episodes_io;

void rrc_stats_note_err_motor(void)
{
    g_err_episodes_motor++;
}

void rrc_stats_note_err_steer(void)
{
    g_err_episodes_steer++;
}

void rrc_stats_note_err_imu(void)
{
    g_err_episodes_imu++;
}

void rrc_stats_note_err_io(void)
{
    g_err_episodes_io++;
}

static uint16_t clamp_u16(uint32_t value)
{
    return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t)value;
}

extern uint8_t rrc_txq_depth(void);
extern uint8_t rrc_txq_high_water(void);
extern uint32_t imu_stream_queue_drops(void);
extern uint32_t rrc_enc_stream_drops(void);
extern uint32_t rrc_batt_stream_drops(void);
extern uint32_t rrc_button_stream_drops(void);

void rrc_fill_stats(rrc_stats_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));

    out->txq_depth = rrc_txq_depth();
    out->txq_high = rrc_txq_high_water();
    out->drops_imu = clamp_u16(imu_stream_queue_drops());
    out->drops_enc = clamp_u16(rrc_enc_stream_drops());
    out->drops_batt = clamp_u16(rrc_batt_stream_drops());
    out->drops_btn = clamp_u16(rrc_button_stream_drops());
    out->err_motor = clamp_u16(g_err_episodes_motor);
    out->err_steer = clamp_u16(g_err_episodes_steer);
    out->err_imu = clamp_u16(g_err_episodes_imu);
    out->err_io = clamp_u16(g_err_episodes_io);
}
