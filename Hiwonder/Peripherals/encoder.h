#pragma once
#include <stdint.h>

typedef struct EncoderObject EncoderObjectTypeDef;

struct EncoderObject {
    uint32_t id;

    /* state */
    uint16_t last_cnt;      /* last raw hardware count (0..1040-1) */
    int32_t  delta_ticks;   /* signed ticks since last handler call */
    int32_t  total_ticks;   /* signed accumulator (optional) */

    /* config (hardcoded ticks/rev in porting; keep here for clarity) */
    uint16_t ticks_per_rev; /* = 1040 */

    /* porting hook: read current raw counter */
    uint16_t (*read_counter)(EncoderObjectTypeDef *self);
};

/* init with sane defaults */
void encoder_object_init(EncoderObjectTypeDef *self);

/* must be called at a fixed period (e.g., 10 ms) */
void encoder_task_handler(EncoderObjectTypeDef *self, uint32_t period_ms);

/* helpers to fetch results */
static inline int32_t encoder_get_delta(EncoderObjectTypeDef *self) { return self->delta_ticks; }
static inline int32_t encoder_get_total(EncoderObjectTypeDef *self) { return self->total_ticks; }
