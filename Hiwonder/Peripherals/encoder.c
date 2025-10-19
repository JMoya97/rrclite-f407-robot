#include "encoder.h"

void encoder_object_init(EncoderObjectTypeDef *self)
{
    self->id = 0;
    self->last_cnt = 0;
    self->delta_ticks = 0;
    self->total_ticks = 0;
    self->ticks_per_rev = 1040;
    self->read_counter = 0;
}

void encoder_task_handler(EncoderObjectTypeDef *self, uint32_t period_ms)
{
    (void)period_ms;  /* not used now; you may use later for speed calc */
    if (!self || !self->read_counter) return;

    uint16_t now = self->read_counter(self);
    int32_t  d   = (int32_t)now - (int32_t)self->last_cnt;

    /* wrap @ 1040 (same logic you use today in TIM7) */
    const int half = self->ticks_per_rev / 2;     /* 520 for 1040 */
    if (d >  half) d -= self->ticks_per_rev;
    if (d < -half) d += self->ticks_per_rev;

    self->delta_ticks = d;
    self->total_ticks += d;
    self->last_cnt = now;
}
