#include "rrc_backoff.h"

#ifndef RRC_BACKOFF_MIN_DELAY_MS
#define RRC_BACKOFF_MIN_DELAY_MS 1U
#endif

void rrc_backoff_init(rrc_backoff_t *backoff,
                      uint32_t initial_ms,
                      float factor,
                      uint32_t max_ms)
{
    if (backoff == NULL) {
        return;
    }

    backoff->initial_ms = (initial_ms == 0U) ? RRC_BACKOFF_MIN_DELAY_MS : initial_ms;
    backoff->factor     = (factor < 1.0f) ? 1.0f : factor;
    backoff->max_ms     = (max_ms == 0U) ? backoff->initial_ms : max_ms;
    if (backoff->max_ms < backoff->initial_ms) {
        backoff->max_ms = backoff->initial_ms;
    }

    backoff->current_ms = backoff->initial_ms;
    backoff->active     = 0U;
}

void rrc_backoff_reset(rrc_backoff_t *backoff)
{
    if (backoff == NULL) {
        return;
    }

    backoff->current_ms = backoff->initial_ms;
    backoff->active     = 0U;
}

uint32_t rrc_backoff_next(rrc_backoff_t *backoff)
{
    if (backoff == NULL) {
        return 0U;
    }

    if (backoff->active == 0U) {
        backoff->active     = 1U;
        backoff->current_ms = backoff->initial_ms;
        return backoff->current_ms;
    }

    float next = (float)backoff->current_ms * backoff->factor;
    if (next < (float)RRC_BACKOFF_MIN_DELAY_MS) {
        next = (float)RRC_BACKOFF_MIN_DELAY_MS;
    }

    uint32_t delay_ms = (uint32_t)next;
    if ((float)delay_ms < next) {
        delay_ms += 1U;
    }

    if (delay_ms < backoff->current_ms) {
        delay_ms = backoff->current_ms;
    }

    if (delay_ms > backoff->max_ms) {
        delay_ms = backoff->max_ms;
    }

    backoff->current_ms = delay_ms;
    return backoff->current_ms;
}
