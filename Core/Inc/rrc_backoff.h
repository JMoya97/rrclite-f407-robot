#ifndef RRC_BACKOFF_H
#define RRC_BACKOFF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t initial_ms;
    uint32_t max_ms;
    uint32_t current_ms;
    float    factor;
    uint8_t  active;
} rrc_backoff_t;

void rrc_backoff_init(rrc_backoff_t *backoff,
                      uint32_t initial_ms,
                      float factor,
                      uint32_t max_ms);

void rrc_backoff_reset(rrc_backoff_t *backoff);

uint32_t rrc_backoff_next(rrc_backoff_t *backoff);

#ifdef __cplusplus
}
#endif

#endif /* RRC_BACKOFF_H */
