#include "rrclite_health.h"

#include <string.h>

static volatile uint16_t s_selftest_bits;

__attribute__((weak)) void rrc_fill_health_impl(rrc_health_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

void rrc_fill_health(rrc_health_t *out)
{
    rrc_fill_health_impl(out);
}

uint16_t rrc_get_selftest_bits(void)
{
    return s_selftest_bits;
}

void rrc_set_selftest_bits(uint16_t bits)
{
    s_selftest_bits = bits;
}

