#pragma once

#include <stdint.h>
#include "cmsis_gcc.h"

#define RRC_DIAG 1

static inline int rrc_in_isr(void) {
    return __get_IPSR() != 0U;
}

#define RRC_ASSERT(x) do { \
    if (!(x)) { \
        /* TODO: add logging/trap hook */ \
    } \
} while (0)

