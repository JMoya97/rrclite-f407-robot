#include "lwmem.h"

uint8_t lwmem_ram[1024 * 38];

lwmem_region_t lwmem_regions[] = {
    { (void*)0x10000000, 1024 * 64 }, /* Order must remain: CCM RAM address is lower than RAM and must be first */
    { (void*)lwmem_ram,  1024 * 38 },
    { NULL, 0}
};
