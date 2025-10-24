#include "lwmem.h"

extern lwmem_region_t lwmem_regions[];
#define LWMEM_CCM_REGION (&lwmem_regions[0])
#define LWMEM_RAM_REGION (&lwmem_regions[1])

#define LWMEM_CCM_MALLOC(size)    lwmem_malloc_ex(NULL, &lwmem_regions[0], (size))
#define LWMEM_RAM_MALLOC(size)    lwmem_malloc_ex(NULL, &lwmem_regions[1], (size))
#define LWMEM_FREE(ptr)           lwmem_free_ex(NULL, (ptr))

