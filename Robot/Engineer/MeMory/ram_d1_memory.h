#ifndef PYRO_ROBOT_RAM_D1_MEMORY_H
#define PYRO_ROBOT_RAM_D1_MEMORY_H

#include "FreeRTOS.h"
#include "pyro_core_dma_heap.h"

#ifdef __cplusplus
extern "C" {
#endif

#define D1_HEAP_TOTAL_SIZE      48000U

void* pvPortD1Malloc(size_t xWantedSize);
void  vPortD1Free(void *pv);
void  vPortGetD1HeapStats(HeapStats_t *pxHeapStats);

#ifdef __cplusplus
}
#endif

#endif //PYRO_ROBOT_RAM_D1_MEMORY_H