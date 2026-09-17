//
// Created by 30148 on 2026/9/13.
//
#include "ram_d1_memory.h"
#include "FreeRTOS.h"
#include "task.h"

#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;
    size_t xBlockSize;
} BlockLink_t;

static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
static size_t xD1BlockAllocatedBit = 0;

/* 堆内存池，定义在本文件内部，放到 .ram_d1_heap 段(RAM_D1) */
__attribute__((section(".ram_d1_heap")))
static uint8_t ucD1Heap[D1_HEAP_TOTAL_SIZE];

static BlockLink_t xD1Start, *pxD1End = NULL;
static size_t xD1FreeBytesRemaining = 0U;
static size_t xD1MinimumEverFreeBytesRemaining = 0U;
static size_t xD1NumberOfSuccessfulAllocations = 0;
static size_t xD1NumberOfSuccessfulFrees = 0;

static void prvD1HeapInit( void );
static void prvInsertD1BlockIntoFreeList( BlockLink_t *pxBlockToInsert );

void *pvPortD1Malloc( size_t xWantedSize )
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void *pvReturn = NULL;

    vTaskSuspendAll();
    {
        if( pxD1End == NULL )
        {
            prvD1HeapInit();
        }

        if( ( xWantedSize & xD1BlockAllocatedBit ) == 0 )
        {
            if( xWantedSize > 0 )
            {
                xWantedSize += xHeapStructSize;
                if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
                {
                    xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
                    configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
                }
            }

            if( ( xWantedSize > 0 ) && ( xWantedSize <= xD1FreeBytesRemaining ) )
            {
                pxPreviousBlock = &xD1Start;
                pxBlock = xD1Start.pxNextFreeBlock;
                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock;
                }

                if( pxBlock != pxD1End )
                {
                    pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                    if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
                    {
                        pxNewBlockLink = (BlockLink_t *)( ( ( uint8_t * ) pxBlock ) + xWantedSize );
                        configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                        pxBlock->xBlockSize = xWantedSize;
                        prvInsertD1BlockIntoFreeList( pxNewBlockLink );
                    }

                    xD1FreeBytesRemaining -= pxBlock->xBlockSize;
                    if( xD1FreeBytesRemaining < xD1MinimumEverFreeBytesRemaining )
                    {
                        xD1MinimumEverFreeBytesRemaining = xD1FreeBytesRemaining;
                    }

                    pxBlock->xBlockSize |= xD1BlockAllocatedBit;
                    pxBlock->pxNextFreeBlock = NULL;
                    xD1NumberOfSuccessfulAllocations++;
                }
            }
        }
        traceMALLOC( pvReturn, xWantedSize );
    }
    ( void ) xTaskResumeAll();

#if( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )
        {
            extern void vApplicationMallocFailedHook( void );
            vApplicationMallocFailedHook();
        }
    }
#endif
    configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
    return pvReturn;
}

void vPortD1Free( void *pv )
{
    uint8_t *puc = ( uint8_t * ) pv;
    BlockLink_t *pxLink;

    if( pv != NULL )
    {
        puc -= xHeapStructSize;
        pxLink = (BlockLink_t *)puc;

        configASSERT( ( pxLink->xBlockSize & xD1BlockAllocatedBit ) != 0 );
        configASSERT( pxLink->pxNextFreeBlock == NULL );

        if( ( pxLink->xBlockSize & xD1BlockAllocatedBit ) != 0 )
        {
            if( pxLink->pxNextFreeBlock == NULL )
            {
                pxLink->xBlockSize &= ~xD1BlockAllocatedBit;
                vTaskSuspendAll();
                {
                    xD1FreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( pv, pxLink->xBlockSize );
                    prvInsertD1BlockIntoFreeList( (BlockLink_t *)pxLink );
                    xD1NumberOfSuccessfulFrees++;
                }
                ( void ) xTaskResumeAll();
            }
        }
    }
}

static void prvD1HeapInit( void )
{
    BlockLink_t *pxFirstFreeBlock;
    uint8_t *pucAlignedHeap;
    size_t uxAddress;
    size_t xTotalHeapSize = D1_HEAP_TOTAL_SIZE;

    uxAddress = ( size_t ) ucD1Heap;
    if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
    {
        uxAddress += ( portBYTE_ALIGNMENT - 1 );
        uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
        xTotalHeapSize -= uxAddress - ( size_t ) ucD1Heap;
    }
    pucAlignedHeap = ( uint8_t * ) uxAddress;

    xD1Start.pxNextFreeBlock = (BlockLink_t *)pucAlignedHeap;
    xD1Start.xBlockSize = ( size_t ) 0;

    uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
    uxAddress -= xHeapStructSize;
    uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
    pxD1End = (BlockLink_t *)uxAddress;

    pxD1End->xBlockSize = 0;
    pxD1End->pxNextFreeBlock = NULL;

    pxFirstFreeBlock = (BlockLink_t *)pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
    pxFirstFreeBlock->pxNextFreeBlock = pxD1End;

    xD1MinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    xD1FreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

    if( xD1BlockAllocatedBit == 0 )
    {
        xD1BlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
    }
}

static void prvInsertD1BlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
    BlockLink_t *pxIterator;
    uint8_t *puc;

    for( pxIterator = &xD1Start;
         pxIterator->pxNextFreeBlock < pxBlockToInsert &&
         pxIterator->pxNextFreeBlock != NULL;
         pxIterator = pxIterator->pxNextFreeBlock )
    {
    }

    puc = ( uint8_t * ) pxIterator;
    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
    {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }

    puc = ( uint8_t * ) pxBlockToInsert;
    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
    {
        if( pxIterator->pxNextFreeBlock != pxD1End )
        {
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        }
        else
        {
            pxBlockToInsert->pxNextFreeBlock = pxD1End;
        }
    }
    else
    {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    if( pxIterator != pxBlockToInsert )
    {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    }
}

void vPortGetD1HeapStats( HeapStats_t *pxHeapStats )
{
    BlockLink_t *pxBlock;
    size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY;

    if( pxD1End == NULL )
    {
        pxHeapStats->xSizeOfLargestFreeBlockInBytes = 0;
        pxHeapStats->xSizeOfSmallestFreeBlockInBytes = 0;
        pxHeapStats->xNumberOfFreeBlocks = 0;
        pxHeapStats->xAvailableHeapSpaceInBytes = 0;
        pxHeapStats->xNumberOfSuccessfulAllocations = 0;
        pxHeapStats->xNumberOfSuccessfulFrees = 0;
        pxHeapStats->xMinimumEverFreeBytesRemaining = 0;
        return;
    }

    vTaskSuspendAll();
    {
        pxBlock = xD1Start.pxNextFreeBlock;
        if( pxBlock != NULL )
        {
            do
            {
                xBlocks++;
                if( pxBlock->xBlockSize > xMaxSize )
                    xMaxSize = pxBlock->xBlockSize;
                if( pxBlock->xBlockSize < xMinSize )
                    xMinSize = pxBlock->xBlockSize;

                pxBlock = pxBlock->pxNextFreeBlock;
            } while( pxBlock != pxD1End );
        }
    }
    xTaskResumeAll();

    pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
    pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
    pxHeapStats->xNumberOfFreeBlocks = xBlocks;

    taskENTER_CRITICAL();
    {
        pxHeapStats->xAvailableHeapSpaceInBytes = xD1FreeBytesRemaining;
        pxHeapStats->xNumberOfSuccessfulAllocations = xD1NumberOfSuccessfulAllocations;
        pxHeapStats->xNumberOfSuccessfulFrees = xD1NumberOfSuccessfulFrees;
        pxHeapStats->xMinimumEverFreeBytesRemaining = xD1MinimumEverFreeBytesRemaining;
    }
    taskEXIT_CRITICAL();
}