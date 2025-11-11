S#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* Heap configuration */
#define HEAP_START_ADDR  ((uint8_t*)0x20001000)   /* start of managed heap */
#define HEAP_SIZE        (4 * 1024)               /* 4 KB */
#define BLOCK_SIZE       16                       /* bytes per block */
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE) /* 256 blocks */

/*
 * block_map[] semantics:
 *   0x00     -> free block
 *   0xFF     -> continuation block (part of a multi-block allocation)
 *   1..254   -> number of blocks allocated (stored in the first block of an allocation)
 */
static uint8_t block_map[BLOCK_COUNT];

/* Initialize the heap allocator (mark all blocks free). */
void heap_init(void)
{
    memset(block_map, 0x00, sizeof(block_map));
}

/* Helper: find contiguous run of `nblocks` free blocks. Returns start index or -1. */
static int find_free_run(size_t nblocks)
{
    if (nblocks == 0 || nblocks > BLOCK_COUNT) return -1;

    size_t run = 0;
    for (size_t i = 0; i < BLOCK_COUNT; ++i) {
        if (block_map[i] == 0x00) {
            run++;
            if (run == nblocks) {
                /* start index = i - nblocks + 1 */
                return (int)(i + 1 - nblocks);
            }
        } else {
            run = 0;
        }
    }
    return -1;
}

/*
 * Allocate size bytes from our fixed-block heap.
 * Returns pointer inside SRAM or NULL on failure.
 */
void* heap_alloc(size_t size)
{
    if (size == 0) return NULL;

    /* Determine how many blocks we need (round up) */
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    if (blocks_needed == 0 || blocks_needed > 254) return NULL; /* reserve 255 as invalid */

    int start_idx = find_free_run(blocks_needed);
    if (start_idx < 0) {
        return NULL; /* no contiguous run found */
    }

    /* Mark block_map: first block gets the count, subsequent blocks get 0xFF */
    block_map[start_idx] = (uint8_t)blocks_needed;
    for (size_t i = 1; i < blocks_needed; ++i) {
        block_map[start_idx + i] = 0xFF;
    }

    /* Return pointer to start of allocated region */
    uint8_t *ptr = HEAP_START_ADDR + (start_idx * BLOCK_SIZE);
    return (void*)ptr;
}

/*
 * Free memory previously returned by heap_alloc.
 * If ptr is invalid or NULL, it safely returns.
 */
void heap_free(void* ptr)
{
    if (ptr == NULL) return;

    unsigned long base   = (unsigned long)HEAP_START_ADDR;
    unsigned long p      = (unsigned long)ptr;

    /* Check range */
    if (p < base) return;
    if (p >= (base + HEAP_SIZE)) return;

    /* Check alignment */
    unsigned long offset = p - base;
    if ((offset % BLOCK_SIZE) != 0) return; /* not aligned to block boundary */

    size_t idx = offset / BLOCK_SIZE;
    if (idx >= BLOCK_COUNT) return;

    uint8_t first = block_map[idx];
    if (first == 0x00 || first == 0xFF) {
        /* Invalid free: either block already free, or pointer inside continuation */
        return;
    }

    size_t blocks_to_free = (size_t)first;
    if ((idx + blocks_to_free) > BLOCK_COUNT) {
        /* Prevent overflow if metadata is corrupted */
        return;
    }

    /* Clear the map entries */
    for (size_t i = 0; i < blocks_to_free; ++i) {
        block_map[idx + i] = 0x00;
    }
}

/* Function to print current block map status */
void heap_print_map(void)
{
    printf("Block Map: ");
    for (size_t i = 0; i < BLOCK_COUNT; ++i) {
        printf("%02X ", block_map[i]);
        if ((i + 1) % 32 == 0) printf("\n");
    }
    printf("\n");
}
