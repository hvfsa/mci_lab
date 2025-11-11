#ifndef HEAP_DRIVER_H
#define HEAP_DRIVER_H

// Include the standard definition header for size_t
// size_t is an unsigned integer type used to represent sizes of objects in bytes.
// It is commonly used in memory allocation functions.
#include <stddef.h>

/*
 * Initializes the custom heap allocator.
 * This function must be called once at the beginning (e.g., in main)
 * before using heap_alloc() or heap_free().
 * It is responsible for setting up internal memory structures,
 * typically marking all memory blocks as free.
 */
void heap_init(void);

/*
 * Allocates a block of memory of at least 'size' bytes.
 * 
 * Parameters:
 *   size - the number of bytes to allocate.
 * 
 * Returns:
 *   A pointer to the beginning of a memory block in SRAM
 *   that is at least 'size' bytes long.
 *   Returns NULL if the memory request cannot be fulfilled (e.g., not enough space).
 * 
 * Notes:
 *   - The implementation usually uses fixed-size blocks (e.g., 16 bytes).
 *   - Internally, it may round the size up and find a set of contiguous free blocks.
 *   - You must not use standard malloc here; the function manages SRAM directly.
 */
void* heap_alloc(size_t size);

/*
 * Frees a previously allocated block of memory.
 * 
 * Parameters:
 *   ptr - a pointer previously returned by heap_alloc.
 *         If ptr is NULL, the function does nothing.
 * 
 * Behavior:
 *   - Identifies which memory blocks were allocated for ptr.
 *   - Marks those blocks as free again so they can be reused.
 * 
 * Notes:
 *   - The implementation must ensure that only valid, aligned pointers are freed.
 *   - If an invalid or already-freed pointer is passed, behavior is undefined.
 */
void heap_free(void* ptr);
void heap_print_map(void);

#endif  // End of include guard
