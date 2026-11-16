/* Game arena: see rc_game_arena.h. This file gets rc_arena_hook.h force-included
 * like every other rcheevos source, so the macros are dropped here: the real
 * malloc/free below are the normal heap. */
#include "rc_game_arena.h"

#undef malloc
#undef calloc
#undef realloc
#undef free

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "esp_heap_caps.h"
#include "multi_heap.h"
#include "freertos/FreeRTOS.h"

static multi_heap_handle_t s_heap;
static uint8_t            *s_base;
static size_t              s_size;
static volatile int        s_window;
static volatile size_t     s_fallbacks;
/* multi_heap takes a portMUX as its lock (portENTER_CRITICAL), the same kind
 * the system heaps use: the arena is touched from httpTask (Core 0, the patch
 * buffer) and the loop task (Core 1, rcheevos). */
static portMUX_TYPE        s_lock = portMUX_INITIALIZER_UNLOCKED;

size_t rc_arena_init(size_t bytes)
{
    if (s_heap)
        return s_size;
    bytes &= ~(size_t)15;
    s_base = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_base)
        return 0;
    s_heap = multi_heap_register(s_base, bytes);
    if (!s_heap) {
        heap_caps_free(s_base);
        s_base = NULL;
        return 0;
    }
    multi_heap_set_lock(s_heap, &s_lock);
    s_size = bytes;
    return s_size;
}

void rc_arena_window(int open)
{
    s_window = open ? 1 : 0;
}

int rc_arena_owns(const void *ptr)
{
    const uint8_t *p = (const uint8_t *)ptr;
    return s_heap && p >= s_base && p < s_base + s_size;
}

void *rc_arena_alloc(size_t size)
{
    return s_heap ? multi_heap_malloc(s_heap, size) : NULL;
}

void *rc_arena_malloc(size_t size)
{
    if (s_window && s_heap) {
        void *p = multi_heap_malloc(s_heap, size);
        if (p)
            return p;
        s_fallbacks++;
    }
    return malloc(size);
}

void *rc_arena_calloc(size_t n, size_t size)
{
    if (s_window && s_heap) {
        if (size && n > SIZE_MAX / size)
            return NULL;
        void *p = multi_heap_malloc(s_heap, n * size);
        if (p) {
            memset(p, 0, n * size);
            return p;
        }
        s_fallbacks++;
    }
    return calloc(n, size);
}

void *rc_arena_realloc(void *ptr, size_t size)
{
    if (!ptr)
        return rc_arena_malloc(size);
    if (!rc_arena_owns(ptr))
        return realloc(ptr, size);
    if (size == 0) {
        multi_heap_free(s_heap, ptr);
        return NULL;
    }
    void *p = multi_heap_realloc(s_heap, ptr, size);
    if (p)
        return p;
    /* No room in the arena: move the block to the normal heap. */
    s_fallbacks++;
    p = malloc(size);
    if (!p)
        return NULL;            /* realloc semantics: the old block stays valid */
    size_t old = multi_heap_get_allocated_size(s_heap, ptr);
    memcpy(p, ptr, old < size ? old : size);
    multi_heap_free(s_heap, ptr);
    return p;
}

void rc_arena_free(void *ptr)
{
    if (!ptr)
        return;
    if (rc_arena_owns(ptr))
        multi_heap_free(s_heap, ptr);
    else
        free(ptr);
}

void rc_arena_get_info(rc_arena_info_t *info)
{
    memset(info, 0, sizeof(*info));
    if (!s_heap)
        return;
    multi_heap_info_t mi;
    multi_heap_get_info(s_heap, &mi);
    info->size             = s_size;
    info->free_bytes       = mi.total_free_bytes;
    info->largest_free     = mi.largest_free_block;
    info->allocated_bytes  = mi.total_allocated_bytes;
    info->allocated_blocks = mi.allocated_blocks;
    info->fallbacks        = s_fallbacks;
}
