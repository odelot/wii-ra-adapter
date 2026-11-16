#ifndef RC_GAME_ARENA_H
#define RC_GAME_ARENA_H
/*
 * Game arena: a fixed PSRAM block, reserved once at boot and run as its own
 * heap (multi_heap), that holds only what one game load builds: the patch
 * download buffer and everything rcheevos allocates while it parses that
 * patch. Nothing else can land in it, so after the game unloads it is whole
 * again.
 *
 * Why (HW 2026-11-15, Kirby -> SMG -> MKW): the shared PSRAM heap had 7.58MB
 * free but only a 3.6MB block. The blockers were 660 bytes of lwIP sockets (the
 * dashboard's TCP connections; lwIP allocates in PSRAM first) created while a
 * game was loaded, sitting between that game's rcheevos data and surviving its
 * unload. A reboot to defragment can't be used over USB (the d2x EHCI module
 * never hands a reconnected adapter back to the OHCI until the next IOS reload).
 *
 * Routing: inside the rcheevos component, malloc/calloc/realloc/free are
 * macros (rc_arena_hook.h, force-included) onto the rc_arena_* functions here.
 * They allocate in the arena only while the window is open (the patch
 * callback); free/realloc go by address, so a block is always returned to the
 * heap it came from, whoever frees it. Arena full -> the normal heap, as
 * before.
 */
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reserve the arena (call once, early in setup). Returns its size, 0 if the
 * block could not be allocated (everything then uses the normal heap). */
size_t rc_arena_init(size_t bytes);

/* Open/close the routing window for the rcheevos allocations. */
void   rc_arena_window(int open);

/* Allocation entry points (rcheevos goes through these via the macros). */
void  *rc_arena_malloc(size_t size);
void  *rc_arena_calloc(size_t n, size_t size);
void  *rc_arena_realloc(void *ptr, size_t size);
void   rc_arena_free(void *ptr);

/* Allocate directly in the arena, ignoring the window (the patch buffer).
 * NULL if the arena does not have it. Grow/shrink/free with the functions
 * above. */
void  *rc_arena_alloc(size_t size);

/* True if ptr lies inside the arena. */
int    rc_arena_owns(const void *ptr);

typedef struct {
    size_t size;            /* arena bytes (0 = no arena)         */
    size_t free_bytes;
    size_t largest_free;
    size_t allocated_bytes;
    size_t allocated_blocks;
    size_t fallbacks;       /* window allocations that did not fit */
} rc_arena_info_t;
void   rc_arena_get_info(rc_arena_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* RC_GAME_ARENA_H */
