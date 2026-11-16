/*
 * Force-included into every rcheevos source (CMakeLists.txt, -include): routes
 * the component's heap calls through the game arena (rc_game_arena.h).
 * Function-like macros, so only calls are rewritten. stdlib.h comes first so
 * its own declarations are seen before the macros exist.
 */
#ifndef RC_ARENA_HOOK_H
#define RC_ARENA_HOOK_H
#include <stdlib.h>
#include <string.h>
#include "rc_game_arena.h"

#define malloc(n)      rc_arena_malloc(n)
#define calloc(n, s)   rc_arena_calloc((n), (s))
#define realloc(p, n)  rc_arena_realloc((p), (n))
#define free(p)        rc_arena_free(p)

#endif /* RC_ARENA_HOOK_H */
