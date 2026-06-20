# rcheevos (PATCHED: do_frame gate + work-stealing eval). Copied from the
# Arduino lib's src/ root, so INCLUDE_DIRS "." mirrors the Arduino include path
# (the flattened public headers live here). The impl .c under src/ resolve their
# internal headers same-dir-first (the dual rc_internal.h pattern). All .c built.
# NOTE: no CONFIGURE_DEPENDS — IDF includes this in cmake script mode (to extract
# REQUIRES) where CONFIGURE_DEPENDS is invalid. Re-run idf.py if .c files change.
file(GLOB_RECURSE RCHEEVOS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
idf_component_register(
    SRCS ${RCHEEVOS_SRCS}
    INCLUDE_DIRS "."
    # esp_timer + freertos: pulled in by our rc_client.c patches (gate profiling
    # via esp_timer_get_time; work-stealing worker via FreeRTOS tasks/semaphores).
    REQUIRES esp_timer freertos
)

# Upstream rcheevos is plain C, not written for IDF's -Werror=all / GCC 14.
#  -w           : hush warnings (e.g. %u for uint32_t = 'long unsigned int').
#  -fpermissive : GCC 14 turned -Wincompatible-pointer-types & friends into
#                 default ERRORS (not via -Werror, so -w can't reach them);
#                 e.g. rc_validate.c passes uint32_t* where unsigned* is wanted
#                 (same width on xtensa but a distinct type). -fpermissive
#                 downgrades those C permerrors back to (then-silenced) warnings.
target_compile_options(${COMPONENT_LIB} PRIVATE -w -fpermissive)

# Dirty/incremental eval: skip cold+clean triggers in rc_client's do_frame eval
# loop (the path to 60fps). PUBLIC so it propagates to `main` (REQUIRES rcheevos)
# — rc_client.c needs it for the struct fields + skip logic, main.cpp for the
# extern counters/toggle in the FRAME log. Runtime A/B via g_rc_dirty_eval_enabled.
# Validated host-side: upstream test suite stays 2313/2314 with the toggle ON.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_DIRTY_EVAL)

# Optimization C: enable the (otherwise-dead) condset short-circuit in
# rc_evaluate_trigger. Logic-neutral for unlocks (hit-target/Measured always
# evaluate; only cosmetic "other" tallies are skipped) and SYNERGIZES with
# dirty-eval (more triggers report cold -> more skips). Runtime A/B via
# g_rc_short_circuit_enabled. Host-validated: test_client clean, 4 cosmetic diffs.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_SHORT_CIRCUIT)

# Optimization B1 (incremental upd, deterministic host-tested slice): in
# rc_update_memref_values, skip re-resolving a NON-INDIRECT value-transform
# modified_memref whose parent+modifier operands provably didn't change this
# frame (CONST/FP, or a memref with .changed==0). INDIRECT_READ chains are NOT
# skipped here -- B2 handles those via the ESP snapshot byte-compare. The
# first-pass guard (incr_warmed, in rc_memref_list_t padding so sizeof is
# unchanged) forces a full resolve before any skipping. Host-validated: suite
# stays 2313/2314; skip-counter repro = 398 skips / 2 resolves over 200 stable
# frames, 0 skips when the leaf changes every frame.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_INCREMENTAL_UPD)

# opt: reverse-hash incremental COLLECT. rc_memrefs_get_pending_addresses skips
# re-walking INDIRECT chains whose inputs are unchanged (incr_clean flag in the
# rc_modified_memref_t padding -> sizeof unchanged). The adapter builds a reverse
# index (address -> chains) via g_rc_chain_read_cb during walks, then per snapshot
# marks only the chains reading a CHANGED address dirty; a periodic full rebuild
# (every N vblanks / on watchlist mutation) re-walks all and resets the index.
# Attacks the 27ms collect_missing spike that drops df/s to 30-52 in heavy scenes.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_INCREMENTAL_COLLECT)
