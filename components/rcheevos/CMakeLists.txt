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
# -O2 -DNDEBUG TESTED 2026-06-29 -> NO benefit, reverted. Controlled same-scene compare
# (bunny convergence, per-trigger evl/de_ev): -Os 89 us/trig vs -O2 -DNDEBUG 92 us/trig =
# equal within noise; df_us p50 18336 vs 17858 = unchanged. The do_frame is PSRAM-memory-
# latency-bound, NOT compute-bound (same reason eval_plan + shadow-array gave 0) -> faster
# code / dropped asserts don't reduce the PSRAM wait. The earlier "evl -50%" read was a
# SCENE artifact (calm-heavy complete log vs convergence-only bunny log). The lever that
# DID work (step 2, RC_COMPACT_OPERAND) cut BYTES/cache-lines, not compute. Keeping -Os +
# asserts (the known-good baseline). To A/B -O2 rigorously: same scene, per-trigger metric.
target_compile_options(${COMPONENT_LIB} PRIVATE -w -fpermissive)

# Dirty/incremental eval: skip cold+clean triggers in rc_client's do_frame eval
# loop (the path to 60fps). PUBLIC so it propagates to `main` (REQUIRES rcheevos)
# — rc_client.c needs it for the struct fields + skip logic, main.cpp for the
# extern counters/toggle in the FRAME log. Runtime A/B via g_rc_dirty_eval_enabled.
# Validated host-side: upstream test suite stays 2313/2314 with the toggle ON.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_DIRTY_EVAL)

# Clean-replay: extends dirty-eval to WARM+CLEAN triggers. Today dirty-eval must
# fully re-evaluate any trigger with hit counts (has_hits); clean-replay instead
# replays it with the CACHED condition truth (rc_test_condition returns is_true
# without re-reading operands) since on a proven-clean frame the raw truth is
# unchanged — the full accrual/pause/measured/state machinery still runs, so the
# result is behavior-identical while skipping the scattered memref->value PSRAM
# reads that dominate the eval (PSRAM-bandwidth-bound; +43% at 80MHz). The replay
# request rides rc_evaluate_trigger's unused_L param as &g_rc_replay_sentinel
# (per-call -> race-free across the 2 work-stealing eval cores). Runtime A/B via
# g_rc_clean_replay_enabled. Host-validated 2313/2314 incl. stress (replay every
# clean trigger = 5983 replays, bit-identical). REQUIRES RC_DIRTY_EVAL.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_CLEAN_REPLAY)

# RC_SHADOW_VALUES: dead experiment (2026-06-29, 0 HW gain — read latency hidden by
# compute). Guarded code kept in-tree for reference. See project_shadow_array.
# target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_SHADOW_VALUES)

# Compact operand (step 2 of the eval compaction): FP-const storage double->float, shrinking
# rc_operand_t 16->8B => rc_condition_t 48->32B (-33% eval streaming) AND rc_modified_memref_t
# 56->36B (-36% upd streaming). EVAL-SAFE (host: only 5 FP parse-storage asserts diverge, eval
# already uses f32; all eval-logic tests pass). The low-risk first cut at the memory-bound eval
# the bloat probe confirmed. Comment out to A/B the gain. See project_compiled_eval.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_COMPACT_OPERAND)

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

# Pointer-aware dirtying: the reverse-index registers ONLY pointer reads (intermediate
# chain levels, recursion depth>0), NOT the top-level leaf read. A chain's leaf address
# only moves when a pointer above it changes; the leaf VALUE changing (most snap_changed
# churn) does NOT need a re-walk. Registering the leaf made chains falsely-dirty -> the
# collect over-walked ~9x (measured: 9 walks per real miss). Skipping it cuts the walk
# on miss-frames (the 10% of frames costing ~40ms ap_us). SAFE: a wrongly-skipped dirty
# is backstopped by the do_frame gate + peek_miss net (deferred frame, never false unlock).
# Device-only (incr-collect not in upstream/host). Comment to A/B. See project_convergence_analysis.
target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_POINTER_AWARE_DIRTY)

# RC_EVAL_PLAN (eval hot/cold split): validated but 0 HW gain — closed 2026-06-30.
# Guarded code kept in-tree. See project_compiled_eval.
#target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_EVAL_PLAN)

# RC_PARALLEL_UPD: net negative on HW (2026-06-29, upd2 is dependency-bound; barrier
# cost > gain). Guarded code kept in-tree. See project_optimization_backlog.
#target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_PARALLEL_UPD)
