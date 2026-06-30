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

# Shadow-array (RC_SHADOW_VALUES): DISABLED 2026-06-29 — HW gave 0 perf gain. The
# memref-value read latency is hidden by compute overlap in the real eval (MEMBENCH's
# 13.7x was isolated-read latency, not the real critical path). Code kept (#ifdef'd,
# compiles out) for reference; the real eval bottleneck is the rc_condition_t struct
# streaming, attacked by the plan/state split instead. See project_shadow_array.
# target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_SHADOW_VALUES)

# DIAGNOSTIC: uncomment to ~2x the rc_condition_t size (inert padding) and double the
# eval's condition-struct streaming. Build once WITH and once WITHOUT, compare evl_us in
# the CATCHUP log: evl rises -> eval is memory-bound on condition streaming (the plan/
# state compaction will pay); evl ~flat -> CPU-bound (compaction won't help, accept 80M).
# Decisive memory-vs-CPU probe before the high-risk eval rewrite. See project_compiled_eval.
# RESULT 2026-06-29: BLOAT added +5.4ms/+35% to evl_us (upd unchanged = clean control) ->
# eval IS memory-bound on condition streaming (~35% of evl). Plan/state compaction justified.
# target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_COND_BLOAT)

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

# Eval HOT/COLD split (eval-read rewrite, steps A-E, project_compiled_eval): a compact
# 16B-per-condition rc_eval_hot_t array (op1/op2/current_hits + type/oper/cmp/flags),
# built once per condset, that rc_test_condition + the condset evaluators stream in the
# common (optimized_comparator) path instead of the 32B rc_condition_t. The COLD fields
# (operand metadata, required_hits value, next, transformed/non-optimized operands) stay
# in rc_condition_t for the rare paths. Host-validated 2308/2314 (-DRC_EVAL_PLAN
# -DRC_COMPACT_OPERAND -DRC_EVAL_PLAN_HOST_SYNC; only the 5 FP-storage + user_agent fails).
# Do NOT define RC_EVAL_PLAN_HOST_SYNC on device: that per-frame hot->cold mirror is host-
# only (it re-touches the 32B cold struct, negating the win) and exists so the host tests /
# serialization can read the cold struct. On device current_hits/is_true live canonically in
# hot; rc_reset_condset resets hot too. Hot arrays go to PSRAM iff main sets g_rc_eval_hot_alloc
# (else libc malloc; a NULL/failed alloc falls back to the cold path, still correct).
# Comment out to A/B the eval-compaction gain. Expected: evl ~-50% of its streaming portion.
#target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_EVAL_PLAN)

# Parallel upd Phase 2 (v0.33): the do_frame memref UPDATE splits into upd1 (leaf peeks,
# ~free) and upd2 (modified_memref pointer-chain resolves = ~100% of upd, ~5.3ms). This
# splits upd2 across both cores BY DEPENDENCY DEPTH: chains are bucketed by their `depth`
# field (0 = parent is a leaf resolved in the free Phase 1; N = parent is a depth-(N-1)
# chain), each depth is work-stolen across the 2 cores (reusing the v0.29 eval worker),
# with a barrier between depths so a chain's parent is always resolved first. SAFE: each
# chain writes only its own value (disjoint), the B2 leaf-unchanged hook is read-only, and
# g_rc_chain_read_cb does NOT fire during upd (g_rc_incr_chain==0 outside collect). Runtime
# A/B via g_par_upd_enabled; uda/udb in the FRAME log show the 2-core balance. Falls back to
# the serial list-order resolve if buckets can't build (alloc fail / depth>=16). Comment to
# A/B. Expected: upd2 ~5.3->~2.7ms -> df_us p50 ~17.9->~15.3ms -> steady 60. RC_PARALLEL_EVAL
# must be on (it is by default). See project_optimization_backlog / the upd-parallel work.
# HW RESULT 2026-06-29 (wii-coelho2.log vs wii-coelho.log): NET NEGATIVE, reverted. Only
# ~15% of upd2 is independent (Pass A); 84% are dependency-bound deep pointer chains (Pass
# B, serial). Pass A (~1.7ms) is too small to amortize the Core-0 worker-wake latency (Core
# 0 is busy with WiFi/lwIP), so the per-frame barrier made df/s p50 50->48 and upd2 didn't
# drop. Code + uda/udb instrumentation kept for reference; flag OFF.
#target_compile_definitions(${COMPONENT_LIB} PUBLIC RC_PARALLEL_UPD)
