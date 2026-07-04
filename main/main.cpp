#include <Arduino.h>
#include <stdlib.h>   /* qsort (chain-root gate) */
#line 1 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
/**
 * wii-ra-adapter.ino
 *
 * Wii RetroAchievements Adapter - ESP32-S3 Firmware
 * Based on gc-ra-adapter by odelot
 *
 * Architecture differences from gc-ra-adapter:
 *   - Wii 6-byte game IDs (e.g. "RSBE01") instead of GameCube IDs
 *   - AdapterState values aligned with ra_status_t so WiiFlow can
 *     poll the status field and know when the game is loaded.
 *   - WiiFlow (PPC) sends LOAD_GAME via EXI before IOS reload;
 *     ra-module (ARM/Starlet) takes over EXI after the game boots.
 *   - No GC-specific install marker handling (0xD0 / 0x9A / 0xFEED).
 *   - STATE_GAME_LOADED: intermediate state between LOADING_GAME and
 *     ACTIVE so WiiFlow can detect "ready to boot" without waiting for
 *     the first SNAPSHOT.
 *
 * Core layout (v0.24.0 — requires Arduino IDE: "Arduino Runs On" = Core 1;
 * recommend "Events Run On" = Core 0):
 *   Core 1 (realtime): loop() -> processSnapshot()/rc_client_do_frame()
 *                      + taskCore1() -> processEXI() (SPI slave)
 *   Core 0 (network):  WiFi/lwIP tasks + httpTask() — ALL blocking HTTP/
 *                      TLS/JSON-strip work. server_call only enqueues;
 *                      rc_client callbacks run on loop() via
 *                      http_drain_done(). Core 1 never waits on the net.
 * 
 * 
 * 

Pin 1: GND
Pin 2: INT   (card detect / interrupt — não usado no projeto)
Pin 3: CS    (Chip Select, active low)
Pin 4: CLK   (serial clock)
Pin 5: DI    (MOSI — Wii escreve, ESP32 lê)
Pin 6: DO    (MISO — ESP32 escreve, Wii lê)
Pin 7: 3.3V

Diagrama de ligação  (pinos exatos vêm de board_config.h / BOARD_VARIANT)

Wii Memory Card Slot A    BOARD_DEV (S3 DevKit)   BOARD_XIAO (XIAO ESP32S3)
────────────────────────  ─────────────────────   ─────────────────────────
Pin 7  3.3V ───────────── 3.3V  (alimentação)      3.3V
Pin 1  GND  ───────────── GND                      GND
Pin 3  CS   ───────────── GPIO10 (EXI_PIN_CS)      GPIO7  (pad D8)
Pin 4  CLK  ───────────── GPIO12 (EXI_PIN_CLK)     GPIO8  (pad D9)
Pin 5  DI   ───────────── GPIO11 (EXI_PIN_MOSI)    GPIO9  (pad D10)
Pin 6  DO   ───────────── GPIO13 (EXI_PIN_MISO)    GPIO5  (pad D4)
Pin 2  INT  ───────────── GPIO14 (EXI_PIN_INT)     GPIO6  (pad D5)

Unlock LED:               WS2812 RGB GPIO48        yellow user LED GPIO21 (active-low)


 */

#include <WiFiManager.h>
#include <EEPROM.h>
#include <StreamString.h>
#include "rc_client.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "SPI.h"
#include <time.h>

// Project headers
#include "board_config.h"   // BOARD_VARIANT switch: EXI pin map + LED back-end
#include "gc_ra_protocol.h"
#include "exi_spi_slave.h"
#include "ra_http.h"
#include "ra_web.h"
#include "wii_game_hashes.h"
#include "driver/gpio.h"
#include "PsramStream.h"

// rcheevos internals — needed for rc_memrefs_get_addresses()
extern "C" {
  #include "rc_internal.h"
  const rc_memrefs_t* rc_client_get_memrefs(const rc_client_t* client);
  /* do_frame profiling split (set in rc_client.c rc_client_do_frame):
   * upd = rc_client_update_memref_values (resolve pointer chains),
   * evl = condition evaluation (achievement loop). */
  extern volatile uint32_t g_rc_update_us;
  extern volatile uint32_t g_rc_eval_us;
  /* v0.33 upd phase split: upd1=leaf peeks (free), upd2=chain resolves (the cost). */
  extern volatile uint32_t g_rc_upd1_us;
  extern volatile uint32_t g_rc_upd2_us;
  /* STEP 0 (2026-06-30): resolve-only time within upd2 (the ~378 rc_get_modified_memref_value
   * calls). skip-check time = upd2 - urs → splits the 6ms between B2 skip-checks and resolves. */
  extern volatile uint32_t g_rc_upd_resolve_us;
  /* v0.33 parallel upd Phase-2: uda=core-1 time, udb=core-0 worker time (0 = serial). */
  extern volatile uint32_t g_upd_a_us;
  extern volatile uint32_t g_upd_b_us;
  /* v0.29.1 — parallel-eval diag: a=core-1 half, b=core-0 worker half, runs=worker fires. */
  extern volatile uint32_t g_par_a_us;
  extern volatile uint32_t g_par_b_us;
  extern volatile uint32_t g_par_runs;
  /* v0.32 cache-primed do_frame gate (resolver/evaluator divergence cure).
   * rc_client_do_frame runs the memref UPDATE, then calls g_rc_doframe_primed_cb;
   * if it returns 0 it rolls the values back (into save_buf) and sets
   * g_rc_doframe_deferred=1 WITHOUT evaluating any condition. */
  uint32_t rc_client_memref_count(const rc_client_t* client);
  uint32_t rc_client_modified_memref_count(const rc_client_t* client);   /* U1 sizing */
  extern int (*g_rc_doframe_primed_cb)(void);
  extern rc_memref_value_t* g_rc_doframe_save_buf;
  extern uint32_t g_rc_doframe_save_cap;
  extern volatile int g_rc_doframe_gate_enabled;
  extern volatile int g_rc_doframe_deferred;
#ifdef RC_DIRTY_EVAL
  /* dirty/incremental eval: cold+clean trigger skipping. enabled=runtime A/B;
   * skipped/evaled = per-frame tallies (we zero them before each do_frame). */
  extern volatile int g_rc_dirty_eval_enabled;
  extern volatile uint32_t g_rc_de_skipped;
  extern volatile uint32_t g_rc_de_evaled;
  /* lb dirty-eval + RP throttle (2026-07-03, the hardcore +3ms fix) */
  extern volatile int g_rc_lb_dirty_enabled;
  extern volatile uint32_t g_rc_lb_skipped;
  extern volatile uint32_t g_rc_lb_evaled;
  extern volatile int g_rc_rp_throttle_enabled;
  extern volatile uint32_t g_rc_lb_stat_active_ok, g_rc_lb_stat_total;  /* boot diag */
#ifdef RC_CLEAN_REPLAY
  extern volatile uint32_t g_rc_de_replayed;   /* warm+clean cached-truth replays (subset of de_ev) */
  extern volatile int g_rc_clean_replay_enabled;
#endif
#endif
}

#define DEBUG 1

/* ----------------------------------------------------------------------
 * Log levels (v0.24.1). Serial is SHARED by all tasks on both cores and
 * the UART drains at 250000 baud ≈ 25 KB/s; when the TX ring buffer
 * fills, Serial.printf BLOCKS the calling task — including the EXI task
 * and loop() on Core 1. The award/load windows used to burst hundreds of
 * chars (URLs, HTTP bodies, rcheevos verbose) and that burst
 * showed up as dropped SNAP fires ("Serial pressure breaks SPI timing").
 *
 *   0 = errors only (+ banner + ACHIEVEMENT, always-on)
 *   1 = info: FRAME/CATCHUP telemetry + lifecycle milestones (game loaded,
 *       login, state transitions, ready) — the default working view
 *   2 = debug: HTTP traces, rcheevos verbose, cap_ops, full FRAME line,
 *       hotbuf, FORCE-collect, WiFi steps, EXI init dumps, SNAP_HDR
 *
 * Raise to 2 only while investigating. Combined with the enlarged UART
 * TX buffer in setup(), level 1 keeps hot windows burst-free. */
#ifndef RA_LOG_LEVEL
#define RA_LOG_LEVEL 2
#endif
#include <stdarg.h>
/* ============================================================================
 * Async log ring (2026-06-19) — the definitive "logging NEVER blocks the EXI
 * core" fix. Any task/core formats into a stack buffer then memcpys it into a
 * ring in INTERNAL SRAM (not PSRAM — keeps the 64KB PSRAM data cache for the hot
 * rcheevos/addr_hash data) under a brief spinlock (~1-2µs, no Serial). A drain
 * task on the idle Core 0 copies chunks OUT (lock held only for the copy, NEVER
 * during Serial.write) and blocks on the UART — so Core 1 never waits on Serial.
 * Ring full → drop the whole line (graceful, never block). Lets us instrument
 * fearlessly. Boot logs (before the drain task) fall back to direct Serial. */
#define LOG_RING_SIZE 8192u   /* +8KB .bss; tune up if drops occur */
static char              log_ring[LOG_RING_SIZE];   /* internal .bss SRAM */
static volatile uint32_t log_head = 0, log_tail = 0;
static portMUX_TYPE      log_mux  = portMUX_INITIALIZER_UNLOCKED;
static volatile bool     g_log_async_ready = false; /* false until the drain task runs */
static volatile uint32_t log_dropped = 0;           /* diag: bytes dropped on overflow */

static inline void ralog_write(const char *buf, uint32_t len) {
    portENTER_CRITICAL(&log_mux);
    uint32_t head = log_head;
    uint32_t used = (head - log_tail + LOG_RING_SIZE) % LOG_RING_SIZE;
    if (len > LOG_RING_SIZE - 1u - used) {           /* won't fit → drop the whole line */
        log_dropped += len;
        portEXIT_CRITICAL(&log_mux);
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        log_ring[head] = buf[i];
        head = (head + 1u == LOG_RING_SIZE) ? 0u : head + 1u;
    }
    log_head = head;
    portEXIT_CRITICAL(&log_mux);
}

static void ralog_vprintf(const char *fmt, va_list ap) {
    char buf[384];   /* FRAME line is ~300ch + margin (vsnprintf truncates) */
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n <= 0) return;
    uint32_t len = (n < (int)sizeof(buf)) ? (uint32_t)n : (uint32_t)(sizeof(buf) - 1);
    if (g_log_async_ready) ralog_write(buf, len);
    else Serial.write((const uint8_t *)buf, len);    /* boot: no EXI yet, blocking ok */
}

/* non-static: also called from exi_spi_slave.cpp via ra_log.h */
void ralog_printf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    ralog_vprintf(fmt, ap);
    va_end(ap);
}

/* Drains the ring to Serial on Core 0. Lock held ONLY for the chunk copy, never
 * during Serial.write — so a Core-1 producer spins at most a few µs. Blocks on
 * the UART (yields IDLE0) when draining, and vTaskDelay when empty (TWDT-safe). */
static void ralog_drain_task(void *pv) {
    (void)pv;
    static char chunk[512];
    uint32_t last_dropped = 0;
    g_log_async_ready = true;
    for (;;) {
        portENTER_CRITICAL(&log_mux);
        uint32_t tail = log_tail;
        uint32_t used = (log_head - tail + LOG_RING_SIZE) % LOG_RING_SIZE;
        uint32_t n = used < sizeof(chunk) ? used : sizeof(chunk);
        for (uint32_t i = 0; i < n; i++) {
            chunk[i] = log_ring[tail];
            tail = (tail + 1u == LOG_RING_SIZE) ? 0u : tail + 1u;
        }
        log_tail = tail;
        portEXIT_CRITICAL(&log_mux);
        if (n) { Serial.write((const uint8_t *)chunk, n); continue; }  /* drained a chunk */
        /* ring empty: if we ever dropped, say so (so silent loss is visible), then yield */
        if (log_dropped != last_dropped) {
            char w[56];
            int wn = snprintf(w, sizeof(w), "LOG_DROPPED total=%lu bytes\r\n",
                              (unsigned long)log_dropped);
            last_dropped = log_dropped;
            if (wn > 0) Serial.write((const uint8_t *)w, (size_t)wn);
        }
        vTaskDelay(1);   /* empty → yield IDLE0 (TWDT-safe) */
    }
}

#define LOG_ERR(...)  ralog_printf(__VA_ARGS__)
#define LOG_INFO(...) do { if (RA_LOG_LEVEL >= 1) ralog_printf(__VA_ARGS__); } while (0)
#define LOG_DBG(...)  do { if (RA_LOG_LEVEL >= 2) ralog_printf(__VA_ARGS__); } while (0)

/* RA_WDOG_VERBOSE: gates the Core-0 deadlock watchdog's serial output. The task
 * always RUNS (tracking srv/wrk/df heartbeat deltas) — this only controls whether
 * it PRINTS. Default OFF because both of its lines are noisy in normal operation:
 *   - "DEBUG=WDOG ..." is an unconditional 1-line/s heartbeat.
 *   - "DEBUG=WDOG-FREEZE ..." fires whenever the servicer didn't tick for a second,
 *     which also happens when the GameCube is simply IDLE (not streaming snapshots),
 *     so it spams with all-zero rings even when nothing is wrong.
 * Flip to 1 at compile time, or set g_wdog_verbose=true at runtime, ONLY when
 * chasing a real freeze (project_exi_robust_handshake / project_spike_root_cause). */
#ifndef RA_WDOG_VERBOSE
#define RA_WDOG_VERBOSE 0
#endif
static volatile bool g_wdog_verbose = RA_WDOG_VERBOSE;

/* LOG_FRAME: per-vblank summary line (one per processed frame). This is the
 * INFO channel — FRAME (and the periodic CATCHUP line) are emitted at
 * RA_LOG_LEVEL >= 1. RA_LOG_FRAME_STATS remains a compile-time master switch
 * to remove the telemetry entirely. "Minimum-noise" mode: RA_LOG_LEVEL 0.
 *
 * Fields printed:
 *   seq    - frame sequence number (PPC VBI counter, sent by d2x)
 *   ok     - Phase D2 integrity: seq+count matched, values accepted (1=ok, 0=skip)
 *   sa     - addr_count from the first SNAPSHOT message (d2x → ESP32)
 *   ms     - all new addresses queued this vblank across every collect_missing_
 *            addresses() call (addAddress resolver + peek_miss safety net).
 *   cm     - cache misses while RESOLVING addAddress chains this vblank: bytes
 *            the resolver could not find in the hash and queued for ADDR_QUERY.
 *            Invariant: next frame's sa == this frame's sa + Σcm (proves the
 *            whole frame resolved in-frame). sa_delta > Σcm means the peek_miss
 *            net caught a chain shape the resolver missed.
 *   it     - addAddress resolution depth (number of ADDR_QUERY round-trips)
 *   mut    - 1 if a watchlist mutation (APPEND or REMOVE_IDX) was minted
 *   cln    - 1 if evict_lru fired this vblank (LRU cleanup)
 *   ap_us  - address processing time µs (SNAPSHOT receipt → new_snapshot=true)
 *   df     - 1 if rc_client_do_frame ran this frame
 *   df_us  - rc_client_do_frame total duration µs
 */
#ifndef RA_LOG_FRAME_STATS
#define RA_LOG_FRAME_STATS 1
#endif
#define LOG_FRAME(...) do { if (RA_LOG_FRAME_STATS && RA_LOG_LEVEL >= 1) ralog_printf(__VA_ARGS__); } while (0)

/* ============================================================================
 * Flight recorder (2026-06-19). The per-frame FRAME line is the bulk of the
 * serial traffic (~60/s × ~300ch ≈ 18KB/s on a 25KB/s UART). Printing it every
 * frame on Core 1 fills the 8KB TX buffer in bursts (catch-up, convergence) →
 * Serial.printf blocks the low-prio loop → steals do_frame CPU and (per the
 * do_frame-starvation analysis) perturbs the EXI critical path. Instead: format
 * each FRAME line into a ring WITHOUT printing, and dump the ring (FR_BEFORE-1
 * before + the bad frame) plus FR_AFTER after-frames ONLY around a bad frame
 * (df_us/ap_us over budget). Good frames never hit Serial → logging cannot impact
 * normal operation, yet every spike comes with full before/after context.
 * Toggle RA_FLIGHT_RECORDER 0 to revert to always-print. ~3KB .bss. */
#ifndef RA_FLIGHT_RECORDER
#define RA_FLIGHT_RECORDER 1
#endif
#if RA_FLIGHT_RECORDER
#include <stdarg.h>
#define FR_BEFORE    8         /* frames of before-context kept in the ring */
#define FR_AFTER     4         /* frames of after-context printed past a spike */
#define FR_LINE_MAX  384       /* a FRAME line is ~300ch + margin (vsnprintf truncates) */
#define FR_BAD_DF_US 20000u    /* do_frame over ~budget (16.6ms) */
#define FR_BAD_AP_US 30000u    /* convergence/EXI spike (matches the d2x SPK >30ms) */
/* Mode toggle: logging is async (ralog ring, never blocks Core 1). DEFAULT here is
 * flight-recorder mode = only the worst frames + before/after context. Set
 * RA_LOG_ALL_FRAMES 1 (or flip g_log_all_frames at runtime) to log EVERY frame. */
#ifndef RA_LOG_ALL_FRAMES
#define RA_LOG_ALL_FRAMES 0
#endif
static volatile bool  g_log_all_frames = RA_LOG_ALL_FRAMES;
static char          fr_ring[FR_BEFORE][FR_LINE_MAX];
static uint8_t        fr_head   = 0;   /* next slot to write = oldest in the ring */
static uint8_t        fr_after  = 0;   /* remaining after-context frames to print */
static unsigned long  fr_spikes = 0;   /* diag: total spikes dumped */

static void fr_record(const char *fmt, ...) {
    if (!RA_LOG_FRAME_STATS || RA_LOG_LEVEL < 1) return;   /* FRAME = INFO channel */
    va_list ap; va_start(ap, fmt);
    if (g_log_all_frames) {                /* log-all mode: emit every frame (async, free) */
        ralog_vprintf(fmt, ap);
        va_end(ap);
        return;
    }
    char *slot = fr_ring[fr_head];         /* flight-recorder mode: ring + dump on spike */
    vsnprintf(slot, FR_LINE_MAX, fmt, ap);
    va_end(ap);
    fr_head = (uint8_t)((fr_head + 1) % FR_BEFORE);
    if (fr_after) { ralog_write(slot, strlen(slot)); fr_after--; }   /* after-window → emit live */
}

/* Call right after recording a frame that exceeded budget. Dumps the ring
 * (before-context + the bad frame) once, then arms the after-window. A second
 * bad frame while still draining just extends the window (no double dump). */
static void fr_trigger(void) {
    if (g_log_all_frames) return;   /* every frame already logged → nothing to dump */
    if (fr_after) { fr_after = FR_AFTER; return; }
    fr_spikes++;
    ralog_printf("---- SPIKE #%lu (flight recorder: %u before + this + %u after) ----\r\n",
               fr_spikes, (unsigned)(FR_BEFORE - 1), (unsigned)FR_AFTER);
    for (uint8_t i = 0; i < FR_BEFORE; i++) {
        char *slot = fr_ring[(fr_head + i) % FR_BEFORE];   /* oldest → newest (the bad frame) */
        if (slot[0]) ralog_write(slot, strlen(slot));
    }
    fr_after = FR_AFTER;
}
#define LOG_FRAME_REC(...) fr_record(__VA_ARGS__)
#else
#define LOG_FRAME_REC(...) LOG_FRAME(__VA_ARGS__)
#endif

/* Execution path for rc_client_do_frame / processSnapshot().
 *
 *   0 (default) — inline in taskCore1 immediately after the vblank snapshot
 *                 is processed. Minimum latency but blocks EXI for the
 *                 duration of do_frame (~26ms on SMG). Acceptable when
 *                 do_frame fits the budget (~16.6ms); a missed SPI window
 *                 only delays by 1 frame.
 *
 *   1           — in the low-priority loop() task (original behaviour).
 *                 EXI never blocks at the cost of ~1ms extra scheduling
 *                 latency. Use when do_frame routinely overruns the budget. */
#ifndef RC_DO_FRAME_IN_LOW_PRIO_TASK
/* 1 since 2026-06-14 — this WAS the nº1 bottleneck. The do_frame is ~26ms,
 * OVERRUNNING the 16.6ms budget, so mode 0's "Acceptable when do_frame fits"
 * premise was violated: the 26ms do_frame on the EXI critical path (prio-19)
 * starved the SPI/handshake service, so the ra-module fired only ~12×/s
 * (losing 4 of every 5 VBI edges) and the ESP ran ~5.4 do_frames/s.
 * MEASURED A/B (wii.log mode-0 vs wii2.log mode-1): cyc_us median 143→28ms
 * (5x), ra-module fire 12→59/s (locked to the 60Hz game), 95.5% of frames
 * <40ms. Cost: do_frame wall-clock inflates (preempted by the EXI task —
 * harmless, CPU-time unchanged) and ~2% of frames spike >80ms (low-prio
 * task occasionally starved). With cycle now ≈ do_frame, the do_frame is
 * finally the thing worth parallelizing. Do NOT flip back to 0. */
#define RC_DO_FRAME_IN_LOW_PRIO_TASK 1
#endif

/* When 0, the patch response is passed to rcheevos as-is (no field stripping).
 * Useful for isolating whether a parse failure is caused by the strip helpers
 * mangling JSON, vs. by something else in the response. The full SMG patch is
 * ~800KB and fits easily in 8MB PSRAM, so disabling strips is safe for triage. */
#define STRIP_JSON_RESPONSE 0

#define EEPROM_SIZE 2048
#define EEPROM_ID_1 142
#define EEPROM_ID_2 210  // Bumped from GC adapter (209) to force re-init on first Wii flash

#define RESET_PIN 8
#define ENABLE_RESET 0
#define RESET_PRESSED_TIME 5000L

// ============================================================================
// State machine — values MUST match ra_status_t in gc_ra_protocol.h so
// WiiFlow can read resp.status and know exactly what the adapter is doing.
// ============================================================================
enum AdapterState {
    STATE_INIT         = RA_STATUS_INITIALIZING,   // 0x00 — booting up
    STATE_WAIT_GAME_ID = RA_STATUS_LOGGED_IN,      // 0x04 — WiFi+login OK, ready for game ID
    STATE_LOADING_GAME = RA_STATUS_LOADING_GAME,   // 0x05 — fetching game data from RA servers
    STATE_GAME_LOADED  = RA_STATUS_GAME_LOADED,    // 0x06 — data ready, WiiFlow may proceed to boot
    STATE_ACTIVE       = RA_STATUS_ACTIVE,          // 0x07 — in-game, processing snapshots
    STATE_ERROR        = RA_STATUS_ERROR_GAME,      // 0xE2 — generic error
    STATE_IDLE         = 0x80,                      // internal only, not exposed via EXI
};

volatile AdapterState state = STATE_INIT;

#line 346 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void dump_cache_lru(void);
#line 429 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void tomb_put(uint32_t addr, uint8_t value);
#line 444 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static bool tomb_get(uint32_t addr, uint8_t *out);
#line 455 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void tomb_clear(void);
#line 529 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void mut_record(uint16_t seq, const uint8_t *resp, uint16_t len);
#line 538 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void mut_ring_clear(void);
#line 545 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static bool mut_deliver_next(void);
#line 665 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
 extern "C" int doframe_primed_cb(void);
#line 684 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void trigger_watchlist_resync(const char *reason);
#line 745 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void * ra_hot_alloc(size_t bytes, bool *from_internal);
#line 755 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static uint32_t hash_addr(uint32_t addr);
#line 759 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void hash_clear();
#line 765 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void hash_insert(uint32_t addr, uint16_t watch_index);
#line 779 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static int32_t hash_lookup(uint32_t addr);
#line 794 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static int is_cached_cb(uint32_t addr, uint32_t num_bytes, void* ud);
#line 819 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static const char * cmd_short_name(uint8_t cmd);
#line 915 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void queue_event(uint8_t type, const uint8_t *data, uint16_t len);
#line 929 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
bool has_pending_event();
#line 933 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
PendingEvent* peek_event();
#line 938 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void pop_event();
#line 947 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void log_message(const char *message, const rc_client_t *client);
#line 951 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void event_handler(const rc_client_event_t *event, rc_client_t *client);
#line 1004 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static uint32_t read_memory_ingame(uint32_t address, uint8_t *buffer, uint32_t num_bytes, rc_client_t *client);
#line 1080 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static uint32_t peek_from_snapshot(uint32_t address, uint32_t num_bytes, void *ud);
#line 1160 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static uint32_t read_memory_nop(uint32_t address, uint8_t *buffer, uint32_t num_bytes, rc_client_t *client);
#line 1165 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void rc_callback(int result, const char *error_message, rc_client_t *client, void *userdata);
#line 1169 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
rc_clock_t get_millisecs(const rc_client_t *client);
#line 1182 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static bool json_quote_is_real(const char* data, size_t idx);
#line 1188 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_remove_whitespace(PsramStream &buf);
#line 1202 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_remove_field(PsramStream &buf, const char* field);
#line 1266 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_clean_field_str(PsramStream &buf, const char* field);
#line 1302 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_clean_field_array(PsramStream &buf, const char* field);
#line 1366 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_keep_only_achievement_id(PsramStream &buf, const uint32_t *target_ids, int n_ids);
#line 1454 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_remove_flags5_achievements(PsramStream &buf);
#line 1502 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_keep_first_n_achievements(PsramStream &buf, int n);
#line 1547 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_keep_first_set(PsramStream &buf);
#line 1627 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static int count_mem_ops(const char* data, int objStart, int objEnd);
#line 1647 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_cap_total_operations(PsramStream &buf, int target_ops);
#line 1719 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void json_strip_patch(PsramStream &buf);
#line 1769 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void http_finish(const http_job_t *job, const char *body, size_t body_len, int status);
#line 1790 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void server_call_blocking(const http_job_t *job);
#line 2046 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void server_call(const rc_api_request_t *request, rc_client_server_callback_t callback, void *callback_data, rc_client_t *rc_client);
#line 2066 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void httpTask(void *pvParameters);
#line 2080 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void http_drain_done(void);
#line 2134 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void send_esp_header_response_4pad(const ra_esp_header_t *hdr);
#line 2174 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void set_new_snapshot(void);
#line 2203 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static uint16_t collect_missing_addresses(void);
#line 2306 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void send_addr_query_response(void);
#line 2376 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void handle_exi_command(const uint8_t *rx_data, size_t rx_len);
#line 3112 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static uint16_t watchlist_append(const uint32_t *addrs, const uint8_t *values, uint16_t count);
#line 3161 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void ensure_watchlist_capacity();
#line 3181 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void watchlist_update_if_changed(uint32_t new_count);
#line 3423 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void on_game_loaded(int result, const char *error_message, rc_client_t *client, void *userdata);
#ifdef RC_SHADOW_VALUES
extern "C" void rc_shadow_set_arena(uint32_t* value, uint32_t* prior, uint8_t* changed, uint32_t cap);  /* memref.c (C linkage) */
#endif
#line 3507 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
static void login_then_load_cb(int result, const char *error_message, rc_client_t *client, void *userdata);
#line 3519 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void loadGame(const char *hash);
#line 3558 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void beginEEPROM(bool force);
#line 3568 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
bool isConfigured();
#line 3570 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void save_configuration_info_eeprom(String ra_user, String ra_pass);
#line 3580 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
String read_ra_user_from_eeprom();
#line 3587 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
String read_ra_pass_from_eeprom();
#line 3603 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
String try_login_RA(String user, String pass);
#line 3790 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void processSnapshot();
#line 4047 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void processEXI();
#line 4095 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void taskCore1(void *pvParameters);
#line 4181 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void setup();
#line 4326 "C:\\dev\\gamecube\\gamecube\\wii-ra-adapter\\wii-ra-adapter\\wii-ra-adapter.ino"
void loop();
// ============================================================================
// Memory tracking - adapted for 32-bit addresses and snapshot model
// ============================================================================
uint32_t *watch_addresses = NULL;     // 32-bit addresses (was uint16_t in fpga-ra-adapter)
uint16_t watch_count = 0;
uint8_t *memory_data = NULL;          // Current values for each watched address
volatile bool new_snapshot = false;    // Flag: new snapshot received from GC
volatile uint32_t frame_counter = 0;
// Set by the RA_CMD_RESET_CREDENTIALS handler; loop() wipes WiFi + RA creds and reboots.
volatile bool g_factory_reset_pending = false;

/* opt B2 (incremental upd, INDIRECT skip — see project_incremental_upd_b).
 * snap_changed[i] = 1 iff memory_data[i] differed from the previous snapshot
 * (computed at snapshot apply, BEFORE the memcpy). The rcheevos hook
 * rc_leaf_unchanged_impl() reads it to skip re-resolving an INDIRECT_READ leaf
 * whose pointer is stable and whose bytes didn't move. Gated OFF for any
 * do_frame where watch_count changed since the last (append/evict/defrag remap
 * the index->address mapping; count-stability within a session == membership
 * stability), so a freshly-cached leaf always resolves at least once. */
uint8_t *snap_changed = NULL;
static volatile bool g_b2_enabled = true;          // master A/B toggle
static uint16_t g_last_doframe_watch_count = 0xFFFF;
extern "C" {
    extern int (*g_rc_leaf_unchanged)(uint32_t address, uint32_t num_bytes);
    extern volatile int g_rc_indirect_skip_enabled;
    extern unsigned long g_rc_b1_skips, g_rc_b2_skips, g_rc_upd_resolves;
    /* U1 leaf-widx cache hooks/state (defined in memref.c; the included rc_internal.h
     * is the flattened "." copy without these decls, so re-declare locally — the same
     * pattern as g_rc_leaf_unchanged above). */
    extern int  (*g_rc_u1_cached)(uint32_t idx);
    extern int  (*g_rc_u1_populate)(uint32_t idx, uint32_t address, uint32_t num_bytes);
    extern void (*g_rc_u1_invalidate)(uint32_t idx);
    /* opt: reverse-hash incremental collect (see incr_* below + memref.c). */
    extern void (*g_rc_chain_read_cb)(uint32_t address, uint8_t num_bytes, void* chain);
    extern volatile int g_rc_incr_collect_enabled;
    extern volatile unsigned long g_rc_collect_skips, g_rc_collect_walks;
    extern volatile int g_rc_collect_walk_budget;   /* lever B: per-vblank chain-walk cap (memref.c) */
    void rc_modified_memref_mark_dirty(void* chain);
    void rc_modified_memrefs_mark_all_dirty(const rc_memrefs_t* memrefs);
#ifdef RC_EVAL_PLAN
    /* eval HOT/COLD split (RC_EVAL_PLAN): point the hot-array alloc at PSRAM so the
     * compact 16B-per-condition arrays don't consume internal RAM (project_compiled_eval).
     * Guarded: the symbol only exists in condset.c under RC_EVAL_PLAN, so an OFF build
     * (flag commented for A/B) must not reference it or the link fails. */
    extern void* (*g_rc_eval_hot_alloc)(size_t);
#endif
}

#ifdef RC_EVAL_PLAN
/* PSRAM allocator for the rcheevos eval-hot arrays (g_rc_eval_hot_alloc hook). A NULL
 * return is handled gracefully by rc_build_eval_hot (falls back to the cold rc_condition_t). */
extern "C" void* rc_eval_hot_psram_alloc(size_t n) {
    return heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
}
#endif
/* Deadlock locator (project_exi_robust_handshake): a Core-0 watchdog dumps these
 * each second. Whichever STOPS advancing during the galaxy convergence is the
 * task that hung; g_servicer_stage says where in the servicer it stalled. */
extern volatile uint32_t g_servicer_ticks;
extern volatile uint8_t  g_servicer_stage;
extern volatile uint8_t  g_servicer_prev_stage;  // stage armed before the blocking get_trans_result
static volatile uint32_t g_worker_ticks  = 0;   // bumped in processEXI (taskCore1)
static volatile uint32_t g_doframe_ticks = 0;   // bumped in processSnapshot

/* Frame-debt catch-up telemetry (v0.27.2) — see processSnapshot. */
static uint32_t g_doframe_count = 0;       // ACTUAL do_frame calls since last CATCHUP log (was += run; catch-up loop is disabled so exactly 1 runs per processSnapshot)
static uint32_t g_ok0_skips     = 0;       // snapshots rejected by Phase D2 (mutation lag / catch-up) since last CATCHUP log
static uint32_t g_df_prev_gameframe = 0;   // frame_counter at last CATCHUP log
/* Profiling (v0.27.3): accumulated µs + counts, reset each CATCHUP log. */
static uint32_t g_df_us = 0, g_df_n = 0;       // do_frame
static uint32_t g_cm_us = 0, g_cm_n = 0;       // collect_missing_addresses
static uint32_t g_cm_skipped = 0;              // collect calls skipped (steady state)
/* v0.28.7 — wall-clock between consecutive processSnapshot bodies (the real
 * processing cycle). cycle = gap + ap_us + df_us, where gap is the overhead
 * NOT spent in our code: ra-module producing/transferring the next SNAPSHOT
 * + EXI wire + handshake. Steady-state timestamps imply cycle ~134ms with
 * df_us only 26ms — gap (~108ms) is the hidden bottleneck capping us at
 * ~7.4 do_frames/s vs the game's 59. This pins it precisely per-frame. */
static uint32_t g_cycle_us = 0;

/* Per-vblank telemetry for LOG_FRAME stats.
 * Written by the EXI task (Core 1) during SNAPSHOT/ADDR_RESPONSE handling;
 * read by processSnapshot (loop task). Each field is uint8/16/32 and
 * volatile so cross-core writes are not cached by the compiler. */
typedef struct {
    uint8_t  data_ok;                /* Phase D2: seq+count matched, values accepted */
    uint16_t snap_addr_count;        /* addr_count from the first SNAPSHOT of the vblank */
    uint16_t collect_cm;             /* cm= : cache misses inside collect_missing
                                      * (peek_from_snapshot calls that hit no cached
                                      * value) this vblank, BEFORE do_frame. The raw
                                      * count of unresolved peeks driving ADDR_QUERY. */
    uint16_t pre_doframe_miss_count; /* peek_miss_count at convergence (before do_frame) */
    uint16_t collect_miss_total;     /* total new addresses discovered by collect_missing_addresses()
                                      * across all ADDR_QUERY iterations this vblank.
                                      * Each one was a watchlist miss: not in hash, queried via ADDR_QUERY.
                                      * Should equal sa_delta between consecutive mut=1 frames. */
    uint8_t  iter_depth;             /* addr_query_iter_count at convergence */
    uint8_t  collect_round;          /* increments on each collect_missing call this vblank */
    uint16_t cm_by_round[8];         /* resolver (part-1) bytes queued in each round —
                                      * round ≈ addAddress chain level, since each round
                                      * commits its level and the next descends. Lets us
                                      * see WHERE the cascade stalls vs. where peek_miss
                                      * picks up the slack (cmr= in LOG_FRAME). */
    uint8_t  had_mutation;           /* wl_seq changed this vblank (APPEND or REMOVE_IDX) */
    uint8_t  had_cleanup;            /* evict_lru fired this vblank */
    uint32_t addr_proc_us;           /* µs from SNAPSHOT receipt to new_snapshot=true
                                      * (WALL CLOCK — includes EXI round-trip waits) */
    uint32_t cm_cpu_us;              /* µs of CPU actually spent inside collect_missing_
                                      * addresses() this vblank (the resolver walk). The
                                      * rest of ap_us (ap_us - cm_cpu_us) is EXI round-trip
                                      * latency + commit — tells CPU-bound from IO-bound. */
    uint32_t snap_start_us;          /* esp_timer timestamp at SNAPSHOT receipt */
    uint16_t wl_seq_at_snap_start;   /* wl_seq when SNAPSHOT arrived */
} vblank_stats_t;
static volatile vblank_stats_t g_vblank_stats;
/* Frozen converged-frame copy for processSnapshot's LOG_FRAME, captured at
 * set_new_snapshot(). Without it, ok=0 snapshots arriving between convergence
 * and the loop task reading the stats reset data_ok/seq/snap_addr_count and
 * splice a stale ap_us onto a later seq (the phantom 432ms "resync" that
 * wasn't). set_new_snapshot is reached ONLY on a converged frame, so this
 * always reflects a real do_frame (data_ok=1). */
static vblank_stats_t g_frame_log;
static uint32_t       g_frame_log_fc;   /* frame_counter frozen with g_frame_log */

/* Phase A.5 — LRU tracking. last_used_frame[i] stores the value of
 * lru_clock at the last hash-hit for watch_addresses[i]. Static entries
 * (and their byte-fanouts) get UINT32_MAX so they're never evicted. */
static uint32_t *last_used_frame = NULL;
static uint32_t lru_clock = 0;  /* = PPC VBI frame_counter (real game frames) since v0.28.8 */
#define LRU_PROTECTED 0xFFFFFFFFu

/* Phase A.5 — STATIC region [0..static_watch_count-1] holds the rcheevos
 * prefetch result EXPANDED to byte-granularity (every byte of every static
 * u8/u16/u32 base gets its own entry). Never evicted — protected via
 * last_used_frame = LRU_PROTECTED. DYNAMIC tail [static_watch_count..]
 * holds chain[1]/[2] addresses and any masked-chain bytes discovered by
 * multi-pass; subject to LRU eviction when watch_count > WATCHLIST_HIGH_WATER. */
static uint16_t static_watch_count = 0;

/* Diagnostic: dump the whole watchlist with per-entry LRU age. Called every
 * N frames from processSnapshot. Age = lru_clock - last_used_frame[i], now
 * in real PPC VBI game-frames (v0.28.8 — lru_clock mirrors frame_counter;
 * static entries are LRU_PROTECTED and have no age). This is HEAVY: it prints
 * thousands of lines and WILL stall SPI timing for a few seconds — only run
 * it sparingly (the user accepts the disruption for debugging). Yields every
 * 64 lines so the Serial backpressure doesn't trip the task watchdog. */
static void dump_cache_lru(void) {
    if (!watch_addresses || !memory_data) return;

    /* Fine-grained age histogram over DYNAMIC entries. age = lru_clock -
     * last_used_frame[i], in PPC VBI game-frames (~60/s). Buckets:
     *   lo[0]=age0  lo[1]=1-9  lo[2]=10-99      (hot working set lives here)
     *   c[0..8] = 100..999 in decades of 100   (c[0]=100-199 ... c[8]=900-999)
     *   k[0..8] = >=1000 in 250-frame bins      (k[0]=1000-1249 ... k[7]=2750-2999,
     *             k[8]=>=3000)
     * The c[]/k[] tail is COLD dead-weight: the ra-module still reads each of
     * those addresses from MEM1 and ships them in EVERY snapshot, but the
     * evaluator hasn't touched them in 1.6s..50s. This quantifies the transfer
     * we pay for data we no longer look at. */
    uint32_t lo[3] = {0,0,0};
    uint32_t c[9]  = {0,0,0,0,0,0,0,0,0};
    uint32_t k[9]  = {0,0,0,0,0,0,0,0,0};
    for (uint16_t i = static_watch_count; i < watch_count; i++) {
        uint32_t luf = last_used_frame ? last_used_frame[i] : 0;
        if (luf == LRU_PROTECTED) continue;
        /* clamp: frame_counter can reset on game restart → lru_clock < luf */
        uint32_t age = (lru_clock >= luf) ? (lru_clock - luf) : 0;
        if      (age == 0)   lo[0]++;
        else if (age < 10)   lo[1]++;
        else if (age < 100)  lo[2]++;
        else if (age < 1000) c[(age - 100) / 100]++;
        else { uint32_t b = (age - 1000) / 250; if (b > 8) b = 8; k[b]++; }
    }
    LOG_DBG("=== CACHE DUMP watch=%u static=%u dyn=%u lru_clock=%lu ===\r\n",
              (unsigned)watch_count, (unsigned)static_watch_count,
              (unsigned)(watch_count - static_watch_count), (unsigned long)lru_clock);
    LOG_DBG("  hot: age0=%lu 1-9=%lu 10-99=%lu\r\n",
              (unsigned long)lo[0], (unsigned long)lo[1], (unsigned long)lo[2]);
    LOG_DBG("  100s: 1xx=%lu 2xx=%lu 3xx=%lu 4xx=%lu 5xx=%lu 6xx=%lu 7xx=%lu 8xx=%lu 9xx=%lu\r\n",
              (unsigned long)c[0], (unsigned long)c[1], (unsigned long)c[2],
              (unsigned long)c[3], (unsigned long)c[4], (unsigned long)c[5],
              (unsigned long)c[6], (unsigned long)c[7], (unsigned long)c[8]);
    LOG_DBG("  1k+: 1.00k=%lu 1.25k=%lu 1.50k=%lu 1.75k=%lu 2.00k=%lu 2.25k=%lu 2.50k=%lu 2.75k=%lu 3k+=%lu\r\n",
              (unsigned long)k[0], (unsigned long)k[1], (unsigned long)k[2],
              (unsigned long)k[3], (unsigned long)k[4], (unsigned long)k[5],
              (unsigned long)k[6], (unsigned long)k[7], (unsigned long)k[8]);

//     for (uint16_t i = 0; i < watch_count; i++) {
//         uint32_t luf = last_used_frame ? last_used_frame[i] : 0;
//         const char *region = (i < static_watch_count) ? "S" : "D";
//         if (luf == LRU_PROTECTED)
//             LOG_FRAME("%u %s 0x%08lX=%02X prot\r\n", (unsigned)i, region,
//                       (unsigned long)watch_addresses[i], (unsigned)memory_data[i]);
//         else
//             LOG_FRAME("%u %s 0x%08lX=%02X age=%lu\r\n", (unsigned)i, region,
//                       (unsigned long)watch_addresses[i], (unsigned)memory_data[i],
//                       (unsigned long)(lru_clock - luf));
//         if ((i & 0x3F) == 0x3F) vTaskDelay(1);   /* yield every 64 lines */
//     }
//     LOG_FRAME("=== END CACHE DUMP ===\r\n");
}   /* per-entry dump above intentionally disabled — histogram is enough */

/* When > 0, pending REMOVE event payload waiting to be emitted on the
 * next response. Holds the addresses (BE wire order) to be removed.
 * Sized for one batch (EVICT_BATCH_SIZE). */
#define EVICT_BATCH_SIZE 256
/* v0.25.0: removal is INDEX-based (RA_EVT_WATCHLIST_REMOVE_IDX). Both
 * replicas are identical pre-removal, so array indices identify entries
 * unambiguously — no address searching on the Wii (was O(R*N) ≈ 25-30ms
 * at N=6144) and no duplicate-address ambiguity class. Host order here;
 * converted to BE u16 at emission. Ascending by construction. */
static uint16_t pending_remove_idx[EVICT_BATCH_SIZE];
static uint16_t pending_remove_count = 0;

/* v0.25.0 — tombstone cache: last known value of recently-evicted
 * addresses. On a peek MISS the evaluator gets the last real value
 * instead of 0, so Delta==Mem → no false "value changed" transitions
 * (the SMG comet/library false unlocks); the miss is still recorded so
 * the multi-pass re-fetches, and the REAL transition lands one frame
 * later with real values. PSRAM (calloc in setup), task-context only. */
#define TOMB_SIZE 4096   /* power of 2 */
#define TOMB_MASK (TOMB_SIZE - 1)
typedef struct {
    uint32_t addr;
    uint8_t  value;
    uint8_t  used;
} tomb_entry_t;
static tomb_entry_t *tomb = NULL;

static void tomb_put(uint32_t addr, uint8_t value) {
    if (!tomb) return;
    uint32_t h = (addr * 0x9E3779B9u) & TOMB_MASK;
    for (uint32_t p = 0; p < 8; p++) {
        uint32_t s = (h + p) & TOMB_MASK;
        if (!tomb[s].used || tomb[s].addr == addr) {
            tomb[s].addr = addr; tomb[s].value = value; tomb[s].used = 1;
            return;
        }
    }
    /* All 8 probe slots busy — overwrite the home slot (LRU-ish enough
     * for a best-effort stale-value cache). */
    tomb[h].addr = addr; tomb[h].value = value; tomb[h].used = 1;
}

static bool tomb_get(uint32_t addr, uint8_t *out) {
    if (!tomb) return false;
    uint32_t h = (addr * 0x9E3779B9u) & TOMB_MASK;
    for (uint32_t p = 0; p < 8; p++) {
        uint32_t s = (h + p) & TOMB_MASK;
        if (!tomb[s].used) return false;
        if (tomb[s].addr == addr) { *out = tomb[s].value; return true; }
    }
    return false;
}

static void tomb_clear(void) {
    if (tomb) memset(tomb, 0, TOMB_SIZE * sizeof(tomb_entry_t));
}

/* ============================================================================
 * Phase D2 (v0.26.0) — VERIFIED watchlist sync via mutation sequence numbers.
 *
 * Every watchlist mutation (APPEND / REMOVE_IDX / full load) gets a
 * monotonic u16 seq. The Wii applies a mutation iff seq == ra_seq+1
 * (anything else is dropped — duplicates and reordering are impossible
 * BY CONSTRUCTION) and echoes its applied seq in every SNAPSHOT header.
 * The ESP therefore KNOWS the Wii's exact position in the mutation
 * stream instead of inferring it from count equality:
 *
 *   ra behind  → deliver exactly the next mutation from the ring
 *                (idempotent — replaces the count-mismatch resender and
 *                 its streak heuristics, deleted in v0.26.0)
 *   impossible → full RESYNC (WATCHLIST_UPDATE + chunk re-fetch in-game)
 *
 * Mutations are RECORDED at mutation time (evict_lru / watchlist_append
 * build the complete event response into the ring) and DELIVERED whenever
 * a response slot is free and the Wii is behind — first delivery and
 * re-delivery are the same code path.
 * ========================================================================= */
/* ----------------------------------------------------------------------
 * Warm-up gate (v0.26.3). The v0.26.2 fastboot starts evaluation ~1s
 * after the game's first VI retrace — which removed an ACCIDENTAL
 * protection: the old ~27s LED boot theater meant the first do_frame ran
 * long after game memory had settled. With fastboot, do_frame runs
 * during the boot discovery storm, where every newly-found masked byte
 * reads 0 then flips to its real value one frame later — "value
 * changed" conditions mass-fired 5 false unlocks 4s into a session
 * (2026-06-12 19:21:55).
 *
 * The fix is NOT to delay do_frame (masked-byte discovery NEEDS the
 * evaluator running — pausing it deadlocks the convergence). Instead:
 * run the warm-up in rcheevos SPECTATOR MODE (no server submissions),
 * swallow trigger events locally (no LED/serial), and when the window
 * ends call rc_client_reset() — every trigger returns to WAITING with
 * hit counts cleared and deltas re-primed from the now-fully-resident
 * memory. Clean start, ~10s after boot, all invisible to the player. */
static bool     g_warmup_active = false;
static uint32_t g_warmup_end_frame = 0;
#define WARMUP_GAME_FRAMES 600u   /* ~10s at 60fps */

static uint16_t wl_seq = 0;       /* last mutation seq applied locally (ESP) */
static uint16_t ra_seq_view = 0;  /* our view of the Wii's applied seq:
                                   * authoritative from the snapshot echo,
                                   * advanced optimistically on delivery */

#define MUT_RING 16               /* power of 2 not required; seq % MUT_RING.
                                   * Was 8 — raised to 16 to accommodate per-round
                                   * watchlist_append() calls during multi-level
                                   * pointer chain discovery (up to ADDR_QUERY_MAX_ITERATIONS
                                   * = 12 mutations per convergence pass). */
/* Largest mutation response = APPEND at ADDR_QUERY_MAX=512:
 *   hdr(6) + append_hdr(4) + 512*4 = 2058 bytes  (REMOVE_IDX max = 522).
 * Was 532, sized for ADDR_QUERY_MAX=128. At 16 slots the ring is now ~33KB,
 * so it is PSRAM-allocated in setup() (calloc, after the extmem threshold)
 * instead of static .bss — it is task-context only (written by mut_record on
 * the EXI task, read by mut_deliver_next, copied into the ISR tx_buf by
 * send_response; never touched by the ISR itself), so PSRAM is safe and keeps
 * SRAM free for mbedTLS's ~45KB handshake. v0.28.7 reverted a naked 512 bump
 * precisely because the old resp[532] could not hold this APPEND. */
#define MUT_RESP_SZ 2068
typedef struct {
    uint16_t seq;
    uint16_t resp_len;            /* 0 = slot empty/overwritten */
    uint8_t  resp[MUT_RESP_SZ];   /* full event response: hdr + payload */
} mut_entry_t;
static mut_entry_t *mut_ring = NULL;   /* [MUT_RING] — calloc'd to PSRAM in setup() */

static void send_response(const uint8_t *data, size_t len);  /* fwd decl */

static void mut_record(uint16_t seq, const uint8_t *resp, uint16_t len) {
    if (!mut_ring) return;
    mut_entry_t *m = &mut_ring[seq % MUT_RING];
    if (len > sizeof(m->resp)) { m->resp_len = 0; return; }
    m->seq = seq;
    m->resp_len = len;
    memcpy(m->resp, resp, len);
}

static void mut_ring_clear(void) {
    if (!mut_ring) return;
    for (int i = 0; i < MUT_RING; i++) mut_ring[i].resp_len = 0;
}

/* Deliver the next mutation the Wii is missing (ra_seq_view+1), if we
 * still hold it. Returns true if a response was sent (slot consumed). */
static bool mut_deliver_next(void) {
    if (!mut_ring || ra_seq_view == wl_seq) return false;  /* nothing pending */
    uint16_t want = (uint16_t)(ra_seq_view + 1);
    mut_entry_t *m = &mut_ring[want % MUT_RING];
    if (m->seq != want || m->resp_len == 0) return false;  /* ring lost it */
    send_response(m->resp, m->resp_len);
    ra_seq_view = want;  /* optimistic; the next snapshot echo corrects */
    return true;
}

/* trigger_watchlist_resync lives further down (after the g_watchlist_*
 * declarations it references — the Arduino preprocessor hoists function
 * prototypes but not variable declarations). */

/* Phase A.5 LRU eviction high-water mark. When watch_count would exceed
 * this on watchlist_append, evict_lru drops up to EVICT_BATCH_SIZE=256
 * age-cold dynamic entries, leaving headroom before the next eviction.
 *
 * History: 900 fit Kirby under MAX=1024; 1800 under MAX=2048 thrashed
 * on SMG; 3800 under MAX=4096 ALSO livelocked — SMG's live working set
 * is ~3700 (119 active cheevos x ~30 chain bytes), i.e. demand sat ON
 * the ceiling and the exact-LRU eviction dropped live bytes that got
 * re-requested next frame (3698->3442->3570 cycle) while masked-chain
 * bytes read 0 on miss and false-fired unlocks. Since v0.25.0 the
 * eviction is age-gated (LRU_MIN_AGE) + index-based + tombstoned, and
 * HIGH_WATER is a "start reclaiming above this" line, not a hard
 * ceiling — RA_MAX_WATCH_ADDRS=6144 is the only hard cap.
 *
 * STRESS_TEST_HIGH_WATER reproduces the exact eviction pressure of the
 * failed v0.24.6 SMG run (HW=3800, demand ~3700 sitting on the line) —
 * the A/B harness for validating Phase D1 under fire. The Wii side
 * (sized for 6144) needs NO rebuild between configs. Expected under
 * stress: REMOVE_IDX evictions logged ("only N/256 cold enough" is
 * normal), ZERO "mismatch PERSISTED", ZERO false unlocks, fires high.
 * After validation, set to 0 for the release config (HW=5888). */
#define STRESS_TEST_HIGH_WATER 0   /* release config — D1/D2 validated under stress 2026-06-12 */
#if STRESS_TEST_HIGH_WATER
#define WATCHLIST_HIGH_WATER 3800
#else
#define WATCHLIST_HIGH_WATER 4608   /* v0.31: 75% of RA_MAX_WATCH_ADDRS=6144 — eviction trigger (was 5888, ~never hit before the cap froze the list) */
#endif

/* v0.32.1 PERIODIC GARBAGE COLLECTOR. Two eviction modes now:
 *   - PRESSURE (watch_count > WATCHLIST_HIGH_WATER): emergency, reclaim half,
 *     relax age 1800→120→30. Unchanged.
 *   - PERIODIC GC (every GC_PERIOD_FRAMES): gentle, remove ONLY entries cold
 *     for ≥ GC_MIN_AGE (genuine dead weight). No target/relax — evicts nothing
 *     if nothing is that cold (cheap scan + early return, no hash rebuild).
 * Keeps the watchlist lean BELOW the high-water so snapshots stay small (less
 * EXI stress, smaller upd) without the aggressive half-dump.
 * NOTE on units: lru_clock ticks one per GAME FRAME (~60Hz), so 600 frames ≈
 * 10s and 3600 ≈ 1 min. The period is a cheap CHECK every ~10s; the age floor
 * is ~1 real minute (genuinely dead → minimal churn, matches "garbage
 * collector"). Both tunable; drop GC_MIN_AGE toward 600 for a leaner-but-
 * churnier list, raise it to trim less. */
#define GC_PERIOD_FRAMES 600u   /* run the periodic GC every ~10s of game frames */
#define GC_MIN_AGE       3600u  /* periodic GC evicts entries cold for ≥ ~1 min */

/* Phase C freshness guard: with the chain window sharing the 8192B snapshot
 * transaction, FULL per-frame slot coverage holds only while
 *   flat <= 8192 - headers - bitmap - blob   (~3786 for SMG's table).
 * Above that the window rotates and slots go 1-2 frames stale (graceful but
 * silent). So in authoritative mode the pressure-eviction threshold is
 * LOWERED to fire before that line (computed per game in on_game_loaded,
 * with a 64B margin); the battle-tested half-dump reclaim then keeps the
 * flat comfortably inside the always-fresh budget. Legacy games keep the
 * compile-time WATCHLIST_HIGH_WATER. */
static uint16_t g_watch_high_water = WATCHLIST_HIGH_WATER;

/* Forward decl. evict_lru lives near watchlist_update_if_changed; the trigger
 * is at the SNAPSHOT handler (converged state, before resolution) since v0.31.
 * pressure=true → HIGH_WATER emergency; pressure=false → periodic GC. */
static void evict_lru(bool pressure);

// Prefetch and watchlist state — used for two-pass processing
#define PREFETCH_MAX 4096   /* rc_memrefs_get_addresses returns (addr,size) pairs; 1024 was too small for SSBM, 2048 risky for SMG's 119-cheevo set (silent truncation = permanently-missing addrs) */
/* v0.24.7: heap-allocated in setup() (≥256B → PSRAM via the extmem
 * threshold) instead of static .bss — 20KB of internal RAM that mbedTLS
 * needs more than we do. Task-context use only (loop/Core 1), never ISR. */
static uint32_t *prefetch_addrs = NULL;
static uint8_t  *prefetch_sizes = NULL;

#define ADDR_QUERY_MAX 512  // max addresses per real-time query.
                            // v0.29.2 RE-RAISED to 512 (was 128 since v0.28.7's revert):
                            // the mut_ring blocker is fixed — resp[] is now MUT_RESP_SZ
                            // (2068, holds the 2058-byte APPEND) and the ring lives in
                            // PSRAM (calloc in setup). EXI buffers are all 8192 and
                            // send_addr_query_response mallocs per-round, so the 512-wide
                            // request (2058B) fits everywhere. d2x already at
                            // RA_ADDR_QUERY_MAX=512 (ra_snap_rx_buf=2080 >= 2058).
                            // CAVEAT: 512 only helps WIDTH-limited transitions (a single
                            // chain level resolving >128 NEW addrs). SMG/Kirby are
                            // narrow-deep (sequential pointer chains) → ~no-op there; the
                            // win is for wide-fanout games. Drop back to 128 if it regresses.
uint32_t query_addrs[ADDR_QUERY_MAX];
uint8_t  query_values[ADDR_QUERY_MAX];
uint16_t query_count = 0;
static volatile bool addr_query_pending = false;

/* peek_from_snapshot miss tracker — captures byte addresses that peek
 * was called for but were not in cache. rc_memrefs_get_addresses outputs
 * UNMASKED resolved addresses (parent_raw_value + offset) but the real
 * trigger evaluator applies masks via the modified_memref chain, so the
 * addresses actually peeked during do_frame can differ from what we
 * cached. Re-queuing peek misses gives those masked/real addresses
 * one frame of latency before they land in the watchlist. */
#define PEEK_MISS_MAX 256
static uint32_t peek_miss_addrs[PEEK_MISS_MAX];
static uint16_t peek_miss_count = 0;

/* Phase C node table (built in on_game_loaded; served to the d2x via
 * RA_CMD_GET_CHAIN_CHUNK). PSRAM; freed/rebuilt per game. count==0 =>
 * Phase C inactive for this game (everything on the legacy path). */
static rc_phasec_node_t* g_phasec_nodes = NULL;
static uint32_t g_phasec_node_count = 0;
static uint32_t g_phasec_blob_bytes = 0;
/* Shadow-phase ingestion state (persistent across rotating windows):
 * blob = raw leaf bytes per shipped slot (offsets = prefix sums of widths),
 * valid/fresh = 1 bit per shipped slot. Compared against the legacy hash
 * (authoritative) every RA_PHASEC_SHADOW_EVERY snapshots. */
static uint8_t*  g_phasec_blob = NULL;       /* [g_phasec_blob_bytes] */
static uint32_t* g_phasec_blob_off = NULL;   /* [shipped+1] prefix sums */
static uint16_t* g_phasec_ship_node = NULL;  /* shipped idx -> node idx */
static uint16_t* g_phasec_node_ship = NULL;  /* node idx -> shipped idx (0xFFFF) */
static uint8_t*  g_phasec_valid = NULL;      /* bitmap, shipped idx */
static uint8_t*  g_phasec_fresh = NULL;      /* bitmap: slot seen at least once */
static uint32_t  g_phasec_shipped = 0;
static uint32_t  g_phasec_win_rx = 0;        /* windows ingested (diag) */
static uint32_t  g_phasec_cmp_ok = 0, g_phasec_cmp_bad = 0,
                 g_phasec_cmp_miss = 0, g_phasec_cmp_inv = 0;

/* ============================================================================
 * SNAPSHOT QUEUE (zero-frame-loss, 2026-07-02) — hardcore-mode fidelity.
 *
 * The old handoff was ONE boolean (new_snapshot): while a >16.67ms do_frame
 * ran on the loop task (the de_ev=160 eval storms), the next snapshot's
 * set_new_snapshot() collapsed into the same boolean and that gameplay frame
 * was never individually evaluated (~8-12 lost/session, 0.03%). Now: when the
 * loop task is busy (or already owes work), the worker task ENQUEUES the raw
 * snapshot payload; the loop task DRAINS the ring after each do_frame,
 * applying each entry (snapshot_apply_payload) and evaluating EVERY frame
 * with ITS OWN exact data — a lagging emulator, not stale values.
 *
 * RULES (see project_snapshot_queue_design):
 *  - Phase D2 seq verification stays at ARRIVAL (worker) — only in-sync
 *    snapshots are enqueued; mutation delivery is value-independent.
 *  - Collect/ADDR_QUERY/evict/GC run ONLY on the fast path (queue empty,
 *    loop idle): the dp resolver must not run against stale do_frame state,
 *    and an ADDR_QUERY under lag would feed FUTURE MEM1 values into a PAST
 *    frame's eval. dp-chains wait 1-2 frames (peek_miss net catches up).
 *  - Single producer (worker task, tail) / single consumer (loop task,
 *    head), both on Core 1 → volatile indexes suffice. Head advances only
 *    AFTER the slot is applied, so the producer never reuses a live slot.
 *  - Ring full (sustained overload — shouldn't happen at steady 6-10ms):
 *    drop the NEW frame + count it (== today's behavior, visible as q_drop).
 */
#define SNAPQ_N 8                       /* 8 x 8KB PSRAM; storms are 1-2 deep */
static uint8_t* g_snapq_buf[SNAPQ_N] = {0};   /* ps_malloc'd in setup() */
static volatile uint16_t g_snapq_len[SNAPQ_N] = {0};
static volatile uint8_t  g_snapq_head = 0;    /* consumer: loop task */
static volatile uint8_t  g_snapq_tail = 0;    /* producer: worker task */
static volatile bool     g_df_busy = false;   /* loop task inside do_frame/drain */
static uint32_t g_q_enq = 0, g_q_drop = 0, g_q_hwm = 0;   /* per-CATCHUP diag */
static inline bool    snapq_empty(void) { return g_snapq_head == g_snapq_tail; }
static inline uint8_t snapq_depth(void) {
    return (uint8_t)((g_snapq_tail - g_snapq_head + SNAPQ_N) % SNAPQ_N);
}
#define RA_SNAP_HDR_LEGACY 8u   /* pre-Phase-C snapshot header size (dual parse) */

/* ============================================================================
 * Phase D — dp verification (the desync detector). The walker resolves
 * DELTA/PRIOR edges from its own frame history (sound because the snapshot
 * queue guarantees walks == do_frames) and SHIPS the parent prev/prior values
 * it used. After the memref UPDATE (the gate's primed callback position) the
 * ESP compares them against its own delta/prior for the same parents:
 * mismatch = the two "previous frame" clocks diverged (a dropped/deferred
 * frame) → DEFER this frame (never evaluate a mixed view). DELTA self-heals
 * after one cleanly evaluated frame; PRIOR on the value's next change.
 * The (parent,kind) list is derived by scanning the node table in order —
 * identical on both sides by construction. dpmm = the mismatch counter IS
 * the instrumentation (each event logged with both values).
 * ============================================================================ */
static uint16_t g_phasec_dp_n = 0;
static uint16_t g_phasec_dp_node[RC_PHASEC_MAX_DP];    /* parent node index */
static uint8_t  g_phasec_dp_kind_a[RC_PHASEC_MAX_DP];  /* 1=delta 2=prior */
static void*    g_phasec_dp_memref[RC_PHASEC_MAX_DP];  /* rc_memref_t* (from keys) */
static uint32_t g_phasec_dp_ship[RC_PHASEC_MAX_DP];    /* values shipped this frame */
static uint8_t  g_phasec_dp_shipvalid[(RC_PHASEC_MAX_DP + 7) / 8]; /* bit i = the
                                       * parent RESOLVED at walk N-1 — compare
                                       * skipped on 0 (unloaded pointer: its
                                       * subtree evaluates as legacy zeros, a
                                       * desync there cannot mis-evaluate) */
static volatile bool g_phasec_dp_have = false;         /* section present for pending frame */
static uint32_t g_pc_dpmm = 0;                         /* mismatches (cumulative diag) */
static uint32_t g_pc_dpskip = 0;                       /* compares skipped (parent unresolved) */

/* v0.32 cache-primed do_frame gate (resolver/evaluator divergence cure).
 * g_update_miss_count is reset right before each rc_client_do_frame and bumped
 * by read_memory_ingame on a NO-TOMBSTONE valid miss (a read that would return
 * 0 — the only thing that can pollute a Delta into a false "became X"). The
 * gate predicate (doframe_primed_cb) defers the frame when it's > 0. A
 * tombstone miss does NOT bump it: that read returns the last real value
 * (Delta==Mem, no fake edge), so it's safe to evaluate while the refresh is
 * resolved in the background. Touched ONLY by read_memory_ingame (loop task)
 * so there is no cross-task race with the EXI task's peek_from_snapshot. */
static volatile uint32_t g_update_miss_count = 0;
static uint32_t g_doframe_deferred_total = 0;   /* CATCHUP telemetry: frames deferred since last log */
/* PSRAM rollback buffer for the gate — sized to the memref count at game load.
 * Holds one rc_memref_value_t per memref so a deferred frame's UPDATE leaves
 * zero trace. ISR never touches it (loop-task only), so PSRAM is fine. */
static rc_memref_value_t *g_memref_save = NULL;

/* Gate predicate: rc_client_do_frame calls this after the memref UPDATE and
 * before condition eval. 1 = cache primed (every read hit) -> evaluate;
 * 0 = a read would have returned 0 -> roll back + defer.
 * extern "C" so its type matches the C-linkage g_rc_doframe_primed_cb pointer
 * (GCC rejects assigning a C++-linkage function to a C-linkage pointer). */
extern "C" int doframe_primed_cb(void) {
    if (g_update_miss_count != 0) return 0;
    /* Phase D dp verification — runs AFTER the memref UPDATE (this callback's
     * position inside the gated do_frame), so the ESP-side delta/prior carry
     * THIS frame's semantics: DELTA = changed ? prior : value (mirrors
     * rc_get_memref_value_value), PRIOR = prior. Any mismatch vs what the
     * walker used ⇒ the two "previous frame" clocks diverged ⇒ defer (the
     * gate rolls the frame back; realignment is automatic). */
    /* Skip the dp compare during warmup: spectator mode makes unlocks
     * impossible, and boot inevitably desyncs the two prev-clocks (the d2x
     * walks frames before the ESP's first do_frame) — comparing there only
     * produced defer noise (9 of wii6.log's 10 dpmm were boot transients).
     * Alignment self-establishes on the first cleanly-evaluated live frame. */
    if (g_phasec_dp_have && !g_warmup_active) {
        for (uint16_t i = 0; i < g_phasec_dp_n; i++) {
            /* validity bit 0 = the parent didn't resolve at walk N-1 (unloaded
             * pointer) — the shipped prev is meaningless and the subtree
             * evaluates as legacy zeros anyway. Skip, don't defer. */
            if (!(g_phasec_dp_shipvalid[i >> 3] & (1u << (i & 7)))) {
                g_pc_dpskip++;
                continue;
            }
            const rc_memref_t* m = (const rc_memref_t*)g_phasec_dp_memref[i];
            uint32_t expect = (g_phasec_dp_kind_a[i] == RC_PHASEC_DP_PRIOR)
                ? m->value.prior
                : (m->value.changed ? m->value.prior : m->value.value);
            if (expect != g_phasec_dp_ship[i]) {
                g_pc_dpmm++;
                static uint32_t last_dp_log = 0;
                uint32_t now_dp = millis();
                if (now_dp - last_dp_log >= 2000) {
                    last_dp_log = now_dp;
                    LOG_DBG("DEBUG=PhaseD dpmm: i=%u kind=%u ship=%08lX expect=%08lX (defer)\r\n",
                            (unsigned)i, (unsigned)g_phasec_dp_kind_a[i],
                            (unsigned long)g_phasec_dp_ship[i], (unsigned long)expect);
                }
                return 0;
            }
        }
    }
    return 1;
}

// Dedicated watchlist buffer for chunked delivery
/* v0.24.7: heap-allocated (→PSRAM) for the same reason as the prefetch
 * arrays — 24KB at MAX=6144. Used from GET_CHUNK handling and
 * watchlist_update_if_changed (task context, never ISR). */
static uint32_t *g_watchlist_addrs = NULL;
static uint16_t g_watchlist_count = 0;
static volatile bool g_watchlist_pending = false;

/* Full watchlist RESYNC (Phase D2) — the repair path for any state the
 * seq machinery can't walk forward from (Wii ahead of us, ring overrun,
 * count mismatch at an agreed seq). Publishes the CURRENT list for
 * chunked re-fetch; the WATCHLIST_UPDATE notify carries wl_seq as the
 * new agreed base. While g_watchlist_pending is set,
 * collect_missing_addresses returns 0, so no new mutations are minted
 * mid-resync. Costs ~300ms once — vs a silently corrupted session. */
static void trigger_watchlist_resync(const char *reason) {
    if (g_watchlist_pending) return;   /* already resyncing */
    if (!g_watchlist_addrs || !watch_addresses) return;
    memcpy(g_watchlist_addrs, watch_addresses, watch_count * sizeof(uint32_t));
    g_watchlist_count   = watch_count;
    g_watchlist_pending = true;
    mut_ring_clear();                  /* history is moot after a reload */
    ralog_printf("ERROR=watchlist RESYNC (%s): esp_seq=%u ra_view=%u count=%u\r\n",
                  reason, (unsigned)wl_seq, (unsigned)ra_seq_view,
                  (unsigned)watch_count);
}

/* ============================================================================
 * Address → watch_index hash table — O(1) lookup instead of linear search
 * across watch_addresses[]. Critical for keeping ESP32 Core 1's processing
 * time below the inter-transaction window (~16-30 ms). Without this, Kirby
 * (391 addrs + up to 128 query_addrs) blows past the budget and ra-module
 * reads a stale/empty SPI descriptor on the next transaction.
 *
 * Open-addressing with linear probing. Size is power-of-two, kept well
 * above the worst-case watchlist so the fill ratio stays low and probes
 * short. Empty bucket marker: watch_index == HASH_EMPTY (0xFFFF).
 *
 * v0.24.7 SIZING LESSON: this was 4096 while RA_MAX_WATCH_ADDRS was also
 * 4096 — at SMG's ~3700 entries the table ran at ~90% load, where linear
 * probing degenerates into chains of hundreds of slots per lookup
 * (x3700 lookups/frame — a hidden contributor to the ~10Hz fire rate),
 * and a full table makes hash_insert spin forever.
 *
 * v0.27.4 EXPERIMENT (reverted): tried 8192 in SRAM to cut do_frame's
 * random PSRAM access. Result: df_us only fell 27.8→26.1ms (-6%) but
 * cm_us ROSE 12.6→16.3ms (+29%, the smaller table = longer probe chains
 * during collect's many lookups). NET NEGATIVE. The lesson: PSRAM was
 * NOT the bottleneck — the 32KB data cache already absorbs the accesses
 * (strong temporal locality: catch-up repeats re-read the same addrs).
 * The real cost is rcheevos trigger-eval CPU (~26ms for SMG's 119-set).
 * So HASH_SIZE back to 16384 (best probe behavior); ra_hot_alloc lets it
 * fall to PSRAM (128KB won't fit internal anyway) — the cache makes that
 * fine. Only memory_data (6KB) stays INTERNAL (cheap, neutral-to-slightly
 * positive, leaves mbedTLS plenty of headroom). */
#define HASH_SIZE 16384
#define HASH_MASK (HASH_SIZE - 1)
#define HASH_EMPTY 0xFFFF

typedef struct {
    uint32_t addr;
    uint16_t watch_index;
} hash_entry_t;

static hash_entry_t *addr_hash = NULL;  /* PSRAM (see ra_hot_alloc note) */

/* ra_hot_alloc — was an experiment (v0.27.4) to put hot buffers in
 * INTERNAL SRAM. REVERTED v0.27.7: the addr_hash (128KB at HASH_SIZE
 * 16384) fit internal at setup time (before WiFi/rcheevos alloc'd) and
 * then STARVED mbedTLS — TLS needs ~45KB/handshake, heap fell to 20KB,
 * the achievement download failed silently, rc_client never reached
 * GAME_LOADED, and WiiFlow booted the game by timeout (state stuck at 5).
 * Profiling had already shown SRAM gave only -6% on do_frame (the cost is
 * trigger-eval CPU, not memory), so it was never worth the internal-RAM
 * pressure. Now allocates straight from PSRAM — the data cache absorbs
 * the random access fine. `from_internal` always reports false. */
static void *ra_hot_alloc(size_t bytes, bool *from_internal) {
    void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(bytes);   /* last-resort fallback */
    if (from_internal) *from_internal = false;
    if (p) memset(p, 0, bytes);
    return p;
}
static bool g_hash_internal = false;
static bool g_memdata_internal = false;

static inline uint32_t hash_addr(uint32_t addr) {
    return (addr * 0x9E3779B9u) & HASH_MASK;  /* Fibonacci hash mod size */
}

/* opt (2026-06-19): L1 widx cache in INTERNAL SRAM in front of the 128KB PSRAM
 * addr_hash. Every do_frame the same hot addresses are looked up over and over
 * (B2 skip-checks ~1035/frame, the peeks, the chain resolves) — each pays an
 * open-addressing probe into the PSRAM hash (cache-thrashing at 128KB > the 64KB
 * data cache). This internal cache answers the hot lookups in one or two SRAM
 * reads, no PSRAM, no probe. ~24KB .bss. The user's "static struct in SRAM"
 * idea, applied surgically to just the lookup RESULT (the widx). Stays correct by
 * invalidating on any watchlist remap: l1_clear() on hash_clear (defrag/rebuild),
 * and the in-place update below on a hash_insert that re-maps an existing addr. */
/* 2-way set-associative (2026-06-19): direct-mapped 4096 thrashed at ~62% hit
 * (the ~4590-addr/frame working set collided in single slots). Same 4096 slots,
 * reorganized as 2048 sets x 2 ways: two addresses that hash to the same set now
 * coexist instead of evicting each other. g_l1_mru = pseudo-LRU (evict the way
 * NOT used last). +2KB for g_l1_mru, otherwise identical SRAM footprint. */
/* 2026-07-02 RESIZE 4096->2048 (the SRAM-reclaim backlog #1): Phase C collapsed
 * the lookup working set from ~5460 byte-addrs (all chain leaves, byte-granular)
 * to ~600-900 (static roots + ~90 legacy dp-chains) — covered chains read their
 * slot blob and never touch the hash. 2048 slots (1024 sets x 2 ways) is still
 * ~2x the working set; frees 13KB of internal .bss (addr 8K + widx 4K + mru 1K),
 * funding the I-cache 32KB re-enable (hardcore-mode frame fidelity). Verify with
 * the l1h/l1m FRAME counters: steady hit% should hold ~86%+; if it craters,
 * bump back to 4096 and reclaim elsewhere. */
#define L1_SLOTS    2048u
#define L1_SETS     (L1_SLOTS / 2u)        /* sets of 2 ways each */
#define L1_SET_MASK (L1_SETS - 1u)
#define L1_EMPTY    0xFFFFFFFFu     /* no valid PPC addr is 0xFFFFFFFF */
static uint32_t g_l1_addr[L1_SLOTS];   /* internal .bss; slot = set*2 + way */
static uint16_t g_l1_widx[L1_SLOTS];
static uint8_t  g_l1_mru[L1_SETS];     /* which way was last used (evict the other) */
/* STEP 0 diagnostic (2026-06-30): per-do_frame L1 hit/miss, reset beside b1sk/b2sk.
 * Tells whether upd2 is PSRAM-lookup-bound (low hit) or the lookups are L1-cheap
 * (high hit -> the upd cost is the operand-eval machinery, not the lookup). Plain
 * uint32: do_frame + collect are cooperative on Core 1 so no mid-frame preemption,
 * and an approximate count is fine for a diagnostic. */
uint32_t g_l1_hits = 0, g_l1_miss = 0;
static inline void l1_clear() {
    for (uint32_t i = 0; i < L1_SLOTS; i++) g_l1_addr[i] = L1_EMPTY;
}

/* opt (2026-06-19): hash_clear is the watchlist REMAP signal — evict/defrag
 * rebuild the hash (widxs shift), whereas a plain APPEND uses hash_insert and
 * never clears. The incremental-collect reverse index only needs a full rebuild
 * on a remap; appends leave existing widx->chain entries valid (new chains are
 * discovered anyway, kept dirty by their missing leaf). Tying the rebuild to this
 * flag instead of "any watch_count change" keeps the convergence collect
 * INCREMENTAL on appends — that append-driven full rebuild was the bunny's 37ms
 * collect spike (cwk=1364, all chains re-walked). Same trigger as l1_clear. */
static volatile bool g_incr_remapped = true;   /* start true -> rebuild first frame */

static void u1_clear_all();     /* U1 leaf-widx cache wipe (defined below); remap-only */

static void hash_clear() {
    for (uint32_t i = 0; i < HASH_SIZE; i++) {
        addr_hash[i].watch_index = HASH_EMPTY;
    }
    l1_clear();              /* watchlist remapped -> L1 widxs are stale */
    u1_clear_all();          /* ...and the U1 leaf-widx cache is stale too */
    g_incr_remapped = true;  /* ...and the incr reverse index needs a rebuild */
}

static void hash_insert(uint32_t addr, uint16_t watch_index) {
    uint32_t h = hash_addr(addr);
    while (addr_hash[h].watch_index != HASH_EMPTY) {
        if (addr_hash[h].addr == addr) {
            addr_hash[h].watch_index = watch_index;
            /* keep a cached L1 entry for this addr coherent (re-map in place) */
            { uint32_t base = ((addr * 0x9E3779B9u) & L1_SET_MASK) << 1;
              if (g_l1_addr[base] == addr)        g_l1_widx[base] = watch_index;
              else if (g_l1_addr[base+1] == addr) g_l1_widx[base+1] = watch_index; }
            return;
        }
        h = (h + 1) & HASH_MASK;
    }
    addr_hash[h].addr = addr;
    addr_hash[h].watch_index = watch_index;
}

/* Returns watch_index (0..watch_count-1) or -1 if not present. L1 internal cache
 * answers the hot lookups; misses fall through to the PSRAM open-addressing hash
 * and populate the L1. */
static inline int32_t hash_lookup(uint32_t addr) {
    uint32_t set  = (addr * 0x9E3779B9u) & L1_SET_MASK;
    uint32_t base = set << 1;
    if (g_l1_addr[base]   == addr) { g_l1_mru[set] = 0; ++g_l1_hits; return (int32_t)g_l1_widx[base]; }
    if (g_l1_addr[base+1] == addr) { g_l1_mru[set] = 1; ++g_l1_hits; return (int32_t)g_l1_widx[base+1]; }
    ++g_l1_miss;   /* fell through to the PSRAM hash */

    uint32_t h = hash_addr(addr);
    while (addr_hash[h].watch_index != HASH_EMPTY) {
        if (addr_hash[h].addr == addr) {
            uint32_t way = (uint32_t)g_l1_mru[set] ^ 1u;   /* populate the non-MRU way */
            g_l1_addr[base + way] = addr;
            g_l1_widx[base + way] = addr_hash[h].watch_index;
            g_l1_mru[set] = (uint8_t)way;
            return (int32_t)addr_hash[h].watch_index;
        }
        h = (h + 1) & HASH_MASK;
    }
    return -1;   /* not present: don't cache (a later insert may add it) */
}

/* rc_is_cached_t for rc_memrefs_get_pending_addresses(): every byte in
 * [addr, addr+num_bytes) must be in the hash for this level to be readable
 * from cache. If any byte is missing, the chain walk stops here and reports
 * addr as pending. */
static int is_cached_cb(uint32_t addr, uint32_t num_bytes, void* ud) {
    (void)ud;
    for (uint32_t b = 0; b < num_bytes; b++) {
        if (hash_lookup(addr + b) < 0) return 0;
    }
    return 1;
}

/* opt B2 hook (registered into rcheevos as g_rc_leaf_unchanged). Returns 1 iff
 * EVERY byte in [addr, addr+num_bytes) is cached AND none changed in the latest
 * snapshot — i.e. an INDIRECT_READ leaf at this (stable) address keeps its
 * value and rc_update_memref_values can skip re-resolving it. A miss returns 0
 * (must resolve so the chain picks up a freshly-cached leaf). Per-frame
 * membership gating (g_rc_indirect_skip_enabled) is handled by the caller. */
extern "C" int rc_leaf_unchanged_impl(uint32_t addr, uint32_t num_bytes) {
    if (!snap_changed) return 0;
    for (uint32_t b = 0; b < num_bytes; b++) {
        int32_t widx = hash_lookup(addr + b);
        if (widx < 0) return 0;            // not cached -> resolve
        if (snap_changed[widx]) return 0;  // moved this snapshot -> resolve
    }
    return 1;
}

// ============================================================================
// U1 — leaf-widx side-array cache (STEP 0 confirmed B2 skip-checks = 66% of upd2,
// ~5us each from 2 operand evals + per-byte hash lookups). When the pointer
// (parent) is stable the leaf address is unchanged -> its watch indices are too.
// Cache them per chain (dense index = the rc_modified_memref_can_skip cursor) so a
// skip-check collapses to one snap_changed[] read: NO operand eval, NO hash lookup.
//
// Correctness (the false-unlock surface — device-only, not host-testable):
//  - Populated only on the stable-pointer path; a stable pointer means the leaf
//    address can't have moved, so the cached widxs stay valid frame-to-frame.
//  - A leaf VALUE change (snap_changed[widx]=1) does NOT invalidate (address is
//    the same) -> resolve the value, keep the cache.
//  - A pointer/modifier MOVE invalidates the entry (rc_u1_invalidate) -> the next
//    stable frame re-populates at the new address.
//  - APPEND never shifts existing widxs (hash_insert, no remap) so it needs no
//    invalidation; evict/defrag DO remap -> hash_clear()->u1_clear_all() wipes all.
//  - do_frame gate backstops a true miss (deferred frame), but a stale-cache wrong
//    value is NOT a miss -> the invalidation logic above is the real guarantee.
// ============================================================================
#define U1_MAX_BYTES 4                 /* up to a 32-bit leaf; bigger -> don't cache */
struct u1_entry_t { uint16_t widx[U1_MAX_BYTES]; uint8_t n; };  /* n=0: empty/uncached */
static u1_entry_t* g_u1 = NULL;        /* [g_u1_cap], internal SRAM */
static uint32_t    g_u1_cap = 0;
volatile int       g_u1_enabled = 1;   /* runtime A/B toggle */
uint32_t           g_u1_fast = 0;      /* per-frame: cached skip-checks (no eval) */
uint32_t           g_u1_pop = 0;       /* per-frame: cache-miss populates (full path) */

static void u1_clear_all() {           /* remap -> every cached widx is stale */
    if (g_u1) memset(g_u1, 0, (size_t)g_u1_cap * sizeof(u1_entry_t));
}

/* -1 = not cached (caller computes addr + calls rc_u1_populate); 0 = a leaf byte
 * moved this snapshot; 1 = leaf unchanged. NO operand eval, NO hash lookup. */
extern "C" int rc_u1_cached(uint32_t idx) {
    if (!g_u1_enabled || !g_u1 || idx >= g_u1_cap || !snap_changed) return -1;
    u1_entry_t* e = &g_u1[idx];
    if (!e->n) return -1;                        /* not populated yet */
    ++g_u1_fast;
    for (uint8_t b = 0; b < e->n; b++)
        if (snap_changed[e->widx[b]]) return 0;  /* moved -> resolve (cache kept) */
    return 1;                                    /* unchanged -> skip */
}

/* cache-miss path: do the per-byte hash lookups, store the widxs, return 1 iff the
 * leaf is unchanged. Falls back to a no-cache direct check when uncacheable. */
extern "C" int rc_u1_populate(uint32_t idx, uint32_t addr, uint32_t num_bytes) {
    ++g_u1_pop;
    if (!g_u1_enabled || !g_u1 || idx >= g_u1_cap || !snap_changed || num_bytes > U1_MAX_BYTES)
        return rc_leaf_unchanged_impl(addr, num_bytes);   /* uncacheable -> original B2 */
    u1_entry_t* e = &g_u1[idx];
    int unchanged = 1;
    for (uint32_t b = 0; b < num_bytes; b++) {
        int32_t widx = hash_lookup(addr + b);
        if (widx < 0) { e->n = 0; return 0; }    /* leaf not cached yet -> resolve, don't store */
        e->widx[b] = (uint16_t)widx;
        if (snap_changed[widx]) unchanged = 0;
    }
    e->n = (uint8_t)num_bytes;                   /* now cached for the stable address */
    return unchanged;
}

/* pointer/modifier moved -> the leaf address may change -> drop the cached widxs. */
extern "C" void rc_u1_invalidate(uint32_t idx) {
    if (g_u1 && idx < g_u1_cap) g_u1[idx].n = 0;
}

// ============================================================================
// rcheevos client
// ============================================================================
rc_client_t *g_client = NULL;
static void *g_callback_userdata = &g_client;

String gameName = "not identified";
String gameId = "0";
TaskHandle_t taskCore1Handle = NULL;

// ============================================================================
// Per-command counters — printed in the Core1 heartbeat. Indexed by ra_gc_command_t.
// Useful for the post-IOS-reload phase where ra-module (no serial) drives EXI;
// we need to see if SNAPSHOT/GET_CHUNK/etc. are arriving and at what rate.
// ============================================================================
static volatile uint32_t cmd_count[16] = {0};   // commands 0x01..0x08 + a few spare
static volatile uint32_t cmd_count_last[16] = {0};
static const char* cmd_short_name(uint8_t cmd) {
    switch (cmd) {
        case 0x01: return "IDENT";
        case 0x02: return "LOAD";
        case 0x03: return "SNAP";
        case 0x04: return "POLL";
        case 0x05: return "RESET";
        case 0x06: return "STAT";
        case 0x07: return "CHUNK";
        case 0x08: return "ADDR";
        default:   return "??";
    }
}
// `WiFiManager wm` was a GLOBAL — its constructor emits Serial debug output, which
// crashes at C++ static-init (before Serial/Arduino is up) under ESP-IDF's
// do_global_ctors (Arduino IDE's init order happened to tolerate it). Now built
// lazily in setup() (see the config-portal block). [IDF migration fix]
String base_url = "https://retroachievements.org/dorequest.php?";
NetworkClientSecure client;
HTTPClient https;


// ============================================================================
// Pending events queue (ESP32 -> GameCube)
// ============================================================================
#define MAX_PENDING_EVENTS 8

volatile int event_head = 0;
volatile int event_tail = 0;
PendingEvent event_queue[MAX_PENDING_EVENTS];

void queue_event(uint8_t type, const uint8_t *data, uint16_t len) {
    int next = (event_head + 1) % MAX_PENDING_EVENTS;
    if (next == event_tail) return;  // Queue full
    event_queue[event_head].type = type;
    if (data && len > 0) {
        uint16_t copy_len = len < RA_MAX_RESPONSE_DATA ? len : RA_MAX_RESPONSE_DATA;
        memcpy((void*)event_queue[event_head].data, data, copy_len);
        event_queue[event_head].data_len = copy_len;
    } else {
        event_queue[event_head].data_len = 0;
    }
    event_head = next;
}

bool has_pending_event() {
    return event_head != event_tail;
}

PendingEvent* peek_event() {
    if (!has_pending_event()) return NULL;
    return &event_queue[event_tail];
}

void pop_event() {
    if (has_pending_event()) {
        event_tail = (event_tail + 1) % MAX_PENDING_EVENTS;
    }
}

// ============================================================================
// rcheevos callbacks - adapted from fpga-ra-adapter
// ============================================================================
static void log_message(const char *message, const rc_client_t *client) {
    LOG_DBG("DEBUG=rcheevos: %s\r\n", message);
}

/* ============================================================================
 * Achievement-unlock LED celebration.
 *
 * Two LED back-ends, selected by board_config.h:
 *   LED_USE_RGB  (BOARD_DEV)  — onboard WS2812 RGB on GPIO48. neopixelWrite()
 *                ships with arduino-esp32 (esp32-hal-rgb-led), is RMT-backed —
 *                the hardware generates the WS2812 waveform, so it does NOT
 *                disable interrupts for a whole frame like a bit-bang would; it
 *                only blocks the CALLING task ~30us waiting on the RMT done.
 *   LED_USE_GPIO (BOARD_XIAO) — single yellow user LED on GPIO21, active-low.
 *                Plain digitalWrite(), no addressable LED on the XIAO module.
 *
 * Cadence matches the d2x disc-slot LED exactly: ra_led_celebrate=54 @ 60Hz
 * produces three 9-frame ON pulses with 9-frame OFF gaps (d2x ra-module
 * main.c ra_poll_thread, the `ra_led_celebrate % 9` loop). 9 frames ≈ 150ms,
 * so we mirror it as 3 × (150ms ON / 150ms OFF) ≈ 0.9s.
 *
 * CRITICAL: the LED write MUST NOT run on Core 1 — that core runs the prio-19
 * EXI servicer and any stall mid-transaction corrupts SPI timing. So the blink
 * runs in a dedicated low-prio task pinned to Core 0; the Core 1 event_handler
 * only bumps g_celebrate_req (a 32-bit atomic on Xtensa). */
#define RA_CELEBRATE_PULSES   3
#define RA_CELEBRATE_ON_MS    150
#define RA_CELEBRATE_OFF_MS   150
#define RA_CELEBRATE_LEVEL    40    /* 0-255 per channel; full white WS2812 is blinding */
static volatile uint32_t g_celebrate_req = 0;   /* bumped by event_handler (Core 1) */

#if defined(LED_USE_RGB)
static inline void led_init(void)  { /* WS2812 needs no pin setup */ }
static inline void led_off(void)   { neopixelWrite(PIN_RGB, 0, 0, 0); }
static inline void led_on(void)    { neopixelWrite(PIN_RGB, 0, RA_CELEBRATE_LEVEL, 0); }
#elif defined(LED_USE_GPIO)
static inline void led_init(void)  { pinMode(PIN_LED_GPIO, OUTPUT); }
#if PIN_LED_ACTIVE_LOW
static inline void led_off(void)   { digitalWrite(PIN_LED_GPIO, HIGH); }
static inline void led_on(void)    { digitalWrite(PIN_LED_GPIO, LOW);  }
#else
static inline void led_off(void)   { digitalWrite(PIN_LED_GPIO, LOW);  }
static inline void led_on(void)    { digitalWrite(PIN_LED_GPIO, HIGH); }
#endif
#else
#error "board_config.h must define LED_USE_RGB or LED_USE_GPIO"
#endif

static void ledCelebrateTask(void *arg) {
    (void)arg;
    led_init();
    led_off();
    uint32_t seen = g_celebrate_req;
    for (;;) {
        if (g_celebrate_req != seen) {
            seen = g_celebrate_req;
            for (int i = 0; i < RA_CELEBRATE_PULSES; i++) {
                led_on();
                vTaskDelay(pdMS_TO_TICKS(RA_CELEBRATE_ON_MS));
                led_off();
                vTaskDelay(pdMS_TO_TICKS(RA_CELEBRATE_OFF_MS));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void event_handler(const rc_client_event_t *event, rc_client_t *client) {
    /* Diagnostic logging: name the event and, for the achievement-bearing ones,
     * print id + title + measured progress so a flickering CHALLENGE indicator can be
     * told apart (same ach SHOWing repeatedly w/o a HIDE = is_primed oscillating) from
     * normal multi-achievement priming (different ach ids). net_shown = running
     * (SHOW - HIDE) tally: it should hover at the count of currently-primed achievements;
     * if it drifts unbounded up (or negative) the SHOW/HIDE events are imbalanced. */
    const rc_client_achievement_t *a = event->achievement;
    switch (event->type) {
        case RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_SHOW:
        case RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_HIDE: {
            static int net_shown = 0;
            int show = (event->type == RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_SHOW);
            net_shown += show ? 1 : -1;
            LOG_INFO("DEBUG=event: CHALLENGE_%s ach=%lu \"%s\" prog=%s net_shown=%d\r\n",
                    show ? "SHOW" : "HIDE",
                    a ? (unsigned long)a->id : 0UL, a ? a->title : "?",
                    (a && a->measured_progress[0]) ? a->measured_progress : "-", net_shown);
#if RA_WEB_DASHBOARD
            ra_web_push_challenge(show, a ? a->id : 0);
#endif
            break;
        }
        case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_SHOW:
        case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_HIDE:
        case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_UPDATE:
            LOG_INFO("DEBUG=event: PROGRESS_%lu ach=%lu \"%s\" prog=%s pct=%d\r\n",
                    (unsigned long)event->type,
                    a ? (unsigned long)a->id : 0UL, a ? a->title : "?",
                    (a && a->measured_progress[0]) ? a->measured_progress : "-",
                    a ? (int)a->measured_percent : 0);
#if RA_WEB_DASHBOARD
            if (a && event->type != RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_HIDE)
                ra_web_push_progress(a->id, (int)a->measured_percent, a->measured_progress);
#endif
            break;
        /* Leaderboards (hardcore experiment 2026-07-03). STARTED/FAILED/
         * SUBMITTED and tracker SHOW/HIDE are rare — log normally. TRACKER_
         * UPDATE fires EVERY FRAME the tracked value changes (a speedrun
         * timer = 60/s) — hard-throttled so the log ring / Core 1 never
         * feel it and the perf measurement stays clean. */
        case RC_CLIENT_EVENT_LEADERBOARD_STARTED:
        case RC_CLIENT_EVENT_LEADERBOARD_FAILED:
        case RC_CLIENT_EVENT_LEADERBOARD_SUBMITTED:
            LOG_INFO("DEBUG=event: LBOARD_%s id=%lu \"%s\"\r\n",
                    event->type == RC_CLIENT_EVENT_LEADERBOARD_STARTED ? "START"
                    : event->type == RC_CLIENT_EVENT_LEADERBOARD_FAILED ? "FAIL"
                    : "SUBMIT",
                    event->leaderboard ? (unsigned long)event->leaderboard->id : 0UL,
                    event->leaderboard ? event->leaderboard->title : "?");
            break;
        case RC_CLIENT_EVENT_LEADERBOARD_TRACKER_SHOW:
        case RC_CLIENT_EVENT_LEADERBOARD_TRACKER_HIDE:
            LOG_INFO("DEBUG=event: LBTRK_%s id=%lu %s\r\n",
                    event->type == RC_CLIENT_EVENT_LEADERBOARD_TRACKER_SHOW ? "SHOW" : "HIDE",
                    event->leaderboard_tracker ? (unsigned long)event->leaderboard_tracker->id : 0UL,
                    event->leaderboard_tracker ? event->leaderboard_tracker->display : "?");
            break;
        case RC_CLIENT_EVENT_LEADERBOARD_TRACKER_UPDATE: {
            static uint32_t last_trk_log = 0;
            uint32_t now_trk = millis();
            if (now_trk - last_trk_log >= 2000) {
                last_trk_log = now_trk;
                LOG_DBG("DEBUG=event: LBTRK_UPD id=%lu %s (2s throttle)\r\n",
                        event->leaderboard_tracker ? (unsigned long)event->leaderboard_tracker->id : 0UL,
                        event->leaderboard_tracker ? event->leaderboard_tracker->display : "?");
            }
            break;
        }
        default:
            LOG_INFO("DEBUG=event: %d\r\n", event->type);
            break;
    }
    switch (event->type) {
        case RC_CLIENT_EVENT_ACHIEVEMENT_TRIGGERED: {
            /* v0.26.3 warm-up: triggers fired against boot-storm garbage
             * are swallowed (no Wii LED, no serial ACHIEVEMENT line, no
             * queue). Spectator mode already blocks the server submit;
             * rc_client_reset at warm-up end wipes the spent state so a
             * GENUINE unlock later re-fires cleanly. */
            if (g_warmup_active) {
                LOG_DBG("DEBUG=warmup: suppressed trigger %lu (%s)\r\n",
                        (unsigned long)event->achievement->id,
                        event->achievement->title);
                break;
            }
            const rc_client_achievement_t *ach = event->achievement;
            // Build achievement event data
            ra_achievement_t ach_data;
            ach_data.achievement_id = ra_host_to_be32(ach->id);
            uint8_t title_len = strlen(ach->title);
            if (title_len > RA_MAX_TITLE_LEN) title_len = RA_MAX_TITLE_LEN;
            ach_data.title_len = title_len;

            uint8_t buf[sizeof(ra_achievement_t) + RA_MAX_TITLE_LEN];
            memcpy(buf, &ach_data, sizeof(ra_achievement_t));
            memcpy(buf + sizeof(ra_achievement_t), ach->title, title_len);

            queue_event(RA_EVT_ACHIEVEMENT, buf, sizeof(ra_achievement_t) + title_len);
            /* Trigger the onboard RGB celebration (3 green pulses, d2x cadence).
             * Just bump the counter — the actual neopixelWrite() runs on Core 0's
             * ledCelebrateTask, NEVER here on the Core 1 EXI critical path. */
            g_celebrate_req++;
            ralog_printf("ACHIEVEMENT=%lu;%s\r\n", (unsigned long)ach->id, ach->title);
#if RA_WEB_DASHBOARD
            /* Instant toast on the phone dashboard. badge_name → the browser
             * builds the media.retroachievements.org URL. */
            ra_web_push_unlock(ach->id, ach->title, ach->badge_name);
#endif
            break;
        }
        default:
            break;
    }
}

// Read memory from our local snapshot — binary search (addresses are sorted)
/* read_memory_ingame is registered with rc_client and called by
 * rc_client_do_frame's trigger evaluator (via rc_update_memref_values
 * + rc_get_modified_memref_value). It MUST behave identically to
 * peek_from_snapshot:
 *   - O(1) hash lookup against the append-only watchlist (NOT a
 *     binary search — watch_addresses is insertion-ordered after
 *     WATCHLIST_APPEND growth so binary search would silently miss
 *     every byte added past the initial sorted prefix).
 *   - Fall back to the per-frame query_addrs cache.
 *   - On miss, record the byte address into peek_miss_addrs so the
 *     next collect_missing_addresses round can queue an ADDR_QUERY.
 *     Without this, the trigger evaluator's reads at MASKED chain
 *     addresses (Kirby's pattern) never feed into our multi-pass
 *     bootstrap, chain[1] remains permanently uncached, and the
 *     cheevo cannot fire even when the player is in a valid level. */
static uint32_t read_memory_ingame(uint32_t address, uint8_t *buffer,
                                   uint32_t num_bytes, rc_client_t *client) {
    (void)client;
    /* Return RAW bytes in address order. rcheevos handles PPC BE-ness
     * INTERNALLY via the RC_MEMSIZE_*_BITS_BE size enums (see memref.c
     * rc_transform_memref_value cases 526-542) — cheevo devs pick the
     * _BE variant for BE-stored values; rcheevos peek returns LE-pack
     * of raw bytes, then transforms when applying the size. If we
     * byte-swap here, the BE size's internal swap reverts our swap and
     * produces the LE-pack value (broken). LE-size cheevos work natively
     * because the dev coded against the LE-pack interpretation. Earlier
     * v0.15.2 BE swap was a misdiagnosis — Kirby v0.22.1-oca-diag log 2026-05-31
     * 20:01:42 showed first_qa=0053F774 = LE-pack of [0x80,0xDC,0x53,0x00]
     * + 0x1AF4, proving cheevo uses RC_MEMSIZE_32_BITS (LE) for pptr. */
    for (uint32_t j = 0; j < num_bytes; j++) {
        uint32_t byte_addr = address + j;
        bool got = false;

        int32_t widx = hash_lookup(byte_addr);
        if (widx >= 0) {
            buffer[j] = memory_data[widx];
            /* Phase A.5: bump last_used (unless static-protected). */
            if (last_used_frame && last_used_frame[widx] != LRU_PROTECTED) {
                last_used_frame[widx] = lru_clock;
            }
            got = true;
        }

        /* (query_addrs fallback removed — v0.27.8. read_memory is now a
         * pure O(1) hash lookup + tombstone; resolved addresses live in
         * the hash, committed per ADDR_RESPONSE round.) */

        if (!got) {
            /* v0.25.0 tombstone: a recently-evicted address re-touched by
             * the evaluator returns its LAST REAL value instead of 0 —
             * Delta==Mem, no false "value changed" transition. The miss
             * is still recorded below, so the multi-pass re-fetches and
             * the genuine transition lands one frame later with real
             * values. (0-on-miss is what mass-fired the SMG comet/library
             * false unlocks.) */
            uint8_t tv;
            bool has_tomb = tomb_get(byte_addr, &tv);
            buffer[j] = has_tomb ? tv : 0;
            /* Sanity-checked miss tracking — same rules as peek_from_snapshot. */
            bool valid_ppc_addr = (byte_addr <= 0x017FFFFF)
                               || (byte_addr >= 0x10000000 && byte_addr <= 0x137FFFFF  /* IOS-reserved top of MEM2 excluded (read-fault guard) */);
            if (valid_ppc_addr) {
                /* v0.32 gate: a NO-tombstone miss is a read that returns 0 —
                 * the only miss that can forge a Delta "became X" edge. Bump
                 * the gate counter so rc_client_do_frame defers (rolls back,
                 * no eval) until this address resolves. A tombstone miss is
                 * safe (last real value, Delta==Mem) and does NOT defer — it
                 * just gets refreshed via the peek_miss net below. */
                if (!has_tomb) g_update_miss_count++;
                if (peek_miss_count < PEEK_MISS_MAX) {
                    bool dup = false;
                    for (uint16_t k = 0; k < peek_miss_count; k++) {
                        if (peek_miss_addrs[k] == byte_addr) { dup = true; break; }
                    }
                    if (!dup) peek_miss_addrs[peek_miss_count++] = byte_addr;
                }
            }
        }
    }
    return num_bytes;
}

// rc_peek_t wrapper: reads from memory_data (watchlist) OR from the
// dynamic query cache (query_addrs/query_values populated by ADDR_RESP).
// Used during rc_memrefs_get_addresses iteration to resolve pointer
// chains progressively across multi-pass ADDR_QUERY rounds.
// Returns: value as little-endian-packed uint32 (rc_memrefs API contract).
// Returns 0 only for true cache miss — pointer chain will yield bogus
// downstream addresses, which the multi-pass converger detects as
// "still missing" and queries on the next round.
static uint32_t peek_from_snapshot(uint32_t address, uint32_t num_bytes, void *ud) {
    uint8_t buf[4] = {0};
    bool found_all = true;

    for (uint32_t j = 0; j < num_bytes && j < 4; j++) {
        uint32_t byte_addr = address + j;
        bool got = false;

        /* Static watchlist: O(1) hash lookup. Hash table maps addr →
         * watch_index; memory_data[watch_index] is the cached value.
         * Insertion order in watch_addresses is preserved (matches ra-
         * module's SNAPSHOT value stream); the hash is just an index. */
        int32_t widx = hash_lookup(byte_addr);
        if (widx >= 0) {
            buf[j] = memory_data[widx];
            /* v0.29.3: do NOT bump last_used_frame here. This is the CHAIN
             * RESOLVER's peek (rc_memrefs_get_pending_addresses), not the
             * trigger evaluator. Bumping here kept every resolver-walked
             * intermediate artificially hot, masking whether the EVALUATOR
             * actually reads it — and hiding any over-resolution (addresses we
             * fetch+ship but the triggers never read). LRU recency is now
             * driven ONLY by read_memory_ingame. Safe today because eviction is
             * dormant (HIGH_WATER never hit); revisit if eviction is enabled
             * and a resolver-only intermediate starts to churn. */
            got = true;
        }

        /* (query_addrs fallback removed — v0.27.8. Resolved addresses are
         * committed straight to the hash per ADDR_RESPONSE round, so the
         * hash lookup above is the single source of truth. query_addrs is
         * now ONLY the collection buffer for misses → ADDR_QUERY.) */

        if (!got) {
            found_all = false;
            /* v0.25.0 tombstone: stale-but-real value for recently-evicted
             * addresses (chain resolution continuity); still counted as
             * missing so the converger re-fetches. */
            uint8_t tv;
            if (tomb_get(byte_addr, &tv)) buf[j] = tv;
            /* Record the miss so collect_missing_addresses can queue this
             * address for next frame's ADDR_QUERY. Catches the case where
             * rc_memrefs_get_addresses outputs the wrong (unmasked) address
             * but the trigger evaluator peeks the correctly-masked one.
             *
             * SANITY filter: only queue addresses that fall in valid PPC
             * MEM1 (0x00000000..0x017FFFFF) or MEM2 (0x10000000..0x13FFFFFF)
             * ranges. Garbage addresses from unresolved chain walks (e.g.
             * 0x80xxxxxx unmasked virtual addrs, or pure offsets at 0xa64
             * that happen to be at MEM1 boot vectors) get passed to ra-module
             * which then asks Swi_MLoad for memory at those addresses; if
             * the address falls outside mapped RAM, the Starlet thread can
             * hang or trip a bus error, taking the whole system down. */
            bool valid_ppc_addr = (byte_addr <= 0x017FFFFF)
                               || (byte_addr >= 0x10000000 && byte_addr <= 0x137FFFFF  /* IOS-reserved top of MEM2 excluded (read-fault guard) */);
            if (valid_ppc_addr && peek_miss_count < PEEK_MISS_MAX) {
                bool dup = false;
                for (uint16_t k = 0; k < peek_miss_count; k++) {
                    if (peek_miss_addrs[k] == byte_addr) { dup = true; break; }
                }
                if (!dup) peek_miss_addrs[peek_miss_count++] = byte_addr;
            }
        }
    }

    /* LE-pack: rcheevos peek contract returns LE-packed u32 from raw
     * bytes. Internal byte-swap (when needed for _BE size variants) is
     * done in rc_transform_memref_value by rcheevos itself. Mirror this
     * here so peek_from_snapshot and read_memory_ingame agree on the
     * value rcheevos sees. */
    /* cm= is now accumulated in collect_missing_addresses() as the number of
     * pending addresses actually appended to the watchlist this vblank (so it
     * equals sa_delta). peek_from_snapshot no longer counts misses here — its
     * job is just to serve cached bytes and feed peek_miss_addrs. */

    uint32_t val = 0;
    for (uint32_t i = 0; i < num_bytes && i < 4; i++)
        val |= ((uint32_t)buf[i]) << (i * 8);
    return val;
}

static uint32_t read_memory_nop(uint32_t address, uint8_t *buffer, uint32_t num_bytes, rc_client_t *client) {
    memset(buffer, 0, num_bytes);
    return num_bytes;
}

static void rc_callback(int result, const char *error_message, rc_client_t *client, void *userdata) {
    LOG_DBG("DEBUG=rc_callback: %d\r\n", result);
}

rc_clock_t get_millisecs(const rc_client_t *client) { return millis(); }

// ============================================================================
// JSON strip functions — in-place on PsramStream
// Ported from nes-esp-firmware (odelot).
// All functions require compact JSON (no whitespace outside strings).
// json_remove_whitespace() must be called first.
// ============================================================================

/* True iff a `"` at position `idx` in `data` is a real (non-escaped) quote.
 * Counts consecutive `\\` chars immediately before idx; the quote is escaped
 * iff that count is odd. Handles `\"` (escaped), `\\"` (real after escaped
 * backslash), `\\\\"` (escaped again), etc. */
static inline bool json_quote_is_real(const char* data, size_t idx) {
    size_t bs = 0;
    while (idx > bs && data[idx - 1 - bs] == '\\') bs++;
    return (bs % 2) == 0;
}

static void json_remove_whitespace(PsramStream &buf) {
    char*  data = buf.data();
    size_t len  = buf.length();
    bool   in_str = false;
    size_t w = 0;
    for (size_t r = 0; r < len; r++) {
        char c = data[r];
        if (c == '"' && json_quote_is_real(data, r)) in_str = !in_str;
        if (in_str || (c != ' ' && c != '\n' && c != '\r' && c != '\t'))
            data[w++] = c;
    }
    buf.setLength(w);
}

static void json_remove_field(PsramStream &buf, const char* field) {
    json_remove_whitespace(buf);
    char*  data = buf.data();
    size_t len  = buf.length();
    size_t flen = strlen(field);
    bool in_str = false, in_array = false, skipping = false;
    size_t r = 0, w = 0, skip_init = 0;
    while (r < len) {
        char c = data[r];
        /* Track string boundaries respecting escape sequences. Without this
         * check, escaped quotes inside long string values (e.g. Lua scripts in
         * RichPresencePatch) flip in_str at the wrong points and the field
         * removal terminates early on a comma that is actually inside the string. */
        if (c == '"' && json_quote_is_real(data, r)) in_str = !in_str;
        if (c == '[' && skipping) in_array = true;
        if (c == ']' && skipping) in_array = false;
        if (in_str && r + 1 + flen + 1 < len &&
            strncmp(data + r + 1, field, flen) == 0 &&
            data[r + flen + 1] == '"') {
            skipping   = true;
            skip_init  = r;
        }
        if (!skipping) data[w++] = c;
        if (skipping && r + 1 < len && data[r+1] == '}') {
            skipping = false;
            if (skip_init > 0 && w > 0 && data[w-1] == ',') w--;
        } else if (skipping && c == ',' && !in_array && !in_str) {
            skipping = false;
        }
        r++;
    }
    buf.setLength(w);
}

static void json_clean_field_str(PsramStream &buf, const char* field) {
    json_remove_whitespace(buf);
    char*  data = buf.data();
    size_t len  = buf.length();
    size_t flen = strlen(field);
    bool in_str = false, skipping = false, remove_next = false;
    size_t r = 0, w = 0, skip_init = 0;
    while (r < len) {
        char c = data[r];
        if (c == '"' && json_quote_is_real(data, r)) {
            in_str = !in_str;
            if (in_str && remove_next) { skipping = true; data[w++] = '"'; }
            if (!in_str && remove_next && r > skip_init) { remove_next = false; skipping = false; }
        }
        if (in_str && r + 1 + flen + 1 < len &&
            strncmp(data + r + 1, field, flen) == 0 &&
            data[r + flen + 1] == '"') {
            remove_next = true;
            skip_init   = r + flen + 2;
        }
        if (!skipping) data[w++] = c;
        r++;
    }
    buf.setLength(w);
}

/* Replace "field":[ ...array... ] with "field":[] (empty the array, keep key).
 *
 * v0.27.x bug: the old version stopped at the FIRST ']' after the key, with no
 * nesting/string awareness. Array elements that contain ']' or '[' inside a
 * string (e.g. a leaderboard title) or a nested array (e.g. the "Mem" field)
 * made it stop early, removing the wrong span and unbalancing the whole JSON
 * (observed on Metroid Prime: after "Leaderboards" clean → depth=4, in_str=1).
 *
 * Fixed: once inside the array value, track brace/bracket DEPTH while honoring
 * string state + backslash escapes, and stop only at the MATCHING ']'. */
static void json_clean_field_array(PsramStream &buf, const char* field) {
    json_remove_whitespace(buf);
    char*  data = buf.data();
    size_t len  = buf.length();
    size_t flen = strlen(field);
    bool   in_str = false, esc = false, skipping = false, remove_next = false;
    int    depth = 0;
    size_t r = 0, w = 0;

    while (r < len) {
        char c = data[r];

        if (skipping) {
            /* Inside "field":[ ... ] — drop content until the matching ']'. */
            if (esc)               { esc = false; }
            else if (in_str)       { if (c == '\\') esc = true; else if (c == '"') in_str = false; }
            else if (c == '"')     { in_str = true; }
            else if (c == '[' || c == '{') { depth++; }
            else if (c == ']' || c == '}') {
                if (--depth == 0) {            /* matching close of the array */
                    skipping = false;
                    remove_next = false;
                    data[w++] = ']';           /* keep "field":[] */
                }
            }
            r++;
            continue;
        }

        if (esc) {
            esc = false;
        } else if (in_str) {
            if (c == '\\') esc = true;
            else if (c == '"') in_str = false;
        } else if (c == '"') {
            in_str = true;
            /* key match: at the opening quote, is this exactly "field"? */
            if (r + 1 + flen + 1 < len &&
                strncmp(data + r + 1, field, flen) == 0 &&
                data[r + flen + 1] == '"') {
                remove_next = true;
            }
        } else if (remove_next && c == '[') {
            /* start of the array value — begin removing its contents */
            data[w++] = '[';
            skipping = true; depth = 1; in_str = false; esc = false;
            r++;
            continue;
        }

        data[w++] = c;
        r++;
    }
    buf.setLength(w);
}

/* DEBUG helper: keep only ONE achievement (by ID) in the patch response.
 * Drastically reduces the watchlist and pointer-chain count so we can
 * validate the bootstrap end-to-end without optimising for scale. Set to 0
 * to disable (keep all achievements). */
#ifndef DEBUG_KEEP_ONLY_ACHIEVEMENT_ID
#define DEBUG_KEEP_ONLY_ACHIEVEMENT_ID 1  /* 0 = keep all achievements (Kirby milestone 2026-05-31 was 557557) */
#endif

static void json_keep_only_achievement_id(PsramStream &buf, const uint32_t *target_ids, int n_ids) {
    json_remove_whitespace(buf);

    int total_kept = 0, total_removed = 0;
    int search_from = 0;

    /* The achievementset response has one "Achievements":[] per Set
     * (typically 2 — the core set and the bonus set). Iterate through
     * each independently so the target cheevo is preserved wherever it
     * lives and everything else is dropped. */
    while (search_from < (int)buf.length()) {
        vTaskDelay(1);  // yield to IDLE0 between achievement-set iterations
        int achvStart = buf.indexOf("\"Achievements\":[", search_from);
        if (achvStart == -1) break;
        char *data = buf.data();
        int arrayStart = buf.indexOf("[", achvStart);
        int arrayEnd   = buf.indexOf("]", arrayStart);
        if (arrayStart == -1 || arrayEnd == -1) break;

        int kept = 0, removed = 0;
        int pos = arrayStart + 1;
        while (pos < arrayEnd) {
            vTaskDelay(1);  // yield to IDLE0 between per-achievement iterations
            int objStart = buf.indexOf("{", pos);
            if (objStart == -1 || objStart > arrayEnd) break;
            int objEnd = objStart, braces = 1;
            while (braces > 0 && objEnd < arrayEnd) {
                objEnd++;
                if (data[objEnd] == '{') braces++;
                else if (data[objEnd] == '}') braces--;
            }
            if (objEnd >= arrayEnd) break;

            bool isTarget = false;
            for (int t = 0; t < n_ids && !isTarget; t++) {
                char id_pattern[32];
                snprintf(id_pattern, sizeof(id_pattern), "\"ID\":%lu,", (unsigned long)target_ids[t]);
                size_t id_pattern_len = strlen(id_pattern);
                for (int i = objStart; i < objEnd - (int)id_pattern_len; i++) {
                    if (strncmp(data + i, id_pattern, id_pattern_len) == 0) {
                        isTarget = true;
                        break;
                    }
                }
            }

            if (!isTarget) {
                int removeStart = objStart;
                int removeEnd   = objEnd;  /* inclusive */

                /* Strip surrounding whitespace first */
                while (removeStart > arrayStart
                       && (data[removeStart-1] == ' ' || data[removeStart-1] == '\n'))
                    removeStart--;
                while (removeEnd + 1 < (int)buf.length()
                       && (data[removeEnd+1] == ' ' || data[removeEnd+1] == '\n'))
                    removeEnd++;

                /* Strip leading comma if present, otherwise trailing comma.
                 * Never both, never neither: that's what keeps the array
                 * valid (`[a,b,c]` minus `b` → `[a,c]`, minus `a` → `[b,c]`,
                 * minus everything → `[]`). The previous logic only stripped
                 * trailing comma on the very first removal, leaving stray
                 * `,` after subsequent removals from the array head. */
                if (removeStart > arrayStart && data[removeStart-1] == ',') {
                    removeStart--;
                } else if (removeEnd + 1 < (int)buf.length() && data[removeEnd+1] == ',') {
                    removeEnd++;
                }

                buf.removeRange(removeStart, removeEnd - removeStart + 1);
                arrayEnd = buf.indexOf("]", arrayStart);
                pos = removeStart;
                data = buf.data();
                removed++;
            } else {
                pos = objEnd + 1;
                kept++;
            }
        }
        total_kept    += kept;
        total_removed += removed;
        search_from   = arrayEnd + 1;
    }
        LOG_DBG("DEBUG=Filtered Achievements (all sets): kept=%d removed=%d\r\n",
                total_kept, total_removed);
}

static void json_remove_flags5_achievements(PsramStream &buf) {
    json_remove_whitespace(buf);
    char* data = buf.data();
    int achvStart = buf.indexOf("\"Achievements\":[");
    if (achvStart == -1) return;
    int arrayStart = buf.indexOf("[", achvStart);
    int arrayEnd   = buf.indexOf("]", arrayStart);
    if (arrayStart == -1 || arrayEnd == -1) return;
    int objCount = 0, pos = arrayStart + 1;
    while (pos < arrayEnd) {
        int objStart = buf.indexOf("{", pos);
        if (objStart == -1 || objStart > arrayEnd) break;
        int objEnd = objStart, braces = 1;
        while (braces > 0 && objEnd < arrayEnd) {
            objEnd++;
            if (data[objEnd] == '{') braces++;
            else if (data[objEnd] == '}') braces--;
        }
        if (objEnd >= arrayEnd) break;
        objCount++;
        bool hasFlags5 = false;
        for (int i = objStart; i < objEnd - 8; i++) {
            if (strncmp(data + i, "\"Flags\":5", 9) == 0) { hasFlags5 = true; break; }
        }
        if (hasFlags5) {
            int removeStart = objStart;
            while (removeStart > arrayStart && (data[removeStart-1] == ' ' || data[removeStart-1] == '\n'))
                removeStart--;
            if (data[removeStart-1] == ',') removeStart--;
            if (objCount == 1 && objEnd + 1 < (int)buf.length() && data[objEnd+1] == ',')
                objEnd++;
            buf.removeRange(removeStart, objEnd - removeStart + 1);
            arrayEnd = buf.indexOf("]", arrayStart);
            pos = removeStart;
        } else {
            pos = objEnd + 1;
        }
    }
}

/* EXPERIMENT (v0.27.6) — keep only the first N achievements in the set,
 * to MEASURE how do_frame cost (df_us) scales with achievement count.
 * The SMG set is 1 "core" set with 161 achievements + 48 leaderboards;
 * the leaderboards are already stripped, so the ~26ms do_frame is the
 * 161 achievements' trigger eval. This is a DIAGNOSTIC, not production —
 * the dropped achievements simply won't be monitored. N=0 keeps all.
 * Expected: df_us ≈ 26ms × (N/161); find the N where df_us < ~16ms
 * (df/s ~60) to quantify "cost per achievement" and confirm the ceiling. */
static void json_keep_first_n_achievements(PsramStream &buf, int n) {
    if (n <= 0) return;
    json_remove_whitespace(buf);
    char* data = buf.data();
    int achvStart = buf.indexOf("\"Achievements\":[");
    if (achvStart == -1) return;
    int arrayStart = buf.indexOf("[", achvStart);
    int arrayEnd   = buf.indexOf("]", arrayStart);
    if (arrayStart == -1 || arrayEnd == -1) return;
    int objCount = 0, pos = arrayStart + 1;
    while (pos < arrayEnd) {
        int objStart = buf.indexOf("{", pos);
        if (objStart == -1 || objStart > arrayEnd) break;
        int objEnd = objStart, braces = 1;
        while (braces > 0 && objEnd < arrayEnd) {
            objEnd++;
            if (data[objEnd] == '{') braces++;
            else if (data[objEnd] == '}') braces--;
        }
        if (objEnd >= arrayEnd) break;
        objCount++;
        if (objCount >= n) {
            /* Keep up to & including this object; drop the rest up to ']'. */
            if (objEnd + 1 < arrayEnd)
                buf.removeRange(objEnd + 1, arrayEnd - objEnd - 1);
            LOG_DBG("DEBUG=keep_first_n: kept %d achievements (was more)\r\n", n);
            return;
        }
        pos = objEnd + 1;
    }
    LOG_DBG("DEBUG=keep_first_n: set had <= %d achievements, no change\r\n", n);
}

/* v0.27.6 — keep only the FIRST set in "Sets":[...], dropping bonus/
 * specialty sets. SMG's full achievementsets response has 2 sets: "core"
 * (23895, 161 achievements) + "bonus" (36024, ~109). rcheevos monitors
 * EVERY set's addresses, so the bonus set inflates the watchlist AND the
 * ~26ms do_frame's trigger-eval. Keeping core-only cuts ~40% of the cost
 * → df_us ~16ms → df/s ~60 (the timer fix), at the cost of not tracking
 * bonus-set achievements (a reasonable product choice; core is the main
 * progression set).
 *
 * Robust brace/bracket scan that SKIPS string contents (a stray { } [ ]
 * inside a Title/Description/Mem would otherwise desync the depth count
 * across this huge cut). */
static void json_keep_first_set(PsramStream &buf) {
    json_remove_whitespace(buf);
    char* data = buf.data();
    int len = (int)buf.length();
    int setsKey = buf.indexOf("\"Sets\":[");
    if (setsKey == -1) return;
    int arrayStart = buf.indexOf("[", setsKey);
    if (arrayStart == -1) return;

    /* 1. End of the FIRST set object: first '{' after '[', then
     *    string-aware brace-match until it closes. */
    int i = arrayStart + 1;
    while (i < len && data[i] != '{') i++;
    if (i >= len) return;
    int braces = 0; bool in_str = false; int objEnd = -1;
    for (; i < len; i++) {
        char c = data[i];
        if (in_str) {
            if (c == '\\') { i++; continue; }
            if (c == '"') in_str = false;
            continue;
        }
        if (c == '"') { in_str = true; continue; }
        if (c == '{') braces++;
        else if (c == '}') { if (--braces == 0) { objEnd = i; break; } }
    }
    if (objEnd == -1) return;

    /* 2. The ']' that closes the Sets array (bracket-depth from
     *    arrayStart, string-aware). */
    int depth = 0; in_str = false; int setsEnd = -1;
    for (i = arrayStart; i < len; i++) {
        char c = data[i];
        if (in_str) {
            if (c == '\\') { i++; continue; }
            if (c == '"') in_str = false;
            continue;
        }
        if (c == '"') { in_str = true; continue; }
        if (c == '[') depth++;
        else if (c == ']') { if (--depth == 0) { setsEnd = i; break; } }
    }
    if (setsEnd == -1 || setsEnd <= objEnd) return;

    /* 3. Drop everything between the first set's '}' and the closing ']'. */
    if (objEnd + 1 < setsEnd) {
        buf.removeRange(objEnd + 1, setsEnd - objEnd - 1);
        LOG_DBG("DEBUG=keep_first_set: dropped sets after first (saved %d bytes)\r\n",
                setsEnd - objEnd - 1);
    }
}

/* ───────────────────────────────────────────────────────────────────────
 * v0.27.x — TOTAL-OPERATION BUDGET CAP  (feature toggle, starts ENABLED)
 *
 * do_frame's trigger-eval cost scales with the TOTAL number of conditions
 * ("operations") across every kept achievement, NOT with the achievement
 * count. SMG's core set is ~32768 ops and measured ~25ms (25274us) on the
 * ESP32 — that alone blows past half of a 16.7ms frame and starves the EXI
 * / comm work that must also finish each frame.
 *
 * Run AFTER every other strip, this pass removes achievements from the
 * HEAVIEST (most operations) to the lightest until the running total drops
 * to <= MAX_TOTAL_OPERATIONS (~20000, ~60% of SMG), buying frame
 * headroom at the cost of a handful of the most expensive achievements.
 * It logs (INFO) how many were dropped and names each one.
 *
 * Operation count = conditions in MemAddr: +1 per '_' (AND within a group),
 * +1 per 'S' group separator (NOT the '0xS' bit6 size prefix — that S is
 * preceded by 'x'), +1 for the first condition. Empty MemAddr counts 0.
 * ─────────────────────────────────────────────────────────────────────── */
#ifndef CAP_TOTAL_OPERATIONS
#define CAP_TOTAL_OPERATIONS 0            // 1 = on, 0 = keep every achievement
#endif
#ifndef MAX_TOTAL_OPERATIONS
#define MAX_TOTAL_OPERATIONS 20000        // target ceiling (~60% of SMG's 32768)
#endif

/* Count rcheevos "operations" (conditions) in one achievement object's
 * MemAddr value, scanning only the [objStart, objEnd] span. */
static int count_mem_ops(const char* data, int objStart, int objEnd) {
    static const char* KEY  = "\"MemAddr\":\"";
    static const int   KLEN = 11;
    int p = -1;
    for (int i = objStart; i <= objEnd - KLEN; i++) {
        if (strncmp(data + i, KEY, KLEN) == 0) { p = i + KLEN; break; }
    }
    if (p == -1) return 0;
    if (data[p] == '"') return 0;          // empty MemAddr
    int ops = 1;                           // first condition
    for (int i = p; i < objEnd; i++) {
        char c = data[i];
        if (c == '\\') { i++; continue; }  // skip escaped char
        if (c == '"') break;               // closing quote → end of value
        if (c == '_') ops++;
        else if (c == 'S' && data[i-1] != 'x') ops++;   // group sep, not 0xS
    }
    return ops;
}

/* Copy the "Title" value of one achievement object's [objStart,objEnd] span into
 * out (NUL-terminated, truncated to outsize). Used to name the achievements the
 * op-cap drops so the INFO log lists exactly what was removed. */
static void cap_extract_title(const char* data, int objStart, int objEnd,
                              char* out, int outsize) {
    static const char* KEY  = "\"Title\":\"";
    static const int   KLEN = 9;
    int p = -1, w = 0;
    for (int i = objStart; i <= objEnd - KLEN; i++) {
        if (strncmp(data + i, KEY, KLEN) == 0) { p = i + KLEN; break; }
    }
    if (p != -1) {
        for (int i = p; i < objEnd && w < outsize - 1; i++) {
            char c = data[i];
            if (c == '\\') { if (i + 1 < objEnd && w < outsize - 1) out[w++] = data[++i]; continue; }
            if (c == '"') break;            // closing quote → end of value
            out[w++] = c;
        }
    }
    out[w] = '\0';
    if (w == 0) snprintf(out, outsize, "(no title)");
}

/* Achievements whose Type is "progression" (RA needs ALL) or "win_condition"
 * (RA needs ANY) gate beaten-game / mastery detection — the op-cap must NEVER
 * drop these, even when heavy, or the player can't get credit for finishing.
 * The patch is whitespace-stripped before the cap runs ("Type":"progression"),
 * but tolerate an optional space for safety. Type core/bonus/null = droppable. */
static bool cap_is_protected(const char* data, int objStart, int objEnd) {
    static const char* KEY  = "\"Type\":";
    static const int   KLEN = 7;
    int p = -1;
    for (int i = objStart; i <= objEnd - KLEN; i++) {
        if (strncmp(data + i, KEY, KLEN) == 0) { p = i + KLEN; break; }
    }
    if (p == -1) return false;
    while (p < objEnd && (data[p] == ' ' || data[p] == '\t')) p++;
    if (p >= objEnd || data[p] != '"') return false;   // null / non-string => droppable
    p++;
    return strncmp(data + p, "progression\"",  12) == 0
        || strncmp(data + p, "win_condition\"", 14) == 0;
}

static void json_cap_total_operations(PsramStream &buf, int target_ops) {
    json_remove_whitespace(buf);
    char* data = buf.data();
    int achvStart = buf.indexOf("\"Achievements\":[");
    if (achvStart == -1) return;
    int arrayStart = buf.indexOf("[", achvStart);
    int arrayEnd   = buf.indexOf("]", arrayStart);
    if (arrayStart == -1 || arrayEnd == -1) return;

    /* Pass 1 — enumerate achievement objects: span + operation count + whether
     * it's a protected (progression/win_condition) achievement the cap can't drop. */
    struct AchSpan { int start, end, ops; bool removed; bool prot; };
    const int MAX_ACH = 512;               // SMG core = 161; generous headroom
    AchSpan* spans = (AchSpan*) ps_malloc(sizeof(AchSpan) * MAX_ACH);
    if (!spans) { LOG_ERR("ERROR=cap_ops: ps_malloc failed, skipping\r\n"); return; }

    int count = 0, total_ops = 0, pos = arrayStart + 1;
    int prot_count = 0, prot_ops = 0;
    while (pos < arrayEnd && count < MAX_ACH) {
        int objStart = buf.indexOf("{", pos);
        if (objStart == -1 || objStart > arrayEnd) break;
        int objEnd = objStart, braces = 1;
        while (braces > 0 && objEnd < arrayEnd) {
            objEnd++;
            if (data[objEnd] == '{') braces++;
            else if (data[objEnd] == '}') braces--;
        }
        if (objEnd >= arrayEnd) break;
        bool prot = cap_is_protected(data, objStart, objEnd);
        spans[count] = { objStart, objEnd, count_mem_ops(data, objStart, objEnd), false, prot };
        total_ops += spans[count].ops;
        if (prot) { prot_count++; prot_ops += spans[count].ops; }
        count++;
        pos = objEnd + 1;
    }

    LOG_DBG("DEBUG=cap_ops: %d achievements, %d total operations (target <= %d), %d protected (%d ops)\r\n",
             count, total_ops, target_ops, prot_count, prot_ops);
    if (total_ops <= target_ops) {
        LOG_DBG("DEBUG=cap_ops: already under target, removed 0 achievements\r\n");
        free(spans);
        return;
    }

    /* Pass 2 — greedily mark the heaviest DROPPABLE achievement until the running
     * total drops to <= target. Protected (progression/win_condition) achievements
     * are NEVER candidates — if only protected remain, we stop above target (better
     * to overshoot the op budget than break beaten-game detection). O(n^2) max-find. */
    int removed = 0, removed_ops = 0, min_removed_ops = -1;
    while (total_ops > target_ops) {
        int maxIdx = -1, maxOps = -1;
        for (int i = 0; i < count; i++)
            if (!spans[i].removed && !spans[i].prot && spans[i].ops > maxOps) { maxOps = spans[i].ops; maxIdx = i; }
        if (maxIdx == -1) break;           // nothing droppable left (rest are protected)
        spans[maxIdx].removed = true;
        total_ops   -= spans[maxIdx].ops;
        removed_ops += spans[maxIdx].ops;
        removed++;
        if (min_removed_ops < 0 || spans[maxIdx].ops < min_removed_ops) min_removed_ops = spans[maxIdx].ops;
        /* Name each dropped achievement (INFO). data offsets are still valid here —
         * the physical removeRange happens in Pass 3 below. Heaviest dropped first. */
        {
            char title[80];
            cap_extract_title(data, spans[maxIdx].start, spans[maxIdx].end, title, sizeof(title));
            LOG_INFO("DEBUG=cap_ops removed: \"%s\" (%d ops)\r\n", title, spans[maxIdx].ops);
        }
    }

    /* Surface the protected achievements that protection actually saved — i.e.
     * ones at least as heavy as the lightest we dropped, which the greedy pass
     * WOULD have cut if they weren't progression/win_condition. */
    if (removed > 0) {
        for (int i = 0; i < count; i++) {
            if (spans[i].prot && spans[i].ops >= min_removed_ops) {
                char title[80];
                cap_extract_title(data, spans[i].start, spans[i].end, title, sizeof(title));
                LOG_INFO("DEBUG=cap_ops KEPT (progression/win): \"%s\" (%d ops)\r\n", title, spans[i].ops);
            }
        }
    }

    /* Pass 3 — physically delete the marked objects, HIGHEST offset first so
     * lower spans' offsets stay valid. Eat the comma joining the object to the
     * array (leading comma normally; trailing comma if it's the first one). */
    for (int i = count - 1; i >= 0; i--) {
        if (!spans[i].removed) continue;
        data = buf.data();                 // re-fetch; buffer shrinks each pass
        int rs = spans[i].start, re = spans[i].end;
        if (rs > 0 && data[rs-1] == ',')                          rs--;
        else if (re + 1 < (int)buf.length() && data[re+1] == ',') re++;
        buf.removeRange(rs, re - rs + 1);
    }

    free(spans);
    LOG_INFO("DEBUG=cap_ops: removed %d achievements (%d ops), %d remain (%d ops), %d protected kept%s\r\n",
             removed, removed_ops, count - removed, total_ops, prot_count,
             (total_ops > target_ops) ? " [target unreachable — protected floor]" : "");
}

// Apply all strips to a patch.php response held in PSRAM
static void json_strip_patch(PsramStream &buf) {
    vTaskDelay(1); json_remove_whitespace(buf);
    vTaskDelay(1); json_remove_field(buf,      "Warning");
    vTaskDelay(1); json_remove_field(buf,      "BadgeLockedURL");
    vTaskDelay(1); json_remove_field(buf,      "BadgeURL");
    vTaskDelay(1); json_remove_field(buf,      "ImageIconURL");
    vTaskDelay(1); json_remove_field(buf,      "Rarity");
    vTaskDelay(1); json_remove_field(buf,      "RarityHardcore");
    vTaskDelay(1); json_remove_field(buf,      "Author");
    vTaskDelay(1); json_remove_field(buf,      "RichPresencePatch");
    vTaskDelay(1); json_clean_field_array(buf, "Leaderboards");
    vTaskDelay(1); json_remove_flags5_achievements(buf);
#if CAP_TOTAL_OPERATIONS
    vTaskDelay(1); json_cap_total_operations(buf, MAX_TOTAL_OPERATIONS);
#endif
}

// ============================================================================
// HTTP — async worker on Core 0 (v0.24.0)
//
// rc_client lives on ONE task (loop, Core 1 — same core as the EXI task).
// HTTP used to run synchronously inside server_call, which meant the
// mbedTLS handshake (~2-3s of crypto), the achievementsets download AND
// the PSRAM JSON strip all executed on Core 1 — starving the EXI task
// (field: SNAP fires dropped to 97/300 during award submits; historically
// TWDT reboots when loop lived on Core 0 next to the WiFi task).
//
// New shape:
//   server_call (Core 1)  → deep-copies the request into an http_job_t and
//                           enqueues it. Returns immediately — do_frame
//                           never blocks on the network again.
//   httpTask   (Core 0)   → dequeues, runs the blocking HTTPClient/TLS/
//                           strip work next to the WiFi/lwIP tasks it
//                           talks to, enqueues an http_done_t.
//   loop       (Core 1)   → drains done-queue and invokes the rc_client
//                           callback there, keeping rc_client single-task.
//
// Generation guard: loadGame destroys/recreates g_client; any in-flight
// response from the previous client generation is discarded on drain
// instead of calling a callback into freed memory.
// ============================================================================
/* http_job_t / http_done_t live in ra_http.h — the Arduino preprocessor
 * hoists its auto-generated function prototypes above any typedef written
 * in the .ino body, so types used in function signatures must come from
 * an #include or the build fails with "does not name a type". */
static QueueHandle_t http_req_q  = NULL;
static QueueHandle_t http_done_q = NULL;
static volatile uint32_t http_gen = 0;

/* Hand a finished response body to the done-queue (called by httpTask). */
static void http_finish(const http_job_t *job, const char *body, size_t body_len, int status) {
    http_done_t done;
    done.body = (char *)ps_malloc(body_len + 1);
    if (!done.body) {
        LOG_ERR("ERROR=http_finish: ps_malloc(%u) failed\r\n", (unsigned)body_len + 1);
        return;
    }
    memcpy(done.body, body, body_len);
    done.body[body_len] = '\0';
    done.body_len      = body_len;
    done.status        = status;
    done.callback      = job->callback;
    done.callback_data = job->callback_data;
    done.gen           = job->gen;
    if (xQueueSend(http_done_q, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        LOG_ERR("ERROR=http_finish: done queue full, response dropped\r\n");
        free(done.body);
    }
}

// HTTP blocking worker body (runs ONLY on httpTask, Core 0).
static void server_call_blocking(const http_job_t *job) {
    // Large responses: rcheevos 11.x uses r=patch, 12.x uses r=achievementsets.
    // Both can be ~500-700KB for games like SSBM.
    // rcheevos puts these in the URL as query params (not in post_data).
    bool is_patch = (strstr(job->url, "r=patch")           != nullptr) ||
                    (strstr(job->url, "r=achievementsets")  != nullptr) ||
                    (job->post_data && strstr(job->post_data, "r=patch")           != nullptr) ||
                    (job->post_data && strstr(job->post_data, "r=achievementsets") != nullptr);
    LOG_DBG("DEBUG=server_call is_patch=%d url=%s data=%s\r\n", (int)is_patch, job->url, job->post_data);

    client.setInsecure();
    https.begin(client, String(job->url));
    /* RA's dorequest can be slow; the default 5s read timeout produced
     * spurious -11 (HTTPC_ERROR_READ_TIMEOUT). 15s: TLS handshake
     * (~2-3s on ESP32) + request + response. Off the hot path since the
     * v0.24 core split — blocking longer here costs nothing on Core 1. */
    https.setTimeout(15000);
    /* Robustness — force a fresh TCP+TLS connection per request. With
     * keep-alive on, a connection left idle (long menu time before launching
     * a game) is silently dropped by the server/NAT, and the reused socket
     * fails on first use ("first request after idle breaks"). The handshake
     * costs ~2-3s but runs on Core 0, off the do_frame hot path. */
    https.setReuse(false);
    https.setUserAgent("WII_RA_ADAPTER/0.1 rcheevos/12.3");
    uint32_t req_start_ms = millis();
    int httpCode = 0;
    if (job->post_data) {
        https.addHeader("Content-Type", "application/x-www-form-urlencoded");
        httpCode = https.POST(job->post_data);
    } else {
        httpCode = https.GET();
    }

    if (httpCode != HTTP_CODE_OK) {
        /* Same semantics as the old synchronous path: no callback on HTTP
         * failure — rcheevos' own retry/ping cadence re-issues the call.
         * Forensics (error path only): elapsed discriminates fast-fail
         * (connect/TLS/power-save wakeup) from a genuine read timeout;
         * heap catches mbedTLS allocation failure (~45KB/session needed);
         * RSSI/status catch RF or association drops. */
        LOG_ERR("DEBUG=HTTP error: %d url=%s elapsed=%lums heap=%u maxblk=%u rssi=%d wifi=%d\r\n",
                httpCode, job->url, (unsigned long)(millis() - req_start_ms),
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (int)WiFi.RSSI(), (int)WiFi.status());
        https.end();
        return;
    }

    int contentLen = https.getSize();  // -1 if chunked/unknown
    LOG_DBG("DEBUG=HTTP ok is_patch=%d content-length=%d psram-free=%u heap-free=%u\r\n",
            (int)is_patch, contentLen, (unsigned)ESP.getFreePsram(), (unsigned)ESP.getFreeHeap());

    if (is_patch) {
        // ----------------------------------------------------------------
        // Large response: allocate in PSRAM, read fully, strip, callback
        // ----------------------------------------------------------------
        /* Wii patch responses are bigger than GameCube/NES — SMG and similar games
         * exceed 800KB. Allocate generously from PSRAM (8MB total, ~7.5MB free here),
         * and cap at 3MB which is still well within budget. */
        size_t buf_size = (contentLen > 0) ? (size_t)(contentLen + 4096) : 2UL * 1024 * 1024;
        if (buf_size > 3UL * 1024 * 1024) buf_size = 3UL * 1024 * 1024;

        PsramStream ps;
        if (!ps.reserve(buf_size)) {
            LOG_ERR("ERROR=PSRAM alloc failed (%u bytes), psram-free=%u\r\n",
                    (unsigned)buf_size, (unsigned)ESP.getFreePsram());
            https.end();
            return;
        }

        // Large response: read in a loop with vTaskDelay(1) between chunks
        // so IDLE0 gets CPU time and can reset its TWDT subscription.
        // Implements the same chunked-transfer and identity encoding logic
        // as HTTPClient::writeToStream but yields every iteration.
        int written = 0;
        bool dl_stalled = false;
        {
            WiFiClient *dl_stream = https.getStreamPtr();
            /* Two independent limits guard the download:
             *  - dl_deadline: absolute cap for a well-behaved but slow link.
             *  - stall guard: abort shortly after the LAST byte arrives. A
             *    TLS-level failure mid-download (mbedTLS -29184 "invalid SSL
             *    record") leaves https.connected() lying true while
             *    readStringUntil/readBytes return empty INSTANTLY on the dead
             *    fd. Without this guard the loop spins re-arming SO_RCVTIMEO on
             *    a closed socket, flooding "setSocketOption EBADF" to the UART
             *    until the 120s deadline — starving IDLE0 → task watchdog, and
             *    (TWDT panic off) wedging the box until a manual reset. */
            const uint32_t dl_deadline = millis() + 120000UL;  // 120s hard cap
            const uint32_t DL_STALL_MS = 4000;                 // no-progress abort (< 5s TWDT)
            uint32_t last_rx_ms = millis();
            int empty_hdrs = 0;                                // fast TLS-death detector
            if (contentLen > 0) {
                // Identity encoding: known length, read in blocks
                uint8_t dl_buf[4096];
                while (written < contentLen && millis() < dl_deadline) {
                    vTaskDelay(1);  // yield to IDLE0 every iteration
                    int avail = dl_stream->available();
                    if (avail > 0) {
                        int n = dl_stream->readBytes(dl_buf,
                                    (size_t)min(avail, (int)sizeof(dl_buf)));
                        if (n > 0) { ps.write(dl_buf, n); written += n; last_rx_ms = millis(); }
                    } else if (!https.connected()) break;
                    if (millis() - last_rx_ms > DL_STALL_MS) { dl_stalled = true; break; }
                }
            } else {
                // Chunked transfer encoding: parse chunk-size headers manually
                uint8_t dl_buf[4096];
                while (millis() < dl_deadline) {
                    vTaskDelay(1);  // yield to IDLE0 every chunk
                    if (!https.connected() && !dl_stream->available()) break;
                    if (millis() - last_rx_ms > DL_STALL_MS) { dl_stalled = true; break; }
                    String chunk_hdr = dl_stream->readStringUntil('\n');
                    chunk_hdr.trim();
                    if (chunk_hdr.length() == 0) {
                        /* Healthy-but-slow link: readStringUntil blocks up to its
                         * ~1s stream timeout before returning empty. Dead TLS fd:
                         * it returns instantly, so a burst of empties == the
                         * connection is gone. Break fast (well under DL_STALL_MS)
                         * to keep the EBADF flood to a handful of lines. */
                        if (++empty_hdrs > 64) { dl_stalled = true; break; }
                        continue;
                    }
                    empty_hdrs = 0;
                    int chunk_sz = (int)strtol(chunk_hdr.c_str(), NULL, 16);
                    if (chunk_sz == 0) break;  // final chunk
                    int remaining = chunk_sz;
                    while (remaining > 0 && millis() < dl_deadline) {
                        vTaskDelay(1);
                        int n = dl_stream->readBytes(dl_buf,
                                    (size_t)min(remaining, (int)sizeof(dl_buf)));
                        if (n > 0) { ps.write(dl_buf, n); written += n; remaining -= n; last_rx_ms = millis(); }
                        else if (!https.connected()) break;
                        if (millis() - last_rx_ms > DL_STALL_MS) { dl_stalled = true; break; }
                    }
                    if (dl_stalled) break;
                    dl_stream->readStringUntil('\n');  // consume trailing CRLF
                }
            }
        }
        if (dl_stalled) {
            /* Forced teardown: the fd is already EBADF, but stop() clears the
             * NetworkClientSecure state so the next request opens a clean
             * socket. No callback → rcheevos re-issues on its ping cadence. */
            LOG_ERR("DEBUG=HTTP patch stalled after %d bytes (TLS drop mid-download?), aborting; heap=%u rssi=%d wifi=%d\r\n",
                    written, (unsigned)ESP.getFreeHeap(), (int)WiFi.RSSI(), (int)WiFi.status());
            client.stop();
            https.end();
            return;
        }
        LOG_DBG("DEBUG=Patch read: %d bytes, stripping (psram-free=%u)...\r\n",
                written, (unsigned)ESP.getFreePsram());
        size_t before = ps.length();


#if DEBUG_KEEP_ONLY_ACHIEVEMENT_ID
        /* DEBUG: filter to a single achievement AND drop RichPresence +
         * Leaderboards so rcheevos doesn't pull in their address sets.
         * Watchlist size shrinks dramatically (from ~671 to ~20) which
         * makes the multi-pass bootstrap fit in 1 ADDR_RESPONSE round
         * and the live `KIRBY ...` dump readable. Leaderboards alone
         * pull in dozens of memrefs per set on Kirby. */
        //static const uint32_t keep_ids[] = { 557557, 577564, 557558 };
        //json_keep_only_achievement_id(ps, keep_ids, 3);
        vTaskDelay(1); json_remove_whitespace(ps);
        /* v0.27.6 — drop bonus/specialty sets, keep only "core". Cuts the
         * SMG do_frame ~40% (270→161 achievements) → df/s toward 60, the
         * timer fix. Set KEEP_ONLY_FIRST_SET to 0 to monitor every set. */
        #ifndef KEEP_ONLY_FIRST_SET
        #define KEEP_ONLY_FIRST_SET 1
        #endif
        #if KEEP_ONLY_FIRST_SET
        vTaskDelay(1); json_keep_first_set(ps);
        #endif
        /* 2026-07-03 EXPERIMENT (user request): keep Leaderboards + RichPresence
         * in the payload to measure the eval/upd/Phase-C cost of their extra
         * conditions (post queue+Phase D the system has headroom to try).
         * RA_KEEP_LB_RP 0 restores the historical strip. Watch on boot: the
         * "Set ..." line (should show 48 leaderboards on SMG), census/table
         * growth vs the d2x cap (RA_MAX_CHAIN_NODES, bumped 2048->3072), and
         * cc= in the PhaseC line — cc < shipped means the bigger blob pushed
         * the window into rotation (the freshness guard only protects flat). */
        #ifndef RA_KEEP_LB_RP
        #define RA_KEEP_LB_RP 1
        #endif
        #if !RA_KEEP_LB_RP
        vTaskDelay(1); json_clean_field_str(ps,   "RichPresencePatch");
        vTaskDelay(1); json_clean_field_array(ps, "Leaderboards");
        #endif
        vTaskDelay(1); json_remove_field(ps,      "Warning");
        vTaskDelay(1); json_remove_field(ps,      "BadgeLockedURL");
        vTaskDelay(1); json_remove_field(ps,      "BadgeURL");
        vTaskDelay(1); json_remove_field(ps,      "ImageIconURL");
        vTaskDelay(1); json_remove_field(ps,      "Rarity");
        vTaskDelay(1); json_remove_field(ps,      "RarityHardcore");
        vTaskDelay(1); json_remove_field(ps,      "Author");
        vTaskDelay(1); json_remove_flags5_achievements(ps);

        /* v0.27.x — cap the total operation budget by dropping the heaviest
         * achievements until total ops <= MAX_TOTAL_OPERATIONS. Runs LAST so
         * it sees the final kept set. Toggle via CAP_TOTAL_OPERATIONS. */
#if CAP_TOTAL_OPERATIONS
        vTaskDelay(1); json_cap_total_operations(ps, MAX_TOTAL_OPERATIONS);
#endif

        // print ths shrinked ps
        LOG_DBG("DEBUG=Patch after stripping fields: size=%u bytes\r\n", (unsigned)ps.length());

        /* v0.27.6 EXPERIMENT — cap the achievement count to measure the
         * do_frame cost curve. Set EXPERIMENT_KEEP_FIRST_N>0 to limit;
         * 0 = keep all. Watch df_us in the CATCHUP log: it should scale
         * ~linearly with the kept count. DIAGNOSTIC ONLY — capped builds
         * don't monitor the dropped achievements. */
        #ifndef EXPERIMENT_KEEP_FIRST_N
        #define EXPERIMENT_KEEP_FIRST_N 0
        #endif
        #if EXPERIMENT_KEEP_FIRST_N > 0
        json_keep_first_n_achievements(ps, EXPERIMENT_KEEP_FIRST_N);
        #endif
#endif
        /* Hand off to the done-queue — the rc_client callback runs on the
         * loop task (Core 1), not here. ps destructor frees its PSRAM. */
        http_finish(job, ps.c_str(), ps.length(), httpCode);
    } else {
        // ----------------------------------------------------------------
        // Small response (login, resolve_hash, etc.) — use heap String
        // ----------------------------------------------------------------
        WiFiClient *stream = https.getStreamPtr();
        String responseStr;
        if (contentLen > 0) responseStr.reserve(contentLen + 1);
        uint8_t chunk[512];
        int total_read = 0;
        uint32_t deadline = millis() + 15000;
        uint32_t last_rx_ms = millis();   // stall guard (same TLS-drop rationale as patch path)
        while ((https.connected() || stream->available()) && millis() < deadline) {
            vTaskDelay(1);  // yield to IDLE0 so it can reset its TWDT subscription
            int avail = stream->available();
            if (avail > 0) {
                int n = stream->readBytes(chunk, min(avail, (int)sizeof(chunk)));
                responseStr.concat((const char*)chunk, n);
                total_read += n;
                last_rx_ms = millis();
            } else {
                delay(1);
            }
            if (contentLen > 0 && total_read >= contentLen) break;
            if (millis() - last_rx_ms > 4000) break;   // no progress: connection gone
        }
        LOG_DBG("DEBUG=HTTP body: declared=%d received=%d\r\n", contentLen, total_read);

        http_finish(job, responseStr.c_str(), responseStr.length(), httpCode);
    }

    https.end();
}

/* rc_client server-call hook — Core 1. Deep-copies the request (rcheevos
 * frees it as soon as we return) and enqueues; never blocks on the net. */
static void server_call(const rc_api_request_t *request, rc_client_server_callback_t callback,
                        void *callback_data, rc_client_t *rc_client) {
    http_job_t job;
    memset(&job, 0, sizeof(job));
    job.url           = strdup(request->url);
    job.post_data     = request->post_data ? strdup(request->post_data) : NULL;
    job.callback      = callback;
    job.callback_data = callback_data;
    job.gen           = http_gen;
    if (!job.url || (request->post_data && !job.post_data) ||
        xQueueSend(http_req_q, &job, pdMS_TO_TICKS(2000)) != pdTRUE) {
        LOG_ERR("ERROR=server_call: enqueue failed, request dropped\r\n");
        free(job.url);
        free(job.post_data);
    }
}

/* HTTP worker task — pinned to Core 0, beside the WiFi/lwIP tasks. All
 * blocking network work (TLS handshake, download, JSON strip) lives here;
 * Core 1 (EXI + rc_client) never waits on it. */
static void httpTask(void *pvParameters) {
    http_job_t job;
    for (;;) {
        if (xQueueReceive(http_req_q, &job, portMAX_DELAY) == pdTRUE) {
            server_call_blocking(&job);
            free(job.url);
            free(job.post_data);
        }
    }
}

/* Drain finished HTTP responses — called from loop() so every rc_client
 * callback executes on the same task as do_frame. Discards responses from
 * a previous client generation (g_client was destroyed/recreated). */
static void http_drain_done(void) {
    http_done_t done;
    while (http_done_q && xQueueReceive(http_done_q, &done, 0) == pdTRUE) {
        if (done.gen == http_gen && done.callback) {
            rc_api_server_response_t server_response;
            memset(&server_response, 0, sizeof(server_response));
            server_response.body             = done.body;
            server_response.body_length      = done.body_len;
            server_response.http_status_code = done.status;

            /* v0.27.8 — measure what the rcheevos internal structure costs.
             * For the big achievementsets response (>100KB), the callback
             * parses it and allocates all the trigger/memref structs; the
             * before/after delta is the resident cost of the game's set. */
            bool big = done.body_len > 100000;
            uint32_t sram_b = 0, psram_b = 0;
            if (big) {
                sram_b  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
                psram_b = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            }
            done.callback(&server_response, done.callback_data);
            if (big) {
                int32_t sram_used  = (int32_t)sram_b  - (int32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
                int32_t psram_used = (int32_t)psram_b - (int32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                LOG_INFO("DEBUG=rcheevos struct cost: body=%uKB sram=%+dKB psram=%+dKB (after: sram-free=%uKB psram-free=%uKB)\r\n",
                         (unsigned)(done.body_len / 1024),
                         (int)(sram_used / 1024), (int)(psram_used / 1024),
                         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
            }
        } else if (done.gen != http_gen) {
            LOG_DBG("DEBUG=http_drain: stale-generation response discarded\r\n");
        }
        free(done.body);
    }
}

// ============================================================================
// EXI Protocol Handler - processes commands from Wii (ra-module / WiiFlow)
// ============================================================================

/* Helper: send an esp_header response with 4 bytes of 0xFF padding prepended.
 * For POLL/STATUS/IDENTIFY/GAME_RESET — all of which use a 4-byte GC request
 * (ra_gc_header_t = magic+cmd+payload_len). The GC consumes the first 4 bytes
 * of the tx_buf during its write phase, then samples the esp_header starting
 * at byte 4 of the read phase. Without the padding, GC reads
 * esp_header.data_len (bytes 4-5 of the struct) and rejects the response.
 *
 * Necessary in the new exi_spi_arm() architecture: previously the cb-at-queue
 * pattern left tx_buf populated with default_ack (which already contained the
 * 4-byte FF padding); now prepare_response writes tx_buf directly, so the
 * caller must include the padding explicitly. GET_CHUNK already builds 6-byte
 * padding inside its buffer (6-byte request); the helper here is only for the
 * common 4-byte-request case. */
static void send_esp_header_response_4pad(const ra_esp_header_t *hdr) {
    uint8_t padded[4 + sizeof(ra_esp_header_t)];
    memset(padded, 0xFF, 4);
    memcpy(padded + 4, hdr, sizeof(*hdr));
    exi_spi_prepare_response(padded, sizeof(padded));
}

/* Multi-pass ADDR_QUERY state. ESP32 iterates rc_memrefs_get_addresses
 * → identify missing addrs → ADDR_QUERY round → ADDR_RESP populates
 * cache → repeat, until the memrefs walk reports no further missing
 * addresses. Only THEN do we set new_snapshot=true so do_frame runs
 * with all real values cached. This avoids the trigger-state advance
 * caused by returning 0 for cache misses (see trigger.c:243 WAITING
 * semantics — first-frame TRUE triggers reset to WAITING, but only if
 * the trigger sees real values; cache-miss zeros transition the
 * trigger to ACTIVE prematurely and the NEXT frame fires falsely). */
static const size_t SNAP_REQ_LEN_PER_VALUE = 1;  /* 1 byte per addr */

/* Index in query_addrs[] where the most recent ADDR_QUERY round
 * started — values from the next ADDR_RESPONSE fill from this index. */
static uint16_t pending_query_start = 0;

/* Hard cap on convergence iterations per snapshot. SMG's deepest
 * pointer chains are ~6 levels; 12 leaves plenty of margin while
 * keeping a runaway loop from stalling the ESP32 forever. */
#define ADDR_QUERY_MAX_ITERATIONS  12

/* v0.27.8 — per-vblank mutation budget. Each ADDR_QUERY round that
 * discovers new addresses mints one APPEND mutation; the d2x MUT_RING
 * holds 8, so we keep resolving (more rounds, deeper chains) within ONE
 * vblank until either convergence (new_missing==0) OR this many mutations
 * have been minted this frame — whichever comes first. Replaces the
 * iteration cap as the practical stop condition. Mutations minted this
 * frame = wl_seq - wl_seq_at_snap_start. */
#define MAX_MUTATIONS_PER_VBLANK   8
static uint8_t addr_query_iter_count = 0;

/* Finalise per-vblank telemetry and arm processSnapshot.
 * Every code path that triggers a frame evaluation calls this instead of
 * writing new_snapshot = true directly, so stats are always consistent. */
static inline void set_new_snapshot(void) {
    g_vblank_stats.addr_proc_us           = (uint32_t)(esp_timer_get_time()
                                              - g_vblank_stats.snap_start_us);
    g_vblank_stats.iter_depth             = addr_query_iter_count;
    g_vblank_stats.pre_doframe_miss_count = peek_miss_count;
    g_vblank_stats.had_mutation           = (wl_seq != g_vblank_stats.wl_seq_at_snap_start)
                                            ? 1u : 0u;
    /* Freeze a consistent copy for processSnapshot's LOG_FRAME — reached ONLY on
     * a converged frame, so g_frame_log always reflects a real do_frame and is
     * immune to later ok=0 snapshots resetting g_vblank_stats. */
    memcpy(&g_frame_log, (const void*)&g_vblank_stats, sizeof(g_frame_log));
    g_frame_log_fc = frame_counter;
    new_snapshot = true;
}

/* Run rc_memrefs_get_pending_addresses with the current cache (hash) and
 * APPEND any NEW addresses (not already in query_addrs and not in watchlist)
 * to query_addrs[query_count..]. Returns the number of new addresses
 * appended. The caller decides whether to send an ADDR_QUERY (return > 0)
 * or to set new_snapshot=true (return == 0).
 *
 * v0.28 — switched from rc_memrefs_get_addresses (which resolves chains
 * against parent->address, frozen since the last do_frame, so it only ever
 * exposes ONE level per frame → it=2) to rc_memrefs_get_pending_addresses,
 * which walks each chain from its static root through the live hash and
 * reports the FIRST uncached level. Each ADDR_RESPONSE commits that level to
 * the hash, so the NEXT collect round exposes the level below it. Looping
 * until it returns 0 resolves an entire multi-level chain within a single
 * vblank. is_cached_cb decides how far each chain can already be walked. */
/* ===========================================================================
 * #1 — Chain-input change gate (kills the periodic 26ms collect_missing force)
 *
 * collect_missing_addresses re-walks ALL ~1330 addAddress pointer chains. The
 * v0.27.3 lazy scheme skips it in steady state, but a periodic FORCE (every 60
 * frames) re-walked them anyway as a safety net — 26ms even when nothing moved
 * (cm=0): the ~1s hitch in the FRAME log.
 *
 * A chain's leaf address only MOVES when a pointer it dereferences changes. We
 * extract the STATIC roots of every chain once (the fixed pointer-base
 * addresses) and, each snapshot, cheaply (O(count) byte-compare) check whether
 * any root changed. If none did, the force is pure waste -> skip it.
 *
 * SAFETY: optimisation, not a correctness change. Intermediate-pointer rewrites
 * (root stable, a pointer at a dynamic address rewritten) are still caught
 * reactively by peek_miss + the v0.32 do_frame gate (1-frame latency; the frame
 * DEFERS instead of reading 0). A missed/mis-extracted root degrades to exactly
 * today's peek_miss behaviour, and a large FORCE stays as an ultimate backstop.
 * Foundation for #2 (root -> chains reverse index for O(changed) resolution). */
#define CHAIN_ROOTS_MAX 4096
static uint32_t *g_root_baddrs = nullptr;   /* PSRAM: sorted byte-addresses of chain roots */
static uint16_t  g_root_baddr_count = 0;
static bool      g_roots_built = false;
/* #1 TOGGLE — the chain-root change gate (inputs_changed drives collect; force
 * backstop relaxes 60->600 to kill the steady-state 26ms hitch). ON by default.
 * EXONERATED 2026-06-15: the galaxy-change freeze was NOT this — bisect proved it
 * froze with this OFF too, and reverting PSRAM/flash 120M->80M fixed it (the
 * convergence storm tipped the 120MHz MSPI/EXI timing). My earlier "60-force is
 * load-bearing" hypothesis was a false culprit. Set false to A/B the steady-state
 * hitch. (The transition storm is a separate, d2x-side convergence issue.) */
static volatile bool g_chain_gate_enabled = true;

static int root_cmp_u32(const void *a, const void *b) {
    uint32_t x = *(const uint32_t*)a, y = *(const uint32_t*)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

static uint8_t root_memsize_bytes(uint8_t size) {
    switch (size) {
        case RC_MEMSIZE_16_BITS: case RC_MEMSIZE_16_BITS_BE:
            return 2;
        case RC_MEMSIZE_24_BITS:  case RC_MEMSIZE_24_BITS_BE:
        case RC_MEMSIZE_32_BITS:  case RC_MEMSIZE_32_BITS_BE:
        case RC_MEMSIZE_FLOAT:    case RC_MEMSIZE_FLOAT_BE:
        case RC_MEMSIZE_MBF32:    case RC_MEMSIZE_MBF32_LE:
        case RC_MEMSIZE_DOUBLE32: case RC_MEMSIZE_DOUBLE32_BE:
            return 4;
        default:
            return 1;  /* 8-bit, bits, nibbles, bitcount */
    }
}

/* Walk every addAddress chain to its STATIC root memref and record the bytes
 * that root reads. One-time (the rcheevos memref structure is fixed per game).
 * Leaves g_roots_built=false if memrefs aren't ready yet (retry next snapshot). */
static void build_chain_roots(void) {
    const rc_memrefs_t *memrefs = (state == STATE_ACTIVE && g_client)
                                  ? rc_client_get_memrefs(g_client) : nullptr;
    if (!memrefs) return;   /* not ready — retry */

    if (!g_root_baddrs) {
        g_root_baddrs = (uint32_t*)ps_malloc(CHAIN_ROOTS_MAX * sizeof(uint32_t));
        if (!g_root_baddrs) { g_roots_built = true; return; }  /* OOM: gate off, FORCE backstop stays */
    }
    g_root_baddr_count = 0;

    const rc_modified_memref_list_t *ml = &memrefs->modified_memrefs;
    while (ml) {
        for (uint16_t i = 0; i < ml->count; i++) {
            const rc_modified_memref_t *mm = &ml->items[i];
            if (mm->modifier_type != RC_OPERATOR_INDIRECT_READ) continue;  /* only pointer derefs */
            /* descend the parent chain to the static (non-modified) root */
            const rc_operand_t *p = &mm->parent;
            int guard = 0;
            while (rc_operand_is_memref(p)
                   && p->value.memref->value.memref_type == RC_MEMREF_TYPE_MODIFIED_MEMREF
                   && guard++ < 24) {
                p = &((const rc_modified_memref_t*)p->value.memref)->parent;
            }
            if (rc_operand_is_memref(p)) {
                uint32_t addr = p->value.memref->address;
                uint8_t  nb   = root_memsize_bytes(p->value.memref->value.size);
                for (uint8_t b = 0; b < nb && g_root_baddr_count < CHAIN_ROOTS_MAX; b++)
                    g_root_baddrs[g_root_baddr_count++] = addr + b;
            }
        }
        ml = ml->next;
    }

    if (g_root_baddr_count > 1) {   /* sort + dedup for binary-search membership */
        qsort(g_root_baddrs, g_root_baddr_count, sizeof(uint32_t), root_cmp_u32);
        uint16_t w = 1;
        for (uint16_t r = 1; r < g_root_baddr_count; r++)
            if (g_root_baddrs[r] != g_root_baddrs[w - 1]) g_root_baddrs[w++] = g_root_baddrs[r];
        g_root_baddr_count = w;
    }
    g_roots_built = true;
    LOG_DBG("DEBUG=chain-root gate: %u pointer-base byte-addrs extracted\r\n",
             (unsigned)g_root_baddr_count);
}

static inline bool is_chain_root(uint32_t baddr) {
    int lo = 0, hi = (int)g_root_baddr_count - 1;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        uint32_t v = g_root_baddrs[mid];
        if (v == baddr) return true;
        if (v < baddr) lo = mid + 1; else hi = mid - 1;
    }
    return false;
}

/* DRY-UPDATE diagnostic (project_exi_robust_handshake): defined in rc_client.c.
 * Runs the evaluator's exact read path on the current cache with save/rollback,
 * populating peek_miss as a side effect — used to tell resolver logic-divergence
 * from frame-latency. Toggle for A/B; off = zero overhead. */
extern "C" void rc_client_diag_dry_update(rc_client_t* client);
static volatile bool g_diag_dry_update = false;  /* OFF for the servicer build — adds ~7ms in taskCore1 */

/* STEP 0 diagnostic (project_collect_resolver_handoff) — "cap collect to 0 walks"
 * causal test. When TRUE, collect_missing_addresses() returns 0 immediately,
 * BEFORE the rc_memrefs_get_pending_addresses walk — so the EXI servicer does
 * ZERO collect CPU (apc_us/cm_cpu_us -> ~0) and convergence is allowed to LAG.
 * Correctness is backstopped by the v0.32 do_frame gate + peek_miss (a missed
 * address -> DEFERred frame, never a false unlock), so this is safe to A/B on
 * real hardware. A/B PROTOCOL: build with FALSE (baseline = current behavior),
 * flash, capture the SAME bunny chase; then build with TRUE (collect capped),
 * flash, capture the SAME scene. Compare CATCHUP df/s in the storm windows:
 *   - dip DISAPPEARS with cap ON  -> the dip is collect-preemption  -> lever B.
 *   - dip PERSISTS  with cap ON   -> residual is the eval spikes (apc already 0
 *                                    on those frames) -> collect levers can't fix
 *                                    it alone; pivot decision. */
static volatile bool g_collect_cap_walks = false;  /* STEP 0 hard cap — OFF (was the confounded test) */

/* lever B (project_collect_resolver_handoff) — per-VBLANK collect walk budget.
 * 0 = OFF (baseline = coelho-capoff.log). >0 caps cumulative chain walks per
 * vblank: the rare mega-spike (rebuild/scene-change walking all ~1373 chains =
 * ~42ms apc) is clipped to ~K x 31us and the remainder carries to the next
 * vblank (left dirty; do_frame gate + peek_miss backstop, never a false unlock).
 * Normal collect (~40 walks) stays under K -> identical to baseline. K=250 ->
 * ~7.7ms apc ceiling. Pushed to g_rc_collect_walk_budget each vblank so it can
 * be A/B'd at runtime. EXPECT: collect-spike half of the dip lifts (apc/cyc
 * spikes collapse, cwk caps at ~250 in the FRAME line); eval-spikes (apc=0)
 * are untouched (separate lever). A/B: 250 vs 0, SAME bunny chase. */
/* 2026-07-02 RETIRED (0 = uncapped): Phase C shrank the walk population from
 * ~1373 chains (the 42ms rebuilds this cap existed for) to ~93 dp/exotic
 * chains — uncapped worst is now ~93 walks x ~31us x rounds ≈ 3ms/round,
 * inside the old K=250 ceiling anyway. The cap's only remaining effect was
 * NEGATIVE for hardcore fidelity: dirty chains carried across frames (leaf
 * values misaligned with their snapshot's do_frame) + self-inflicted gate
 * defers (wii5.log: cwk=250 hit twice, dfr=3). Set back to 250 only if a
 * future game shows big legacy populations (census elig% tells you at load). */
static volatile int g_collect_walk_budget = 0;

/* ====================================================================
 * opt: reverse-hash incremental collect (project_incremental_collect)
 * --------------------------------------------------------------------
 * collect_missing_addresses re-walks ~1938 INDIRECT chains every vblank
 * (~27ms in heavy scenes -> df/s 30-52). Most chains are stable. This
 * reverse index lets us dirty only the chains reading a CHANGED address
 * so rc_memrefs_get_pending_addresses skips the rest (incr_clean flag).
 *
 *   incr_rev_head[watch_index] -> singly-linked list of chains (opaque
 *   rc_modified_memref_t*) that READ that address. Built during the walk
 *   via incr_chain_read (registered as g_rc_chain_read_cb). Per snapshot,
 *   incr_mark_dirty() walks snap_changed[] and dirties reverse[i]'s chains.
 *   A full rebuild (watchlist mutated / every N vblanks / pool overflow)
 *   resets the index and dirties every chain. The v0.32 do_frame gate is
 *   the correctness backstop: a wrongly-skipped chain whose moved leaf is
 *   never queried -> do_frame read misses -> DEFER (no false unlock). */
/* Pool sized for byte-granular records: each chain read contributes num_bytes
 * (widx,chain) pairs (snap_changed is per-byte, so a u32 leaf needs all 4 widxs
 * recorded). ~2000 chains x ~3 levels x ~4 bytes ~= 24k worst case. Generous so
 * the steady state never overflow-churns; overflow just forces a safe rebuild. */
#define INCR_REV_POOL_MAX 40000      /* (watch_index,chain) pair capacity */
#define INCR_REBUILD_EVERY 0         /* TIME-BASED periodic rebuild cadence (vblanks); 0 = DISABLED.
                                      * 2026-06-29: 60 -> 600 -> 0 (disabled). The every-N rebuild does
                                      * mark_all_dirty -> the collect then walks ALL ~1552 chains = the
                                      * periodic ~30ms apc spike (mk=0/cwk=1552 frames). It was only a
                                      * staleness backstop + a missed-dirty correctness backstop; the
                                      * latter is moot (dirty-tracking bugs long fixed; do_frame gate +
                                      * peek_miss are the real net), and the former is now NEED-based via
                                      * the OVERFLOW trigger (rebuild when the 40000-pair pool fills).
                                      * A/B at 600 confirmed staleness stays bounded (overflow 5->4, no
                                      * thrash) so the time-based periodic is removed. remap (evict/
                                      * defrag) + overflow rebuilds still fire. NOTE: collect is NOT the
                                      * df/s gate (proven: cutting walks 28% didn't move df/s) — this is
                                      * EXI-task hygiene, not a df/s win; the df/s lever is do_frame work. */
static volatile bool g_incr_collect_active = true;   /* A/B master toggle */
static uint16_t *incr_rev_head = NULL;               /* [RA_MAX_WATCH_ADDRS]; 0xFFFF=empty */
struct incr_rev_node { void* chain; uint16_t next; };
static struct incr_rev_node *incr_rev_pool = NULL;   /* [INCR_REV_POOL_MAX] */
static uint16_t incr_rev_count = 0;
static bool incr_rev_overflow = false;
static uint16_t g_incr_last_watch_count = 0xFFFF;
static uint16_t g_incr_since_rebuild = 0;
static unsigned long g_incr_rebuilds = 0;            /* diag (reset per FRAME) */
static unsigned long g_incr_marks = 0;               /* chains dirtied this period (diag) */

static void incr_rev_reset(void) {
    if (!incr_rev_head) return;
    for (uint32_t i = 0; i < RA_MAX_WATCH_ADDRS; i++) incr_rev_head[i] = 0xFFFF;
    incr_rev_count = 0;
    incr_rev_overflow = false;
}

/* g_rc_chain_read_cb: record each byte of [address,address+n) -> chain. O(1)
 * append, NO dedup: within one walk every (chain,widx) is already unique (a chain
 * reads each address once), so a dedup scan never matches yet costs O(list) per
 * read -> a hot shared root read by N chains made collect O(N^2) (the 340ms bug).
 * Duplicates that accumulate across multi-pass rounds / incremental re-walks are
 * harmless (dirty marking is idempotent) and are bounded by overflow->rebuild +
 * the periodic rebuild reset. */
extern "C" void incr_chain_read(uint32_t address, uint8_t num_bytes, void* chain) {
    if (!incr_rev_head || incr_rev_overflow) return;
    for (uint8_t b = 0; b < num_bytes; b++) {
        int32_t widx = hash_lookup(address + b);
        if (widx < 0) continue;
        if (incr_rev_count >= INCR_REV_POOL_MAX) { incr_rev_overflow = true; return; }
        incr_rev_pool[incr_rev_count].chain = chain;
        incr_rev_pool[incr_rev_count].next  = incr_rev_head[widx];
        incr_rev_head[widx] = incr_rev_count++;
    }
}

/* Per-snapshot dirty marking. Called from the snapshot apply right after
 * snap_changed[] is computed (count = this snapshot's watch-entry count). */
static void incr_mark_dirty(uint16_t count) {
    if (!g_incr_collect_active || !incr_rev_head || !snap_changed) return;
    const rc_memrefs_t* memrefs = (state == STATE_ACTIVE && g_client)
                                  ? rc_client_get_memrefs(g_client) : NULL;
    if (!memrefs) return;

    bool remapped   = g_incr_remapped; g_incr_remapped = false;  /* evict/defrag/init, NOT append */
    bool periodic   = (INCR_REBUILD_EVERY != 0) && (++g_incr_since_rebuild >= INCR_REBUILD_EVERY);
    if (remapped || periodic || incr_rev_overflow) {
        /* FULL REBUILD: reset the index, dirty every chain (they all walk this
         * vblank, repopulating the index from scratch). Only on a true remap now
         * (evict/defrag shift widxs) or the periodic/overflow backstop — a plain
         * APPEND no longer forces this (it left the index valid), which is what
         * kept the bunny convergence collect at 37ms. */
        incr_rev_reset();
        rc_modified_memrefs_mark_all_dirty(memrefs);
        g_incr_since_rebuild = 0;
        g_incr_rebuilds++;
    } else {
        /* INCREMENTAL: dirty only the chains reading an address that moved. */
        uint16_t lim = (count < watch_count) ? count : watch_count;
        for (uint16_t i = 0; i < lim; i++) {
            if (!snap_changed[i]) continue;
            for (uint16_t n = incr_rev_head[i]; n != 0xFFFF; n = incr_rev_pool[n].next) {
                rc_modified_memref_mark_dirty(incr_rev_pool[n].chain);
                g_incr_marks++;
            }
        }
    }
    g_incr_last_watch_count = watch_count;
    /* Enable the resolver skip ONLY now that a marking pass has actually run
     * (pool allocated, chains correctly dirtied). Setting it in setup would let
     * the resolver skip clean chains before anything marks them -> frozen collect.
     * Once on it stays on; a collect without a fresh snapshot has no new changes,
     * so the clean flags remain valid. */
    g_rc_incr_collect_enabled = 1;
}

/* ============================================================================
 * Phase C shadow walker — ESP-side mirror of the d2x chain walker, used to
 * cross-check the ingested slot blob against the legacy hash (authoritative
 * during the shadow phase). Also the future ingestion core once slots become
 * authoritative. Semantics mirror the wire spec in gc_ra_protocol.h.
 * ============================================================================ */
static bool g_phasec_shadow_enabled = false;  /* bring-up diag (validated 2026-07-01:
                                               * 1.93M compares, bad=0.055% read-skew);
                                               * OFF in authoritative mode */

static inline uint32_t phasec_xform(uint32_t v, uint8_t psize) {
    switch (psize) {
        case RA_CN_SZ_8:     return v & 0xFFu;
        case RA_CN_SZ_16:    return v & 0xFFFFu;
        case RA_CN_SZ_24:    return v & 0xFFFFFFu;
        case RA_CN_SZ_32:    return v;
        case RA_CN_SZ_16_BE: return ((v & 0xFF00u) >> 8) | ((v & 0xFFu) << 8);
        case RA_CN_SZ_24_BE: return ((v & 0xFF0000u) >> 16) | (v & 0xFF00u)
                                  | ((v & 0xFFu) << 16);
        case RA_CN_SZ_32_BE: return __builtin_bswap32(v);
        default:             return v;
    }
}

static inline uint32_t phasec_combine(uint32_t pv, uint32_t o, uint8_t op) {
    switch (op) {
        case RA_CN_OP_MULT:       return pv * o;
        case RA_CN_OP_DIV:        return o ? pv / o : 0;
        case RA_CN_OP_AND:        return pv & o;
        case RA_CN_OP_XOR:        return pv ^ o;
        case RA_CN_OP_MOD:        return o ? pv % o : 0;
        case RA_CN_OP_ADD:        return pv + o;
        case RA_CN_OP_SUB:        return pv - o;
        case RA_CN_OP_SUB_PARENT: return o - pv;
        case RA_CN_OP_ADD_ACC:    return pv + o;
        case RA_CN_OP_SUB_ACC:    return pv - o;
        default:                  return 0;
    }
}

/* Raw (LE-pack) value of node idx. Shipped derefs read the ingested blob;
 * roots read the legacy hash; combines/immediates compute. false = value
 * unavailable (root byte not cached / slot never ingested / slot invalid). */
static bool phasec_node_value(uint16_t idx, uint32_t* out) {
    if (idx >= g_phasec_node_count) return false;
    const rc_phasec_node_t* nd = &g_phasec_nodes[idx];
    uint8_t op = RC_PHASEC_OP(nd);

    if (op == (uint8_t)RA_CN_OP_NONE) { *out = nd->operand; return true; }

    if (op == (uint8_t)RA_CN_OP_DEREF) {
        uint8_t w = RC_PHASEC_WIDTH(nd);
        if (RC_PHASEC_SHIPPED(nd)) {
            uint16_t s = g_phasec_node_ship[idx];
            if (s == 0xFFFF ||
                !(g_phasec_fresh[s >> 3] & (1u << (s & 7))) ||
                !(g_phasec_valid[s >> 3] & (1u << (s & 7))))
                return false;
            const uint8_t* b = g_phasec_blob + g_phasec_blob_off[s];
            uint32_t v = 0;
            for (uint8_t k = 0; k < w; k++) v |= (uint32_t)b[k] << (8u * k);
            *out = v;
            return true;
        }
        /* root: read through the legacy hash (authoritative) */
        uint32_t v = 0;
        for (uint8_t k = 0; k < w; k++) {
            int32_t widx = hash_lookup(nd->operand + k);
            if (widx < 0) return false;
            v |= (uint32_t)memory_data[widx] << (8u * k);
        }
        *out = v;
        return true;
    }

    /* combine: parent value -> edge transform -> ALU */
    {
        uint32_t pv;
        if (nd->parent == RC_PHASEC_PARENT_NONE) return false;
        if (!phasec_node_value(nd->parent, &pv)) return false;
        *out = phasec_combine(phasec_xform(pv, nd->psize), nd->operand, op);
        return true;
    }
}

/* ============================================================================
 * Phase C AUTHORITATIVE mode — covered chains read their leaf straight from
 * the d2x-walked slot blob (rc_upd_resolve_one hook) and are skipped by the
 * collect walk (their leaves never enter the flat watchlist). The legacy
 * path remains for dp/exotic chains + static roots; the v0.32 do_frame gate
 * stays as the correctness net. g_phasec_auth kills it at runtime (full
 * legacy fallback — the d2x keeps shipping the window, we just ignore it).
 * ============================================================================ */
static bool      g_phasec_auth = false;      /* armed per-game in on_game_loaded */
static uint16_t* g_phasec_mm_ship = NULL;    /* dense mm idx -> shipped slot (0xFFFF)
                                              * INTERNAL SRAM — hot path O(1) */
static uint32_t  g_phasec_mm_total = 0;
static uint32_t  g_pc_reads = 0, g_pc_inv = 0, g_pc_defer = 0;   /* cumulative diag */

/* upd leaf source (called from rc_upd_resolve_one via g_rc_phasec_read).
 * Returns: 0 not covered; 1 *raw = LE-packed leaf bytes; 2 slot not ready
 * (bumps the do_frame gate -> defer). Invalid slot (d2x range-check failed)
 * mirrors legacy read_memory_ingame semantics for out-of-range addresses:
 * 0-fill WITHOUT a gate bump (line ~1590: valid_ppc_addr guards the bump). */
static int phasec_read_cb(uint32_t mm_idx, uint32_t* raw) {
    uint16_t s;
    if (!g_phasec_auth || mm_idx >= g_phasec_mm_total) return 0;
    s = g_phasec_mm_ship[mm_idx];
    if (s == 0xFFFF) return 0;
    if (!(g_phasec_fresh[s >> 3] & (1u << (s & 7)))) {
        g_update_miss_count++;      /* never eval on a hole — gate defers */
        g_pc_defer++;
        return 2;
    }
    if (!(g_phasec_valid[s >> 3] & (1u << (s & 7)))) {
        *raw = 0;
        g_pc_inv++;
        return 1;
    }
    {
        const uint8_t* b = g_phasec_blob + g_phasec_blob_off[s];
        uint32_t w = g_phasec_blob_off[s + 1] - g_phasec_blob_off[s];
        uint32_t v = 0;
        for (uint32_t k = 0; k < w; k++) v |= (uint32_t)b[k] << (8u * k);
        *raw = v;
    }
    g_pc_reads++;
    return 1;
}

/* collect skip (g_rc_phasec_covered): covered chains never need EXI
 * resolution — keeps their leaves OUT of the flat watchlist for good. */
static int phasec_covered_cb(uint32_t mm_idx) {
    return g_phasec_auth && mm_idx < g_phasec_mm_total &&
           g_phasec_mm_ship[mm_idx] != 0xFFFF;
}

/* Cross-check a slice of the just-ingested window: recompute each slot's
 * leaf address from the table (like the d2x did) and compare the blob bytes
 * against the legacy hash. Slice-capped so the EXI task never stalls. Small
 * steady 'bad' noise is expected (flat reads and the walk happen ms apart on
 * the d2x; game writes in between) — watch the TREND, not zero.
 * AUTHORITATIVE-phase caveat: covered leaves LEAVE the flat hash, so the
 * compare can only cover slots whose leaf still happens to be hashed —
 * bring-up diagnostic only, default OFF now. */
#define RA_PHASEC_SHADOW_SLICE 64
static void phasec_shadow_compare(uint16_t win_first, uint16_t win_count) {
    if (!g_phasec_shadow_enabled || !memory_data) return;
    static uint32_t rot = 0;
    uint16_t n = win_count < RA_PHASEC_SHADOW_SLICE ? win_count
                                                    : RA_PHASEC_SHADOW_SLICE;
    for (uint16_t i = 0; i < n; i++) {
        uint32_t s = win_first + (uint32_t)((rot + i) % win_count);
        uint16_t node = g_phasec_ship_node[s];
        const rc_phasec_node_t* nd = &g_phasec_nodes[node];
        if (!(g_phasec_valid[s >> 3] & (1u << (s & 7)))) { g_phasec_cmp_inv++; continue; }
        uint32_t pv, addr;
        if (nd->parent == RC_PHASEC_PARENT_NONE) {
            addr = nd->operand;                 /* shipped root (not emitted today) */
        } else {
            if (!phasec_node_value(nd->parent, &pv)) { g_phasec_cmp_miss++; continue; }
            addr = phasec_xform(pv, nd->psize) + nd->operand;
        }
        uint8_t w = RC_PHASEC_WIDTH(nd);
        const uint8_t* b = g_phasec_blob + g_phasec_blob_off[s];
        bool bad = false, miss = false;
        for (uint8_t k = 0; k < w; k++) {
            int32_t widx = hash_lookup(addr + k);
            if (widx < 0) { miss = true; break; }
            if (memory_data[widx] != b[k]) { bad = true; break; }
        }
        if (miss)      g_phasec_cmp_miss++;
        else if (bad)  g_phasec_cmp_bad++;
        else           g_phasec_cmp_ok++;
    }
    rot += n;
}

/* Apply ONE verified snapshot payload: parse (dual: legacy 8B / v2 14B header),
 * advance frame_counter + lru_clock, chain-root input check, snap_changed diff,
 * memory_data memcpy, incremental-collect dirtying, Phase C window ingest.
 * THE single accept path — called from the arrival fast path (worker task) AND
 * the snapshot-queue drain (loop task). Caller guarantees the payload passed
 * Phase D2 seq+count verification at arrival. Returns inputs_changed (the
 * chain-root gate; drain callers ignore it — collect never runs under lag). */
static bool snapshot_apply_payload(const uint8_t* rx_data, uint32_t rx_len) {
    const ra_snapshot_header_t *snap =
        (const ra_snapshot_header_t*)(rx_data + sizeof(ra_gc_header_t));
    uint16_t count = ra_be16_to_host(snap->addr_count);
    bool snap_v2 = (rx_len >= sizeof(ra_gc_header_t)
                    + sizeof(ra_snapshot_header_t) + count);
    uint16_t ch_first = 0, ch_count = 0, ch_blob = 0;
    bool inputs_changed = false;

    if (snap_v2) {
        ch_first = ra_be16_to_host(snap->chain_first);
        ch_count = ra_be16_to_host(snap->chain_count);
        ch_blob  = ra_be16_to_host(snap->chain_blob_len);
    }
    frame_counter = ra_be32_to_host(snap->frame_counter);
    lru_clock = frame_counter;

    const uint8_t *values = rx_data + sizeof(ra_gc_header_t)
        + (snap_v2 ? sizeof(ra_snapshot_header_t) : RA_SNAP_HDR_LEGACY);

    if (!memory_data) return false;

    /* #1 chain-input gate (toggle, default OFF): BEFORE overwriting
     * memory_data, check whether any pointer-base byte changed. */
    if (g_chain_gate_enabled) {
        if (!g_roots_built) build_chain_roots();
        if (!g_roots_built) {
            inputs_changed = true;   /* memrefs not ready -> don't suppress collect */
        } else if (g_root_baddr_count && watch_addresses) {
            uint16_t lim = (count < watch_count) ? count : watch_count;
            for (uint16_t i = 0; i < lim; i++) {
                if (values[i] != memory_data[i] && is_chain_root(watch_addresses[i])) {
                    inputs_changed = true;
                    break;
                }
            }
        }
    }
    /* opt B2 + incremental collect: snapshot diff BEFORE overwriting
     * memory_data — snap_changed[i] marks entries whose value moved. */
    if ((g_b2_enabled || g_incr_collect_active) && snap_changed) {
        uint16_t lim = (count < watch_count) ? count : watch_count;
        for (uint16_t i = 0; i < lim; i++)
            snap_changed[i] = (values[i] != memory_data[i]) ? 1 : 0;
    }
    memcpy(memory_data, values, count);
    /* incremental collect: dirty only chains whose inputs moved (uses the
     * fresh snap_changed[]; must precede this vblank's collect rounds). */
    incr_mark_dirty(count);
    g_vblank_stats.data_ok = 1;  /* Phase D2 integrity check passed */

    /* Phase C ingest: copy this frame's chain window into the persistent slot
     * blob + validity/fresh bitmaps. Length/offset checks make a malformed
     * window a silent no-op. Shadow compare only if the diag toggle is on. */
    if (snap_v2 && ch_count && g_phasec_blob && g_phasec_shipped) {
        uint32_t bm_bytes = ((uint32_t)ch_count + 7u) / 8u;
        const uint8_t *bm = values + count;
        const uint8_t *cb = bm + bm_bytes;
        if ((uint32_t)ch_first + ch_count <= g_phasec_shipped &&
            rx_len >= sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t)
                    + (uint32_t)count + bm_bytes + ch_blob &&
            g_phasec_blob_off[ch_first + ch_count]
              - g_phasec_blob_off[ch_first] == ch_blob) {
            memcpy(g_phasec_blob + g_phasec_blob_off[ch_first], cb, ch_blob);
            for (uint16_t i = 0; i < ch_count; i++) {
                uint32_t s = (uint32_t)ch_first + i;
                if (bm[i >> 3] & (1u << (i & 7)))
                    g_phasec_valid[s >> 3] |=  (uint8_t)(1u << (s & 7));
                else
                    g_phasec_valid[s >> 3] &= (uint8_t)~(1u << (s & 7));
                g_phasec_fresh[s >> 3] |= (uint8_t)(1u << (s & 7));
            }
            g_phasec_win_rx++;
            phasec_shadow_compare(ch_first, ch_count);
        }
    }

    /* Phase D: dp verification section (after the window blob). Only accepted
     * when the count matches OUR derived list (else ignored — e.g. a table/fw
     * mismatch); the compare itself runs post-upd in doframe_primed_cb. */
    g_phasec_dp_have = false;
    if (snap_v2 && g_phasec_dp_n) {
        uint16_t dpc = ra_be16_to_host(snap->dp_count);
        uint32_t bm_bytes = ((uint32_t)ch_count + 7u) / 8u;
        uint32_t dpv_bytes = ((uint32_t)dpc + 7u) / 8u;
        if (dpc == g_phasec_dp_n &&
            rx_len >= (uint32_t)(values - rx_data) + (uint32_t)count
                    + bm_bytes + ch_blob + 4u * dpc + dpv_bytes) {
            const uint8_t* dp  = values + count + bm_bytes + ch_blob;
            const uint8_t* dpv = dp + 4u * dpc;   /* validity bitmap */
            for (uint16_t i = 0; i < dpc; i++) {
                uint32_t v;
                memcpy(&v, dp + 4u * i, 4);
                g_phasec_dp_ship[i] = ra_be32_to_host(v);
            }
            memcpy(g_phasec_dp_shipvalid, dpv, dpv_bytes);
            g_phasec_dp_have = true;
        }
    }
    return inputs_changed;
}

static uint16_t collect_missing_addresses(void) {
    const rc_memrefs_t *memrefs = (state == STATE_ACTIVE && g_client)
                                  ? rc_client_get_memrefs(g_client) : NULL;
    if (!memrefs || g_watchlist_pending) return 0;

    /* STEP 0 cap-collect test (g_collect_cap_walks): short-circuit the whole
     * resolver walk so apc_us drops to ~0 and convergence lags. See the toggle's
     * comment for the A/B protocol. No-op when the flag is FALSE. */
    if (g_collect_cap_walks) return 0;

    uint32_t cm_t0 = (uint32_t)esp_timer_get_time();
    uint32_t n = rc_memrefs_get_pending_addresses(
        memrefs, prefetch_addrs, prefetch_sizes, PREFETCH_MAX,
        peek_from_snapshot, is_cached_cb, NULL);

    uint16_t start_count = query_count;

    for (uint32_t pi = 0; pi < n && query_count < ADDR_QUERY_MAX; pi++) {
        for (uint8_t b = 0; b < prefetch_sizes[pi] && query_count < ADDR_QUERY_MAX; b++) {
            uint32_t byte_addr = prefetch_addrs[pi] + b;

            /* Sanity: reject addresses outside MEM1/MEM2. Should be rare now
             * — rc_memrefs_get_pending_addresses resolves each level from a
             * cached (real) parent value, so a reported address is a real
             * pointer + offset, not garbage off an unresolved 0 pointer. Kept
             * as a guard: a genuinely NULL/wild pointer in game RAM could
             * still land here, and asking ra-module to Swi_MLoad an invalid
             * address can hang the Starlet thread. */
            if (!((byte_addr <= 0x017FFFFF)
                  || (byte_addr >= 0x10000000 && byte_addr <= 0x137FFFFF  /* IOS-reserved top of MEM2 excluded (read-fault guard) */))) continue;

            /* In static watchlist? O(1) hash lookup. */
            if (hash_lookup(byte_addr) >= 0) continue;

            /* Already in this frame's query cache (from previous round)? */
            bool dup = false;
            for (uint16_t q = 0; q < query_count; q++) {
                if (query_addrs[q] == byte_addr) { dup = true; break; }
            }
            if (!dup) {
                query_addrs[query_count++] = byte_addr;
            }
        }
    }

    /* cm= : cache misses encountered WHILE RESOLVING addAddress chains — the
     * bytes the new resolver could not find in the hash this round and is
     * about to fetch. Counted here (before the peek_miss safety net) so it
     * reflects only the addAddress resolution, not do_frame leftovers. The
     * invariant to watch: next snapshot's sa == this snapshot's sa + Σcm.
     * If sa_delta > Σcm, the peek_miss net below caught something the
     * resolver missed (a chain shape not covered) — a useful red flag. */
    uint16_t part1 = (uint16_t)(query_count - start_count);
    g_vblank_stats.collect_cm += part1;
    /* Per-round breakdown: which addAddress level this collect resolved. */
    if (g_vblank_stats.collect_round < 8)
        g_vblank_stats.cm_by_round[g_vblank_stats.collect_round] = part1;
    if (g_vblank_stats.collect_round < 255)
        g_vblank_stats.collect_round++;

    /* Process peek misses accumulated since the last collect (covers BOTH
     * the resolver's peek calls just now AND the previous frame's do_frame
     * trigger evaluations). Safety net for any address the resolver did not
     * surface (e.g. an exotic modifier shape). */
    uint16_t before_pm = query_count;
    for (uint16_t i = 0; i < peek_miss_count && query_count < ADDR_QUERY_MAX; i++) {
        uint32_t addr = peek_miss_addrs[i];
        if (hash_lookup(addr) >= 0) continue;
        bool dup = false;
        for (uint16_t q = 0; q < query_count; q++) {
            if (query_addrs[q] == addr) { dup = true; break; }
        }
        if (!dup) query_addrs[query_count++] = addr;
    }
    peek_miss_count = 0;

    /* Diagnostic: the resolver found NOTHING (part1==0) yet do_frame's
     * peek_miss surfaced addresses the evaluator actually read. Log a few so
     * we can see WHAT shape the chain walker doesn't cover. Throttled 2s. */
    uint16_t pm_added = (uint16_t)(query_count - before_pm);
    if (part1 == 0 && pm_added > 0) {
        /* DISAMBIGUATION (project_exi_robust_handshake): is the galaxy freeze a
         * logical multi-pass loop or EXI corruption? Track how many CONSECUTIVE
         * frames the SAME first peek-miss address re-surfaces. A persistent same
         * address = it's queried+committed every frame yet the evaluator re-misses
         * it (resolver-divergence or post-commit eviction) — a LOGICAL loop, not
         * EXI (which would garble DIFFERENT addresses each time). Also report
         * watch_count so we see if the list is still growing (commit alive). */
        static uint32_t g_stuck_addr  = 0;
        static uint16_t g_stuck_count = 0;
        uint32_t first = query_addrs[before_pm];
        if (first == g_stuck_addr) {
            if (g_stuck_count < 0xFFFF) g_stuck_count++;
        } else {
            g_stuck_addr  = first;
            g_stuck_count = 0;
        }
        static uint32_t last_pm_log = 0;
        uint32_t now_pm = millis();
        if (now_pm - last_pm_log >= 2000) {
            last_pm_log = now_pm;
            uint16_t nlog = pm_added < 8 ? pm_added : 8;
            char buf[160]; int off = 0;
            for (uint16_t k = 0; k < nlog && off < (int)sizeof(buf) - 12; k++)
                off += snprintf(buf + off, sizeof(buf) - off, " %08lX",
                                (unsigned long)query_addrs[before_pm + k]);
            LOG_DBG("DEBUG=resolver-miss pm=%u stuck=%ux@%08lX watch=%u (resolver found 0):%s\r\n",
                      (unsigned)pm_added, (unsigned)g_stuck_count,
                      (unsigned long)g_stuck_addr, (unsigned)watch_count, buf);

            /* THE DECISIVE TEST: run the evaluator's EXACT read path on THIS SAME
             * cache and see what IT misses. peek_miss_count is 0 here (reset just
             * above). If the dry-UPDATE finds misses the resolver (part1==0) did
             * NOT -> LOGIC divergence (fix #1 or #2/#3). If it finds none -> the
             * resolver was right on this cache and the resolver-miss is just
             * FRAME-LATENCY (stale last-frame peek_miss) -> #2/#3 this-frame exact
             * is the answer, NOT patching the resolver. Restores peek_miss +
             * g_update_miss_count so the real flow is untouched. */
            if (g_diag_dry_update && g_client) {
                uint32_t umc_save = g_update_miss_count;
                peek_miss_count = 0;
                rc_client_diag_dry_update(g_client);   /* evaluator, SAME cache */
                uint16_t dry = peek_miss_count;
                char dbuf[120]; int doff = 0;
                uint16_t dn = dry < 8 ? dry : 8;
                for (uint16_t k = 0; k < dn && doff < (int)sizeof(dbuf) - 12; k++)
                    doff += snprintf(dbuf + doff, sizeof(dbuf) - doff, " %08lX",
                                     (unsigned long)peek_miss_addrs[k]);
                LOG_DBG("DEBUG=DRY-DIAG dry_miss=%u verdict=%s:%s\r\n",
                          (unsigned)dry,
                          dry > 0 ? "LOGIC-DIVERGENCE" : "frame-latency(resolver-ok)",
                          dbuf);
                peek_miss_count     = 0;          /* discard the dry-UPDATE's misses */
                g_update_miss_count = umc_save;   /* don't perturb the gate */
            }
        }
    }

    uint16_t new_count = (uint16_t)(query_count - start_count);
    /* ms= : all new addresses queued this vblank (resolver + peek_miss net).
     * cm= (the addAddress-resolution subset) was already accumulated above. */
    g_vblank_stats.collect_miss_total += new_count;
    /* CPU spent in the resolver walk this call — accumulated across all the
     * vblank's rounds. ap_us - cm_cpu_us ≈ EXI round-trip wait. */
    g_vblank_stats.cm_cpu_us += (uint32_t)esp_timer_get_time() - cm_t0;
    return new_count;
}

/* Build & ship an ADDR_QUERY response with the addresses appended in
 * the most recent collect_missing_addresses() call: range
 * [pending_query_start .. query_count-1]. */
static void send_addr_query_response(void) {
    uint16_t round_count = query_count - pending_query_start;
    if (round_count == 0) {
        /* Zero-round = nothing to send. Should not happen in normal flow.
         * If it does, we silently no-op (leaves addr_query_pending alone). */
        return;
    }

    /* Throttled trace — useful when multi-pass is active. */
    {
        static uint32_t last_aq_log = 0;
        uint32_t now_aq = millis();
        if (now_aq - last_aq_log >= 1000) {
            last_aq_log = now_aq;
            LOG_DBG("DEBUG=>> send_addr_query iter=%u round=%u total_qc=%u first_qa=0x%08lX\r\n",
                    (unsigned)addr_query_iter_count, (unsigned)round_count,
                    (unsigned)query_count,
                    (unsigned long)query_addrs[pending_query_start]);
        }
    }

    /* GALAXY-FREEZE forensics: garbage pointers (read in-flux during a galaxy load)
     * resolve to LOW addresses (a pointer read as ~0, + offset). The ra-module
     * Swi_MLoad's every queried address from PPC RAM; a wild low/unmapped address
     * is the prime suspect for the Starlet fault -> IOS hang -> freeze. Flag the
     * low ones so the last ADDR_QUERY before a freeze names the culprit. */
    {
        uint16_t n_low = 0, n_high = 0;
        uint32_t min_qa = 0xFFFFFFFFu, max_qa = 0;
        for (uint16_t i = 0; i < round_count; i++) {
            uint32_t a = query_addrs[pending_query_start + i];
            if (a < min_qa) min_qa = a;
            if (a > max_qa) max_qa = a;
            if (a < 0x00100000u) { if (n_low  < 6) LOG_DBG("DEBUG=LOWQA=0x%08lX\r\n",  (unsigned long)a); n_low++;  }
            if (a > 0x13000000u) { if (n_high < 6) LOG_DBG("DEBUG=HIGHQA=0x%08lX\r\n", (unsigned long)a); n_high++; }
        }
        LOG_DBG("DEBUG=ADDR_QUERY round=%u min=0x%08lX max=0x%08lX n_low=%u n_high=%u\r\n",
                (unsigned)round_count, (unsigned long)min_qa, (unsigned long)max_qa,
                (unsigned)n_low, (unsigned)n_high);
    }

    addr_query_pending = true;
    uint16_t payload_len = sizeof(ra_addr_query_t) + round_count * 4;

    ra_esp_header_t resp;
    resp.magic       = RA_MAGIC_ESP_TO_GC;
    resp.status      = (uint8_t)state;
    resp.event_type  = RA_EVT_ADDR_QUERY;
    resp.event_count = has_pending_event() ? 1 : 0;
    resp.data_len    = ra_host_to_be16(payload_len);

    uint8_t *buf = (uint8_t*)malloc(sizeof(ra_esp_header_t) + payload_len);
    if (!buf) {
        LOG_ERR("ERROR=send_addr_query_response: malloc(%u) FAILED\n",
                (unsigned)(sizeof(ra_esp_header_t) + payload_len));
        addr_query_pending = false;
        set_new_snapshot();  /* fail-open: process frame with what we have */
        return;
    }
    memcpy(buf, &resp, sizeof(resp));
    ra_addr_query_t aq;
    aq.addr_count = ra_host_to_be16(round_count);
    memcpy(buf + sizeof(resp), &aq, sizeof(aq));
    uint32_t *aq_addrs = (uint32_t*)(buf + sizeof(resp) + sizeof(aq));
    for (uint16_t i = 0; i < round_count; i++)
        aq_addrs[i] = ra_host_to_be32(query_addrs[pending_query_start + i]);

    send_response(buf, sizeof(ra_esp_header_t) + payload_len);
    free(buf);
}

/* Freeze forensics rings (dumped by the Core-0 WDOG when the servicer stalls).
 * cmd  = commands the Wii SENT us (03=SNAPSHOT 08=ADDR_RESPONSE ...).
 * evt  = events WE sent back (event_type at offset 2 of each response).
 * Together they replay the exact exchange + mutation sequence (REMOVE_IDX/
 * APPEND/ADDR_QUERY) right before a freeze — WITHOUT per-frame Serial spam
 * (dumped only on stall), so they don't shift the Heisenbug timing. */
#define CMD_RING 16
volatile uint8_t g_cmd_ring[CMD_RING] = {0};
volatile uint8_t g_cmd_ring_idx = 0;
volatile uint8_t g_evt_ring[CMD_RING] = {0};   /* event_type of each response sent */
volatile uint8_t g_evt_ring_idx = 0;

/* Response sender for Phase B commands (SNAPSHOT, ADDR_RESPONSE). Places
 * the response at tx_buf[0] so the Wii's dedicated read CS-low clocks it
 * out from byte 0 — no leading padding. WiiFlow's Phase A commands
 * (IDENTIFY/POLL/CHUNK/GAME_RESET) use separate handlers that bake their
 * own padding for the single-CS-low write+read pattern. */
static void send_response(const uint8_t *data, size_t len) {
    if (len > EXI_MAX_TRANSACTION_SIZE) {
        LOG_ERR("DEBUG=send_response: too big len=%u\n", (unsigned)len);
        return;
    }
    /* Forensics: record the event_type we're shipping (offset 2). */
    if (len >= 3) {
        g_evt_ring[g_evt_ring_idx] = data[2];
        g_evt_ring_idx = (uint8_t)((g_evt_ring_idx + 1) & (CMD_RING - 1));
    }
    /* DIAG: log wire-format ONLY for non-ACK events (event_type at offset 2). */
    if (len >= 6 && state == STATE_ACTIVE && data[2] != 0x00) {
        LOG_DBG("DEBUG=>>WIRE len=%u hdr=[%02X %02X %02X %02X %02X %02X]\r\n",
                (unsigned)len,
                data[0], data[1], data[2], data[3], data[4], data[5]);
    }
    exi_spi_prepare_response(data, len);
}

void handle_exi_command(const uint8_t *rx_data, size_t rx_len) {
    if (rx_len < sizeof(ra_gc_header_t)) {
        static uint32_t last_short = 0; uint32_t now = millis();
        if (now - last_short >= 1000) { last_short = now;
            LOG_DBG("DEBUG=EXI: short pkt rx_len=%u\r\n", (unsigned)rx_len); }
        return;
    }

    const ra_gc_header_t *hdr = (const ra_gc_header_t*)rx_data;
    if (hdr->magic != RA_MAGIC_GC_TO_ESP) {
        static uint32_t last_magic = 0; uint32_t now = millis();
        if (now - last_magic >= 1000) { last_magic = now;
            LOG_ERR("DEBUG=EXI: bad magic 0x%02X rx_len=%u bytes=%02X %02X %02X %02X\r\n",
                hdr->magic, (unsigned)rx_len,
                rx_data[0], rx_data[1],
                rx_len>2 ? rx_data[2] : 0, rx_len>3 ? rx_data[3] : 0); }
        return;
    }

    // Freeze forensics: ring of recent commands. When the Wii stops talking
    // (servicer srv=+0), the WDOG dumps this to show the command sequence that
    // led to the freeze — pinning what the ra-module was doing when it died.
    g_cmd_ring[g_cmd_ring_idx] = hdr->command;
    g_cmd_ring_idx = (uint8_t)((g_cmd_ring_idx + 1) & (CMD_RING - 1));

    // Per-command counter (for heartbeat rate analysis, especially post-IOS-reload)
    if (hdr->command < 16) cmd_count[hdr->command]++;

    // Throttled per-command log — 5s to keep Serial from saturating at 30Hz.
    {
        static uint32_t last_cmd = 0; uint32_t now = millis();
        if (now - last_cmd >= 5000) { last_cmd = now;
            LOG_DBG("DEBUG=EXI: cmd=0x%02X rx_len=%u state=%d\r\n",
                      hdr->command, (unsigned)rx_len, (int)state); }
    }

    switch (hdr->command) {
        case RA_CMD_IDENTIFY: {
            // Respond with device ID.
            // IMPORTANT: The GC sends 4 bytes (IDENTIFY cmd) then reads 10 bytes in the
            // SAME CS-low window (14-byte total transaction). The ESP32 sends tx_buf[0..13]
            // throughout. The GC captures tx_buf[4..13] as rx[0..9]. So the actual response
            // (magic + status + ...) must start at byte 4 of the TX buffer, preceded by
            // 4 bytes of 0xFF padding that are clocked out during the GC's write phase.
            uint32_t device_id = ra_host_to_be32(RA_DEVICE_ID);
            ra_esp_header_t resp;
            resp.magic = RA_MAGIC_ESP_TO_GC;
            resp.status = (uint8_t)state;
            resp.event_type = RA_EVT_NONE;
            resp.event_count = 0;
            resp.data_len = ra_host_to_be16(4);
            uint8_t buf[4 + sizeof(ra_esp_header_t) + 4];  /* 4 pad + 6 hdr + 4 devid = 14 */
            buf[0] = buf[1] = buf[2] = buf[3] = 0xFF;      /* padding (GC write phase) */
            memcpy(buf + 4, &resp, sizeof(resp));
            memcpy(buf + 4 + sizeof(resp), &device_id, 4);
            exi_spi_prepare_response(buf, sizeof(buf));
            LOG_DBG("DEBUG=EXI: IDENTIFY\r\n");
            break;
        }
        
        case RA_CMD_LOAD_GAME: {
            // game_id (6 bytes) is mandatory; disc_number/has_hash/md5_hash
            // follow per ra_load_game_t. Since v0.27.0 WiiFlow computes the
            // RA hash ON CONSOLE (rc_hash_wii_disc port over the WBFS/ISO
            // image) and ships it here — identification works for ANY game
            // RA knows, exactly like Dolphin. The game-ID table remains the
            // fallback for physical discs / unsupported image formats.
            if (rx_len < sizeof(ra_gc_header_t) + RA_GAME_ID_LEN) break;
            const uint8_t *p = rx_data + sizeof(ra_gc_header_t);
            char gid[RA_GAME_ID_LEN + 1];
            memcpy(gid, p, RA_GAME_ID_LEN);
            gid[RA_GAME_ID_LEN] = '\0';
            LOG_INFO("DEBUG=EXI: LOAD_GAME id=%s rx_len=%u\r\n", gid, (unsigned)rx_len);

            uint8_t has_hash = 0;
            char console_hash[RA_HASH_LEN + 1];
            if (rx_len >= sizeof(ra_gc_header_t) + RA_GAME_ID_LEN + 2 + RA_HASH_LEN) {
                has_hash = p[RA_GAME_ID_LEN + 1];
                if (has_hash == 1) {
                    memcpy(console_hash, p + RA_GAME_ID_LEN + 2, RA_HASH_LEN);
                    console_hash[RA_HASH_LEN] = '\0';
                    /* sanity: 32 lowercase hex chars */
                    for (int hx = 0; hx < RA_HASH_LEN; hx++) {
                        char c = console_hash[hx];
                        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
                            has_hash = 0;
                            break;
                        }
                    }
                }
            }

            if (has_hash == 1) {
                LOG_DBG("DEBUG=Console-computed hash: %s\r\n", console_hash);
                gameId = String(console_hash);
                state = STATE_LOADING_GAME;
            } else {
                // Fallback: lookup hash from the Wii game-ID table
                const char *hash = wii_lookup_game_hash(gid);
                if (hash) {
                    LOG_DBG("DEBUG=Hash found in table: %s (%s)\r\n", hash, wii_lookup_game_name(gid));
                    gameId = String(hash);  // store MD5 hash for loadGame()
                    state = STATE_LOADING_GAME;
                } else {
                    LOG_ERR("ERROR=No hash for Wii game ID %s (no console hash either)\r\n", gid);
                    gameId = String(gid);
                    state = STATE_ERROR;
                }
            }
            
            // ACK
            ra_esp_header_t resp;
            resp.magic = RA_MAGIC_ESP_TO_GC;
            resp.status = (uint8_t)state;
            resp.event_type = RA_EVT_NONE;
            resp.event_count = 0;
            resp.data_len = 0;
            exi_spi_prepare_response((uint8_t*)&resp, sizeof(resp));
            break;
        }
        
        case RA_CMD_SNAPSHOT: {
            // Accept snapshots even before STATE_ACTIVE so that WATCHLIST_UPDATE
            // can be delivered while the watchlist is being built.
            // STATE_GAME_LOADED transitions to STATE_ACTIVE on the first snapshot
            // received from ra-module (i.e. after WiiFlow has booted the game).
            if (state == STATE_INIT || state == STATE_ERROR) break;
            if (state == STATE_GAME_LOADED) {
                state = STATE_ACTIVE;
                g_roots_built = false;   /* rebuild chain-root gate for this game's memrefs */
                LOG_INFO("DEBUG=*** STATE 6 -> 7: first SNAPSHOT received from ra-module ***\r\n");
            }
            /* Dual-parse: legacy snapshot header is 8B (frame/count/seq); the
             * Phase C v2 header is 14B (+chain_first/count/blob_len). The Wii
             * writes EXACT lengths, so rx_len disambiguates: legacy rx_len is
             * always 4+8+count < 4+14+count, the v2 minimum. A legacy d2x
             * against this fw therefore parses cleanly with no chain data.
             * (RA_SNAP_HDR_LEGACY is file-scope, by the snapshot queue.) */
            if (rx_len < sizeof(ra_gc_header_t) + RA_SNAP_HDR_LEGACY) break;

            const ra_snapshot_header_t *snap = (const ra_snapshot_header_t*)(rx_data + sizeof(ra_gc_header_t));
            uint16_t count = ra_be16_to_host(snap->addr_count);
            frame_counter = ra_be32_to_host(snap->frame_counter);
            bool snap_v2 = (rx_len >= sizeof(ra_gc_header_t)
                            + sizeof(ra_snapshot_header_t) + count);
            uint16_t ch_first = 0, ch_count = 0, ch_blob = 0;
            if (snap_v2) {
                ch_first = ra_be16_to_host(snap->chain_first);
                ch_count = ra_be16_to_host(snap->chain_count);
                ch_blob  = ra_be16_to_host(snap->chain_blob_len);
            }

            /* v0.28.8 — LRU clock = PPC VBI game-frame counter (was ++ per
             * SNAPSHOT ≈7.4Hz, which made the eviction age tiers worth ~8x
             * their intended seconds). Now age = real game-frames since last
             * touch — fixed relation to wall-clock and robust to dropped/
             * skipped snapshots. frame_counter set just above from the header.
             * (A game restart can reset frame_counter; the age clamps at the
             * eviction/histogram sites guard the resulting underflow.) */
            lru_clock = frame_counter;

            /* Reset per-vblank telemetry for this frame. */
            g_vblank_stats.data_ok               = 0;
            g_vblank_stats.snap_addr_count        = count;
            g_vblank_stats.collect_cm             = 0;
            g_vblank_stats.pre_doframe_miss_count = 0;
            g_vblank_stats.collect_miss_total     = 0;
            g_vblank_stats.cm_cpu_us              = 0;
            g_vblank_stats.iter_depth             = 0;
            g_vblank_stats.collect_round          = 0;
            for (uint8_t r = 0; r < 8; r++) g_vblank_stats.cm_by_round[r] = 0;
            g_vblank_stats.had_mutation           = 0;
            g_vblank_stats.had_cleanup            = 0;
            g_vblank_stats.snap_start_us          = (uint32_t)esp_timer_get_time();
            g_vblank_stats.wl_seq_at_snap_start   = wl_seq;

            // Throttled log: confirm ra_do_frame VBlank hook is firing on GC side.
            // frame_counter should increment ~60x/sec. If stuck at 0, hook is not installed.
            static uint32_t last_snap_log_ms = 0;
            uint32_t snap_now = (uint32_t)(esp_timer_get_time() / 1000);
            if (snap_now - last_snap_log_ms >= 5000) {
                last_snap_log_ms = snap_now;
                LOG_DBG("DEBUG=SNAP frame=%lu snap_count=%u esp_watch=%u state=%d\r\n",
                         (unsigned long)frame_counter, (unsigned)count,
                         (unsigned)watch_count, (int)state);
                if (g_phasec_shipped) {
                    /* Authoritative health: rd/inv/dfr = CUMULATIVE slot reads /
                     * invalid-slot 0-reads / gate defers from not-ready slots.
                     * win=windows ingested; cf/cc=this frame's window (cc should
                     * equal shipped once the flat list shrinks). Shadow counters
                     * shown too when the diag toggle is on. */
                    LOG_DBG("DEBUG=PhaseC%s: rd=%lu inv=%lu dfr=%lu win=%lu cf=%u cc=%u dp=%u dpmm=%lu dpskip=%lu lbA=%lu/%lu | shadow ok=%lu bad=%lu miss=%lu\r\n",
                            g_phasec_auth ? " AUTH" : "",
                            (unsigned long)g_pc_reads, (unsigned long)g_pc_inv,
                            (unsigned long)g_pc_defer, (unsigned long)g_phasec_win_rx,
                            (unsigned)ch_first, (unsigned)ch_count,
                            (unsigned)g_phasec_dp_n, (unsigned long)g_pc_dpmm,
                            (unsigned long)g_pc_dpskip,
                            (unsigned long)g_rc_lb_stat_active_ok, (unsigned long)g_rc_lb_stat_total,
                            (unsigned long)g_phasec_cmp_ok, (unsigned long)g_phasec_cmp_bad,
                            (unsigned long)g_phasec_cmp_miss);
                }
            }
            const uint8_t *values = rx_data + sizeof(ra_gc_header_t)
                + (snap_v2 ? sizeof(ra_snapshot_header_t) : RA_SNAP_HDR_LEGACY);
            bool inputs_changed = false;   /* #1 gate: a chain-root (pointer base) byte changed this snapshot */

            /* ---- Phase D2 (v0.26.0): VERIFIED sync ----
             * The snapshot echoes the seq of the last mutation the Wii
             * applied. Equality with our wl_seq (plus matching count) is
             * PROOF of positional alignment — only then do we accept the
             * values and run the frame. Behind → deliver exactly the next
             * mutation from the ring (idempotent: the Wii drops anything
             * != ra_seq+1). Anything else → RESYNC. This replaces the
             * v0.24.6/v0.25.1 count-mismatch streak heuristics entirely. */
            uint16_t wl_seq_ra = ra_be16_to_host(snap->wl_seq);

            if (state == STATE_ACTIVE) {
                ra_seq_view = wl_seq_ra;          /* authoritative echo */

                if (wl_seq_ra != wl_seq) {
                    static uint32_t last_behind_log = 0;
                    uint32_t now_bh = millis();
                    if (now_bh - last_behind_log >= 2000) {
                        last_behind_log = now_bh;
                        LOG_DBG("DEBUG=SNAP seq: ra=%u esp=%u — delivering next mutation\r\n",
                                (unsigned)wl_seq_ra, (unsigned)wl_seq);
                    }
                    if (!mut_deliver_next()) {
                        /* Wii ahead of us, or the ring no longer holds the
                         * mutation it needs — unrecoverable by walking. */
                        trigger_watchlist_resync("seq-gap");
                        ra_esp_header_t ack;
                        ack.magic       = RA_MAGIC_ESP_TO_GC;
                        ack.status      = (uint8_t)state;
                        ack.event_type  = RA_EVT_NONE;
                        ack.event_count = 0;
                        ack.data_len    = 0;
                        send_response((uint8_t*)&ack, sizeof(ack));
                    }
                    g_ok0_skips++;
                    break;   /* out of sync — no memcpy, no do_frame */
                }

                if (count != watch_count) {
                    /* Same mutation point but different length = replica
                     * corruption the seq stream cannot explain. */
                    trigger_watchlist_resync("count@seq");
                    ra_esp_header_t ack;
                    ack.magic       = RA_MAGIC_ESP_TO_GC;
                    ack.status      = (uint8_t)state;
                    ack.event_type  = RA_EVT_NONE;
                    ack.event_count = 0;
                    ack.data_len    = 0;
                    send_response((uint8_t*)&ack, sizeof(ack));
                    g_ok0_skips++;
                    break;
                }

                /* SNAPSHOT QUEUE (zero-frame-loss): the loop task still owes
                 * work (do_frame in flight, an unstarted fast-path frame, or
                 * a non-empty ring) — enqueue this verified payload RAW and
                 * ACK. Apply happens at drain time, serialized with ITS OWN
                 * do_frame. No collect/evict/convergence while lagging (the
                 * dp resolver must not run on stale do_frame state; an
                 * ADDR_QUERY here would feed future MEM1 values into a past
                 * frame). See the ring's comment block. */
                if (g_snapq_buf[0] && (g_df_busy || new_snapshot || !snapq_empty())) {
                    uint8_t nt = (uint8_t)((g_snapq_tail + 1) % SNAPQ_N);
                    if (nt == g_snapq_head) {
                        g_q_drop++;   /* ring full (pathological) — counted, frame lost */
                    } else {
                        memcpy(g_snapq_buf[g_snapq_tail], rx_data, rx_len);
                        g_snapq_len[g_snapq_tail] = (uint16_t)rx_len;
                        g_snapq_tail = nt;
                        g_q_enq++;
                        uint8_t d = snapq_depth();
                        if (d > g_q_hwm) g_q_hwm = d;
                    }
                    ra_esp_header_t ack;
                    ack.magic       = RA_MAGIC_ESP_TO_GC;
                    ack.status      = (uint8_t)state;
                    ack.event_type  = RA_EVT_NONE;
                    ack.event_count = 0;
                    ack.data_len    = 0;
                    send_response((uint8_t*)&ack, sizeof(ack));
                    break;
                }

                /* Verified in sync — accept the values (shared apply, ALSO the
                 * queue drain's applier — all accept logic lives in there). */
                inputs_changed = snapshot_apply_payload(rx_data, rx_len);
            }

            /* CRITICAL: only reset multi-pass state when no ADDR_QUERY is in
             * flight. ra-module's arm-after-prepare protocol means a
             * SNAPSHOT can arrive on Core 1 AFTER we've prepared an
             * ADDR_QUERY response but BEFORE its matching ADDR_RESPONSE
             * loops back — if we reset query_count here we wipe the
             * query_addrs/query_values from the in-flight ADDR_QUERY, and
             * when its ADDR_RESPONSE lands the fill loop sees query_count=0
             * → CONVERGE qc=0 → watchlist_append no-op → watch frozen at
             * 18. The diag at 19:52:04 caught this red-handed: ADDR_RESP
             * entry query_count=0 peek_miss=4 right after a SNAPSHOT that
             * had wiped state. Solution: skip the reset (and the
             * collect_missing call) when addr_query_pending. The intermediate
             * SNAPSHOT just refreshes memory_data and sends a plain ACK
             * for ra-module to discard; the original multi-pass continues
             * on the eventual ADDR_RESPONSE. See [[project-snapshot-wipes-
             * inflight-query]]. */
            uint16_t n_missing = 0;
            /* addr_query_pending gates the multi-pass state. If a prior
             * ADDR_QUERY is still outstanding (ra-module hasn't sent
             * ADDR_RESP back yet), don't wipe query_count — the in-flight
             * round's bookkeeping is still needed. See
             * [[project-snapshot-wipes-inflight-query]]. */
            if (!addr_query_pending) {
                /* EVICTION (v0.31): trigger at the CONVERGED state — both
                 * replicas are identical here (count just matched in Phase D2)
                 * — and BEFORE this frame's resolution. evict_lru defrags the
                 * ESP list + queues REMOVE_IDX(s); the collect_missing below
                 * then re-resolves any evicted-but-still-reachable byte (re-walk
                 * → re-query → re-append age 0), so nothing the do_frame needs is
                 * stranded (this is WHY it goes before resolution, not after).
                 * The REMOVE ships via the existing convergence mut_deliver; the
                 * seq machinery drains multi-batch in order. Only fires under
                 * watchlist pressure — light games (Kirby ~978 << 4608) never
                 * hit it. d2x unchanged (REMOVE_IDX handler already in v0.30).
                 *
                 * v0.32.1 — TWO triggers now, same safe (converged, pre-
                 * resolution) placement. PRESSURE first (emergency half-dump);
                 * else the PERIODIC GC every GC_PERIOD_FRAMES (gentle dead-weight
                 * sweep, age ≥ GC_MIN_AGE). last_gc_frame resets on either so the
                 * GC clock restarts after a pressure dump too. */
                if (state == STATE_ACTIVE) {
                    static uint32_t last_gc_frame = 0;
                    if (watch_count > g_watch_high_water) {
                        evict_lru(true);            /* pressure: reclaim half */
                        last_gc_frame = frame_counter;
                    } else if ((uint32_t)(frame_counter - last_gc_frame) >= GC_PERIOD_FRAMES) {
                        last_gc_frame = frame_counter;
                        evict_lru(false);           /* periodic GC: dead weight only */
                    }
                }
                query_count = 0;
                pending_query_start = 0;
                addr_query_iter_count = 0;
                /* v0.27.3 — SKIP the expensive memref walk in steady state.
                 * collect_missing_addresses() iterates all ~3500 SMG memrefs
                 * (rc_memrefs_get_addresses peeks every chain) EVERY snapshot,
                 * which starves the do_frame catch-up of CPU (measured df/s=16
                 * vs 60 needed → bunny timer ran 27% speed). But once chains
                 * have converged it finds nothing new. peek_miss_count carries
                 * the PREVIOUS do_frame's unresolved peeks: if it's 0, every
                 * address the evaluator touched was already cached, so the
                 * walk is pure waste — skip it and converge immediately,
                 * freeing the frame for catch-up do_frames. A pointer that
                 * moves (galaxy change) makes the next do_frame miss → walk
                 * runs the frame after (1-frame latency, same as always). A
                 * periodic force every FORCE_COLLECT_EVERY frames catches any
                 * address only reachable via the memref walk, not do_frame. */
                /* #1 gate (toggle g_chain_gate_enabled, default OFF):
                 *   OFF -> ORIGINAL: peek_miss || every-60-force. The 60-force
                 *          proactively pre-fetches at scene transitions — proven safe.
                 *   ON  -> chain-root gate: inputs_changed drives it, force relaxes
                 *          to 600. (Death-spiraled at galaxy changes — #2 is the fix.) */
                const uint16_t force_every = g_chain_gate_enabled ? 600u : 60u;
                static uint16_t collect_skip_run = 0;
                if (state == STATE_ACTIVE) {
                    bool force  = (++collect_skip_run >= force_every);
                    bool by_pm  = (peek_miss_count > 0);             /* capture: collect resets it */
                    bool by_inp = g_chain_gate_enabled && inputs_changed;
                    if (by_pm || by_inp || force) {
                        collect_skip_run = 0;
                        uint32_t t0 = (uint32_t)esp_timer_get_time();
                        n_missing = collect_missing_addresses();
                        g_cm_us += (uint32_t)esp_timer_get_time() - t0;
                        g_cm_n++;
                        if (g_chain_gate_enabled && force && !by_pm && !by_inp && n_missing > 0)
                            LOG_DBG("DEBUG=FORCE-collect found %u addr(s) the chain-root gate missed\r\n",
                                     (unsigned)n_missing);
                    } else {
                        n_missing = 0;     /* steady state — converge now */
                        g_cm_skipped++;
                    }
                }
            }

            /* Throttled snapshot diagnostic — 2s cadence keeps Serial
             * pressure low while giving a heartbeat that SNAPshots are
             * arriving and what state they're in. */
            if (state == STATE_ACTIVE) {
                static uint32_t last_snap_diag = 0;
                uint32_t now_sd = millis();
                if (now_sd - last_snap_diag >= 2000) {
                    last_snap_diag = now_sd;
                    LOG_DBG("DEBUG=SNAP_HDR n_missing=%u qc=%u peek_miss=%u first_qa=%08lX\r\n",
                                  (unsigned)n_missing, (unsigned)query_count,
                                  (unsigned)peek_miss_count,
                                  query_count > 0 ? (unsigned long)query_addrs[0] : 0UL);
                }
            }

            // Build response
            ra_esp_header_t resp;
            resp.magic       = RA_MAGIC_ESP_TO_GC;
            resp.status      = (uint8_t)state;
            resp.event_count = has_pending_event() ? 1 : 0;

            /* Mutation delivery has top priority. collect_missing →
             * watchlist_append may have just minted an eviction
             * (REMOVE_IDX) and/or an append on THIS frame — the ring
             * holds them in seq order; ship the next one the Wii lacks.
             * memory_data stays locally coherent (evict_lru defrags it
             * with the addresses), so do_frame on this frame's values
             * is safe. */
            if (state == STATE_ACTIVE && ra_seq_view != wl_seq) {
                if (mut_deliver_next()) {
                    set_new_snapshot();
                    break;
                }
                trigger_watchlist_resync("ring-miss");
                /* fall through to plain ACK below via the event-less path */
            }

            if (n_missing > 0) {
                /* Multi-pass round 1: send ADDR_QUERY for the addresses we
                 * just discovered. ADDR_RESPONSE handler will receive
                 * values, then call collect_missing_addresses again — if
                 * pointer chains revealed new addresses, another round
                 * fires. Convergence ends when collect returns 0. */
                pending_query_start = 0;
                addr_query_iter_count = 1;
                send_addr_query_response();
            } else if (g_watchlist_pending) {
                // Notify GC: new watchlist ready (initial load OR Phase D2
                // resync). seq = the mutation base both sides hold after
                // the chunk fetch completes.
                uint16_t num_chunks = RA_WATCHLIST_NUM_CHUNKS(g_watchlist_count);
                ra_watchlist_notify_t notify;
                notify.total_addr_count = ra_host_to_be16(g_watchlist_count);
                notify.num_chunks       = ra_host_to_be16(num_chunks);
                notify.seq              = ra_host_to_be16(wl_seq);
                resp.event_type = RA_EVT_WATCHLIST_UPDATE;
                resp.data_len   = ra_host_to_be16(sizeof(notify));
                uint8_t buf[sizeof(ra_esp_header_t) + sizeof(notify)];
                memcpy(buf, &resp, sizeof(resp));
                memcpy(buf + sizeof(resp), &notify, sizeof(notify));
                send_response(buf, sizeof(buf));
                set_new_snapshot();  // no missing addrs, process immediately
            } else {
                // No missing addresses, no watchlist update. Process immediately.
                set_new_snapshot();
                PendingEvent *evt = peek_event();
                if (evt) {
                    resp.event_type = evt->type;
                    resp.data_len   = ra_host_to_be16(evt->data_len);
                    uint8_t buf[sizeof(ra_esp_header_t) + RA_MAX_RESPONSE_DATA];
                    memcpy(buf, &resp, sizeof(resp));
                    memcpy(buf + sizeof(resp), (void*)evt->data, evt->data_len);
                    send_response(buf, sizeof(resp) + evt->data_len);
                    pop_event();
                } else {
                    resp.event_type = RA_EVT_NONE;
                    resp.data_len   = 0;
                    send_response((uint8_t*)&resp, sizeof(resp));
                }
            }
            break;
        }

        case RA_CMD_ADDR_RESPONSE: {
            /* ra-module returned values for the LAST ADDR_QUERY round we
             * sent (range [pending_query_start .. query_count-1]).
             * Fill those slots in query_values, then run the next pass
             * of collect_missing_addresses — if pointer chains revealed
             * NEW addresses, ship another ADDR_QUERY. Otherwise, the
             * cache is complete for this frame and we can do_frame. */
            if (rx_len < sizeof(ra_gc_header_t) + sizeof(ra_addr_response_t)) break;
            const ra_addr_response_t *ar = (const ra_addr_response_t*)(rx_data + sizeof(ra_gc_header_t));
            uint16_t resp_count = ra_be16_to_host(ar->addr_count);
            const uint8_t *resp_values = rx_data + sizeof(ra_gc_header_t) + sizeof(ra_addr_response_t);

            /* Throttled trace — useful when multi-pass is active. At 1s
             * cadence we see the rounds firing during chain discovery but
             * stay silent in steady state. */
            {
                static uint32_t last_resp_log = 0;
                uint32_t now_resp = millis();
                if (now_resp - last_resp_log >= 1000) {
                    last_resp_log = now_resp;
                    LOG_DBG("DEBUG=<<ADDR_RESP qc=%u resp_count=%u iter=%u first_qa=0x%08lX\r\n",
                            (unsigned)query_count, (unsigned)resp_count,
                            (unsigned)addr_query_iter_count,
                            query_count > 0 ? (unsigned long)query_addrs[0] : 0UL);
                }
            }

            /* Commit this batch immediately to the permanent watchlist + hash.
             * This is the key to multi-level chain resolution in a single vblank:
             * instead of accumulating all rounds into query_addrs and committing
             * only at convergence, we commit each round right here so the next
             * collect_missing_addresses() call finds these addresses via hash_lookup()
             * (O(1) hit → skip), exposing only the NEXT level of the pointer chain
             * as new misses. Without this, query_count hits ADDR_QUERY_MAX=128 on
             * round 1 and the second collect() call immediately returns 0 (cap full),
             * capping discovery at 128 addrs/vblank regardless of chain depth. */
            uint16_t round_start  = pending_query_start;
            uint16_t round_end    = (pending_query_start + resp_count < query_count)
                                    ? pending_query_start + resp_count
                                    : query_count;
            uint16_t round_actual = round_end - round_start;
            uint16_t round_new_addrs = 0;
            if (state == STATE_ACTIVE && round_actual > 0) {
                uint16_t before_round = watch_count;
                watchlist_append(query_addrs + round_start, resp_values, round_actual);
                if (watch_count > before_round)
                    round_new_addrs = watch_count - before_round;
            }

            /* Reset query buffer — all addresses from this round are now in
             * the hash, so we don't need to keep them in query_addrs for
             * fallback lookups in read_memory_ingame / peek_from_snapshot. */
            query_count        = 0;
            pending_query_start = 0;

            addr_query_pending = false;

            /* Run next convergence pass — peek_from_snapshot now sees the
             * newly committed addresses via hash_lookup (not query_addrs),
             * so deeper pointer chain levels resolve correctly. */
            uint16_t new_missing = (state == STATE_ACTIVE)
                                   ? collect_missing_addresses() : 0;
            addr_query_iter_count++;

            /* v0.27.8 — keep resolving within this vblank until convergence
             * OR the per-vblank mutation budget (8 = d2x MUT_RING) is spent.
             * Mutations minted this frame = wl_seq - wl_seq_at_snap_start. */
            uint16_t muts_this_vblank =
                (uint16_t)(wl_seq - g_vblank_stats.wl_seq_at_snap_start);
            if (new_missing > 0
                && muts_this_vblank < MAX_MUTATIONS_PER_VBLANK
                && addr_query_iter_count < ADDR_QUERY_MAX_ITERATIONS) {
                /* Another round needed — send ADDR_QUERY for just the
                 * new addresses. ra-module sees event_type=ADDR_QUERY
                 * and loops back with another ADDR_RESPONSE. */
                send_addr_query_response();
            } else {
                /* Converged (or iteration cap hit). All batches were already
                 * committed to the watchlist + hash per-round above, so no
                 * additional watchlist_append() is needed here.
                 * query_count == 0 at this point (reset after last round). */
                uint16_t new_addr_count = round_new_addrs;  /* from last round */
                set_new_snapshot();

                static uint32_t last_conv_log = 0;
                uint32_t now_ms = millis();
                if (now_ms - last_conv_log >= 2000) {
                    last_conv_log = now_ms;
                    LOG_DBG("DEBUG=CONVERGE iter=%u new_addrs(last)=%u watch=%u\r\n",
                            (unsigned)addr_query_iter_count,
                            (unsigned)new_addr_count,
                            (unsigned)watch_count);
                }

                /* Phase D2: any mutations minted by watchlist_append above
                 * (an eviction REMOVE_IDX and/or the APPEND itself) are in
                 * the ring in seq order — deliver the next one the Wii
                 * lacks. The Wii applies it and its next snapshot echoes
                 * the new seq; remaining mutations ship one per response. */
                if (state == STATE_ACTIVE && ra_seq_view != wl_seq) {
                    if (!mut_deliver_next()) {
                        trigger_watchlist_resync("ring-miss-conv");
                        ra_esp_header_t ack;
                        ack.magic       = RA_MAGIC_ESP_TO_GC;
                        ack.status      = (uint8_t)state;
                        ack.event_type  = RA_EVT_NONE;
                        ack.event_count = 0;
                        ack.data_len    = 0;
                        send_response((uint8_t*)&ack, sizeof(ack));
                    } else if (new_addr_count > 0) {
                        static uint32_t last_app_log = 0;
                        uint32_t now_a = millis();
                        if (now_a - last_app_log >= 2000) {
                            last_app_log = now_a;
                            LOG_DBG("DEBUG=CONVERGE delivering mutation seq=%u (appended %u, watch=%u)\r\n",
                                    (unsigned)ra_seq_view, (unsigned)new_addr_count,
                                    (unsigned)watch_count);
                        }
                    }
                } else {
                    /* Nothing to deliver — plain ack. */
                    ra_esp_header_t ack;
                    ack.magic       = RA_MAGIC_ESP_TO_GC;
                    ack.status      = (uint8_t)state;
                    ack.event_type  = RA_EVT_NONE;
                    ack.event_count = 0;
                    ack.data_len    = 0;
                    send_response((uint8_t*)&ack, sizeof(ack));
                }

                if (addr_query_iter_count >= ADDR_QUERY_MAX_ITERATIONS && new_missing > 0) {
                    static uint32_t last_iter_warn = 0; uint32_t now = millis();
                    if (now - last_iter_warn >= 5000) { last_iter_warn = now;
                        LOG_DBG("DEBUG=ADDR_QUERY iteration cap (%u) hit with %u still missing\r\n",
                                ADDR_QUERY_MAX_ITERATIONS, new_missing);
                    }
                }
            }
            break;
        }
        
        case RA_CMD_DEBUG_LOG: {
            /* Format: u8 msg_len, then msg_len bytes of ASCII text. */
            if (rx_len < sizeof(ra_gc_header_t) + 1) {
                LOG_DBG("DEBUG=GC: (DEBUG_LOG rx too short: %u bytes)\n",
                        (unsigned)rx_len);
                break;
            }
            const uint8_t *p = rx_data + sizeof(ra_gc_header_t);
            uint8_t msg_len = *p;
            if (rx_len < sizeof(ra_gc_header_t) + 1 + msg_len) {
                LOG_DBG("DEBUG=GC: (DEBUG_LOG truncated: have %u bytes, need %u)\n",
                        (unsigned)rx_len,
                        (unsigned)(sizeof(ra_gc_header_t) + 1 + msg_len));
                break;
            }

            /* ra-module diag passthrough (VBI/PHB timing, etc). These GC messages
             * are throttled (~2 per 5s); emitted at LOG_DBG (RA_LOG_LEVEL >= 2). */
            LOG_DBG("DEBUG=GC: %.*s\r\n", (int)msg_len, (const char *)(p + 1));

            /* Minimal ACK — we don't care about the response on the GC side. */
            ra_esp_header_t ack;
            ack.magic       = RA_MAGIC_ESP_TO_GC;
            ack.status      = (uint8_t)state;
            ack.event_type  = RA_EVT_NONE;
            ack.event_count = 0;
            ack.data_len    = 0;
            exi_spi_prepare_response((uint8_t*)&ack, sizeof(ack));
            break;
        }

        case RA_CMD_GET_WATCHLIST_CHUNK: {
            if (rx_len < sizeof(ra_gc_header_t) + sizeof(ra_watchlist_chunk_req_t)) {
                LOG_DBG("DEBUG=GET_CHUNK: rx_len too short (%u bytes)\n",
                        (unsigned)rx_len);
                break;
            }
            const ra_watchlist_chunk_req_t *req =
                (const ra_watchlist_chunk_req_t*)(rx_data + sizeof(ra_gc_header_t));
            uint16_t chunk_idx = ra_be16_to_host(req->chunk_index);

            uint16_t total      = g_watchlist_count;
            uint16_t n_chunks   = (uint16_t)RA_WATCHLIST_NUM_CHUNKS(total);
            uint16_t start      = chunk_idx * RA_WATCHLIST_CHUNK_ADDRS;
            uint16_t n_in_chunk = (start < total)
                ? (uint16_t)((total - start) < RA_WATCHLIST_CHUNK_ADDRS
                             ? (total - start) : RA_WATCHLIST_CHUNK_ADDRS)
                : 0;
            uint8_t is_last = (chunk_idx + 1 >= n_chunks) ? 1 : 0;

            uint32_t resp_data_len = sizeof(ra_watchlist_chunk_t) + n_in_chunk * 4;
            /* GC's GET_CHUNK request is 6 bytes (ra_gc_header_t + 2-byte
             * chunk_index payload). On the next transaction (the read), GC
             * clocks 6 bytes of MOSI during its write phase that ESP32 sends
             * as the start of tx_buf and the GC discards.
             *
             * So we must prepend 6 bytes of 0xFF padding before the actual
             * response — analogous to the 4-byte padding the IDENTIFY handler
             * uses for the 4-byte IDENTIFY write. Without this padding, GC
             * reads bytes [6..] of the response and sees chunk_hdr where it
             * expects ra_esp_header_t → magic check fails. */
            const uint32_t GC_WRITE_PADDING = sizeof(ra_gc_header_t)
                                            + sizeof(ra_watchlist_chunk_req_t); /* 6 */
            uint32_t total_len = GC_WRITE_PADDING + sizeof(ra_esp_header_t)
                                                 + resp_data_len;

            /* Unconditional diagnostic — chunk traffic is infrequent so we
             * never need to throttle it. */
            LOG_DBG("DEBUG=GET_CHUNK req: idx=%u (total_addrs=%u, n_chunks=%u, this_chunk=%u, is_last=%u, resp_len=%u, padded_len=%u)\n",
                    chunk_idx, total, n_chunks, n_in_chunk, is_last,
                    (unsigned)(sizeof(ra_esp_header_t) + resp_data_len),
                    (unsigned)total_len);

            uint8_t *buf = (uint8_t*)malloc(total_len);
            if (!buf) {
                LOG_ERR("DEBUG=GET_CHUNK: malloc(%u) FAILED\n", (unsigned)total_len);
                break;
            }

            /* 6 bytes of padding — consumed by GC's write phase, discarded. */
            memset(buf, 0xFF, GC_WRITE_PADDING);

            ra_esp_header_t hdr;
            hdr.magic       = RA_MAGIC_ESP_TO_GC;
            hdr.status      = (uint8_t)state;
            hdr.event_type  = RA_EVT_NONE;
            hdr.event_count = 0;
            hdr.data_len    = ra_host_to_be16((uint16_t)resp_data_len);
            memcpy(buf + GC_WRITE_PADDING, &hdr, sizeof(hdr));

            ra_watchlist_chunk_t chunk_hdr;
            chunk_hdr.chunk_index = ra_host_to_be16(chunk_idx);
            chunk_hdr.addr_count  = ra_host_to_be16(n_in_chunk);
            chunk_hdr.is_last     = is_last;
            chunk_hdr.reserved    = 0;
            memcpy(buf + GC_WRITE_PADDING + sizeof(ra_esp_header_t),
                   &chunk_hdr, sizeof(chunk_hdr));

            uint32_t *addr_out = (uint32_t*)(buf + GC_WRITE_PADDING
                                              + sizeof(ra_esp_header_t)
                                              + sizeof(chunk_hdr));
            for (uint16_t i = 0; i < n_in_chunk; i++)
                addr_out[i] = ra_host_to_be32(g_watchlist_addrs[start + i]);

            /* Log first 4 addresses so we can confirm the data on the GC side.
             * Build the whole line then emit once (the async ring is line-oriented;
             * piecewise writes from other producers could interleave). */
            if (n_in_chunk > 0 && RA_LOG_LEVEL >= 2) {
                char ab[96]; int ap = 0;
                ap += snprintf(ab + ap, sizeof(ab) - ap, "DEBUG=GET_CHUNK first addrs:");
                for (uint16_t i = 0; i < n_in_chunk && i < 4; i++)
                    ap += snprintf(ab + ap, sizeof(ab) - ap, " 0x%08lX",
                                   (unsigned long)g_watchlist_addrs[start + i]);
                if (n_in_chunk > 4) ap += snprintf(ab + ap, sizeof(ab) - ap, " ...");
                LOG_DBG("%s\r\n", ab);
            }

            /* Log first 22 bytes of the prepared response: 6 padding + the
             * first 16 bytes of actual content. */
            if (RA_LOG_LEVEL >= 2) {
                char hb[112]; int hp = 0;
                hp += snprintf(hb + hp, sizeof(hb) - hp, "DEBUG=GET_CHUNK resp head:");
                for (uint32_t i = 0; i < total_len && i < 22; i++)
                    hp += snprintf(hb + hp, sizeof(hb) - hp, " %02X", buf[i]);
                LOG_DBG("%s\r\n", hb);
            }

            exi_spi_prepare_response(buf, total_len);
            free(buf);
            if (is_last) g_watchlist_pending = false;
            break;
        }

        case RA_CMD_GET_CHAIN_CHUNK: {
            /* Phase C: serve the chain descriptor table built at game load.
             * No table (GC adapter semantics / emit failure) => node_count=0,
             * is_last=1 on ANY index — the console disables Phase C. Wire
             * node ints are BE (the 8B layout matches rc_phasec_node_t). */
            if (rx_len < sizeof(ra_gc_header_t) + sizeof(ra_chain_chunk_req_t))
                break;
            const ra_chain_chunk_req_t *req =
                (const ra_chain_chunk_req_t*)(rx_data + sizeof(ra_gc_header_t));
            uint16_t chunk_idx = ra_be16_to_host(req->chunk_index);

            uint32_t total = g_phasec_node_count;
            if (total > RA_MAX_CHAIN_NODES) {
                /* Console cap exceeded — disable Phase C rather than truncate
                 * (a truncated DAG has dangling parents). */
                LOG_ERR("ERROR=PhaseC: table %u > console cap %u — serving empty\r\n",
                        (unsigned)total, (unsigned)RA_MAX_CHAIN_NODES);
                total = 0;
            }
            uint32_t start = (uint32_t)chunk_idx * RA_CHAIN_CHUNK_NODES;
            uint16_t n_in_chunk = (start < total)
                ? (uint16_t)((total - start) < RA_CHAIN_CHUNK_NODES
                             ? (total - start) : RA_CHAIN_CHUNK_NODES)
                : 0;
            uint8_t is_last = (start + n_in_chunk >= total) ? 1 : 0;

            uint32_t resp_data_len = sizeof(ra_chain_chunk_t)
                                   + (uint32_t)n_in_chunk * RA_CHAIN_NODE_SIZE;
            const uint32_t GC_WRITE_PADDING = sizeof(ra_gc_header_t)
                                            + sizeof(ra_chain_chunk_req_t); /* 6 */
            uint32_t total_len = GC_WRITE_PADDING + sizeof(ra_esp_header_t)
                                                  + resp_data_len;
            uint8_t *buf = (uint8_t*)malloc(total_len);
            if (!buf) {
                LOG_ERR("ERROR=CHAIN_CHUNK: malloc(%u) FAILED\r\n", (unsigned)total_len);
                break;
            }
            memset(buf, 0xFF, GC_WRITE_PADDING);

            ra_esp_header_t hdr;
            hdr.magic       = RA_MAGIC_ESP_TO_GC;
            hdr.status      = (uint8_t)state;
            hdr.event_type  = RA_EVT_NONE;
            hdr.event_count = 0;
            hdr.data_len    = ra_host_to_be16((uint16_t)resp_data_len);
            memcpy(buf + GC_WRITE_PADDING, &hdr, sizeof(hdr));

            ra_chain_chunk_t chdr;
            chdr.chunk_index = ra_host_to_be16(chunk_idx);
            chdr.node_count  = ra_host_to_be16(n_in_chunk);
            chdr.total_nodes = ra_host_to_be16((uint16_t)total);
            chdr.is_last     = is_last;
            chdr.reserved    = 0;
            memcpy(buf + GC_WRITE_PADDING + sizeof(ra_esp_header_t), &chdr, sizeof(chdr));

            uint8_t *out = buf + GC_WRITE_PADDING + sizeof(ra_esp_header_t)
                               + sizeof(ra_chain_chunk_t);
            for (uint16_t i = 0; i < n_in_chunk; i++) {
                const rc_phasec_node_t *nd = &g_phasec_nodes[start + i];
                uint32_t ob = ra_host_to_be32(nd->operand);
                uint16_t pb = ra_host_to_be16(nd->parent);
                memcpy(out,     &ob, 4);
                memcpy(out + 4, &pb, 2);
                out[6] = nd->op;
                out[7] = nd->psize;
                out += RA_CHAIN_NODE_SIZE;
            }

            LOG_DBG("DEBUG=CHAIN_CHUNK req: idx=%u n=%u total=%u last=%u\r\n",
                    chunk_idx, n_in_chunk, (unsigned)total, is_last);
            exi_spi_prepare_response(buf, total_len);
            free(buf);

            /* CAPABILITY NEGOTIATION: the console just fetched the whole
             * table — it WILL walk chains and ship windows. Arm the
             * authoritative mode now (hooks were registered at game load but
             * no-op'd). Also lower the pressure-eviction threshold so the
             * flat list can never push the chain window into rotation
             * (freshness guard; dp verification section reserved too). */
            if (is_last && total > 0 && g_phasec_mm_ship && g_phasec_blob &&
                !g_phasec_auth) {
                uint32_t bm = (g_phasec_shipped + 7u) / 8u;
                uint32_t budget = 8192u - sizeof(ra_gc_header_t)
                                - sizeof(ra_snapshot_header_t)
                                - bm - g_phasec_blob_bytes
                                - 4u * g_phasec_dp_n;
                budget = (budget > 64u) ? budget - 64u : 0u;
                if (budget < g_watch_high_water)
                    g_watch_high_water = (uint16_t)budget;
                g_phasec_auth = true;
                LOG_DBG("DEBUG=PhaseC AUTH armed by console fetch: hw=%u dp=%u\r\n",
                        (unsigned)g_watch_high_water, (unsigned)g_phasec_dp_n);
            }
            break;
        }

        case RA_CMD_STATUS: {
            ra_esp_header_t resp;
            resp.magic = RA_MAGIC_ESP_TO_GC;
            resp.status = (uint8_t)state;
            resp.event_type = RA_EVT_NONE;
            resp.event_count = has_pending_event() ? 1 : 0;
            resp.data_len = 0;
            send_esp_header_response_4pad(&resp);
            break;
        }

        case RA_CMD_POLL: {
            /* WiiFlow polls every ~300ms during pre-boot LOAD_GAME wait, checking the
             * status byte until it sees GAME_LOADED (0x06). Response must reflect the
             * CURRENT state so WiiFlow can progress.  Without this handler the response
             * buffer would still hold the default_ack (status=0x01, stale) and WiiFlow
             * would time out — see [[project-wiiflow-exi]] poll-loop semantics. */
            ra_esp_header_t resp;
            resp.magic = RA_MAGIC_ESP_TO_GC;
            resp.status = (uint8_t)state;
            resp.event_type = RA_EVT_NONE;
            resp.event_count = has_pending_event() ? 1 : 0;
            resp.data_len = 0;
            send_esp_header_response_4pad(&resp);
            break;
        }

        case RA_CMD_GAME_RESET: {
            LOG_INFO("DEBUG=EXI: GAME_RESET\r\n");
            // TODO: reset rcheevos state
            state = STATE_WAIT_GAME_ID;
            ra_esp_header_t resp;
            resp.magic = RA_MAGIC_ESP_TO_GC;
            resp.status = (uint8_t)state;
            resp.event_type = RA_EVT_NONE;
            resp.event_count = 0;
            resp.data_len = 0;
            send_esp_header_response_4pad(&resp);
            break;
        }

        case RA_CMD_RESET_CREDENTIALS: {
            // WiiFlow asks us to forget the stored WiFi + RA credentials and
            // reboot into the config portal (replaces the nes-ra-adapter's
            // physical memory-card reset button). We can't safely erase NVS +
            // restart from inside the EXI servicer, so just ACK and arm the
            // flag — loop() performs the wipe + ESP.restart() on Core 1. The
            // ACK is best-effort: by the time the Wii reads, we may already be
            // rebooting, which is fine — the Wii side doesn't depend on it.
            LOG_INFO("DEBUG=EXI: RESET_CREDENTIALS requested\r\n");
            g_factory_reset_pending = true;
            ra_esp_header_t resp;
            resp.magic = RA_MAGIC_ESP_TO_GC;
            resp.status = (uint8_t)state;
            resp.event_type = RA_EVT_NONE;
            resp.event_count = 0;
            resp.data_len = 0;
            send_esp_header_response_4pad(&resp);
            break;
        }
    }
}

// ============================================================================
// Watchlist helpers
// ============================================================================

/* APPEND new (addr, value) pairs to watch_addresses/memory_data.
 * - Keeps INSERTION ORDER (not sorted!) — ra-module sends SNAPSHOT
 *   values in the same insertion order, so watch_addresses[i] and
 *   memory_data[i] correspond to ra-module's index i. Sorting here
 *   would desync ra-module's value-stream from our address-stream.
 * - peek_from_snapshot / collect_missing use linear search across
 *   watch_addresses; with watch_count typically ≤ 500 the cost is
 *   negligible vs an ADDR_QUERY round-trip.
 * - Buffers are pre-allocated at MAX size by ensure_watchlist_capacity()
 *   so this function never reallocs — safe to call from Core 1 while
 *   Core 0's do_frame is reading via peek_from_snapshot.
 * - watch_count is incremented LAST so a concurrent reader sees either
 *   the old size (and ignores the appended entries) or the new size
 *   (with all entries fully written).
 * - Caller must guarantee no duplicates (collect_missing already dedupes).
 * - Returns the new watch_count (== old + count on success, == old on fail). */
static uint16_t watchlist_append(const uint32_t *addrs, const uint8_t *values, uint16_t count) {
    if (count == 0) return watch_count;
    uint16_t old_count = watch_count;

    /* UNIFY (v0.30): NO filter, NO eviction here. The ra-module appends these
     * SAME addresses in-band when it receives the ADDR_QUERY, so both replicas
     * grow identically and Phase D2's count check verifies the append — the
     * wl_seq channel is now REMOVE-only.
     *  - No filter: collect_missing_addresses already deduped query_addrs vs
     *    the hash; filtering here (but NOT on the ra-module) would desync the
     *    replicas. A stray dup would land on BOTH sides -> still in sync.
     *  - No evict_lru: the REMOVE_IDX-vs-in-band-append ordering is the
     *    eviction milestone's job. Until then only the hard cap bounds growth
     *    (SMG peaks ~5300 < 6144, so it won't bite in normal sessions). */
    uint32_t new_count = (uint32_t)old_count + count;
    if (new_count > RA_MAX_WATCH_ADDRS) {
        LOG_ERR("DEBUG=watchlist_append: at hard cap (%u + %u > %u) — skipped (eviction pending)\r\n",
                (unsigned)old_count, (unsigned)count, (unsigned)RA_MAX_WATCH_ADDRS);
        return old_count;
    }
    if (!watch_addresses || !memory_data) return old_count;

    memcpy(watch_addresses + old_count, addrs, count * sizeof(uint32_t));
    memcpy(memory_data + old_count, values, count);
    /* Phase A.5: tag each new entry as freshly used so it doesn't get
     * evicted on the very next round. last_used_frame is alloc'd alongside
     * watch_addresses by ensure_watchlist_capacity. */
    if (last_used_frame) {
        for (uint16_t i = 0; i < count; i++) {
            last_used_frame[old_count + i] = lru_clock;
        }
    }
    /* Update hash table for fast lookups in peek_from_snapshot. */
    for (uint16_t i = 0; i < count; i++) {
        hash_insert(addrs[i], old_count + i);
    }
    /* Release-ordered publication: writer-visible memory barrier so
     * Core 0 sees the new entries before the new count. */
    __atomic_store_n(&watch_count, (uint16_t)new_count, __ATOMIC_RELEASE);

    /* UNIFY (v0.30): NO mutation minted. The ra-module already appended these
     * addresses in-band on ADDR_QUERY receipt; the snapshot count check is the
     * sync proof. (Was: build a WATCHLIST_APPEND event into mut_ring + wl_seq++.) */
    return (uint16_t)new_count;
}

/* Grow watch_addresses/memory_data to MAX capacity at game-load so
 * subsequent watchlist_append() calls never need to realloc (which
 * would race with Core 0's reads via peek_from_snapshot). */
static void ensure_watchlist_capacity() {
    if (!watch_addresses)
        watch_addresses = (uint32_t*)malloc(RA_MAX_WATCH_ADDRS * sizeof(uint32_t));
    /* memory_data is the hottest do_frame buffer (1 access per peek) —
     * force INTERNAL SRAM (v0.27.4). watch_addresses/last_used_frame are
     * cold during do_frame (collect/evict only), so they stay in PSRAM. */
    if (!memory_data) {
        memory_data = (uint8_t*)ra_hot_alloc(RA_MAX_WATCH_ADDRS, &g_memdata_internal);
        LOG_DBG("DEBUG=hotbuf: memdata=%s internal-free=%uKB\r\n",
                 g_memdata_internal ? "SRAM" : "PSRAM",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }
    if (!last_used_frame)
        last_used_frame = (uint32_t*)calloc(RA_MAX_WATCH_ADDRS, sizeof(uint32_t));
    if (!snap_changed)
        snap_changed = (uint8_t*)calloc(RA_MAX_WATCH_ADDRS, 1);  // B2 snapshot diff
    /* incremental collect reverse index (PSRAM): head table + node pool. */
    if (!incr_rev_head)
        incr_rev_head = (uint16_t*)ps_malloc(RA_MAX_WATCH_ADDRS * sizeof(uint16_t));
    if (!incr_rev_pool)
        incr_rev_pool = (struct incr_rev_node*)ps_malloc((size_t)INCR_REV_POOL_MAX * sizeof(struct incr_rev_node));
    if (incr_rev_head && incr_rev_pool)
        incr_rev_reset();
    else
        g_incr_collect_active = false;  // alloc failed -> fall back to full collect
}

// Rebuild watch_addresses/memory_data from a prefetch result, pre-expanding
// each (base, size) pair into individual byte addresses so chain[0] u32/u16
// reads land entirely in the hash from frame 1. The expanded entries form
// the STATIC region [0..static_watch_count-1] which is never evicted.
static void watchlist_update_if_changed(uint32_t new_count) {
    // Sort prefetch result IN-PLACE (insertion sort) by base address.
    for (uint32_t i = 1; i < new_count; i++) {
        uint32_t ka = prefetch_addrs[i];
        uint8_t  ks = prefetch_sizes[i];
        int j = (int)i - 1;
        while (j >= 0 && prefetch_addrs[j] > ka) {
            prefetch_addrs[j+1] = prefetch_addrs[j];
            prefetch_sizes[j+1] = prefetch_sizes[j];
            j--;
        }
        prefetch_addrs[j+1] = ka;
        prefetch_sizes[j+1] = ks;
    }

    ensure_watchlist_capacity();
    if (!watch_addresses || !memory_data || !last_used_frame) return;

    /* Phase A.5: pre-expand byte-fanouts. Each prefetch entry (base, size)
     * becomes `size` byte addresses in watch_addresses[]. Deduped against
     * already-inserted addresses via hash. Skips entries that would
     * overflow RA_MAX_WATCH_ADDRS. */
    hash_clear();
    uint16_t expanded = 0;
    for (uint32_t pi = 0; pi < new_count; pi++) {
        uint8_t sz = prefetch_sizes[pi];
        if (sz == 0) sz = 1;
        for (uint8_t b = 0; b < sz; b++) {
            if (expanded >= RA_MAX_WATCH_ADDRS) break;
            uint32_t a = prefetch_addrs[pi] + b;
            if (hash_lookup(a) >= 0) continue;  // dedup
            watch_addresses[expanded] = a;
            memory_data[expanded] = 0;
            last_used_frame[expanded] = LRU_PROTECTED;  // static = never evict
            hash_insert(a, expanded);
            expanded++;
        }
    }
    __atomic_store_n(&watch_count, expanded, __ATOMIC_RELEASE);

    /* Phase A.5: static_watch_count is now the BYTE-EXPANDED count. Anything
     * appended later (chain[1]/[2] discoveries) goes into the dynamic
     * region and is LRU-evictable. */
    static_watch_count = expanded;
    pending_remove_count = 0;
    lru_clock = 1;
    tomb_clear();  /* stale values from a previous game are poison */
    /* Phase D2: fresh mutation stream for the new list. The Wii's
     * ra_wl_seq resets to the notify's seq (0) when it fetches chunks. */
    wl_seq = 0;
    ra_seq_view = 0;
    mut_ring_clear();

    // Publish into dedicated watchlist buffer for chunked delivery
    memcpy(g_watchlist_addrs, watch_addresses, expanded * sizeof(uint32_t));
    g_watchlist_count   = expanded;
    g_watchlist_pending = true;

    LOG_DBG("DEBUG=Watchlist updated: %u addresses (%u chunks) [static_byte_count=%u, base_addrs=%u]\r\n",
             (unsigned)expanded, RA_WATCHLIST_NUM_CHUNKS(expanded),
             (unsigned)static_watch_count, (unsigned)new_count);
}

/* LRU eviction of dynamic entries (v0.25.0 — index-based, age-gated).
 *
 * Selects up to EVICT_BATCH_SIZE entries from the dynamic region
 * [static_watch_count..watch_count-1] that are age-cold (untouched for
 * >= LRU_MIN_AGE snapshots). Tombstones their last values (peek-miss
 * continuity), defrags watch_addresses/memory_data/last_used_frame
 * (shifts survivors down), and queues their array INDICES in
 * pending_remove_idx[] for emission via RA_EVT_WATCHLIST_REMOVE_IDX.
 *
 * Static entries (last_used_frame == LRU_PROTECTED) are NEVER picked,
 * preserving chain[0]/chain[1] byte-fanouts that the trigger evaluator
 * depends on. Insertion order of survivors is preserved → ra-module's
 * single-pass index compaction arrives at the same array. */
static void evict_lru(bool pressure) {
    if (watch_count <= static_watch_count) return;

    uint16_t dyn_start = static_watch_count;
    uint16_t dyn_count = watch_count - dyn_start;
    if (dyn_count == 0) return;

    /* v0.26.3 — RECLAIM HALF, ADAPTIVELY. The v0.25 single-256-batch +
     * fixed 10s age floor lost the race on SMG: galaxy transitions append
     * thousands of YOUNG entries, the floor found nothing cold exactly
     * when pressure peaked ("only 0/256 cold"), the list hit MAX and
     * appends got REJECTED (326x in one session) — rejected bytes read 0
     * forever, which both broke timers (bunny cheevo lost its ResetIf
     * clock) and poisoned "value changed" conditions.
     *
     * New shape (user-specified): target HALF the dynamic region, minted
     * as up to EVICT_MUT_MAX sequential REMOVE_IDX mutations (the Phase
     * D2 ring delivers them in order, one per response slot — multi-frame
     * cleanup is fine, the seq machinery guarantees lockstep). Age tiers
     * relax under pressure: prefer 10s-cold; if scarce take 2s-cold, then
     * 0.5s-cold. Entries the evaluator touched this/last frame (age<30)
     * are never evicted — dropping genuinely-hot bytes never helps.
     * (v0.28.8: these seconds are now ACCURATE — lru_clock counts game-frames
     * at ~59Hz, so 600/120/30 = 10s/2s/0.5s. Pre-0.28.8 it ticked per-snapshot
     * ≈7.4Hz, so the same tiers were really ~81s/16s/4s — far too conservative.)
     * Values are tombstoned so a later re-touch peeks the last real
     * value instead of 0. */
    #define EVICT_MUT_MAX 4   /* ≤4 mutations per call; MUT_RING=8 keeps room for appends */
    uint16_t target;
    uint32_t age_tiers[3];
    int num_tiers;
    if (pressure) {
        /* HIGH_WATER emergency: reclaim HALF, relaxing age under pressure.
         * Top tier 1800 (30s-cold); relaxes to 2s/0.5s only under burst
         * pressure (cap safety, per the v0.26.3 "galaxy appends thousands
         * young" lesson). */
        target = dyn_count / 2;
        if (target < EVICT_BATCH_SIZE) target = EVICT_BATCH_SIZE;
        if (target > EVICT_BATCH_SIZE * EVICT_MUT_MAX)
            target = EVICT_BATCH_SIZE * EVICT_MUT_MAX;
        age_tiers[0] = 1800u; age_tiers[1] = 120u; age_tiers[2] = 30u;
        num_tiers = 3;
    } else {
        /* Periodic GC: a SINGLE age floor (GC_MIN_AGE), NO target and NO
         * relaxation. Evicts every dead-weight entry up to the per-call cap;
         * if none qualifies, the scan finds nothing and we early-return below
         * (no hash rebuild = no soluço). */
        target = EVICT_BATCH_SIZE * EVICT_MUT_MAX;
        age_tiers[0] = GC_MIN_AGE;
        num_tiers = 1;
    }
    uint16_t evicted_total = 0;
    uint16_t mutations = 0;
    uint16_t before_all = watch_count;

    for (int tier = 0; tier < num_tiers && evicted_total < target
                       && mutations < EVICT_MUT_MAX; tier++) {
        uint32_t min_age = age_tiers[tier];

        for (;;) {
            if (evicted_total >= target || mutations >= EVICT_MUT_MAX) break;

            /* Select up to one batch of cold entries (ascending indices). */
            uint16_t want = (uint16_t)(target - evicted_total);
            if (want > EVICT_BATCH_SIZE) want = EVICT_BATCH_SIZE;
            pending_remove_count = 0;
            for (uint16_t i = dyn_start;
                 i < watch_count && pending_remove_count < want; i++) {
                if (last_used_frame[i] == LRU_PROTECTED) continue;
                /* clamp against frame_counter reset (game restart) so an
                 * underflowed age can't mass-evict freshly-touched entries */
                uint32_t age = (lru_clock >= last_used_frame[i])
                               ? (lru_clock - last_used_frame[i]) : 0;
                if (age < min_age) continue;
                pending_remove_idx[pending_remove_count++] = i;
                tomb_put(watch_addresses[i], memory_data[i]);
            }
            if (pending_remove_count == 0) break;  /* tier exhausted */

            /* Mint mutation #(wl_seq+1): complete REMOVE_IDX response into
             * the ring; mut_deliver_next() ships it on the next free slot.
             * Indices are pre-defrag, matching the Wii's skip-cursor. */
            {
                uint16_t seq = (uint16_t)(wl_seq + 1);
                wl_seq = seq;
                static uint8_t mbuf[sizeof(ra_esp_header_t)
                                    + sizeof(ra_watchlist_remove_idx_t)
                                    + EVICT_BATCH_SIZE * 2];
                ra_esp_header_t evt;
                evt.magic       = RA_MAGIC_ESP_TO_GC;
                evt.status      = (uint8_t)state;
                evt.event_type  = RA_EVT_WATCHLIST_REMOVE_IDX;
                evt.event_count = 0;
                uint16_t data_len = sizeof(ra_watchlist_remove_idx_t)
                                  + pending_remove_count * 2;
                evt.data_len    = ra_host_to_be16(data_len);
                memcpy(mbuf, &evt, sizeof(evt));
                ra_watchlist_remove_idx_t wri;
                wri.seq       = ra_host_to_be16(seq);
                wri.idx_count = ra_host_to_be16(pending_remove_count);
                memcpy(mbuf + sizeof(evt), &wri, sizeof(wri));
                uint8_t *idx_dst = mbuf + sizeof(evt) + sizeof(wri);
                for (uint16_t k = 0; k < pending_remove_count; k++) {
                    uint16_t be = ra_host_to_be16(pending_remove_idx[k]);
                    memcpy(idx_dst + k * 2, &be, 2);
                }
                mut_record(seq, mbuf, sizeof(evt) + data_len);
            }

            /* Defrag: shift survivors down, skipping the (ascending)
             * selected indices with a single cursor. No inline hash
             * deletes (tombstones broke probe chains — v0.18.3 saga);
             * rebuild afterwards. */
            uint16_t skip = 0;
            uint16_t write_idx = dyn_start;
            for (uint16_t read_idx = dyn_start; read_idx < watch_count; read_idx++) {
                if (skip < pending_remove_count
                    && pending_remove_idx[skip] == read_idx) {
                    skip++;
                    continue;
                }
                if (read_idx != write_idx) {
                    watch_addresses[write_idx] = watch_addresses[read_idx];
                    memory_data[write_idx]     = memory_data[read_idx];
                    last_used_frame[write_idx] = last_used_frame[read_idx];
                }
                write_idx++;
            }
            __atomic_store_n(&watch_count, write_idx, __ATOMIC_RELEASE);

            /* v0.31.1: hash rebuild deferred to ONCE after all batches (below) —
             * the batch loop selects by index from last_used_frame, never via
             * hash_lookup, so a stale hash mid-eviction is harmless. */
            evicted_total += pending_remove_count;
            mutations++;
            pending_remove_count = 0;
        }
    }

    if (evicted_total == 0) {
        /* Common for the periodic GC in steady state — nothing is dead yet.
         * No hash rebuild past this point, so this path is cheap (no soluço). */
        LOG_DBG("DEBUG=evict_lru[%s]: nothing evictable (dyn=%u, all age<%u)\r\n",
                pressure ? "press" : "gc",
                (unsigned)dyn_count, (unsigned)age_tiers[num_tiers - 1]);
        return;
    }
    g_vblank_stats.had_cleanup = 1;  /* cln=1 only when we actually evicted */
    /* v0.31.1: rebuild the hash ONCE after all batches, not per-batch. Only the
     * final array needs a correct hash (before the next collect_missing /
     * read_memory); the batches in between never hash_lookup. Cuts the ~40ms
     * ap_us seen on cln=1 frames to ~12ms. */
    hash_clear();
    for (uint16_t i = 0; i < watch_count; i++)
        hash_insert(watch_addresses[i], i);
    LOG_DBG("DEBUG=evict_lru[%s]: %u → %u (dropped %u in %u mutations, target=%u, seq=%u)\r\n",
            pressure ? "press" : "gc",
            (unsigned)before_all, (unsigned)watch_count,
            (unsigned)evicted_total, (unsigned)mutations,
            (unsigned)target, (unsigned)wl_seq);
}

#if RA_WEB_DASHBOARD
/* Refresh the cross-core snapshot the Core-0 web dashboard serves. Reads the
 * rc_client, so it must run ONLY on the rc_client/loop task (same task as
 * do_frame) — never from the HTTP handler. Cheap: two struct reads + a spinlock
 * copy. Called from on_game_loaded and throttled (~1 Hz) from loop(). */
static void publish_web_state(void) {
    ra_web_pub_t p;
    memset(&p, 0, sizeof(p));
    p.state = (uint8_t)state;
    const rc_client_game_t *g = g_client ? rc_client_get_game_info(g_client) : NULL;
    if (g) {
        p.game_id = g->id;
        strncpy(p.game_title, g->title ? g->title : "", sizeof(p.game_title) - 1);
        rc_client_user_game_summary_t s;
        rc_client_get_user_game_summary(g_client, &s);
        p.total           = (uint16_t)s.num_core_achievements;
        p.unlocked        = (uint16_t)s.num_unlocked_achievements;
        p.points_total    = s.points_core;
        p.points_unlocked = s.points_unlocked;
    } else {
        strncpy(p.game_title, gameName.c_str(), sizeof(p.game_title) - 1);
    }
    ra_web_publish(&p);
}

/* Full /api/state JSON (header + every achievement with LIVE state/progress).
 * Built on the loop task — the SAME task as do_frame, so rc_client reads are
 * never concurrent with it and the internal pthread mutex is uncontended (no
 * cross-core stall). GROUPING_PROGRESS asks rcheevos to bucket each achievement
 * as it stands THIS frame: ACTIVE_CHALLENGE (primed) / ALMOST_THERE (in
 * progress) / LOCKED / UNLOCKED — so "currently in progress" is live, not a
 * frozen snapshot. Gated by ra_web_client_active() in loop(), so it only runs
 * while a phone is actually watching — zero cost otherwise. */
static char  *g_web_json = NULL;      /* reused PSRAM render buffer */
static size_t g_web_json_cap = 0;

/* Category label the front-end filters/sorts on (derived from the bucket). */
static const char *web_cat_name(uint8_t bucket_type) {
    switch (bucket_type) {
        case RC_CLIENT_ACHIEVEMENT_BUCKET_UNLOCKED:
        case RC_CLIENT_ACHIEVEMENT_BUCKET_RECENTLY_UNLOCKED: return "unlocked";
        case RC_CLIENT_ACHIEVEMENT_BUCKET_ACTIVE_CHALLENGE:  return "challenge";
        case RC_CLIENT_ACHIEVEMENT_BUCKET_ALMOST_THERE:      return "progress";
        case RC_CLIENT_ACHIEVEMENT_BUCKET_UNSUPPORTED:       return "unsupported";
        default:                                             return "locked";
    }
}

/* Append s to buf[o..cap) as a JSON string body (quotes/backslashes/ctrl). */
static size_t web_json_esc(char *buf, size_t cap, size_t o, const char *s) {
    for (size_t i = 0; s && s[i] && o + 7 < cap; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') { buf[o++] = '\\'; buf[o++] = (char)c; }
        else if (c < 0x20)         { o += snprintf(buf + o, cap - o, "\\u%04x", c); }
        else                       { buf[o++] = (char)c; }
    }
    return o;
}

static void build_and_publish_web_json(void) {
    if (!g_client) return;
    rc_client_achievement_list_t *list = rc_client_create_achievement_list(
        g_client, RC_CLIENT_ACHIEVEMENT_CATEGORY_CORE,
        RC_CLIENT_ACHIEVEMENT_LIST_GROUPING_PROGRESS);
    if (!list) return;

    uint32_t total = 0;
    for (uint32_t b = 0; b < list->num_buckets; b++)
        total += list->buckets[b].num_achievements;

    size_t need = (size_t)total * 320 + 1024;   /* generous per-entry budget */
    if (need > g_web_json_cap) {
        char *nb = (char *)heap_caps_realloc(g_web_json, need, MALLOC_CAP_SPIRAM);
        if (!nb) { rc_client_destroy_achievement_list(list); return; }
        g_web_json = nb; g_web_json_cap = need;
    }
    char  *buf = g_web_json;
    size_t cap = g_web_json_cap;
    size_t o   = 0;

    rc_client_user_game_summary_t s;
    rc_client_get_user_game_summary(g_client, &s);
    const rc_client_game_t *g = rc_client_get_game_info(g_client);

    o += snprintf(buf + o, cap - o, "{\"state\":%u,\"game_id\":%lu,\"title\":\"",
                  (unsigned)state, (unsigned long)(g ? g->id : 0));
    o = web_json_esc(buf, cap, o, g ? g->title : "");
    o += snprintf(buf + o, cap - o,
                  "\",\"total\":%lu,\"unlocked\":%lu,\"points_total\":%lu,"
                  "\"points_unlocked\":%lu,\"achievements\":[",
                  (unsigned long)s.num_core_achievements,
                  (unsigned long)s.num_unlocked_achievements,
                  (unsigned long)s.points_core, (unsigned long)s.points_unlocked);

    int first = 1;
    for (uint32_t b = 0; b < list->num_buckets; b++) {
        const rc_client_achievement_bucket_t *bk = &list->buckets[b];
        for (uint32_t i = 0; i < bk->num_achievements; i++) {
            const rc_client_achievement_t *a = bk->achievements[i];
            if (o + 400 > cap) break;                /* safety: never overrun */
            if (!first) buf[o++] = ',';
            first = 0;
            o += snprintf(buf + o, cap - o, "{\"id\":%lu,\"t\":\"", (unsigned long)a->id);
            o = web_json_esc(buf, cap, o, a->title);
            o += snprintf(buf + o, cap - o, "\",\"d\":\"");
            o = web_json_esc(buf, cap, o, a->description);
            o += snprintf(buf + o, cap - o,
                          "\",\"badge\":\"%s\",\"pts\":%lu,\"st\":%u,\"pct\":%d,\"m\":\"",
                          a->badge_name, (unsigned long)a->points, (unsigned)a->state,
                          (int)a->measured_percent);
            o = web_json_esc(buf, cap, o, a->measured_progress);
            o += snprintf(buf + o, cap - o, "\",\"cat\":\"%s\",\"rare\":%d}",
                          web_cat_name(bk->bucket_type), (int)a->rarity);
        }
    }
    o += snprintf(buf + o, cap - o, "]}");
    rc_client_destroy_achievement_list(list);

    ra_web_set_state_json(buf, (unsigned)o);
}
#endif

// ============================================================================
// Game loading - runs on Core 0 when state == STATE_LOADING_GAME
// ============================================================================

static void on_game_loaded(int result, const char *error_message, rc_client_t *client, void *userdata) {
    if (result != RC_OK) {
        LOG_ERR("ERROR=rc_client: game load failed: %s\r\n", error_message ? error_message : "?");
        state = STATE_ERROR;
        return;
    }
    LOG_DBG("DEBUG=Game loaded, building initial watchlist...\r\n");

    const rc_memrefs_t *memrefs = rc_client_get_memrefs(client);
    uint32_t n = 0;
    if (memrefs)
        n = rc_memrefs_get_addresses(memrefs, prefetch_addrs, prefetch_sizes, PREFETCH_MAX, NULL, NULL);

    watch_count = 0;
    free(watch_addresses); watch_addresses = NULL;
    free(memory_data);     memory_data     = NULL;
    watchlist_update_if_changed(n);

    rc_client_set_read_memory_function(client, read_memory_ingame);

    /* v0.32 cache-primed do_frame gate: size a PSRAM rollback buffer to the
     * memref count and arm the gate. rc_client_do_frame will run the UPDATE,
     * call doframe_primed_cb, and defer (roll back, no eval) if any read would
     * have returned 0 — killing the resolver/evaluator divergence false
     * unlocks. ISR never touches this buffer (loop-task only) → PSRAM is safe. */
    {
        uint32_t mrc = rc_client_memref_count(client);
        free(g_memref_save); g_memref_save = NULL;
        g_rc_doframe_primed_cb = NULL;          /* disarm until the buffer is ready */
        g_rc_doframe_save_buf  = NULL;
        g_rc_doframe_save_cap  = 0;
        if (mrc > 0) {
            g_memref_save = (rc_memref_value_t*)ps_malloc((size_t)mrc * sizeof(rc_memref_value_t));
        }
        if (g_memref_save) {
            g_rc_doframe_save_buf     = g_memref_save;
            g_rc_doframe_save_cap     = mrc;
            g_rc_doframe_gate_enabled = 1;
            g_rc_doframe_primed_cb    = doframe_primed_cb;   /* arm last (all fields set) */
            LOG_DBG("DEBUG=doframe gate armed: %u memrefs, %u KB PSRAM rollback buf\r\n",
                     (unsigned)mrc, (unsigned)((size_t)mrc * sizeof(rc_memref_value_t) / 1024));
        } else {
            LOG_ERR("ERROR=doframe gate: ps_malloc(%u memrefs) failed — gate DISABLED\r\n",
                    (unsigned)mrc);
        }
    }

    /* (U1 alloc moved BELOW the Phase C block — its placement depends on
     * whether the authoritative mode armed for this game.) */

    /* Phase C step-0 census v2: how many AddAddress chains can the d2x walker
     * resolve? Nodes: INDIRECT = read(mask(parent)+mod); combine =
     * ALU_op(mask(parent), mod). v1 scored SMG 0% because Wii sets route every
     * pointer through a `& 0x01FFFFFF` / `- 0x80000000` combine — v2 walks
     * through combines and histograms the ALU ops + parent sizes the d2x must
     * implement. Ineligible chains stay on legacy ADDR_QUERY (Nintendont/GC
     * keep legacy). One-shot, read-only diagnostic. */
    {
        rc_phasec_census_t cs;
        rc_memrefs_phasec_census(rc_client_get_memrefs(client), &cs);
        uint32_t elig_all = cs.eligible + cs.eligible_mref_mod;
        uint32_t pct = cs.mm_indirect ? (elig_all * 100u) / cs.mm_indirect : 0;
        LOG_DBG("DEBUG=PhaseC census v3: mm=%u indirect=%u elig=%u eligMmod=%u (%u%%) combine=%u | "
                "inel: dp=%u xform=%u nonmem=%u chain=%u mod=%u op=%u\r\n",
                (unsigned)cs.mm_total, (unsigned)cs.mm_indirect, (unsigned)cs.eligible,
                (unsigned)cs.eligible_mref_mod, (unsigned)pct,
                (unsigned)(cs.mm_total - cs.mm_indirect),
                (unsigned)cs.inel_parent_dp, (unsigned)cs.inel_parent_xform,
                (unsigned)cs.inel_parent_nonmem, (unsigned)cs.inel_parent_chain,
                (unsigned)cs.inel_modifier, (unsigned)cs.inel_op);
        /* levels = mm->depth + 1 (depth 0 == single-deref chain) */
        LOG_DBG("DEBUG=PhaseC levels: 1=%u 2=%u 3=%u 4=%u 5=%u 6+=%u | "
                "leaf bytes: 1=%u 2=%u 3=%u 4=%u | wire=%uB\r\n",
                (unsigned)cs.depth_hist[0], (unsigned)cs.depth_hist[1],
                (unsigned)cs.depth_hist[2], (unsigned)cs.depth_hist[3],
                (unsigned)cs.depth_hist[4],
                (unsigned)(cs.depth_hist[5] + cs.depth_hist[6] + cs.depth_hist[7]),
                (unsigned)cs.leaf_bytes_hist[1], (unsigned)cs.leaf_bytes_hist[2],
                (unsigned)cs.leaf_bytes_hist[3], (unsigned)cs.leaf_bytes_hist[4],
                (unsigned)cs.eligible_value_bytes);
        {   /* which ALU ops + parent mask sizes the d2x walker must implement,
             * and WHAT the non-memref parents are (recall => legacy forever). */
            static const char* const kOp[18] = {
                "eq","lt","le","gt","ge","ne","none","mul","div","and",
                "xor","mod","add","sub","subp","adda","suba","ind" };
            static const char* const kOpnd[10] = {
                "addr","delta","const","fp","func","prior","bcd","inv","recall","?" };
            char buf[192]; int o = 0;
            for (unsigned s = 0; s < 18; s++)
                if (cs.op_hist[s] && o < (int)sizeof(buf) - 20)
                    o += snprintf(buf + o, sizeof(buf) - o, " %s=%u",
                                  kOp[s], (unsigned)cs.op_hist[s]);
            o += snprintf(buf + o, sizeof(buf) - o, " | nonmem:");
            for (unsigned s = 0; s < 10; s++)
                if (cs.nonmem_hist[s] && o < (int)sizeof(buf) - 20)
                    o += snprintf(buf + o, sizeof(buf) - o, " %s=%u",
                                  kOpnd[s], (unsigned)cs.nonmem_hist[s]);
            o += snprintf(buf + o, sizeof(buf) - o, " | psz:");
            for (unsigned s = 0; s < 32; s++)
                if (cs.parent_size_hist[s] && o < (int)sizeof(buf) - 16)
                    o += snprintf(buf + o, sizeof(buf) - o, " sz%u=%u",
                                  s, (unsigned)cs.parent_size_hist[s]);
            o += snprintf(buf + o, sizeof(buf) - o, " | rcl=%u cbase=%u",
                          (unsigned)cs.recall_hops, (unsigned)cs.const_base);
            LOG_DBG("DEBUG=PhaseC ops:%s\r\n", buf);
        }
    }

    /* Phase C node-table emission — the dependency-ordered DAG the d2x walker
     * will execute. Sizing gate: table bytes must fit the d2x module gap
     * (~144KB total incl. code) and blob bytes ride every SNAPSHOT. refused
     * counts ALL non-emitted INDIRECT chains (census-ineligible + the
     * emitter's stricter edge-size/immediate rules) — expect ≈ indirect -
     * (elig+eligMmod) + eligMmod; a big excess = edge shapes to look at. */
    {
        enum { PHASEC_NODE_CAP = 4096 };
        /* Disarm the authoritative hooks FIRST (a do_frame may run between
         * games), then free per-game state. */
        g_phasec_auth = false;
        g_rc_phasec_read = NULL;
        g_rc_phasec_covered = NULL;
        g_watch_high_water = WATCHLIST_HIGH_WATER;   /* legacy default until auth re-arms */
        free(g_phasec_mm_ship);   g_phasec_mm_ship = NULL; g_phasec_mm_total = 0;
        g_pc_reads = g_pc_inv = g_pc_defer = 0;
        g_phasec_dp_n = 0; g_phasec_dp_have = false; g_pc_dpmm = 0; g_pc_dpskip = 0;   /* Phase D */
        g_rc_lb_stat_active_ok = 0; g_rc_lb_stat_total = 0;   /* lb dirty-eval boot diag */
        free(g_phasec_nodes);     g_phasec_nodes = NULL;
        free(g_phasec_blob);      g_phasec_blob = NULL;
        free(g_phasec_blob_off);  g_phasec_blob_off = NULL;
        free(g_phasec_ship_node); g_phasec_ship_node = NULL;
        free(g_phasec_node_ship); g_phasec_node_ship = NULL;
        free(g_phasec_valid);     g_phasec_valid = NULL;
        free(g_phasec_fresh);     g_phasec_fresh = NULL;
        g_phasec_node_count = 0; g_phasec_blob_bytes = 0; g_phasec_shipped = 0;
        g_phasec_win_rx = 0;
        g_phasec_cmp_ok = g_phasec_cmp_bad = g_phasec_cmp_miss = g_phasec_cmp_inv = 0;
        rc_phasec_node_t* nodes =
            (rc_phasec_node_t*)ps_malloc(PHASEC_NODE_CAP * sizeof(rc_phasec_node_t));
        const void** keys =
            (const void**)ps_malloc(PHASEC_NODE_CAP * sizeof(void*));
        if (nodes && keys) {
            uint32_t blob = 0, shipped = 0, refused = 0;
            uint32_t n = rc_memrefs_phasec_emit(rc_client_get_memrefs(client),
                                                nodes, keys, PHASEC_NODE_CAP,
                                                &blob, &shipped, &refused);
            if (n == 0xFFFFFFFFu) {
                LOG_ERR("ERROR=PhaseC emit: node cap %u overflow — Phase C OFF this game\r\n",
                        (unsigned)PHASEC_NODE_CAP);
                free(nodes);
            } else {
                g_phasec_nodes = nodes;
                g_phasec_node_count = n;
                g_phasec_blob_bytes = blob;
                LOG_DBG("DEBUG=PhaseC table: nodes=%u (%uB) shipped=%u blob=%uB refused=%u\r\n",
                        (unsigned)n, (unsigned)(n * sizeof(rc_phasec_node_t)),
                        (unsigned)shipped, (unsigned)blob, (unsigned)refused);
                /* Shadow-ingestion side arrays: shipped<->node maps, blob
                 * prefix offsets, validity/fresh bitmaps, slot blob. Any
                 * alloc failure disables ingestion (blob NULL gates it). */
                if (n > 0 && shipped > 0 && blob > 0) {
                    uint32_t bm = (shipped + 7u) / 8u;
                    g_phasec_blob      = (uint8_t*) ps_malloc(blob);
                    g_phasec_blob_off  = (uint32_t*)ps_malloc((shipped + 1) * 4);
                    g_phasec_ship_node = (uint16_t*)ps_malloc(shipped * 2);
                    g_phasec_node_ship = (uint16_t*)ps_malloc(n * 2);
                    g_phasec_valid     = (uint8_t*) ps_malloc(bm);
                    g_phasec_fresh     = (uint8_t*) ps_malloc(bm);
                    if (g_phasec_blob && g_phasec_blob_off && g_phasec_ship_node &&
                        g_phasec_node_ship && g_phasec_valid && g_phasec_fresh) {
                        uint32_t s = 0, off = 0;
                        memset(g_phasec_valid, 0, bm);
                        memset(g_phasec_fresh, 0, bm);
                        memset(g_phasec_blob, 0, blob);
                        for (uint32_t i = 0; i < n; i++) {
                            g_phasec_node_ship[i] = 0xFFFF;
                            if (RC_PHASEC_SHIPPED(&nodes[i])) {
                                g_phasec_ship_node[s] = (uint16_t)i;
                                g_phasec_node_ship[i] = (uint16_t)s;
                                g_phasec_blob_off[s]  = off;
                                off += RC_PHASEC_WIDTH(&nodes[i]);
                                s++;
                            }
                        }
                        g_phasec_blob_off[s] = off;   /* == blob */
                        g_phasec_shipped = s;
                    } else {
                        free(g_phasec_blob);      g_phasec_blob = NULL;
                        free(g_phasec_blob_off);  g_phasec_blob_off = NULL;
                        free(g_phasec_ship_node); g_phasec_ship_node = NULL;
                        free(g_phasec_node_ship); g_phasec_node_ship = NULL;
                        free(g_phasec_valid);     g_phasec_valid = NULL;
                        free(g_phasec_fresh);     g_phasec_fresh = NULL;
                        LOG_ERR("ERROR=PhaseC shadow arrays alloc failed — ingest OFF\r\n");
                    }
                }

                /* AUTHORITATIVE map: dense mm index (U1-cursor / list order) ->
                 * shipped slot, from the emitter's dedup keys (keys[node] =
                 * memref ptr; the mm ptr equals its embedded memref ptr) —
                 * built BEFORE keys are freed. INTERNAL SRAM: this is the hot
                 * upd path (one O(1) read per chain per frame). Hooks armed
                 * only when everything (blob arrays + map) is in place. */
                if (g_phasec_blob && g_phasec_shipped > 0) {
                    const rc_memrefs_t* mrs = rc_client_get_memrefs(client);
                    const rc_modified_memref_list_t* l;
                    uint32_t total = 0;
                    for (l = &mrs->modified_memrefs; l; l = l->next)
                        total += l->count;
                    g_phasec_mm_ship = (uint16_t*)heap_caps_malloc(
                        (size_t)total * sizeof(uint16_t),
                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                    if (g_phasec_mm_ship) {
                        uint32_t g = 0, covered = 0;
                        for (l = &mrs->modified_memrefs; l; l = l->next) {
                            const rc_modified_memref_t* mm = l->items;
                            for (uint16_t k = 0; k < l->count; k++, mm++, g++) {
                                uint16_t ship = 0xFFFF;
                                if (mm->modifier_type == RC_OPERATOR_INDIRECT_READ) {
                                    for (uint32_t i2 = 0; i2 < n; i2++) {
                                        if (keys[i2] == (const void*)mm) {
                                            ship = g_phasec_node_ship[i2];
                                            break;
                                        }
                                    }
                                }
                                g_phasec_mm_ship[g] = ship;
                                if (ship != 0xFFFF) covered++;
                            }
                        }
                        g_phasec_mm_total  = g;
                        /* Phase D: derive the dp verification list — the SAME
                         * table scan the d2x does (order = node order, dedup
                         * on (parent,kind)) — and resolve each parent node to
                         * its ESP memref via the emitter keys (keys[node] =
                         * memref ptr; dp parents are never immediate nodes). */
                        g_phasec_dp_n = 0;
                        for (uint32_t i2 = 0; i2 < n; i2++) {
                            uint8_t dpk = (uint8_t)(nodes[i2].psize >> RC_PHASEC_PSZ_DP_SHIFT);
                            uint16_t pn = nodes[i2].parent;
                            uint16_t j2;
                            if (!dpk || pn == RC_PHASEC_PARENT_NONE) continue;
                            for (j2 = 0; j2 < g_phasec_dp_n; j2++)
                                if (g_phasec_dp_node[j2] == pn &&
                                    g_phasec_dp_kind_a[j2] == dpk) break;
                            if (j2 < g_phasec_dp_n) continue;
                            if (g_phasec_dp_n >= RC_PHASEC_MAX_DP) break; /* emitter caps; belt+suspenders */
                            g_phasec_dp_node[g_phasec_dp_n]   = pn;
                            g_phasec_dp_kind_a[g_phasec_dp_n] = dpk;
                            g_phasec_dp_memref[g_phasec_dp_n] = (void*)keys[pn];
                            g_phasec_dp_n++;
                        }
                        g_rc_phasec_read    = phasec_read_cb;
                        g_rc_phasec_covered = phasec_covered_cb;
                        /* CAPABILITY NEGOTIATION (Nintendont/legacy-console
                         * compat, 2026-07-02): auth does NOT arm here. The
                         * hooks are registered but no-op while g_phasec_auth
                         * is false — the console proves Phase C support by
                         * FETCHING the chain table (RA_CMD_GET_CHAIN_CHUNK
                         * handler arms on the last chunk). A console that
                         * never fetches (Nintendont, GC) runs pure legacy:
                         * collect resolves everything, slots stay ignored.
                         * (Pre-fix, arming at load left covered chains
                         * permanently unresolvable on Nintendont.) */
                        LOG_DBG("DEBUG=PhaseC READY: mm=%u covered=%u map=%uB SRAM dp=%u (auth arms on console fetch)\r\n",
                                (unsigned)g, (unsigned)covered,
                                (unsigned)(total * sizeof(uint16_t)),
                                (unsigned)g_phasec_dp_n);
                    } else {
                        LOG_ERR("ERROR=PhaseC mm_ship alloc failed — staying legacy\r\n");
                    }
                }
            }
        } else {
            LOG_ERR("ERROR=PhaseC emit: alloc failed — Phase C OFF this game\r\n");
            free(nodes);
        }
        free(keys);   /* dedup map only lives through the build */
    }

    /* U1 leaf-widx cache: size it to the exact modified_memref count (the chains
     * rc_modified_memref_can_skip walks). Placement is the Phase C SRAM dividend:
     * with the authoritative mode armed, U1 only serves the ~90 legacy (dp/exotic)
     * chains per frame, so it lives in PSRAM and returns its 18KB of internal
     * SRAM to the WiFi/TLS budget (the 2026-07-01 heap=304B network outage was
     * exactly this budget crossing zero). Without Phase C (no table / emit fail)
     * U1 is the hot per-chain path again -> INTERNAL SRAM as before. Register
     * the hooks only on a successful alloc; otherwise leave them NULL so
     * memref.c keeps the original B2 path (g_rc_leaf_unchanged). */
    {
        free(g_u1); g_u1 = NULL; g_u1_cap = 0;
        g_rc_u1_cached = NULL; g_rc_u1_populate = NULL; g_rc_u1_invalidate = NULL;
        uint32_t mmc = rc_client_modified_memref_count(client);
        /* Placement keys on TABLE-READY, not g_phasec_auth (which only arms
         * when the console fetches — AFTER this point; keying on auth put U1
         * back in internal SRAM and sank the heap floor to 17.5KB, wii6.log).
         * Table ready + Wii console → U1 serves ~5 exotic chains → PSRAM is
         * free SRAM. Table ready + legacy console (Nintendont never fetches)
         * → U1 serves ALL chains from PSRAM: ~0.3µs/entry on GC-sized sets
         * (~0.15ms/frame) — acceptable, still far better than no U1. */
        bool u1_psram = (g_phasec_nodes != NULL && g_phasec_shipped > 0);
        uint32_t caps = u1_psram ? MALLOC_CAP_SPIRAM
                                 : (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (mmc > 0) {
            g_u1 = (u1_entry_t*)heap_caps_malloc((size_t)mmc * sizeof(u1_entry_t), caps);
        }
        if (g_u1) {
            g_u1_cap = mmc;
            u1_clear_all();                 /* n=0 everywhere -> all miss until populated */
            g_rc_u1_cached     = rc_u1_cached;
            g_rc_u1_populate   = rc_u1_populate;
            g_rc_u1_invalidate = rc_u1_invalidate;
            LOG_DBG("DEBUG=U1 cache: %u chains, %u KB %s\r\n",
                     (unsigned)mmc, (unsigned)((size_t)mmc * sizeof(u1_entry_t) / 1024),
                     u1_psram ? "PSRAM (Phase C dividend)" : "internal SRAM");
        } else {
            LOG_ERR("ERROR=U1 cache: alloc(%u chains) failed — U1 OFF, original B2 path\r\n",
                    (unsigned)mmc);
        }
    }

#ifdef RC_SHADOW_VALUES
    /* Shadow-array arena: compact PSRAM {value,prior,changed} so the eval reads memref
     * values densely instead of strided 56-byte structs (MEMBENCH: 13.7x cheaper read;
     * the lever to crave 60 df/s at 80MHz). Sized to the exact memref count, re-allocated
     * each game load. rc_shadow_set_arena resets the slot counter; rc_de_build_game (first
     * do_frame) assigns the slots. Alloc fail -> arena NULL -> eval falls back to struct. */
    {
        static uint32_t* sv_value = NULL;
        static uint32_t* sv_prior = NULL;
        static uint8_t*  sv_changed = NULL;
        uint32_t svc = rc_client_memref_count(client);
        free(sv_value);   sv_value = NULL;
        free(sv_prior);   sv_prior = NULL;
        free(sv_changed); sv_changed = NULL;
        if (svc > 0) {
            sv_value   = (uint32_t*)ps_malloc((size_t)svc * sizeof(uint32_t));
            sv_prior   = (uint32_t*)ps_malloc((size_t)svc * sizeof(uint32_t));
            sv_changed = (uint8_t*)ps_malloc((size_t)svc);
        }
        if (sv_value && sv_prior && sv_changed) {
            rc_shadow_set_arena(sv_value, sv_prior, sv_changed, svc);
            LOG_DBG("DEBUG=shadow arena: %u memrefs, %u KB PSRAM\r\n",
                    (unsigned)svc, (unsigned)(((size_t)svc * 9) / 1024));
        } else {
            free(sv_value); sv_value = NULL;
            free(sv_prior); sv_prior = NULL;
            free(sv_changed); sv_changed = NULL;
            rc_shadow_set_arena(NULL, NULL, NULL, 0);   /* shadow off -> struct fallback */
            LOG_ERR("ERROR=shadow arena alloc failed (%u memrefs) -> shadow disabled\r\n", (unsigned)svc);
        }
    }
#endif

    const rc_client_game_t *game = rc_client_get_game_info(client);
    if (game) {
        gameName = String(game->title);
        LOG_INFO("DEBUG=Game: %s\r\n", game->title);
    }

    /* v0.27.8 — memory free right before the vblank stream starts (game
     * fully loaded: rcheevos structs allocated, watchlist built). This is
     * the steady-state baseline the do_frame/collect loop runs against. */
    LOG_INFO("DEBUG=mem at game-loaded: sram-free=%uKB (maxblk=%uKB) psram-free=%uKB watch=%u\r\n",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)watch_count);

    /* v0.27.1 warm-up: NOW the session exists (startsession fired during
     * begin_load_game), so spectator mode is togglable — not LOCKED. It
     * stays on through the boot discovery storm; processSnapshot lifts
     * it (plus rc_client_reset) after WARMUP_GAME_FRAMES. */
    rc_client_set_spectator_mode_enabled(client, 1);
    g_warmup_active = true;
    g_warmup_end_frame = 0;

    /* Transition to GAME_LOADED so WiiFlow sees status 0x06 (RA_STATUS_GAME_LOADED)
     * and knows it can proceed with the IOS reload + game boot.
     * STATE_ACTIVE (0x07) is set on the first SNAPSHOT received from ra-module. */
    state = STATE_GAME_LOADED;
    LOG_INFO("DEBUG=Game loaded, watching %u addresses — waiting for first SNAPSHOT\r\n", watch_count);

#if RA_WEB_DASHBOARD
    /* First snapshot for the LAN dashboard now that the game + summary exist,
     * and tell any already-connected phone to re-fetch the full list. */
    publish_web_state();
    ra_web_push_reload();
#endif
}

/* Login→load chaining (v0.24.0). server_call is async now, so the old
 * "login then immediately begin_load_game" sequence — which relied on the
 * login HTTP round-trip completing synchronously — must be event-driven:
 * begin_load_game only fires from the login completion callback. */
static char g_pending_hash[64];

static void login_then_load_cb(int result, const char *error_message, rc_client_t *client, void *userdata) {
    if (result != RC_OK) {
        LOG_ERR("ERROR=RA login failed (%d): %s\r\n", result,
                error_message ? error_message : "?");
        state = STATE_ERROR;
        return;
    }
    LOG_INFO("DEBUG=Login OK — loading game with hash: %s\r\n", g_pending_hash);
    // on_game_loaded sets state = STATE_ACTIVE or STATE_ERROR.
    rc_client_begin_load_game(g_client, g_pending_hash, on_game_loaded, NULL);
}

void loadGame(const char *hash) {
    // Destroy previous client if any. Bump the HTTP generation so any
    // in-flight response addressed to the old client is discarded by
    // http_drain_done instead of calling into freed memory.
    if (g_client) {
        http_gen++;
        rc_client_destroy(g_client);
        g_client = NULL;
    }

    g_client = rc_client_create(read_memory_nop, server_call);
    g_rc_leaf_unchanged = rc_leaf_unchanged_impl;   /* opt B2 INDIRECT-skip hook */
#ifdef RC_EVAL_PLAN
    g_rc_eval_hot_alloc = rc_eval_hot_psram_alloc;  /* RC_EVAL_PLAN hot arrays -> PSRAM */
#endif
    /* opt: reverse-hash incremental collect. Register the reverse-index builder;
     * the resolver skip is enabled later by incr_mark_dirty's first successful
     * marking pass (NOT here — see the note there; enabling before the pool is
     * allocated + chains dirtied would freeze the collect). */
    g_rc_chain_read_cb = incr_chain_read;
    LOG_DBG("DEBUG=incr_collect: %s (reverse-hash; rebuild every %d vblanks)\r\n",
             g_incr_collect_active ? "ACTIVE" : "off", INCR_REBUILD_EVERY);
#ifdef RC_INCREMENTAL_UPD
    LOG_DBG("DEBUG=optB: RC_INCREMENTAL_UPD ACTIVE (B1+B2 wired into live do_frame update)\r\n");
#else
    LOG_DBG("DEBUG=optB: RC_INCREMENTAL_UPD NOT compiled in\r\n");
#endif
    rc_client_enable_logging(g_client, RC_CLIENT_LOG_LEVEL_VERBOSE, log_message);
    rc_client_set_event_handler(g_client, event_handler);
    /* 2026-07-03 EXPERIMENT: HARDCORE ON — rc_client only ACTIVATES
     * leaderboards in hardcore (wii7.log: 48 lb loaded, 0 lb events in
     * softcore → their eval load was never exercised). Submissions may not
     * validate server-side (unvalidated client) — irrelevant for the load
     * test. RA_HARDCORE_MODE 0 restores softcore. Must be set BEFORE
     * begin_load_game (lb activation happens at load). */
    #ifndef RA_HARDCORE_MODE
    #define RA_HARDCORE_MODE 0
    #endif
    rc_client_set_hardcore_enabled(g_client, RA_HARDCORE_MODE);
    rc_client_set_get_time_millisecs_function(g_client, get_millisecs);

    /* v0.27.1: warm-up spectator is enabled in on_game_loaded, NOT here.
     * Enabling it BEFORE begin_load_game made rc_client skip the server
     * session entirely and LOCK spectator for the whole game ("Spectator
     * mode cannot be disabled if it was enabled prior to loading game" —
     * rc_client.c) → zero unlock submissions all session + the synthetic
     * "Unknown Emulator" warning 101000001 (field 2026-06-12 21:2x, was
     * mistaken for an ESP network problem). Enabled AFTER the load, the
     * session exists and spectator is freely togglable. */

    strncpy(g_pending_hash, hash, sizeof(g_pending_hash) - 1);
    g_pending_hash[sizeof(g_pending_hash) - 1] = '\0';

    // rc_client needs its own login session with correct URLs via server_call.
    // The game load is chained from the login callback (async HTTP).
    String ra_user = read_ra_user_from_eeprom();
    String ra_pass = read_ra_pass_from_eeprom();
    rc_client_begin_login_with_password(g_client, ra_user.c_str(), ra_pass.c_str(),
                                        login_then_load_cb, g_callback_userdata);
}

// ============================================================================
// EEPROM functions (reused from fpga-ra-adapter)
// ============================================================================
void beginEEPROM(bool force) {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(0) != EEPROM_ID_1 || EEPROM.read(1) != EEPROM_ID_2 || force) {
        EEPROM.write(0, EEPROM_ID_1);
        EEPROM.write(1, EEPROM_ID_2);
        for (int i = 2; i < EEPROM_SIZE; i++) EEPROM.write(i, 0);
        EEPROM.commit();
    }
}

bool isConfigured() { return EEPROM.read(2) == 1; }

void save_configuration_info_eeprom(String ra_user, String ra_pass) {
    int ul = ra_user.length(), pl = ra_pass.length();
    EEPROM.write(2, 1);
    EEPROM.write(3, ul);
    for (int i = 0; i < ul; i++) EEPROM.write(4 + i, ra_user[i]);
    EEPROM.write(4 + ul, pl);
    for (int i = 0; i < pl; i++) EEPROM.write(5 + ul + i, ra_pass[i]);
    EEPROM.commit();
}

String read_ra_user_from_eeprom() {
    int len = EEPROM.read(3);
    String s = "";
    for (int i = 0; i < len; i++) s += (char)EEPROM.read(4 + i);
    return s;
}

String read_ra_pass_from_eeprom() {
    int ul = EEPROM.read(3), len = EEPROM.read(4 + ul);
    String s = "";
    for (int i = 0; i < len; i++) s += (char)EEPROM.read(5 + ul + i);
    return s;
}

// ============================================================================
// WiFiManager setup (reused from fpga-ra-adapter with renamed AP)
// ============================================================================
const char head[] = "<style>#l,#i,#z{text-align:center}button{background-color:#0000FF;}</style>";
const char html_p1[] = "<p id='z'>Enter RetroAchievements credentials:</p>";
WiFiManagerParameter custom_p1(html_p1);
WiFiManagerParameter custom_user("un", "RA Username", "", 24, " required");
WiFiManagerParameter custom_pass("up", "RA Password", "", 14, " type='password' required");

String try_login_RA(String user, String pass) {
    String path = "r=login&u=" + user + "&p=" + pass;
    client.setInsecure();
    https.begin(client, base_url + path);
    https.setUserAgent("WII_RA_ADAPTER/0.1");
    int code = https.GET();
    if (code != HTTP_CODE_OK) { https.end(); return "null"; }
    String resp = https.getString();
    https.end();
    // Extract token
    int start = resp.indexOf("\"Token\":\"");
    if (start == -1) return "null";
    start += 9;
    int end = resp.indexOf("\"", start);
    return resp.substring(start, end);
}

// ============================================================================
// Core tasks
// ============================================================================
static void process_one_frame(void);

/* SNAPSHOT QUEUE wrapper (zero-frame-loss): run the fast-path frame if one is
 * pending, then DRAIN the lag ring — each queued entry is applied
 * (snapshot_apply_payload) and evaluated with its own exact data, in order.
 * g_df_busy tells the worker task to enqueue instead of touching live state;
 * a race at the busy=false boundary self-resolves on the next loop() tick
 * (this function also triggers on a non-empty ring, not just new_snapshot). */
void processSnapshot() {
    if (state != STATE_ACTIVE || !g_client) return;
    if (!new_snapshot && snapq_empty()) return;
    g_df_busy = true;
    if (new_snapshot) {
        new_snapshot = false;
        process_one_frame();
    }
    while (!snapq_empty()) {
        /* OVERLOAD collapse (wii8 lesson): a (near-)full ring means SUSTAINED
         * over-budget do_frames — draining 7 old entries one by one keeps us
         * permanently ~8 frames behind while the producer drops the NEWEST
         * (worst of both). Skip to the newest queued entry instead: the
         * skipped frames are lost either way (counted in g_q_drop, same
         * semantic), but lag stays bounded at ~1 frame == the legacy-collapse
         * behavior, gracefully. Consumer owns head — safe. */
        if (snapq_depth() >= SNAPQ_N - 1) {
            while (snapq_depth() > 1) {
                g_snapq_head = (uint8_t)((g_snapq_head + 1) % SNAPQ_N);
                g_q_drop++;
            }
        }
        uint8_t h = g_snapq_head;
        snapshot_apply_payload(g_snapq_buf[h], g_snapq_len[h]);
        g_snapq_head = (uint8_t)((h + 1) % SNAPQ_N);   /* release slot AFTER apply */
        process_one_frame();
    }
    g_df_busy = false;
}

/* One frame's evaluation: warmup window, frame-debt bookkeeping, the v0.32
 * gated do_frame, FRAME/CATCHUP telemetry. Reads the globals the applier set
 * (frame_counter, memory_data, blob...). Formerly the body of processSnapshot. */
static void process_one_frame() {
    /* Snapshot the frozen converged-frame stats up front: do_frame below runs
     * ~25ms during which later snapshots rewrite g_frame_log, so capture now to
     * keep the LOG_FRAME consistent with THIS frame (not a later convergence). */
    vblank_stats_t fl;
    memcpy(&fl, &g_frame_log, sizeof(fl));
    uint32_t fl_fc = g_frame_log_fc;

    /* v0.28.7 — measure the real processing cycle: wall-clock since the
     * previous processSnapshot body started. cycle - ap_us - df_us = the
     * overhead spent waiting on the ra-module/EXI for the next snapshot. */
    {
        static uint32_t last_ps_us = 0;
        uint32_t now_ps_us = (uint32_t)esp_timer_get_time();
        g_cycle_us = (last_ps_us > 0) ? (now_ps_us - last_ps_us) : 0;
        last_ps_us = now_ps_us;
    }

    /* All required addresses are guaranteed cached by the time we get
     * here — the SNAPSHOT/ADDR_RESPONSE handler chain iterates
     * ADDR_QUERY rounds until rc_memrefs_get_addresses reports no
     * missing addresses before setting new_snapshot=true. So the very
     * first do_frame sees all REAL values; rcheevos's per-trigger
     * WAITING state naturally suppresses spurious triggers (see
     * trigger.c:243 — WAITING + condition TRUE → reset to WAITING).
     *
     * FRAME-DEBT CATCH-UP (v0.26.1): rcheevos hit-count timers assume
     * exactly one do_frame per GAME frame ("3600 hits" = 60s @ 60fps).
     * Our delivered/evaluated rate is 55-95% of the game's — snapshots
     * skipped for convergence, mutation delivery or missed fires made
     * timers run SLOW (the SMG bunny "under 1 minute" cheevo unlocked
     * after >1 real minute, 2026-06-12: 18224 evaluations over 33236
     * game frames = the "minute" lasted 109s). The snapshot header now
     * carries the PPC VBI counter (true game-frame clock); we owe one
     * do_frame per elapsed game frame and pay the debt here, capped per
     * snapshot to bound CPU. Calling do_frame repeatedly with the SAME
     * memory image is semantically correct: Delta==Mem on the repeats
     * (no false edges) while frame-counted hits accumulate — exactly
     * what those frames meant on the console. Debt is itself capped:
     * a stall beyond ~10s means a load screen / pause, where the sets'
     * own ResetIf guards rule, not the timer. */
    /* v0.26.3 warm-up window: track the end frame on the first evaluated
     * snapshot; when the window closes, wipe every trigger state that was
     * primed against boot-storm garbage and go live. */
    if (g_warmup_active) {
        if (g_warmup_end_frame == 0) {
            g_warmup_end_frame = frame_counter + WARMUP_GAME_FRAMES;
            LOG_DBG("DEBUG=warmup: spectator until game frame %lu\r\n",
                     (unsigned long)g_warmup_end_frame);
        } else if (frame_counter >= g_warmup_end_frame) {
            rc_client_reset(g_client);
            rc_client_set_spectator_mode_enabled(g_client, 0);
            g_warmup_active = false;
            LOG_DBG("DEBUG=warmup complete: triggers reset, going live (watch=%u)\r\n",
                     (unsigned)watch_count);
        }
    }

    {
        static uint32_t prev_game_frame = 0;
        static uint32_t frame_debt = 0;
        /* RUN_MAX 4→32 (v0.27.2). processSnapshot runs on the LOW-priority
         * loopTask, which the prio-19 EXI task preempts at will — so a long
         * catch-up burst here never delays the SPI/snapshot path. The old
         * cap of 4 throttled catch-up below the game's 60 fps whenever
         * processSnapshot ran < 15×/s (it collapses multiple arrived
         * snapshots into one boolean new_snapshot, so it runs far less
         * often than snapshots arrive). The debt then pinned at the cap
         * and frames were dropped permanently → timers ran slow → the SMG
         * bunny <1min cheevo false-unlocked after >1 real minute.
         * CAP 600→1800 (~30s) gives a deeper recovery reservoir while
         * still discarding genuinely absurd jumps (ra-module restart). */
        #define FRAME_DEBT_RUN_MAX   32u
        #define FRAME_DEBT_CAP       1800u
        uint32_t elapsed = frame_counter - prev_game_frame;
        if (prev_game_frame == 0 || elapsed == 0 || elapsed > FRAME_DEBT_CAP)
            elapsed = 1;   /* first frame / frozen counter / absurd jump */
        prev_game_frame = frame_counter;
        frame_debt += elapsed;
        if (frame_debt > FRAME_DEBT_CAP) frame_debt = FRAME_DEBT_CAP;

        uint32_t run = (frame_debt < FRAME_DEBT_RUN_MAX)
                       ? frame_debt : FRAME_DEBT_RUN_MAX;
        frame_debt -= run;
        uint32_t this_df_us = 0;
        int df_ran = 0;
        if (run > 0) {
            uint32_t t0 = (uint32_t)esp_timer_get_time();
            /* v0.32 cache-primed gate: reset the per-frame miss counter, then
             * run the gated do_frame. If it DEFERS (a no-tombstone read missed),
             * the memref values were rolled back and NOTHING was evaluated —
             * the misses feed the next convergence round and we retry next
             * frame. Fail-open after MAX consecutive defers so a permanently
             * wild pointer can't freeze evaluation forever (tombstone/WAITING
             * guard the rare residual). */
            #define MAX_DOFRAME_DEFER 4u
            static uint8_t consec_defer = 0;
            g_update_miss_count = 0;
            g_rc_doframe_gate_enabled = (consec_defer < MAX_DOFRAME_DEFER) ? 1 : 0;
            /* opt B2: allow INDIRECT skipping only when the watchlist membership
             * is unchanged since the last do_frame (append/evict/defrag remap
             * indices -> snap_changed[] would be stale; a freshly-cached leaf
             * must resolve once). Count-stability within a session == membership
             * stability. Off-frames just do a full resolve (safe). */
            g_rc_collect_walk_budget = g_collect_walk_budget;   /* lever B: per-vblank walk cap */
            g_rc_indirect_skip_enabled = (g_b2_enabled && watch_count == g_last_doframe_watch_count) ? 1 : 0;
            g_last_doframe_watch_count = watch_count;
            g_rc_b1_skips = 0; g_rc_b2_skips = 0; g_rc_upd_resolves = 0;  /* per-frame */
            g_l1_hits = 0; g_l1_miss = 0;            /* STEP 0: per-frame L1 hit/miss */
            g_rc_upd_resolve_us = 0;                 /* STEP 0: resolve-only time within upd2 */
            g_u1_fast = 0; g_u1_pop = 0;             /* U1: per-frame fast skips / cache-miss populates */
#ifdef RC_DIRTY_EVAL
            g_rc_de_skipped = 0; g_rc_de_evaled = 0;  /* per-frame skip% */
            g_rc_lb_skipped = 0; g_rc_lb_evaled = 0;  /* per-frame lb skip% */
#ifdef RC_CLEAN_REPLAY
            g_rc_de_replayed = 0;  /* per-frame replay count */
#endif
#endif
            // disable the loop to pay for missing do_frame
            //for (uint32_t f = 0; f < run; f++) {
                rc_client_do_frame(g_client);
            //}
            if (g_rc_doframe_deferred) {
                /* Deferred: un-spend the frame debt (the frame wasn't evaluated)
                 * and skip the do_frame accounting; eval lands next frame once
                 * the cache primes. */
                consec_defer++;
                g_doframe_deferred_total++;
                frame_debt += run;
                if (frame_debt > FRAME_DEBT_CAP) frame_debt = FRAME_DEBT_CAP;
            } else {
                consec_defer = 0;
                df_ran = 1;
                this_df_us = (uint32_t)esp_timer_get_time() - t0;
                g_df_us += this_df_us;
                g_df_n  += 1;   /* exactly one do_frame ran (catch-up loop disabled) */
            }
        }

        /* Per-vblank frame summary (LOG_FRAME channel = INFO, RA_LOG_LEVEL >= 1).
         * Set RA_LOG_LEVEL 0 for minimum-noise mode (errors/banner/ACHIEVEMENT only).
         *
         * seq    = frame sequence number (PPC VBI counter from d2x)
         * ok     = Phase D2: seq+count matched, values accepted (1=ok, 0=skipped)
         * sa     = addr_count in the first SNAPSHOT message (d2x → ESP32)
         * cm     = cache misses in collect_missing (before do_frame)
         * it     = addAddress resolution depth (ADDR_QUERY iteration count)
         * mut    = 1 if watchlist mutation minted this vblank (APPEND/REMOVE_IDX)
         * cln    = 1 if evict_lru fired this vblank (LRU cleanup)
         * ap_us  = address processing time µs (SNAPSHOT receipt → new_snapshot)
         * df     = 1 if rc_client_do_frame executed this frame
         * df_us  = rc_client_do_frame total duration µs */
        /* cmr= : per-round resolver yield (≈ per addAddress level). Tells us
         * whether the cascade is climbing levels (e.g. 128,90,40,...) or
         * stalling at round 0 while peek_miss carries the rest (0 here with
         * ms>0 = the resolver isn't surfacing what do_frame reads). */
        char cmr_buf[56]; cmr_buf[0] = 0;
        {
            uint8_t nr = fl.collect_round; if (nr > 8) nr = 8;
            int off = 0;
            for (uint8_t r = 0; r < nr && off < (int)sizeof(cmr_buf) - 8; r++)
                off += snprintf(cmr_buf + off, sizeof(cmr_buf) - off,
                                r ? ",%u" : "%u", (unsigned)fl.cm_by_round[r]);
        }
        /* df_us splits into upd (pointer-chain resolve = rc_update_memref_values,
         * the 68KB of modified_memrefs) and evl (condition evaluation). Whichever
         * dominates decides the optimization: upd→move memrefs to SRAM (fits);
         * evl→fewer active achievements. */
#ifdef RC_DIRTY_EVAL
        unsigned long de_sk = (unsigned long)g_rc_de_skipped, de_ev = (unsigned long)g_rc_de_evaled;
#else
        unsigned long de_sk = 0, de_ev = 0;
#endif
#ifdef RC_CLEAN_REPLAY
        unsigned long de_rp = (unsigned long)g_rc_de_replayed;  /* warm+clean replays (subset of de_ev) */
#else
        unsigned long de_rp = 0;
#endif
        /* FRAME line has two forms, chosen at compile time by RA_LOG_LEVEL:
         *   level 1 (INFO)  = LEAN — health + the perf outcome only:
         *                     seq ok sa df df_us upd_us evl_us de_ev
         *   level 2 (DEBUG) = FULL — the LEAN set PLUS every mechanism/diagnostic
         *                     counter (collect ms/cm/cmr/it, dirty-eval de_sk/de_rp,
         *                     skips/cache/parallel/EXI, l1h/l1m/urs/u1f/u1p).
         * To move a field between the two, edit the lists below. */
#if RA_LOG_LEVEL >= 2
        LOG_FRAME_REC("FRAME seq=%lu ok=%d sa=%u ms=%u cm=%u cmr=%s it=%u mut=%d cln=%d ap_us=%lu apc_us=%lu df=%d df_us=%lu upd_us=%lu upd1=%lu upd2=%lu uda=%lu udb=%lu evl_us=%lu cyc_us=%lu par_a=%lu par_b=%lu runs=%lu de_sk=%lu de_ev=%lu de_rp=%lu lbs=%lu lbe=%lu b1sk=%lu b2sk=%lu upr=%lu csk=%lu cwk=%lu crb=%lu rvc=%u ov=%d mk=%lu l1h=%lu l1m=%lu urs=%lu u1f=%lu u1p=%lu\r\n",
                  (unsigned long)fl_fc,
                  (int)fl.data_ok,
                  (unsigned)fl.snap_addr_count,
                  (unsigned)fl.collect_miss_total,
                  (unsigned)fl.collect_cm,
                  cmr_buf,
                  (unsigned)fl.iter_depth,
                  (int)fl.had_mutation,
                  (int)fl.had_cleanup,
                  (unsigned long)fl.addr_proc_us,
                  (unsigned long)fl.cm_cpu_us,
                  df_ran,   /* v0.32: 1=evaluated, 0=deferred (cache not primed) */
                  (unsigned long)this_df_us,
                  (unsigned long)g_rc_update_us,
                  (unsigned long)g_rc_upd1_us,
                  (unsigned long)g_rc_upd2_us,
                  (unsigned long)g_upd_a_us,
                  (unsigned long)g_upd_b_us,
                  (unsigned long)g_rc_eval_us,
                  (unsigned long)g_cycle_us,
                  (unsigned long)g_par_a_us,
                  (unsigned long)g_par_b_us,
                  (unsigned long)g_par_runs,
                  de_sk, de_ev, de_rp,
                  (unsigned long)g_rc_lb_skipped, (unsigned long)g_rc_lb_evaled,
                  (unsigned long)g_rc_b1_skips, (unsigned long)g_rc_b2_skips,
                  (unsigned long)g_rc_upd_resolves,
                  (unsigned long)g_rc_collect_skips, (unsigned long)g_rc_collect_walks,
                  (unsigned long)g_incr_rebuilds,
                  (unsigned)incr_rev_count, (int)incr_rev_overflow, (unsigned long)g_incr_marks,
                  (unsigned long)g_l1_hits, (unsigned long)g_l1_miss,
                  (unsigned long)g_rc_upd_resolve_us,
                  (unsigned long)g_u1_fast, (unsigned long)g_u1_pop);
#else
        (void)cmr_buf; (void)de_sk; (void)de_rp;   /* level-2-only fields (computed above) */
        LOG_FRAME_REC("FRAME seq=%lu ok=%d sa=%u df=%d df_us=%lu upd_us=%lu evl_us=%lu de_ev=%lu\r\n",
                  (unsigned long)fl_fc,
                  (int)fl.data_ok,
                  (unsigned)fl.snap_addr_count,
                  df_ran,   /* v0.32: 1=evaluated, 0=deferred (cache not primed) */
                  (unsigned long)this_df_us,
                  (unsigned long)g_rc_update_us,
                  (unsigned long)g_rc_eval_us,
                  de_ev);
#endif
#if RA_FLIGHT_RECORDER
        /* Bad frame (over budget / convergence spike)? Dump the ring = the
         * FR_BEFORE-1 frames leading up + this one + the next FR_AFTER. */
        if (this_df_us > FR_BAD_DF_US || (unsigned long)fl.addr_proc_us > FR_BAD_AP_US)
            fr_trigger();
#endif
        /* incremental-collect counters reset per FRAME so csk/cwk/crb/mk are per-FRAME.
         * rvc=pool fill, ov=1 means overflow (reverse index incomplete -> bump pool).
         * mk=chains dirtied (0 in steady state with no movement is normal). */
        g_rc_collect_skips = 0; g_rc_collect_walks = 0; g_incr_rebuilds = 0; g_incr_marks = 0;

        /* Every 1000 frames: dump the full cache + LRU ages. Heavy (stalls
         * SPI for a few seconds) but by 1000 frames we already have enough
         * FRAME lines to debug; the user accepts the disruption. */
        static uint32_t last_cache_dump = 0;
        if (frame_counter - last_cache_dump >= 1000) {
            last_cache_dump = frame_counter;
            dump_cache_lru();
        }

        /* Catch-up + profiling telemetry (every 5s). df/s should track the
         * game's ~60 fps. df_us = mean do_frame µs, cm_us = mean
         * collect_missing µs, cm_skip = walks skipped in steady state. If
         * df/s << 60 with debt high: CPU-bound — the µs breakdown says
         * whether do_frame or the memref walk dominates. */
        g_doframe_count += df_ran;   /* count REAL do_frame EVALUATIONS (0 on a gate-deferred frame) */
        static uint32_t last_df_log = 0;
        uint32_t df_now = millis();
        if (df_now - last_df_log >= 5000) {
            uint32_t dt = df_now - last_df_log;
            uint32_t gf = frame_counter - g_df_prev_gameframe;
            {
#ifdef RC_SHADOW_VALUES
            extern uint32_t g_rc_shadow_count; extern volatile int g_rc_shadow_enabled;
            unsigned long sv_slots = (unsigned long)g_rc_shadow_count, sv_on = (unsigned long)g_rc_shadow_enabled;
#else
            unsigned long sv_slots = 0, sv_on = 0;
#endif
            LOG_FRAME("DEBUG=CATCHUP df/s=%lu gf/s=%lu ok0/s=%lu dfr/s=%lu debt=%lu | df_us=%lu cm_us=%lu cm_n=%lu cm_skip=%lu sv=%lu/%lu q=%lu/%lu/%lu\r\n",
                     (unsigned long)(g_doframe_count * 1000UL / (dt ? dt : 1)),
                     (unsigned long)(gf * 1000UL / (dt ? dt : 1)),
                     (unsigned long)(g_ok0_skips * 1000UL / (dt ? dt : 1)),
                     (unsigned long)(g_doframe_deferred_total * 1000UL / (dt ? dt : 1)),
                     (unsigned long)frame_debt,
                     (unsigned long)(g_df_n ? g_df_us / g_df_n : 0),
                     (unsigned long)(g_cm_n ? g_cm_us / g_cm_n : 0),
                     (unsigned long)g_cm_n,
                     (unsigned long)g_cm_skipped,
                     sv_slots, sv_on,
                     (unsigned long)g_q_enq, (unsigned long)g_q_hwm,
                     (unsigned long)g_q_drop);   /* q=enqueued/high-water/dropped */
            }
            g_doframe_count = 0;
            g_ok0_skips = 0;
            g_doframe_deferred_total = 0;
            g_df_prev_gameframe = frame_counter;
            g_df_us = g_df_n = g_cm_us = g_cm_n = g_cm_skipped = 0;
            g_q_enq = 0; g_q_hwm = 0;   /* q_drop cumulative (should stay 0) */
            last_df_log = df_now;
        }
    }
    /* INTENTIONAL: do NOT call watchlist_update_if_changed here.
     *
     * The initial watchlist (computed in on_game_loaded) is FROZEN for
     * the rest of the session. Dynamic pointer-chain addresses that
     * rcheevos discovers during do_frame are NOT added to the watchlist;
     * instead they are resolved each frame via the ADDR_QUERY multi-pass
     * inside the SNAPSHOT/ADDR_RESPONSE handlers.
     *
     * Why: updating the watchlist mid-stream wipes memory_data to zeros
     * (calloc), and ra-module on the GC side doesn't react to
     * WATCHLIST_UPDATE — it keeps sending SNAPSHOTs for the OLD address
     * list. The result is memory_data[i] containing the value of the
     * OLD addr[i] while watch_addresses[i] is the NEW addr[i] — total
     * cache corruption that causes spurious achievement triggers.
     *
     * Like Dolphin/MiSTer-PSX/NES-adapter, we treat memory access as
     * "live read" rather than "snapshot-then-cache-then-rebuild". The
     * ADDR_QUERY round trip IS our live read for dynamic addresses. */

    /* INTENTIONAL: do NOT reset query_count here. Core 1's SNAPSHOT
     * handler is the sole owner of query_count's per-frame reset (line
     * 1326). If we reset here too, do_frame's ~5-10 ms duration means
     * Core 0's tail can race PAST Core 1's start-of-next-frame collect_
     * missing and wipe its just-populated query_count to 0 — convergence
     * then sees query_count==0 and skips watchlist_append, so chain[2]
     * bytes never get committed and ADDR_QUERY spins forever on the same
     * misses. Symptom: SNAPSHOT addr_count stuck while ADDR_RESP fires
     * every frame returning the same 4 bytes. See heartbeat log dated
     * 2026-05-31 with FC BF 00 32 repeating ~25 Hz for the canonical
     * trace. */
}

void processEXI() {
    /* SOLE-OWNER worker step (project_exi_robust_handshake): the servicer task
     * owns the SPI driver and hands us a REQUEST via exi_spi_wait_request. We
     * run handle_exi_command (collect_missing, prepare_response), then signal the
     * servicer (exi_spi_submit_response) to arm the response + INT. The servicer
     * arms the RECEIVE descriptor for response-reads itself — so our (heavy)
     * collect_missing never holds the re-arm hostage. */
    g_worker_ticks++;
    static uint8_t buf[EXI_MAX_TRANSACTION_SIZE];  // STATIC to avoid stack overflow!
    size_t len = 0;
    if (exi_spi_wait_request(buf, &len, 20)) {
        if (len > 0)
            handle_exi_command(buf, len);
        exi_spi_submit_response();   // ALWAYS submit (even len==0) so the servicer,
                                     // which is blocked on g_resp_sem, never deadlocks
    }
}

void taskCore1(void *pvParameters) {
    uint32_t last_heartbeat = 0;
    uint8_t  last_published_state = 0xFE;  /* impossible AdapterState — forces 1st publish */
    for (;;) {
        /* Keep the default_ack template's status byte in sync with current state
         * BEFORE polling — so the next GC POLL/STATUS read returns the live state.
         * Only call when state actually changes, to avoid touching shared buffers
         * unnecessarily (no need to write the same byte every millisecond). */
        if ((uint8_t)state != last_published_state) {
            LOG_INFO("DEBUG=state transition %02X -> %02X\r\n",
                     last_published_state, (uint8_t)state);
            last_published_state = (uint8_t)state;
            exi_spi_update_status(last_published_state);
        }

        processEXI();
#if !RC_DO_FRAME_IN_LOW_PRIO_TASK
        processSnapshot();  /* do_frame on the EXI critical path */
#endif
        vTaskDelay(1);  // Yield to prevent watchdog
        
        // Heartbeat every 5 seconds
        uint32_t now = millis();
        if (now - last_heartbeat >= 5000) {
            last_heartbeat = now;
            int cs = gpio_get_level((gpio_num_t)EXI_PIN_CS);
            LOG_DBG("DEBUG=Core1 alive | CS=%d | transactions=%lu | rx_bytes=%lu | state=%d | heap=%u maxblk=%u\n",
                    cs, (unsigned long)exi_spi_get_transaction_count(),
                    (unsigned long)exi_spi_get_total_bytes_rx(), (int)state,
                    (unsigned)ESP.getFreeHeap(),
                    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            /* Per-command totals + delta since last heartbeat. Only print rows that
             * have nonzero total to keep the line short. Format:
             *   DEBUG=cmds | SNAP:1234(+300) POLL:5(+0) CHUNK:1(+0) IDENT:2(+0)
             * The (+N) is the count in the last 5s window — that's the post-IOS
             * cadence indicator (SNAP should hit ~300/5s = 60Hz). */
            char line[256];
            int pos = snprintf(line, sizeof(line), "DEBUG=cmds |");
            for (uint8_t c = 1; c <= 8 && pos < (int)sizeof(line) - 32; c++) {
                if (cmd_count[c] == 0) continue;
                uint32_t delta = cmd_count[c] - cmd_count_last[c];
                cmd_count_last[c] = cmd_count[c];
                pos += snprintf(line + pos, sizeof(line) - pos, " %s:%lu(+%lu)",
                                cmd_short_name(c), (unsigned long)cmd_count[c],
                                (unsigned long)delta);
            }
            LOG_DBG("%s\n", line);
        }
    }
}

// ============================================================================
// Arduino setup() and loop()
// ============================================================================
static void exi_watchdog_task(void *arg);  // defined below loop(); fwd-decl for setup()

#ifndef RA_MEMBENCH
#define RA_MEMBENCH 1
#endif
#if RA_MEMBENCH
/* Decisive pre-surgery probe for the shadow-array (memref->value -> SRAM) lever.
 * Measures, on the real S3 @80MHz, the read cost the eval pays vs the shadow target,
 * with NO rcheevos changes. Three patterns over ~SMG memref count:
 *   strided_psram = read one u32 out of every 56-byte slot (= the eval reading
 *                   ->value.value from rc_modified_memref_t structs; 56B stride busts
 *                   the 64KB cache at N*56=149KB working set -> true PSRAM cost).
 *   compact_psram = read from an 8-byte-packed array in PSRAM (N*8=21KB FITS cache
 *                   -> shows if compactness alone, via cache residency, is the win).
 *   compact_sram  = same packed array in INTERNAL SRAM (off the MSPI bus -> the
 *                   shadow-array ideal; immune to the 80M bandwidth cut + cache pressure).
 * If strided >> compact_sram the surgery pays (proportional to read volume). If
 * compact_psram already ~= compact_sram, compactness suffices (less invasive: keep
 * the shadow in PSRAM, no SRAM budget fight). If all three ~equal -> eval is CPU-bound,
 * ABORT the surgery (would be another clean-replay). */
static void ra_membench(void) {
  const int N = 2666;       /* ~SMG distinct memref count */
  const int STRIDE = 56;    /* ~sizeof(rc_modified_memref_t): the eval's read stride */
  const int PASSES = 8;     /* ~reads-per-memref per do_frame (≈8000 reads / ~1336 distinct) */
  uint8_t*  strided = (uint8_t*)heap_caps_malloc((size_t)N * STRIDE, MALLOC_CAP_SPIRAM);
  uint32_t* cpsram  = (uint32_t*)heap_caps_malloc((size_t)N * 8, MALLOC_CAP_SPIRAM);
  uint32_t* csram   = (uint32_t*)heap_caps_malloc((size_t)N * 8, MALLOC_CAP_INTERNAL);
  if (!strided || !cpsram || !csram) {
    ralog_printf("MEMBENCH alloc fail strided=%p cpsram=%p csram=%p\n",
                 (void*)strided, (void*)cpsram, (void*)csram);
    if (strided) heap_caps_free(strided);
    if (cpsram) heap_caps_free(cpsram);
    if (csram) heap_caps_free(csram);
    return;
  }
  for (int i = 0; i < N; i++) { *(volatile uint32_t*)(strided + (size_t)i * STRIDE) = (uint32_t)i; cpsram[i*2] = i; csram[i*2] = i; }
  volatile uint32_t sink = 0;
  unsigned long t0, t1, t2, t3;
  t0 = micros();
  for (int p = 0; p < PASSES; p++) for (int i = 0; i < N; i++) sink += *(uint32_t*)(strided + (size_t)i * STRIDE);
  t1 = micros();
  for (int p = 0; p < PASSES; p++) for (int i = 0; i < N; i++) sink += cpsram[i*2];
  t2 = micros();
  for (int p = 0; p < PASSES; p++) for (int i = 0; i < N; i++) sink += csram[i*2];
  t3 = micros();
  ralog_printf("MEMBENCH N=%d passes=%d sink=%u | strided_psram=%luus compact_psram=%luus compact_sram=%luus\n",
               N, PASSES, (unsigned)sink,
               (unsigned long)(t1 - t0), (unsigned long)(t2 - t1), (unsigned long)(t3 - t2));
  heap_caps_free(strided); heap_caps_free(cpsram); heap_caps_free(csram);
}
#endif

void setup() {
    /* v0.24.3 — route big allocations to PSRAM. Field forensics showed
     * the INTERNAL heap collapsing to 2.6KB after game load (rcheevos
     * parse objects + our arrays + the v0.24.x additions: httpTask
     * stack, UART TX ring, doubled watchlist), while mbedTLS needs
     * ~45KB per TLS session — every gameplay-time HTTP failed with -1
     * in 30ms (alloc fail) or -11 (limping into the read timeout). The
     * "network problem" was an OOM problem all along.
     *
     * Threshold 256, not 4096 (v0.24.4): two field findings. (a) The
     * Arduino prebuilt mbedTLS allocates with MALLOC_CAP_INTERNAL
     * explicitly — its 2x16KB record buffers CANNOT be sent to PSRAM
     * from here, so internal heap must always hold ~45KB contiguous
     * for TLS. (b) The post-load internal residency (~40KB) is not big
     * blocks — it's rcheevos' small-struct storm (119 achievements +
     * hundreds of memrefs, each allocation < 512B), which a 4096
     * threshold left internal (field: heap=43K maxblk=32756, and the
     * second 16KB TLS buffer missed fitting by ~50 bytes → -11). At
     * 256, those structs go to PSRAM (rcheevos allocates at load time,
     * not per-frame — PSRAM latency is irrelevant there). Task stacks,
     * FreeRTOS objects, the UART ring and the WiFi driver allocate
     * with explicit internal caps and are unaffected; our SPI/ISR
     * buffers are static .bss (internal) by construction. */
    heap_caps_malloc_extmem_enable(256);

    /* Big working buffers — allocated AFTER the extmem threshold is set
     * so they land in PSRAM (all ≥256B, task-context-only, never ISR). */
    prefetch_addrs   = (uint32_t*)calloc(PREFETCH_MAX, sizeof(uint32_t));
    prefetch_sizes   = (uint8_t*) calloc(PREFETCH_MAX, sizeof(uint8_t));
    g_watchlist_addrs= (uint32_t*)calloc(RA_MAX_WATCH_ADDRS, sizeof(uint32_t));
    addr_hash        = (hash_entry_t*)ra_hot_alloc(HASH_SIZE * sizeof(hash_entry_t), &g_hash_internal);
    tomb             = (tomb_entry_t*)calloc(TOMB_SIZE, sizeof(tomb_entry_t));
    /* Phase D2 mutation ring — MUT_RING * MUT_RESP_SZ ≈ 33KB → PSRAM (≥256B,
     * lands in PSRAM via heap_caps_malloc_extmem_enable(256) set above).
     * calloc zero-inits resp_len=0 so every slot starts empty. */
    mut_ring         = (mut_entry_t*)calloc(MUT_RING, sizeof(mut_entry_t));
    /* SNAPSHOT QUEUE ring: SNAPQ_N × 8KB PSRAM (raw verified payloads). Any
     * alloc failure leaves g_snapq_buf[0] NULL → the lag branch never takes
     * (full legacy collapse behavior) — graceful. */
    for (int qi = 0; qi < SNAPQ_N; qi++) {
        g_snapq_buf[qi] = (uint8_t*)ps_malloc(EXI_MAX_TRANSACTION_SIZE);
        if (!g_snapq_buf[qi]) {
            for (int qj = 0; qj < qi; qj++) { free(g_snapq_buf[qj]); g_snapq_buf[qj] = NULL; }
            g_snapq_buf[0] = NULL;
            LOG_ERR("ERROR=setup: snapq alloc failed — queue OFF (legacy collapse)\r\n");
            break;
        }
    }
    if (!prefetch_addrs || !prefetch_sizes || !g_watchlist_addrs || !addr_hash || !tomb || !mut_ring) {
        LOG_ERR("ERROR=setup: PSRAM buffer alloc failed\r\n");
    }
    /* Report where the hot buffers landed + remaining internal headroom. */
    LOG_DBG("DEBUG=hotbuf: hash=%s(%uKB) memdata=deferred internal-free=%uKB maxblk=%uKB\r\n",
             g_hash_internal ? "SRAM" : "PSRAM",
             (unsigned)(HASH_SIZE * sizeof(hash_entry_t) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));

    pinMode(RESET_PIN, INPUT);
    beginEEPROM(false);
    /* v0.18.10: bumped 115200 → 250000 (max of vscode serial monitor)*/
    /* v0.24.1: TX ring buffer (default 256B). Serial.printf blocks the
     * CALLING task when the ring is full — with the EXI task and loop()
     * sharing the UART with httpTask, a log burst used to stall the
     * realtime core mid-frame. 8KB absorbs any level-1 burst and drains
     * at ~25 KB/s in the background; was 16KB in v0.24.1, halved in
     * v0.24.3 to give internal heap back to mbedTLS. Must be set BEFORE
     * Serial.begin. */
    Serial.setTxBufferSize(8192);
    Serial.begin(250000);
    delay(250);
    /* Start the async log drain on the idle Core 0. From here, all ralog_printf /
     * LOG_* / flight-recorder output goes through the SRAM ring → Core 1 never
     * blocks on Serial. (Before this point g_log_async_ready=false → direct.) */
    xTaskCreatePinnedToCore(ralog_drain_task, "LogDrain", 4096, nullptr, 1, nullptr, 0);
    /* Mark all hash buckets as empty BEFORE any potential lookup.
     * Static .bss gives us all-zero which would alias to "addr=0 is here". */
    hash_clear();

    /* Bump on any meaningful change so the user can verify the flash actually
     * landed by looking at the serial log. Format: vMAJOR.MINOR.BUILDID where
     * BUILDID is __DATE__ __TIME__ from preprocessor. */
    ralog_printf("WII-RA-ADAPTER v0.32.2-wsteal (build %s %s)\n", __DATE__, __TIME__);
    /* lever B: arm the per-vblank collect walk budget from boot (also refreshed
     * each vblank). 0 = baseline; >0 = capped. Logged so the A/B run is labelled. */
    g_rc_collect_walk_budget = g_collect_walk_budget;
    ralog_printf("DEBUG=leverB: collect walk budget=%d (0=off)\n", g_collect_walk_budget);

#if RA_MEMBENCH
    /* one-shot shadow-array feasibility probe (read-only; see ra_membench comment) */
    ra_membench();
#endif

    // Initialize SPI slave for EXI
    if (!exi_spi_init(NULL)) {
        LOG_ERR("ERROR=SPI init failed!\r\n");
        while(1) yield();
    }
    LOG_DBG("DEBUG=SPI slave initialized\r\n");

    // WiFi + RA login (same flow as fpga-ra-adapter)
    if (!isConfigured()) {
        // Heap (not stack: the captive portal's web server is stack-hungry on the
        // 8KB loop task) and constructed HERE, with Serial up — not at static-init.
        WiFiManager* wm = new WiFiManager();
        wm->setDebugOutput(false);
        wm->setCustomHeadElement(head);
        wm->setDarkMode(true);
        wm->addParameter(&custom_p1);
        wm->addParameter(&custom_user);
        wm->addParameter(&custom_pass);
        wm->setAPStaticIPConfig(IPAddress(192,168,1,1), IPAddress(192,168,1,1), IPAddress(255,255,255,0));
        
        while (!isConfigured()) {
            LOG_DBG("DEBUG=Connect to 'WII_RA_ADAPTER' WiFi, open http://192.168.1.1\r\n");
            if (wm->startConfigPortal("WII_RA_ADAPTER", "12345678")) {
                String token = try_login_RA(custom_user.getValue(), custom_pass.getValue());
                if (token != "null") {
                    save_configuration_info_eeprom(custom_user.getValue(), custom_pass.getValue());
                }
            }
        }
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.begin();
        while (WiFi.status() != WL_CONNECTED) yield();
        LOG_DBG("DEBUG=WiFi OK\r\n");
        String token = try_login_RA(read_ra_user_from_eeprom(), read_ra_pass_from_eeprom());
        if (token != "null") {
            LOG_DBG("DEBUG=RA login OK\r\n");
        }
    }

    /* Kill the WiFi modem power-save (default WIFI_PS_MIN_MODEM). With
     * the radio dozing between DTIM beacons, the first TLS connect after
     * a network-idle stretch fails FAST (-1/-11) — which is exactly the
     * gameplay pattern: login + 214KB patch fetch work perfectly at load
     * (continuous traffic keeps the radio awake), then pings/awards fail
     * minutes later. Costs ~50mA — irrelevant, we're not on battery. */
    WiFi.setSleep(false);

    state = STATE_WAIT_GAME_ID;
    LOG_INFO("DEBUG=Ready, waiting for GameCube...\r\n");

    /* EXI task priority 19 (v0.24.2; was 1). Arduino-ESP32 leaves the
     * lwIP tcpip task UNPINNED (NO_AFFINITY, prio ~18) — during HTTP
     * activity it freely hops onto Core 1 and, at prio 18 vs our old
     * prio 1, preempted the EXI task mid-frame (the residual fire-rate
     * dips that survived the v0.24.0 core split). 19 sits above tcpip
     * and below the WiFi driver (23). Safe: taskCore1 blocks on the SPI
     * result queue / vTaskDelay every iteration, never busy-spins. */
    xTaskCreatePinnedToCore(taskCore1, "EXI_Task", 16384, nullptr, 19, &taskCore1Handle, 1);

    /* HTTP worker on Core 0, beside the WiFi/lwIP tasks. 12KB stack —
     * TLS+HTTPClient ran for months on loopTask's default 8KB, so 12KB
     * has margin while giving 8KB of internal heap back to mbedTLS
     * (stacks must stay internal RAM). Queue depth 8 is generous:
     * rcheevos issues at most a couple of concurrent calls (ping +
     * award); the single worker serializes them. */
    http_req_q  = xQueueCreate(8, sizeof(http_job_t));
    http_done_q = xQueueCreate(8, sizeof(http_done_t));
    xTaskCreatePinnedToCore(httpTask, "HTTP_Task", 12288, nullptr, 1, nullptr, 0);

    // Core-0 deadlock watchdog (prio 2, Core 0) — survives a Core-1 hang.
    xTaskCreatePinnedToCore(exi_watchdog_task, "EXI_WDOG", 3072, nullptr, 2, nullptr, 0);

    /* Achievement-unlock LED celebration. Dedicated task on Core 0 so the
     * RMT-backed neopixelWrite() never stalls the Core 1 EXI realtime path.
     * Prio 0, tiny stack (no rc_client here). */
    xTaskCreatePinnedToCore(ledCelebrateTask, "LED_Task", 2048, nullptr, 0, nullptr, 0);

#if RA_WEB_DASHBOARD
    /* LAN dashboard: mDNS (http://wii-ra.local/) + esp_http_server, pinned to
     * Core 0 beside the WiFi/lwIP + httpTask stack. WiFi is already connected
     * above. The Core 1 EXI/do_frame path is never touched by any HTTP work. */
    ra_web_init();
#endif
}

/* Core-0 deadlock watchdog (project_exi_robust_handshake). Runs on Core 0 so it
 * keeps printing even if Core 1 (servicer + worker/taskCore1 + loop/do_frame)
 * deadlocks. Dumps per-task heartbeat DELTAS each second — a delta of 0 on a
 * task that should be busy == that task hung; g_servicer_stage pinpoints where. */
static void exi_watchdog_task(void *arg) {
    (void)arg;
    uint32_t ls = 0, lw = 0, ld = 0;
    extern volatile uint8_t g_cmd_ring[CMD_RING];
    extern volatile uint8_t g_cmd_ring_idx;
    extern volatile uint8_t g_evt_ring[CMD_RING];
    extern volatile uint8_t g_evt_ring_idx;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t s = g_servicer_ticks, w = g_worker_ticks, d = g_doframe_ticks;
        if (!g_wdog_verbose) { ls = s; lw = w; ld = d; continue; }  /* silent unless chasing a hang */
        LOG_DBG("DEBUG=WDOG srv=+%lu(st=%u) wrk=+%lu df=+%lu\r\n",
                (unsigned long)(s - ls), (unsigned)g_servicer_stage,
                (unsigned long)(w - lw), (unsigned long)(d - ld));
        /* Servicer stalled (Wii stopped talking) -> dump the freeze forensics:
         * prev_stage (2=waiting for the Wii to READ a response; 3=waiting for the
         * Wii's next REQUEST) + the last 16 commands the Wii SENT and the last 16
         * events WE sent back, newest last. Together they replay the exchange +
         * mutation sequence (08->REMOVE/APPEND/ADDR_QUERY) that led to the freeze. */
        if (s == ls) {
            char cmds[80], evts[80]; int p = 0, q = 0;
            for (int i = 0; i < CMD_RING; i++) {
                p += snprintf(cmds + p, sizeof(cmds) - p, "%02X ",
                              g_cmd_ring[(g_cmd_ring_idx + i) & (CMD_RING - 1)]);
                q += snprintf(evts + q, sizeof(evts) - q, "%02X ",
                              g_evt_ring[(g_evt_ring_idx + i) & (CMD_RING - 1)]);
            }
            LOG_DBG("DEBUG=WDOG-FREEZE prev_stage=%u lastcmds=[ %s] lastevts=[ %s]\r\n",
                    (unsigned)g_servicer_prev_stage, cmds, evts);
        }
        ls = s; lw = w; ld = d;
    }
}

/* Wipe stored WiFi + RetroAchievements credentials and reboot into the config
 * portal. Triggered from WiiFlow via RA_CMD_RESET_CREDENTIALS — the software
 * replacement for the nes-ra-adapter's physical memory-card reset button.
 * Runs on Core 1 (loop task), NOT from the EXI servicer, so the NVS write +
 * restart can't corrupt an in-flight SPI transaction. */
static void do_factory_reset() {
    LOG_INFO("DEBUG=Factory reset: wiping WiFi + RA credentials\r\n");
    // 1. RA username/password + the "configured" flag live in our EEPROM blob.
    //    Force-wipe rewrites the header and zeros the rest -> isConfigured()==0.
    beginEEPROM(true);
    // 2. WiFi SSID/password are stored by the WiFi stack. resetSettings() erases
    //    them so the next boot falls back into the captive portal. Construct on
    //    heap with debug off (the global ctor is Serial-noisy — see note above).
    WiFiManager *wm = new WiFiManager();
    wm->setDebugOutput(false);
    wm->resetSettings();
    LOG_INFO("DEBUG=Factory reset done — rebooting into 'WII_RA_ADAPTER' portal\r\n");
    delay(300);   // let the Serial drain + the best-effort EXI ACK clock out
    ESP.restart();
}

void loop() {
    g_doframe_ticks++;
    // Service a pending credentials reset BEFORE anything that can block (the
    // WiFi reconnect loop below spins forever if creds were just erased).
    if (g_factory_reset_pending) {
        g_factory_reset_pending = false;
        do_factory_reset();   // does not return (ESP.restart())
    }
    // Reconnect WiFi if needed
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin();
        while (WiFi.status() != WL_CONNECTED) yield();
    }

    /* Deliver finished HTTP responses to rc_client (callbacks run HERE,
     * on the same task as do_frame — rc_client is not thread-safe). */
    http_drain_done();

#if RA_WEB_DASHBOARD
    /* Refresh the web dashboard snapshot ~1 Hz. Runs on THIS task (the rc_client
     * task) so reading rc_client is safe; the Core-0 HTTP handler only ever sees
     * the published copy. */
    {
        static uint32_t last_web_pub = 0;
        if (state >= STATE_GAME_LOADED && (millis() - last_web_pub) > 1000) {
            last_web_pub = millis();
            publish_web_state();                     /* tiny header — always cheap */
            /* Full achievement list only while a phone is actually watching, so
             * normal gameplay pays nothing for it. */
            if (ra_web_client_active(15000)) {
                build_and_publish_web_json();
                /* Rich-presence line — push only when it changes. */
                if (g_client) {
                    static char last_rp[128] = {0};
                    char rp[128];
                    if (rc_client_get_rich_presence_message(g_client, rp, sizeof(rp)) > 0
                        && strcmp(rp, last_rp) != 0) {
                        strncpy(last_rp, rp, sizeof(last_rp) - 1);
                        ra_web_push_rp(rp);
                    }
                }
            }
        }
    }
#endif

    // State machine. loadGame is async now (login → load → on_game_loaded
    // all via callbacks), so guard against re-entering it every iteration
    // while state is still LOADING.
    static bool load_started = false;
    if (state == STATE_LOADING_GAME) {
        if (!load_started) {
            load_started = true;
            loadGame(gameId.c_str());
        }
    } else {
        load_started = false;  // re-arm for the next LOAD_GAME / reset
    }

#if RC_DO_FRAME_IN_LOW_PRIO_TASK
    processSnapshot();
#endif
    vTaskDelay(1);
}

