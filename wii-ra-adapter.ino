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
 * Core 0: loop() -> processSnapshot() -> rc_client_do_frame()
 * Core 1: taskCore1() -> processEXI() -> SPI slave communication
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

Diagrama de ligação

Wii Memory Card Slot A          ESP32-S3 DevKit
────────────────────────        ─────────────────────
Pin 7  3.3V ──────────────────── 3.3V  (alimentação)
Pin 1  GND  ──────────────────── GND       
Pin 3  CS   ──────────────────── GPIO10  (EXI_PIN_CS)
Pin 4  CLK  ──────────────────── GPIO12  (EXI_PIN_CLK)
Pin 5  DI   ──────────────────── GPIO11  (EXI_PIN_MOSI)
Pin 6  DO   ──────────────────── GPIO13  (EXI_PIN_MISO)
Pin 2  INT  ──── não conecta


roxo
laranja
verde
marrom
vermelho
preto


 */

#include <WiFiManager.h>
#include <EEPROM.h>
#include <StreamString.h>
#include "rc_client.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "SPI.h"
#include <time.h>
#include <FastLED.h>
#include <Ticker.h>

// Project headers
#include "gc_ra_protocol.h"
#include "exi_spi_slave.h"
#include "wii_game_hashes.h"
#include "driver/gpio.h"
#include "PsramStream.h"

// rcheevos internals — needed for rc_memrefs_get_addresses()
extern "C" {
  #include "rc_internal.h"
  const rc_memrefs_t* rc_client_get_memrefs(const rc_client_t* client);
}

#define DEBUG 1

/* When 0, the patch response is passed to rcheevos as-is (no field stripping).
 * Useful for isolating whether a parse failure is caused by the strip helpers
 * mangling JSON, vs. by something else in the response. The full SMG patch is
 * ~800KB and fits easily in 8MB PSRAM, so disabling strips is safe for triage. */
#define STRIP_JSON_RESPONSE 0

#define EEPROM_SIZE 2048
#define EEPROM_ID_1 142
#define EEPROM_ID_2 210  // Bumped from GC adapter (209) to force re-init on first Wii flash

#define PIN_RGB 48
#define NUM_LEDS 1
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

// ============================================================================
// LED (reused from fpga-ra-adapter)
// ============================================================================
enum LedMode { LED_OFF, LED_ON, LED_BLINK_SLOW, LED_BLINK_MEDIUM, LED_BLINK_FAST };
enum LedColor { LED_RED, LED_GREEN, LED_YELLOW };

struct Led {
    LedMode mode;
    bool ledState;
    Ticker ticker;
    CRGB color;
};

CRGB leds[NUM_LEDS];
Led ledRGB = { LED_OFF, false, Ticker(), CRGB::Black };

void updateLed(Led* led) {
    /*if (led->mode == LED_OFF) { leds[0] = CRGB::Black; }
    else if (led->mode == LED_ON) { leds[0] = led->color; }
    else { led->ledState = !led->ledState; leds[0] = led->ledState ? led->color : CRGB::Black; }
    FastLED.show(5);*/
}

void configureTicker(Led* led) {
   /* led->ticker.detach();
    float interval = 0;
    switch (led->mode) {
        case LED_BLINK_SLOW: interval = 0.9; break;
        case LED_BLINK_MEDIUM: interval = 0.6; break;
        case LED_BLINK_FAST: interval = 0.3; break;
        default: updateLed(led); return;
    }
    led->ticker.attach(interval, updateLed, led);*/
}

void setSemaphoreLED(LedMode mode, LedColor color) {
    /*ledRGB.mode = mode;
    switch (color) {
        case LED_RED: ledRGB.color = CRGB::Red; break;
        case LED_GREEN: ledRGB.color = CRGB::Green; break;
        case LED_YELLOW: ledRGB.color = CRGB::Yellow; break;
    }
    configureTicker(&ledRGB);*/
}

// ============================================================================
// Memory tracking - adapted for 32-bit addresses and snapshot model
// ============================================================================
uint32_t *watch_addresses = NULL;     // 32-bit addresses (was uint16_t in fpga-ra-adapter)
uint16_t watch_count = 0;
uint8_t *memory_data = NULL;          // Current values for each watched address
volatile bool new_snapshot = false;    // Flag: new snapshot received from GC
volatile uint32_t frame_counter = 0;

/* Phase A.5 — LRU tracking. last_used_frame[i] stores the value of
 * lru_clock at the last hash-hit for watch_addresses[i]. Static entries
 * (and their byte-fanouts) get UINT32_MAX so they're never evicted. */
static uint32_t *last_used_frame = NULL;
static uint32_t lru_clock = 0;  /* monotonic, ticks each SNAPSHOT */
#define LRU_PROTECTED 0xFFFFFFFFu

/* Phase A.5 — STATIC region [0..static_watch_count-1] holds the rcheevos
 * prefetch result EXPANDED to byte-granularity (every byte of every static
 * u8/u16/u32 base gets its own entry). Never evicted — protected via
 * last_used_frame = LRU_PROTECTED. DYNAMIC tail [static_watch_count..]
 * holds chain[1]/[2] addresses and any masked-chain bytes discovered by
 * multi-pass; subject to LRU eviction when watch_count > WATCHLIST_HIGH_WATER. */
static uint16_t static_watch_count = 0;

/* When > 0, pending REMOVE event payload waiting to be emitted on the
 * next response. Holds the addresses (BE wire order) to be removed.
 * Sized for one batch (EVICT_BATCH_SIZE). */
#define EVICT_BATCH_SIZE 256
static uint32_t pending_remove_addrs[EVICT_BATCH_SIZE];  /* BE-encoded */
static uint16_t pending_remove_count = 0;
/* DIAG (v0.18.7): set after REMOVE emission, consumed at next SNAPSHOT
 * to print ra-module's actual post-REMOVE count vs ESP's expectation. */
bool snap_post_remove_armed = false;
uint16_t snap_post_remove_expected = 0;

/* ra_module_expected_count tracks what we BELIEVE ra-module's watchlist
 * count is at the moment its NEXT read of our prepped response fires. This
 * is essential for canonical TX padding: ra-module's RX offset = its NEXT
 * TX width = 10 + (count after it processes our response). If we pad based
 * on the count it currently shows in its SNAPSHOT (stale by 1 tx after a
 * REMOVE/APPEND emit), the response data lands at the wrong byte and
 * ra-module misses it. Updated on every count-changing emit AND on every
 * SNAPSHOT receive (truth sync). */
static uint16_t ra_module_expected_count = 0;
#ifndef WATCHLIST_HIGH_WATER
/* Phase A.5 ENABLED: trigger LRU eviction when watch_count would exceed
 * the high-water mark. evict_lru drops up to EVICT_BATCH_SIZE=256 LRU
 * dynamic entries (chain-root-safe), leaving headroom before next eviction.
 *
 * v0.19.3-60hz: lowered from 768 to 500 to force eviction during
 * the very first multi-pass round (boot → initial CONVERGE adds ~128 →
 * 359 + 128 = 487 close; next round pushes past 500). This makes the
 * off-by-2 reproducible without needing to play to a specific gameplay
 * state. Will return to 768 once the off-by-N root cause is fixed.
 *
 * v0.19.3-60hz: raised to 900. Phase B SPI is now reliable (the SETTLE
 * ra_sleep(1) in ra_send_phase_b fixed the slave-not-armed race for large
 * responses). Remaining issue was LRU thrashing: 500 was forcing chain[0]
 * + chain[1] to be evicted on every chain[2] arrival, then chain[2] on
 * every chain[1] re-fetch, oscillating forever and never letting the
 * trigger evaluator reach chain[3] (the ability byte at 0x00000A64).
 *
 * Capacity check for Kirby RtDL chain navigation:
 *   359 static + 4 chain[0] + 128 chain[1] + 128 chain[2] + ~1-4 chain[3]
 *   = ~620-625 worst case. 900 leaves comfortable headroom and still
 *   activates eviction long before hitting RA_MAX_WATCH_ADDRS=1024. */
#define WATCHLIST_HIGH_WATER 900
#endif
/* Forward decl — watchlist_append calls evict_lru, but evict_lru lives
 * near watchlist_update_if_changed (after the watchlist_append def). */
static void evict_lru(void);

// Prefetch and watchlist state — used for two-pass processing
#define PREFETCH_MAX 2048   /* rc_memrefs_get_addresses returns (addr,size) pairs; 1024 was too small for SSBM */
static uint32_t prefetch_addrs[PREFETCH_MAX];
static uint8_t  prefetch_sizes[PREFETCH_MAX];

#define ADDR_QUERY_MAX 128  // max addresses per real-time query
uint32_t query_addrs[ADDR_QUERY_MAX];
uint8_t  query_values[ADDR_QUERY_MAX];
uint16_t query_count = 0;
static volatile bool addr_query_pending = false;

/* Soft cap on dynamic watchlist growth — defends against ra-module lockup
 * observed at watch=855 with full Kirby cheevo set (2026-05-31 / 2026-06-01).
 * Beyond this count, watchlist_append rejects new entries (instead of
 * growing past hardware limits). Proper LRU eviction comes in a later
 * milestone (requires d2x rebuild for RA_EVT_WATCHLIST_TRUNCATE). */
#ifndef WATCHLIST_SOFT_CAP
#define WATCHLIST_SOFT_CAP 512
#endif

/* Rejected-address tracking — prevents multi-pass spin when SOFT CAP
 * rejects a batch. Without this, rc_memrefs_get_addresses keeps emitting
 * the same rejected addresses every frame, collect_missing keeps queuing
 * ADDR_QUERY for them, ra-module keeps reading PPC for those bytes, ADDR_
 * RESPONSE fills query_values, convergence tries to append → SOFT CAP
 * rejects again → infinite loop at full SPI speed, starving the WiFi/HTTP
 * stack on Core 0 (observed Jun 1 2026: HTTP timeout -11 with watch=408
 * and ADDR rate 28Hz). Rejecting an addr ONCE marks it here so subsequent
 * collect_missing skips it. Ring-buffer; oldest entries evicted on wrap. */
#define REJECT_SET_MAX 512
static uint32_t reject_set[REJECT_SET_MAX];
static uint16_t reject_count = 0;
static uint16_t reject_write_idx = 0;

static bool is_rejected(uint32_t addr) {
    uint16_t n = (reject_count < REJECT_SET_MAX) ? reject_count : REJECT_SET_MAX;
    for (uint16_t i = 0; i < n; i++) {
        if (reject_set[i] == addr) return true;
    }
    return false;
}

static void mark_rejected(uint32_t addr) {
    if (is_rejected(addr)) return;
    reject_set[reject_write_idx] = addr;
    reject_write_idx = (reject_write_idx + 1) % REJECT_SET_MAX;
    if (reject_count < REJECT_SET_MAX) reject_count++;
}

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

// Dedicated watchlist buffer for chunked delivery
static uint32_t g_watchlist_addrs[RA_MAX_WATCH_ADDRS];
static uint16_t g_watchlist_count = 0;
static volatile bool g_watchlist_pending = false;

/* ============================================================================
 * Address → watch_index hash table — O(1) lookup instead of linear search
 * across watch_addresses[]. Critical for keeping ESP32 Core 1's processing
 * time below the inter-transaction window (~16-30 ms). Without this, Kirby
 * (391 addrs + up to 128 query_addrs) blows past the budget and ra-module
 * reads a stale/empty SPI descriptor on the next transaction.
 *
 * Open-addressing with linear probing. Size is power-of-two = 2x the
 * worst-case watchlist so fill ratio stays ~25% and avg probe is ~1.25.
 * Empty bucket marker: watch_index == HASH_EMPTY (0xFFFF).
 * ========================================================================= */
#define HASH_SIZE 4096
#define HASH_MASK (HASH_SIZE - 1)
#define HASH_EMPTY 0xFFFF

typedef struct {
    uint32_t addr;
    uint16_t watch_index;
} hash_entry_t;

static hash_entry_t addr_hash[HASH_SIZE];

static inline uint32_t hash_addr(uint32_t addr) {
    return (addr * 0x9E3779B9u) & HASH_MASK;  /* Fibonacci hash mod size */
}

static void hash_clear() {
    for (uint32_t i = 0; i < HASH_SIZE; i++) {
        addr_hash[i].watch_index = HASH_EMPTY;
    }
}

static void hash_insert(uint32_t addr, uint16_t watch_index) {
    uint32_t h = hash_addr(addr);
    while (addr_hash[h].watch_index != HASH_EMPTY) {
        if (addr_hash[h].addr == addr) {
            addr_hash[h].watch_index = watch_index;
            return;
        }
        h = (h + 1) & HASH_MASK;
    }
    addr_hash[h].addr = addr;
    addr_hash[h].watch_index = watch_index;
}

/* Returns watch_index (0..watch_count-1) or -1 if not present. */
static inline int32_t hash_lookup(uint32_t addr) {
    uint32_t h = hash_addr(addr);
    while (addr_hash[h].watch_index != HASH_EMPTY) {
        if (addr_hash[h].addr == addr) {
            return (int32_t)addr_hash[h].watch_index;
        }
        h = (h + 1) & HASH_MASK;
    }
    return -1;
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
WiFiManager wm;
String base_url = "https://retroachievements.org/dorequest.php?";
NetworkClientSecure client;
HTTPClient https;

// ============================================================================
// Async marker queue — decouples SPI receive from Serial.printf
//
// handle_exi_command() enqueues 0xD0xxxxxx markers without printing.
// drain_marker_queue() is called from the processEXI loop AFTER the SPI
// slave has been re-armed, so Serial.printf never blocks the receive path.
// This prevents consecutive back-to-back markers from being lost.
// ============================================================================
#define MARKER_QUEUE_SIZE 32

static volatile uint32_t marker_queue[MARKER_QUEUE_SIZE];
static volatile int      marker_head = 0;
static volatile int      marker_tail = 0;

static void enqueue_marker(uint32_t m) {
    int next = (marker_head + 1) % MARKER_QUEUE_SIZE;
    if (next != marker_tail) {
        marker_queue[marker_head] = m;
        marker_head = next;
    }
    // If queue is full the marker is silently dropped — better than blocking.
}

static void drain_marker_queue(void) {
    while (marker_tail != marker_head) {
        uint32_t m   = marker_queue[marker_tail];
        marker_tail  = (marker_tail + 1) % MARKER_QUEUE_SIZE;

        uint8_t top  = (m >> 24) & 0xFF;
        uint8_t sub  = (m >> 16) & 0xFF;
        uint8_t ch   = ((m >>  8) >> 4) & 0xF;
        uint8_t dev  =  (m >>  8)       & 0xF;
        uint16_t crumb = (uint16_t)(m & 0xFFFF);

        if (top == 0x9A) {
            /* Swiss ra_install.c step markers */
            switch (sub) {
                case 0x01: Serial.println("DEBUG=STEP: [1] workspace allocated");           break;
                case 0x02: Serial.printf( "DEBUG=STEP: [2] blob copy done, addr_lo=0x%04X\n", crumb); break;
                case 0x03: Serial.println("DEBUG=STEP: [3] before VAR_AREA init writes");   break;
                case 0x04: Serial.println("DEBUG=STEP: [4] after VAR_AREA init writes");    break;
                case 0x05: Serial.printf( "DEBUG=STEP: [5] VIRetraceHook addr_lo=0x%04X%s\n", crumb, crumb ? "" : " (NULL!)"); break;
                case 0x06: Serial.printf( "DEBUG=STEP: [6] install_site addr_lo=0x%04X%s\n",  crumb, crumb ? "" : " (NULL!)"); break;
                case 0x07: Serial.println("DEBUG=STEP: [7] hook[24]=bl written OK");         break;
                case 0x08: Serial.println("DEBUG=STEP: [8] *install_site=b written OK");    break;
                case 0x0F: Serial.println("DEBUG=STEP: [F] ra_agent_install() COMPLETE");   break;
                default:   Serial.printf( "DEBUG=STEP: 0x%08X\n", (unsigned)m);             break;
            }
            return;
        }

        switch (m) {
            case 0xD000C0DE: Serial.println("DEBUG=MARKER: ra_agent_install() entered"); break;
            case 0xD000FA11: Serial.println("DEBUG=MARKER: all probes FAILED");           break;
            default:
                if      (sub == 0xBB) Serial.printf("DEBUG=MARKER: breadcrumb %04X\n",        crumb);
                else if (sub == 0xB1) Serial.printf("DEBUG=MARKER: loop iter %u entered\n",   (unsigned)(m & 0xF));
                else if (sub == 0xB2) Serial.printf("DEBUG=MARKER: loop iter %u chdv OK\n",   (unsigned)(m & 0xF));
                else if (sub == 0xE0) Serial.printf("DEBUG=MARKER: probing ch=%d dev=%d\n",   ch, dev);
                else if (sub == 0xE1) Serial.printf("DEBUG=MARKER: probe PASS ch=%d dev=%d\n",ch, dev);
                else if (sub == 0x50) Serial.printf("DEBUG=MARKER: ra_init ch=%d dev=%d\n",   ch, dev);
                else if (sub == 0xAA) Serial.printf("DEBUG=MARKER: beacon ch=%d dev=%d\n",    ch, dev);
                else                  Serial.printf("DEBUG=MARKER: 0x%08X\n", (unsigned)m);
                break;
        }
    }
}

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
    if (DEBUG) Serial.printf("DEBUG=rcheevos: %s\r\n", message);
}

static void event_handler(const rc_client_event_t *event, rc_client_t *client) {
    if (DEBUG) Serial.printf("DEBUG=event: %d\r\n", event->type);
    switch (event->type) {
        case RC_CLIENT_EVENT_ACHIEVEMENT_TRIGGERED: {
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
            setSemaphoreLED(LED_BLINK_MEDIUM, LED_GREEN);
            Serial.printf("ACHIEVEMENT=%lu;%s\r\n", (unsigned long)ach->id, ach->title);
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
     * v0.15.2 BE swap was a misdiagnosis — Kirby v0.19.3-60hz log 2026-05-31
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

        if (!got) {
            for (uint16_t q = 0; q < query_count; q++) {
                if (query_addrs[q] == byte_addr) {
                    buffer[j] = query_values[q];
                    got = true;
                    break;
                }
            }
        }

        if (!got) {
            buffer[j] = 0;
            /* Sanity-checked miss tracking — same rules as peek_from_snapshot. */
            bool valid_ppc_addr = (byte_addr <= 0x017FFFFF)
                               || (byte_addr >= 0x10000000 && byte_addr <= 0x13FFFFFF);
            if (valid_ppc_addr && peek_miss_count < PEEK_MISS_MAX) {
                bool dup = false;
                for (uint16_t k = 0; k < peek_miss_count; k++) {
                    if (peek_miss_addrs[k] == byte_addr) { dup = true; break; }
                }
                if (!dup) peek_miss_addrs[peek_miss_count++] = byte_addr;
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
            /* Phase A.5: bump last_used (unless static-protected). */
            if (last_used_frame && last_used_frame[widx] != LRU_PROTECTED) {
                last_used_frame[widx] = lru_clock;
            }
            got = true;
        }

        /* Dynamic query cache: linear search (small N <= ADDR_QUERY_MAX,
         * usually <= 128, cheap relative to ADDR_QUERY round trip). */
        if (!got) {
            for (uint16_t q = 0; q < query_count; q++) {
                if (query_addrs[q] == byte_addr) {
                    buf[j] = query_values[q];
                    got = true;
                    break;
                }
            }
        }

        if (!got) {
            found_all = false;
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
                               || (byte_addr >= 0x10000000 && byte_addr <= 0x13FFFFFF);
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
    uint32_t val = 0;
    for (uint32_t i = 0; i < num_bytes && i < 4; i++)
        val |= ((uint32_t)buf[i]) << (i * 8);
    (void)found_all;
    return val;
}

static uint32_t read_memory_nop(uint32_t address, uint8_t *buffer, uint32_t num_bytes, rc_client_t *client) {
    memset(buffer, 0, num_bytes);
    return num_bytes;
}

static void rc_callback(int result, const char *error_message, rc_client_t *client, void *userdata) {
    if (DEBUG) Serial.printf("DEBUG=rc_callback: %d\r\n", result);
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

static void json_clean_field_array(PsramStream &buf, const char* field) {
    json_remove_whitespace(buf);
    char*  data = buf.data();
    size_t len  = buf.length();
    size_t flen = strlen(field);
    bool in_str = false, skipping = false, remove_next = false;
    size_t r = 0, w = 0;
    while (r < len) {
        char c = data[r];
        if (c == '[' && remove_next) { skipping = true; data[w++] = '['; }
        if (c == ']' && remove_next) { remove_next = false; skipping = false; }
        if (c == '"' && json_quote_is_real(data, r)) in_str = !in_str;
        if (in_str && r + 1 + flen + 1 < len &&
            strncmp(data + r + 1, field, flen) == 0 &&
            data[r + flen + 1] == '"') {
            remove_next = true;
        }
        if (!skipping) data[w++] = c;
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
        int achvStart = buf.indexOf("\"Achievements\":[", search_from);
        if (achvStart == -1) break;
        char *data = buf.data();
        int arrayStart = buf.indexOf("[", achvStart);
        int arrayEnd   = buf.indexOf("]", arrayStart);
        if (arrayStart == -1 || arrayEnd == -1) break;

        int kept = 0, removed = 0;
        int pos = arrayStart + 1;
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
    Serial.printf("DEBUG=Filtered Achievements (all sets): kept=%d removed=%d\r\n",
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

// Apply all strips to a patch.php response held in PSRAM
static void json_strip_patch(PsramStream &buf) {
    json_remove_whitespace(buf);
    json_clean_field_str(buf,   "Description");
    json_remove_field(buf,      "Warning");
    json_remove_field(buf,      "BadgeLockedURL");
    json_remove_field(buf,      "BadgeURL");
    json_remove_field(buf,      "ImageIconURL");
    json_remove_field(buf,      "Rarity");
    json_remove_field(buf,      "RarityHardcore");
    json_remove_field(buf,      "Author");
    json_remove_field(buf,      "RichPresencePatch");
    json_clean_field_array(buf, "Leaderboards");
    json_remove_flags5_achievements(buf);
}

// HTTP server call (same as fpga-ra-adapter)
static void server_call(const rc_api_request_t *request, rc_client_server_callback_t callback,
                        void *callback_data, rc_client_t *rc_client) {
    // Large responses: rcheevos 11.x uses r=patch, 12.x uses r=achievementsets.
    // Both can be ~500-700KB for games like SSBM.
    // rcheevos puts these in the URL as query params (not in post_data).
    bool is_patch = (strstr(request->url, "r=patch")           != nullptr) ||
                    (strstr(request->url, "r=achievementsets")  != nullptr) ||
                    (request->post_data && strstr(request->post_data, "r=patch")           != nullptr) ||
                    (request->post_data && strstr(request->post_data, "r=achievementsets") != nullptr);
    if (DEBUG) Serial.printf("DEBUG=server_call is_patch=%d url=%s data=%s\r\n", (int)is_patch, request->url, request->post_data);

    client.setInsecure();
    https.begin(client, String(request->url));
    //https.setReuse(false);    // desabilita keep-alive: força conexão nova em vez de reusar stale
    //https.setTimeout(15000);  // 15s: TLS handshake (~2-3s no ESP32) + request + response
    https.setUserAgent("WII_RA_ADAPTER/0.1 rcheevos/12.3");
    int httpCode = 0;
    if (request->post_data) {
        https.addHeader("Content-Type", "application/x-www-form-urlencoded");
        httpCode = https.POST(request->post_data);
    } else {
        httpCode = https.GET();
    }

    if (httpCode != HTTP_CODE_OK) {
        if (DEBUG) Serial.printf("DEBUG=HTTP error: %d url=%s\r\n", httpCode, request->url);
        https.end();
        return;
    }

    int contentLen = https.getSize();  // -1 if chunked/unknown
    Serial.printf("DEBUG=HTTP ok is_patch=%d content-length=%d psram-free=%u heap-free=%u\r\n",
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
            Serial.printf("ERROR=PSRAM alloc failed (%u bytes), psram-free=%u\r\n",
                          (unsigned)buf_size, (unsigned)ESP.getFreePsram());
            https.end();
            return;
        }

        // writeToStream handles chunked transfer decoding internally.
        // Direct getStreamPtr() reads bypass chunked decoding and leave
        // hex chunk-size headers (e.g. "ff58\r\n") in the buffer.
        int written = https.writeToStream(&ps);
        Serial.printf("DEBUG=Patch read: %d bytes, stripping (psram-free=%u)...\r\n",
                      written, (unsigned)ESP.getFreePsram());
        Serial.printf("DEBUG=Patch first64 before: %.64s\r\n", ps.c_str());
        size_t before = ps.length();
#if STRIP_JSON_RESPONSE
        json_strip_patch(ps);
        Serial.printf("DEBUG=Patch stripped: %u → %u bytes (saved %u)\r\n",
                      (unsigned)before, (unsigned)ps.length(), (unsigned)(before - ps.length()));
#else
        Serial.printf("DEBUG=Patch strip DISABLED, raw size=%u bytes\r\n",
                      (unsigned)ps.length());
#endif
#if DEBUG_KEEP_ONLY_ACHIEVEMENT_ID
        /* DEBUG: filter to a single achievement AND drop RichPresence +
         * Leaderboards so rcheevos doesn't pull in their address sets.
         * Watchlist size shrinks dramatically (from ~671 to ~20) which
         * makes the multi-pass bootstrap fit in 1 ADDR_RESPONSE round
         * and the live `KIRBY ...` dump readable. Leaderboards alone
         * pull in dozens of memrefs per set on Kirby. */
        //static const uint32_t keep_ids[] = { 557557, 577564, 557558 };
        //json_keep_only_achievement_id(ps, keep_ids, 3);
        json_clean_field_str(ps,   "RichPresencePatch");
        json_clean_field_array(ps, "Leaderboards");
#endif
        Serial.printf("DEBUG=Patch first64 after: %.64s\r\n", ps.c_str());

        /* Diagnostic: dump area surrounding ConsoleId so we can see if the
         * strip corrupted brace nesting and made the field appear nested. */
        {
            const char *body = ps.c_str();
            const char *p = strstr(body, "ConsoleID");
            if (!p) p = strstr(body, "ConsoleId");
            if (p) {
                size_t off = (size_t)(p - body);
                Serial.printf("DEBUG=Patch ConsoleID found at offset %u: %.80s\r\n",
                              (unsigned)off, p);
                /* Print the 160 bytes leading up to ConsoleId — looking for unbalanced
                 * braces from a bad RichPresencePatch / Description strip. */
                size_t pre_start = (off > 160) ? off - 160 : 0;
                size_t pre_len   = off - pre_start;
                Serial.printf("DEBUG=Patch pre-ConsoleId[%u..%u]: %.*s\r\n",
                              (unsigned)pre_start, (unsigned)off,
                              (int)pre_len, body + pre_start);

                /* Count unbalanced braces from start to ConsoleId.
                 * In valid JSON, just before a top-level field we expect depth=1. */
                int depth = 0; bool in_str = false; bool esc = false;
                for (size_t i = 0; i < off; i++) {
                    char c = body[i];
                    if (esc) { esc = false; continue; }
                    if (c == '\\' && in_str) { esc = true; continue; }
                    if (c == '"') in_str = !in_str;
                    if (in_str) continue;
                    if (c == '{') depth++;
                    else if (c == '}') depth--;
                }
                Serial.printf("DEBUG=Patch brace depth at ConsoleId offset: %d (expect 1)\r\n", depth);
            } else {
                Serial.println("DEBUG=Patch: ConsoleID/ConsoleId NOT in body");
            }
        }

        rc_api_server_response_t server_response;
        memset(&server_response, 0, sizeof(server_response));
        server_response.body             = ps.c_str();
        server_response.body_length      = ps.length();
        server_response.http_status_code = httpCode;
        callback(&server_response, callback_data);
        // ps destructor frees PSRAM automatically
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
        while ((https.connected() || stream->available()) && millis() < deadline) {
            int avail = stream->available();
            if (avail > 0) {
                int n = stream->readBytes(chunk, min(avail, (int)sizeof(chunk)));
                responseStr.concat((const char*)chunk, n);
                total_read += n;
            } else {
                delay(1);
            }
            if (contentLen > 0 && total_read >= contentLen) break;
        }
        Serial.printf("DEBUG=HTTP body: declared=%d received=%d\r\n", contentLen, total_read);

        rc_api_server_response_t server_response;
        memset(&server_response, 0, sizeof(server_response));
        server_response.body             = responseStr.c_str();
        server_response.body_length      = responseStr.length();
        server_response.http_status_code = httpCode;
        callback(&server_response, callback_data);
    }

    https.end();
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

/* Captured from the most recent SNAPSHOT so the ADDR_RESPONSE handler
 * can recompute padding (= GC's SNAPSHOT request size). */
static size_t last_snap_req_len = 0;

/* Index in query_addrs[] where the most recent ADDR_QUERY round
 * started — values from the next ADDR_RESPONSE fill from this index. */
static uint16_t pending_query_start = 0;

/* Hard cap on convergence iterations per snapshot. SMG's deepest
 * pointer chains are ~6 levels; 12 leaves plenty of margin while
 * keeping a runaway loop from stalling the ESP32 forever. */
#define ADDR_QUERY_MAX_ITERATIONS  12
static uint8_t addr_query_iter_count = 0;

/* Run rc_memrefs_get_addresses with the current peek (which sees both
 * the static watchlist and the dynamic query cache), and APPEND any
 * NEW addresses (not already in query_addrs and not in watchlist) to
 * query_addrs[query_count..]. Returns the number of new addresses
 * appended. The caller decides whether to send an ADDR_QUERY (return
 * > 0) or to set new_snapshot=true (return == 0). */
static uint16_t collect_missing_addresses(void) {
    const rc_memrefs_t *memrefs = (state == STATE_ACTIVE && g_client)
                                  ? rc_client_get_memrefs(g_client) : NULL;
    if (!memrefs || g_watchlist_pending) return 0;

    uint32_t n = rc_memrefs_get_addresses(
        memrefs, prefetch_addrs, prefetch_sizes, PREFETCH_MAX,
        peek_from_snapshot, NULL);

    uint16_t start_count = query_count;

    for (uint32_t pi = 0; pi < n && query_count < ADDR_QUERY_MAX; pi++) {
        for (uint8_t b = 0; b < prefetch_sizes[pi] && query_count < ADDR_QUERY_MAX; b++) {
            uint32_t byte_addr = prefetch_addrs[pi] + b;

            /* Sanity: reject addresses outside MEM1/MEM2. rc_memrefs_get_
             * addresses can emit garbage when chain walks see unresolved
             * pointers (peek=0 → addr = 0 + offset, which can fall
             * anywhere). Asking ra-module to Swi_MLoad invalid addrs can
             * crash the Starlet thread. */
            if (!((byte_addr <= 0x017FFFFF)
                  || (byte_addr >= 0x10000000 && byte_addr <= 0x13FFFFFF))) continue;

            /* In static watchlist? O(1) hash lookup. */
            if (hash_lookup(byte_addr) >= 0) continue;

            /* Previously rejected by SOFT CAP? Don't waste a round trip. */
            if (is_rejected(byte_addr)) continue;

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

    /* Process peek misses accumulated since the last collect (covers BOTH
     * the rc_memrefs_get_addresses peek calls just now AND the previous
     * frame's do_frame trigger evaluations). These are the addresses
     * actually asked for at evaluation time — different from
     * rc_memrefs_get_addresses' unmasked outputs when AddAddress chains
     * include mask modifiers (Kirby pattern). */
    for (uint16_t i = 0; i < peek_miss_count && query_count < ADDR_QUERY_MAX; i++) {
        uint32_t addr = peek_miss_addrs[i];
        if (hash_lookup(addr) >= 0) continue;
        if (is_rejected(addr)) continue;
        bool dup = false;
        for (uint16_t q = 0; q < query_count; q++) {
            if (query_addrs[q] == addr) { dup = true; break; }
        }
        if (!dup) query_addrs[query_count++] = addr;
    }
    peek_miss_count = 0;

    return (uint16_t)(query_count - start_count);
}

/* Build & ship an ADDR_QUERY response with the addresses appended in
 * the most recent collect_missing_addresses() call: range
 * [pending_query_start .. query_count-1]. */
static void send_addr_query_response(size_t pad_len) {
    uint16_t round_count = query_count - pending_query_start;
    if (round_count == 0) {
        /* DIAG v0.18.8: zero-round = nothing to send. Should not happen in
         * normal flow. If it does, we silently no-op which leaves
         * addr_query_pending unchanged. Log so we know. */
        Serial.printf("DEBUG=>> send_addr_query SKIPPED: round_count=0 (qc=%u pending=%u)\r\n",
                      (unsigned)query_count, (unsigned)pending_query_start);
        return;
    }

    /* DIAG v0.18.8: trace every ADDR_QUERY emission. */
    Serial.printf("DEBUG=>> send_addr_query iter=%u round_count=%u total_qc=%u first_qa=0x%08lX pad=%u\r\n",
                  (unsigned)addr_query_iter_count, (unsigned)round_count,
                  (unsigned)query_count,
                  (unsigned long)query_addrs[pending_query_start],
                  (unsigned)pad_len);

    addr_query_pending = true;
    Serial.printf("DEBUG=  aqp <- true (ADDR_QUERY sent)\r\n");
    uint16_t payload_len = sizeof(ra_addr_query_t) + round_count * 4;

    ra_esp_header_t resp;
    resp.magic       = RA_MAGIC_ESP_TO_GC;
    resp.status      = (uint8_t)state;
    resp.event_type  = RA_EVT_ADDR_QUERY;
    resp.event_count = has_pending_event() ? 1 : 0;
    resp.data_len    = ra_host_to_be16(payload_len);

    uint8_t *buf = (uint8_t*)malloc(sizeof(ra_esp_header_t) + payload_len);
    if (!buf) {
        Serial.printf("DEBUG=send_addr_query_response: malloc(%u) FAILED\n",
                      (unsigned)(sizeof(ra_esp_header_t) + payload_len));
        addr_query_pending = false;
        new_snapshot = true;  /* fail-open: process frame with what we have */
        return;
    }
    memcpy(buf, &resp, sizeof(resp));
    ra_addr_query_t aq;
    aq.addr_count = ra_host_to_be16(round_count);
    memcpy(buf + sizeof(resp), &aq, sizeof(aq));
    uint32_t *aq_addrs = (uint32_t*)(buf + sizeof(resp) + sizeof(aq));
    for (uint16_t i = 0; i < round_count; i++)
        aq_addrs[i] = ra_host_to_be32(query_addrs[pending_query_start + i]);

    send_padded_response(buf, sizeof(ra_esp_header_t) + payload_len, pad_len);
    free(buf);
}

/* Response sender for SNAPSHOT / ADDR_RESPONSE (Phase B commands).
 *
 * v0.19.1 — `pad` IS NOW IGNORED. Phase B uses a separate CS-low for the
 * Wii's read phase, which always starts reading from tx_buf[0]. Leading
 * padding bytes would corrupt the magic check on the Wii side
 * (`ra_parse_response` reads the header at offset 0). With pad=0 the
 * response sits at tx_buf[0] and the Wii parses it directly.
 *
 * Why pad existed under Phase A: ra_send used a single CS-low for write
 * + read, so the Wii's write phase clocked the leading pad bytes through
 * MISO (the Wii ignored them), and the Wii's read phase landed on the
 * actual response at tx_buf[pad]. Phase B's separate read CS-low makes
 * the pad strictly harmful.
 *
 * Phase A WiiFlow commands (POLL/STATUS/IDENTIFY/CHUNK/GAME_RESET) use
 * different code paths (send_esp_header_response_4pad + bespoke handlers)
 * that bake their own padding into the response — those continue to work
 * because WiiFlow still does single-CS-low write+read.
 *
 * The `pad` parameter is kept in the signature so the existing call sites
 * compile unchanged; once Phase B is settled, we'll drop it entirely. */
static void send_padded_response(const uint8_t *data, size_t len, size_t pad) {
    (void)pad;  /* Phase B: ignored */
    if (len > EXI_MAX_TRANSACTION_SIZE) {
        Serial.printf("DEBUG=send_padded_response: too big len=%u\n", (unsigned)len);
        return;
    }
    /* DIAG: log wire-format ONLY for non-ACK events (event_type at offset 2). */
    if (len >= 6 && state == STATE_ACTIVE && data[2] != 0x00) {
        Serial.printf("DEBUG=>>WIRE len=%u hdr=[%02X %02X %02X %02X %02X %02X]\r\n",
                      (unsigned)len,
                      data[0], data[1], data[2], data[3], data[4], data[5]);
    }
    exi_spi_prepare_response(data, len);
}

void handle_exi_command(const uint8_t *rx_data, size_t rx_len) {
    if (rx_len < sizeof(ra_gc_header_t)) {
        static uint32_t last_short = 0; uint32_t now = millis();
        if (now - last_short >= 1000) { last_short = now;
            Serial.printf("DEBUG=EXI: short pkt rx_len=%u\r\n", (unsigned)rx_len); }
        return;
    }

    const ra_gc_header_t *hdr = (const ra_gc_header_t*)rx_data;
    if (hdr->magic != RA_MAGIC_GC_TO_ESP) {
        static uint32_t last_magic = 0; uint32_t now = millis();
        if (now - last_magic >= 1000) { last_magic = now;
            Serial.printf("DEBUG=EXI: bad magic 0x%02X rx_len=%u bytes=%02X %02X %02X %02X\r\n",
                hdr->magic, (unsigned)rx_len,
                rx_data[0], rx_data[1],
                rx_len>2 ? rx_data[2] : 0, rx_len>3 ? rx_data[3] : 0); }
        return;
    }

    // Per-command counter (for heartbeat rate analysis, especially post-IOS-reload)
    if (hdr->command < 16) cmd_count[hdr->command]++;

    // Throttled per-command log — 5s to keep Serial from saturating at 30Hz.
    {
        static uint32_t last_cmd = 0; uint32_t now = millis();
        if (now - last_cmd >= 5000) { last_cmd = now;
            Serial.printf("DEBUG=EXI: cmd=0x%02X rx_len=%u state=%d\r\n",
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
            if (DEBUG) Serial.println("DEBUG=EXI: IDENTIFY");
            break;
        }
        
        case RA_CMD_LOAD_GAME: {
            // Only game_id (6 bytes) is mandatory; md5_hash is optional.
            // ra_load_game_t is 40 bytes total — Wii/WiiFlow only sends 8 bytes.
            if (rx_len < sizeof(ra_gc_header_t) + RA_GAME_ID_LEN) break;
            const uint8_t *p = rx_data + sizeof(ra_gc_header_t);
            char gid[RA_GAME_ID_LEN + 1];
            memcpy(gid, p, RA_GAME_ID_LEN);
            gid[RA_GAME_ID_LEN] = '\0';
            Serial.printf("DEBUG=EXI: LOAD_GAME id=%s rx_len=%u\r\n", gid, (unsigned)rx_len);

            // Lookup hash from Wii game table
            const char *hash = wii_lookup_game_hash(gid);
            if (hash) {
                Serial.printf("DEBUG=Hash found: %s (%s)\r\n", hash, wii_lookup_game_name(gid));
                gameId = String(hash);  // store MD5 hash for loadGame()
                state = STATE_LOADING_GAME;
            } else {
                Serial.printf("ERROR=No hash found for Wii game ID: %s\r\n", gid);
                gameId = String(gid);
                state = STATE_ERROR;
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
                Serial.println("DEBUG=*** STATE 6 -> 7: first SNAPSHOT received from ra-module ***");
            }
            if (rx_len < sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t)) break;

            const ra_snapshot_header_t *snap = (const ra_snapshot_header_t*)(rx_data + sizeof(ra_gc_header_t));
            uint16_t count = ra_be16_to_host(snap->addr_count);
            frame_counter = ra_be32_to_host(snap->frame_counter);

            /* Phase A.5: SNAPSHOT count is ground truth for ra-module's
             * watchlist size right NOW. Sync our expected. Lru_clock ticks
             * each SNAPSHOT for LRU eviction freshness. */

            /* DIAG: log first SNAPSHOT after every REMOVE emission so we see
             * ra-module's actual post-REMOVE count vs what we expected.
             * snap_post_remove_armed is set right after we emit REMOVE; the
             * next SNAPSHOT prints the raw delta. This bypasses the mismatch
             * detector throttle and tells us if off-by-N still happens. */
            extern bool snap_post_remove_armed;
            extern uint16_t snap_post_remove_expected;
            if (snap_post_remove_armed) {
                snap_post_remove_armed = false;
                int32_t delta = (int32_t)count - (int32_t)snap_post_remove_expected;
                Serial.printf("DEBUG=POST-REMOVE SNAP: ra-module count=%u, ESP expected=%u, delta=%+ld, esp_watch=%u\r\n",
                              (unsigned)count, (unsigned)snap_post_remove_expected,
                              (long)delta, (unsigned)watch_count);
            }

            ra_module_expected_count = count;
            lru_clock++;

            // Throttled log: confirm ra_do_frame VBlank hook is firing on GC side.
            // frame_counter should increment ~60x/sec. If stuck at 0, hook is not installed.
            static uint32_t last_snap_log_ms = 0;
            uint32_t snap_now = (uint32_t)(esp_timer_get_time() / 1000);
            if (snap_now - last_snap_log_ms >= 5000) {
                last_snap_log_ms = snap_now;
                Serial.printf("DEBUG=SNAP frame=%lu snap_count=%u esp_watch=%u state=%d\r\n",
                              (unsigned long)frame_counter, (unsigned)count,
                              (unsigned)watch_count, (int)state);
            }
            const uint8_t *values = rx_data + sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t);

            // Update memory_data and run multi-pass cache fill when fully active
            if (state == STATE_ACTIVE) {
                if (count <= watch_count && memory_data)
                    memcpy(memory_data, values, count);
            }

            /* Re-sync: if SNAPSHOT carries FEWER values than our watchlist
             * size, ra-module missed a WATCHLIST_APPEND event from a prior
             * round (race on its parse). Re-send the missing tail BEFORE
             * doing anything else for this frame. We must skip do_frame
             * this iteration because peek_from_snapshot would read STALE
             * memory_data[count..watch_count-1] (those slots haven't been
             * refreshed since the original append) and could mis-evaluate
             * triggers (including the player ability chain leaf that
             * gates "Kirby to the Past"). */
            /* Track the count we last sent a mismatch-APPEND for. If the
             * NEXT SNAPSHOT carries the same count (ra-module hasn't
             * processed our APPEND yet due to the 1-tx arm-after-prepare
             * delay), do NOT re-send the APPEND — it would land as a
             * second copy in ra-module's read, causing it to grow its
             * watchlist by 2× the intended count, breaking the canonical
             * padding invariant on subsequent transactions and freezing
             * multi-pass forever (observed 2026-06-01 with eviction at
             * watch=759, ra-module locked at count=431). */
            static uint16_t mismatch_last_sent_count = 0xFFFF;

            if (state == STATE_ACTIVE && count < watch_count
                && count == mismatch_last_sent_count) {
                /* Same mismatch as last tx — skip re-send. Reply with a
                 * plain ACK padded to the canonical SNAPSHOT width so
                 * ra-module's RX offset stays aligned. */
                const size_t SNAP_REQ_LEN = sizeof(ra_gc_header_t)
                                          + sizeof(ra_snapshot_header_t)
                                          + count;
                last_snap_req_len = SNAP_REQ_LEN;
                ra_esp_header_t ack;
                ack.magic       = RA_MAGIC_ESP_TO_GC;
                ack.status      = (uint8_t)state;
                ack.event_type  = RA_EVT_NONE;
                ack.event_count = 0;
                ack.data_len    = 0;
                send_padded_response((uint8_t*)&ack, sizeof(ack), SNAP_REQ_LEN);
                break;
            }

            if (state == STATE_ACTIVE && count < watch_count) {
                uint16_t missing = (uint16_t)(watch_count - count);
                if (missing > ADDR_QUERY_MAX) missing = ADDR_QUERY_MAX;
                mismatch_last_sent_count = count;  /* arm dup-suppression */
                /* DIAG: log EVERY mismatch (was throttled 5s) so we see the
                 * post-REMOVE off-by-N that the throttle was hiding. */
                Serial.printf("DEBUG=SNAPSHOT mismatch: ra-module count=%u < esp watch=%u — re-sending APPEND for %u tail addrs\r\n",
                              (unsigned)count, (unsigned)watch_count, (unsigned)missing);

                static uint8_t buf[sizeof(ra_esp_header_t)
                                   + sizeof(ra_watchlist_append_t)
                                   + ADDR_QUERY_MAX * 4];
                ra_esp_header_t evt;
                evt.magic       = RA_MAGIC_ESP_TO_GC;
                evt.status      = (uint8_t)state;
                evt.event_type  = RA_EVT_WATCHLIST_APPEND;
                evt.event_count = 0;
                uint16_t data_len = sizeof(ra_watchlist_append_t) + missing * 4;
                evt.data_len    = ra_host_to_be16(data_len);
                memcpy(buf, &evt, sizeof(evt));
                ra_watchlist_append_t wa;
                wa.addr_count = ra_host_to_be16(missing);
                memcpy(buf + sizeof(evt), &wa, sizeof(wa));
                uint8_t *addr_dst = buf + sizeof(evt) + sizeof(wa);
                for (uint16_t i = 0; i < missing; i++) {
                    uint32_t a = ra_host_to_be32(watch_addresses[count + i]);
                    memcpy(addr_dst + i * 4, &a, 4);
                }
                /* Phase A.5 canonical padding (CORRECTED): pad with CURRENT
                 * ra-module count (= 10 + count). ra-module's NEXT TX is
                 * still 10 + count because it parses APPEND AFTER reading.
                 * Update expected_count to post-APPEND value for subsequent. */
                const size_t SNAP_REQ_LEN = sizeof(ra_gc_header_t)
                                          + sizeof(ra_snapshot_header_t)
                                          + count;
                send_padded_response(buf, sizeof(evt) + data_len, SNAP_REQ_LEN);
                ra_module_expected_count = count + missing;
                last_snap_req_len = sizeof(ra_gc_header_t)
                                  + sizeof(ra_snapshot_header_t)
                                  + ra_module_expected_count;
                /* Skip do_frame this iteration — wait for ra-module's
                 * next SNAPSHOT to confirm catch-up. */
                break;
            }

            /* No mismatch this SNAPSHOT — ra-module caught up. Reset
             * the dup-suppression marker so a FUTURE mismatch (e.g. after
             * another eviction) goes through. */
            mismatch_last_sent_count = 0xFFFF;

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
            /* DIAG v0.18.8: capture addr_query_pending at SNAP_HDR entry —
             * if it's stuck=true and no ADDR_RESP is arriving, multi-pass
             * is frozen. */
            bool aqp_at_entry = addr_query_pending;
            if (!addr_query_pending) {
                query_count = 0;
                pending_query_start = 0;
                addr_query_iter_count = 0;
                n_missing = (state == STATE_ACTIVE)
                            ? collect_missing_addresses() : 0;
            }

            /* DIAG v0.18.10: throttled 2s — reduce Serial pressure. */
            if (state == STATE_ACTIVE) {
                static uint32_t last_snap_diag = 0;
                uint32_t now_sd = millis();
                if (now_sd - last_snap_diag >= 2000) {
                    last_snap_diag = now_sd;
                    Serial.printf("DEBUG=SNAP_HDR exit: n_missing=%u query_count=%u peek_miss=%u first_qa=%08lX\r\n",
                                  (unsigned)n_missing, (unsigned)query_count,
                                  (unsigned)peek_miss_count,
                                  query_count > 0 ? (unsigned long)query_addrs[0] : 0UL);
                }
            }

            /* Padding for SNAPSHOT response = ra-module's SNAPSHOT request size.
             * GC writes 4 (gc_hdr) + 6 (snap_hdr) + count (values) = 10+count
             * bytes during its write phase; ESP32 must clock out exactly that
             * many filler bytes before the response header so GC's read phase
             * lands on the esp_header.magic. Without this pad, the SNAPSHOT
             * response is dropped on the floor (ra-module reads garbage past
             * its own write tail) and ADDR_QUERY never reaches the GC side. */
            const size_t SNAP_REQ_LEN = sizeof(ra_gc_header_t)
                                      + sizeof(ra_snapshot_header_t)
                                      + count;

            // Build response
            ra_esp_header_t resp;
            resp.magic       = RA_MAGIC_ESP_TO_GC;
            resp.status      = (uint8_t)state;
            resp.event_count = has_pending_event() ? 1 : 0;

            /* Save snapshot request length so ADDR_RESPONSE handler can
             * pad subsequent ADDR_QUERY responses with the same width
             * (the GC's response read for ADDR_RESPONSE has the same
             * write-then-read shape as SNAPSHOT — ra-module is built
             * to always read a fixed 1024 bytes regardless of cmd). */
            last_snap_req_len = SNAP_REQ_LEN;

            /* Phase A.5 — REMOVE has top priority. evict_lru() queued
             * specific addresses to drop; emit them so ra-module can
             * defrag in lockstep. Canonical pad for ra-module's NEXT TX
             * (= 10 + ra_module_expected_count, already decremented by
             * evict_lru when it filled pending_remove_*). */
            if (pending_remove_count > 0) {
                uint16_t n = pending_remove_count;
                uint16_t data_len = sizeof(ra_watchlist_remove_t) + n * 4;
                static uint8_t buf[sizeof(ra_esp_header_t)
                                   + sizeof(ra_watchlist_remove_t)
                                   + EVICT_BATCH_SIZE * 4];
                ra_esp_header_t evt;
                evt.magic       = RA_MAGIC_ESP_TO_GC;
                evt.status      = (uint8_t)state;
                evt.event_type  = RA_EVT_WATCHLIST_REMOVE;
                evt.event_count = 0;
                evt.data_len    = ra_host_to_be16(data_len);
                memcpy(buf, &evt, sizeof(evt));
                ra_watchlist_remove_t wr;
                wr.addr_count = ra_host_to_be16(n);
                memcpy(buf + sizeof(evt), &wr, sizeof(wr));
                memcpy(buf + sizeof(evt) + sizeof(wr),
                       pending_remove_addrs, n * 4);
                /* Phase A.5 canonical padding (CORRECTED): pad with CURRENT
                 * ra-module count = SNAP_REQ_LEN (= 10 + count from this
                 * SNAPSHOT). ra-module's NEXT TX is still using OLD count
                 * because it parses REMOVE AFTER reading. Update
                 * last_snap_req_len for subsequent responses. */
                send_padded_response(buf, sizeof(evt) + data_len, SNAP_REQ_LEN);
                last_snap_req_len = sizeof(ra_gc_header_t)
                                  + sizeof(ra_snapshot_header_t)
                                  + ra_module_expected_count;
                Serial.printf("DEBUG=Sent WATCHLIST_REMOVE n=%u (ra_expected_post=%u, esp_watch=%u)\r\n",
                              (unsigned)n, (unsigned)ra_module_expected_count,
                              (unsigned)watch_count);
                pending_remove_count = 0;
                snap_post_remove_armed = true;
                snap_post_remove_expected = ra_module_expected_count;
                new_snapshot = true;
                break;
            }

            /* DIAG v0.18.8: log the path SNAP_HDR takes — REMOVE/APPEND/
             * ADDR_QUERY/ACK. Critical for finding the multi-pass freeze. */
            const char *path =
                (pending_remove_count > 0) ? "REMOVE-already-emitted"
                : (n_missing > 0) ? "ADDR_QUERY"
                : (g_watchlist_pending) ? "WATCHLIST_UPDATE"
                : "ACK";
            Serial.printf("DEBUG=SNAP_HDR path=%s aqp_entry=%d aqp_now=%d n_missing=%u qc=%u\r\n",
                          path, (int)aqp_at_entry, (int)addr_query_pending,
                          (unsigned)n_missing, (unsigned)query_count);

            if (n_missing > 0) {
                /* Multi-pass round 1: send ADDR_QUERY for the addresses we
                 * just discovered. ADDR_RESPONSE handler will receive
                 * values, then call collect_missing_addresses again — if
                 * pointer chains revealed new addresses, another round
                 * fires. Convergence ends when collect returns 0. */
                pending_query_start = 0;
                addr_query_iter_count = 1;
                send_addr_query_response(SNAP_REQ_LEN);
            } else if (g_watchlist_pending) {
                // Notify GC: new watchlist ready
                uint16_t num_chunks = RA_WATCHLIST_NUM_CHUNKS(g_watchlist_count);
                ra_watchlist_notify_t notify;
                notify.total_addr_count = ra_host_to_be16(g_watchlist_count);
                notify.num_chunks       = ra_host_to_be16(num_chunks);
                resp.event_type = RA_EVT_WATCHLIST_UPDATE;
                resp.data_len   = ra_host_to_be16(sizeof(notify));
                uint8_t buf[sizeof(ra_esp_header_t) + sizeof(notify)];
                memcpy(buf, &resp, sizeof(resp));
                memcpy(buf + sizeof(resp), &notify, sizeof(notify));
                send_padded_response(buf, sizeof(buf), SNAP_REQ_LEN);
                new_snapshot = true;  // no missing addrs, process immediately
            } else {
                // No missing addresses, no watchlist update. Process immediately.
                new_snapshot = true;
                PendingEvent *evt = peek_event();
                if (evt) {
                    resp.event_type = evt->type;
                    resp.data_len   = ra_host_to_be16(evt->data_len);
                    uint8_t buf[sizeof(ra_esp_header_t) + RA_MAX_RESPONSE_DATA];
                    memcpy(buf, &resp, sizeof(resp));
                    memcpy(buf + sizeof(resp), (void*)evt->data, evt->data_len);
                    send_padded_response(buf, sizeof(resp) + evt->data_len, SNAP_REQ_LEN);
                    pop_event();
                } else {
                    resp.event_type = RA_EVT_NONE;
                    resp.data_len   = 0;
                    send_padded_response((uint8_t*)&resp, sizeof(resp), SNAP_REQ_LEN);
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

            /* DIAG v0.18.8: UNTHROTTLED — log every ADDR_RESP entry so we
             * can correlate exactly which round the freeze happens in. */
            {
                Serial.printf("DEBUG=<<ADDR_RESP entry: aqp=%d qc=%u pending_start=%u resp_count=%u first_qa=0x%08lX peek_miss=%u iter=%u\r\n",
                              (int)addr_query_pending,
                              (unsigned)query_count, (unsigned)pending_query_start,
                              (unsigned)resp_count,
                              query_count > 0 ? (unsigned long)query_addrs[0] : 0UL,
                              (unsigned)peek_miss_count,
                              (unsigned)addr_query_iter_count);
            }

            /* Values land at the offset where the most recent round of
             * collect_missing_addresses appended new addresses. */
            for (uint16_t i = 0; i < resp_count
                              && (pending_query_start + i) < query_count; i++) {
                query_values[pending_query_start + i] = resp_values[i];
            }

            addr_query_pending = false;
            Serial.printf("DEBUG=  aqp <- false (ADDR_RESP processed)\r\n");

            /* Padding for the ack/next-query response. ra-module v0.15+
             * pads ALL its writes to the SNAPSHOT canonical length so
             * the read offset stays stable regardless of which command
             * was last sent. We MUST mirror that here: pad with
             * last_snap_req_len, NOT the actual ADDR_RESPONSE request
             * size (which would be smaller). See [[project-ra-module-
             * write-len-padding]] for the architectural rationale. */
            const size_t ADDR_RESP_REQ_LEN = last_snap_req_len;
            (void)resp_count;  /* no longer used for padding */

            /* Run next convergence pass — peek_from_snapshot now sees
             * the values we just received, so deeper pointer chains
             * may resolve to brand new addresses. */
            pending_query_start = query_count;
            uint16_t new_missing = (state == STATE_ACTIVE)
                                   ? collect_missing_addresses() : 0;
            addr_query_iter_count++;

            if (new_missing > 0 && addr_query_iter_count < ADDR_QUERY_MAX_ITERATIONS) {
                /* Another round needed — send ADDR_QUERY for just the
                 * new addresses. ra-module sees event_type=ADDR_QUERY
                 * and loops back with another ADDR_RESPONSE. */
                send_addr_query_response(ADDR_RESP_REQ_LEN);
            } else {
                /* Converged (or iteration cap hit). Commit query_addrs/
                 * query_values to the permanent watchlist FIRST, THEN set
                 * new_snapshot=true. If we set new_snapshot=true before the
                 * append, Core 0's processSnapshot can race in, run do_frame,
                 * and (used to) reset query_count=0 at its tail — wiping the
                 * value we needed for watchlist_append a few microseconds
                 * later. Order is now: read query_count → append → publish. */
                uint16_t new_addr_count = 0;
                uint16_t qc_snapshot = query_count;
                if (state == STATE_ACTIVE && qc_snapshot > 0) {
                    uint16_t before = watch_count;
                    watchlist_append(query_addrs, query_values, qc_snapshot);
                    /* watchlist_append may have triggered evict_lru() if
                     * the batch crossed WATCHLIST_HIGH_WATER. In that case
                     * watch_count dropped BELOW `before` then re-grew to
                     * static_count + actual_appended — `before - watch_count`
                     * would wrap. Guard the subtraction. */
                    if (watch_count > before) {
                        new_addr_count = watch_count - before;
                    } else {
                        new_addr_count = 0;
                    }
                }
                new_snapshot = true;

                static uint32_t last_conv_log = 0;
                uint32_t now_ms = millis();
                if (now_ms - last_conv_log >= 2000) {
                    last_conv_log = now_ms;
                    Serial.printf("DEBUG=CONVERGE qc=%u new_addrs=%u watch=%u\r\n",
                                  (unsigned)qc_snapshot, (unsigned)new_addr_count,
                                  (unsigned)watch_count);
                }

                /* Phase A.5 — REMOVE takes priority over WATCHLIST_APPEND.
                 * If watchlist_append() crossed HIGH_WATER it called
                 * evict_lru() which queued specific addresses in
                 * pending_remove_addrs[]. Emit REMOVE with canonical pad
                 * for ra-module's NEXT TX (= 10 + ra_module_expected_count
                 * which evict_lru already decremented). */
                if (pending_remove_count > 0) {
                    uint16_t n = pending_remove_count;
                    uint16_t data_len = sizeof(ra_watchlist_remove_t) + n * 4;
                    static uint8_t buf[sizeof(ra_esp_header_t)
                                       + sizeof(ra_watchlist_remove_t)
                                       + EVICT_BATCH_SIZE * 4];
                    ra_esp_header_t evt;
                    evt.magic       = RA_MAGIC_ESP_TO_GC;
                    evt.status      = (uint8_t)state;
                    evt.event_type  = RA_EVT_WATCHLIST_REMOVE;
                    evt.event_count = 0;
                    evt.data_len    = ra_host_to_be16(data_len);
                    memcpy(buf, &evt, sizeof(evt));
                    ra_watchlist_remove_t wr;
                    wr.addr_count = ra_host_to_be16(n);
                    memcpy(buf + sizeof(evt), &wr, sizeof(wr));
                    memcpy(buf + sizeof(evt) + sizeof(wr),
                           pending_remove_addrs, n * 4);
                    /* Phase A.5 canonical padding (CORRECTED): pad with the
                     * CURRENT ra-module count (= ADDR_RESP_REQ_LEN). NOTE:
                     * evict_lru() already decremented ra_module_expected_count
                     * by n. So the CURRENT (pre-REMOVE-parse) ra-module count
                     * is (ra_module_expected_count + n). We use ADDR_RESP_REQ_LEN
                     * which was last_snap_req_len set by SNAPSHOT — that's
                     * pre-REMOVE count (because SNAPSHOT was processed before
                     * we called evict_lru in watchlist_append). */
                    send_padded_response(buf, sizeof(evt) + data_len, ADDR_RESP_REQ_LEN);
                    last_snap_req_len = sizeof(ra_gc_header_t)
                                      + sizeof(ra_snapshot_header_t)
                                      + ra_module_expected_count;
                    Serial.printf("DEBUG=CONVERGE sending WATCHLIST_REMOVE n=%u (ra_expected_post=%u, esp_watch=%u)\r\n",
                                  (unsigned)n, (unsigned)ra_module_expected_count,
                                  (unsigned)watch_count);
                    pending_remove_count = 0;
                    snap_post_remove_armed = true;
                    snap_post_remove_expected = ra_module_expected_count;
                } else if (new_addr_count > 0) {
                    static uint32_t last_app_log = 0;
                    uint32_t now_a = millis();
                    if (now_a - last_app_log >= 2000) {
                        last_app_log = now_a;
                        Serial.printf("DEBUG=CONVERGE sending WATCHLIST_APPEND for %u addrs (watch=%u)\r\n",
                                      (unsigned)new_addr_count, (unsigned)watch_count);
                    }
                    /* Build response: esp_header + ra_watchlist_append_t + N*4 addrs.
                     * Tell ra-module the EXACT addrs to append, in
                     * insertion order (ra-module just appends to its tail
                     * so its order matches ours). Static buf to avoid
                     * stack pressure on the EXI core's loopTask. */
                    static uint8_t buf[sizeof(ra_esp_header_t)
                                       + sizeof(ra_watchlist_append_t)
                                       + ADDR_QUERY_MAX * 4];
                    ra_esp_header_t evt;
                    evt.magic       = RA_MAGIC_ESP_TO_GC;
                    evt.status      = (uint8_t)state;
                    evt.event_type  = RA_EVT_WATCHLIST_APPEND;
                    evt.event_count = 0;
                    uint16_t data_len = sizeof(ra_watchlist_append_t) + new_addr_count * 4;
                    evt.data_len    = ra_host_to_be16(data_len);
                    memcpy(buf, &evt, sizeof(evt));
                    ra_watchlist_append_t wa;
                    wa.addr_count = ra_host_to_be16(new_addr_count);
                    memcpy(buf + sizeof(evt), &wa, sizeof(wa));
                    /* Read from the TAIL of watch_addresses, NOT query_addrs.
                     * watchlist_append may have filtered out duplicates
                     * (defensive dedup against hash false-misses post-
                     * eviction), so the addresses actually appended are
                     * watch_addresses[watch_count - new_addr_count ..
                     * watch_count - 1] — not the first new_addr_count of
                     * query_addrs. Mismatching these would emit a phantom
                     * APPEND to ra-module for an address ESP32 didn't
                     * actually add, breaking the SNAPSHOT positional invariant. */
                    uint8_t *addr_dst = buf + sizeof(evt) + sizeof(wa);
                    uint16_t base_idx = (uint16_t)(watch_count - new_addr_count);
                    for (uint16_t i = 0; i < new_addr_count; i++) {
                        uint32_t a = ra_host_to_be32(watch_addresses[base_idx + i]);
                        memcpy(addr_dst + i * 4, &a, 4);
                    }
                    /* Phase A.5 canonical padding (CORRECTED): pad with the
                     * CURRENT ra-module count (= last_snap_req_len). ra-module
                     * READS this APPEND with its TX still at the OLD width —
                     * it only grows its count AFTER parsing this response.
                     * Update expected_count AFTER the emit so subsequent
                     * responses (which ra-module reads in later txs, post-
                     * APPEND-parse) use the new width. */
                    send_padded_response(buf, sizeof(evt) + data_len, ADDR_RESP_REQ_LEN);
                    ra_module_expected_count += new_addr_count;
                    last_snap_req_len = sizeof(ra_gc_header_t)
                                      + sizeof(ra_snapshot_header_t)
                                      + ra_module_expected_count;
                } else {
                    /* No new addrs to announce — plain ack. ra-module's
                     * count doesn't change so pad = 10 + expected_count. */
                    ra_esp_header_t ack;
                    ack.magic       = RA_MAGIC_ESP_TO_GC;
                    ack.status      = (uint8_t)state;
                    ack.event_type  = RA_EVT_NONE;
                    ack.event_count = 0;
                    ack.data_len    = 0;
                    send_padded_response((uint8_t*)&ack, sizeof(ack), ADDR_RESP_REQ_LEN);
                }

                if (addr_query_iter_count >= ADDR_QUERY_MAX_ITERATIONS && new_missing > 0) {
                    static uint32_t last_iter_warn = 0; uint32_t now = millis();
                    if (now - last_iter_warn >= 5000) { last_iter_warn = now;
                        Serial.printf("DEBUG=ADDR_QUERY iteration cap (%u) hit with %u still missing\r\n",
                                      ADDR_QUERY_MAX_ITERATIONS, new_missing);
                    }
                }
            }
            break;
        }
        
        case RA_CMD_DEBUG_LOG: {
            /* Format: u8 msg_len, then msg_len bytes of ASCII text. */
            if (rx_len < sizeof(ra_gc_header_t) + 1) {
                Serial.printf("DEBUG=GC: (DEBUG_LOG rx too short: %u bytes)\n",
                              (unsigned)rx_len);
                break;
            }
            const uint8_t *p = rx_data + sizeof(ra_gc_header_t);
            uint8_t msg_len = *p;
            if (rx_len < sizeof(ra_gc_header_t) + 1 + msg_len) {
                Serial.printf("DEBUG=GC: (DEBUG_LOG truncated: have %u bytes, need %u)\n",
                              (unsigned)rx_len,
                              (unsigned)(sizeof(ra_gc_header_t) + 1 + msg_len));
                break;
            }

            Serial.print("DEBUG=GC: ");
            Serial.write(p + 1, msg_len);
            Serial.println();

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
                Serial.printf("DEBUG=GET_CHUNK: rx_len too short (%u bytes)\n",
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
            Serial.printf("DEBUG=GET_CHUNK req: idx=%u (total_addrs=%u, n_chunks=%u, this_chunk=%u, is_last=%u, resp_len=%u, padded_len=%u)\n",
                          chunk_idx, total, n_chunks, n_in_chunk, is_last,
                          (unsigned)(sizeof(ra_esp_header_t) + resp_data_len),
                          (unsigned)total_len);

            uint8_t *buf = (uint8_t*)malloc(total_len);
            if (!buf) {
                Serial.printf("DEBUG=GET_CHUNK: malloc(%u) FAILED\n",
                              (unsigned)total_len);
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

            /* Log first 4 addresses so we can confirm the data on the GC side. */
            if (n_in_chunk > 0) {
                Serial.printf("DEBUG=GET_CHUNK first addrs:");
                for (uint16_t i = 0; i < n_in_chunk && i < 4; i++) {
                    Serial.printf(" 0x%08lX",
                                  (unsigned long)g_watchlist_addrs[start + i]);
                }
                if (n_in_chunk > 4) Serial.printf(" ...");
                Serial.println();
            }

            /* Log first 22 bytes of the prepared response: 6 padding + the
             * first 16 bytes of actual content. */
            Serial.printf("DEBUG=GET_CHUNK resp head:");
            for (uint32_t i = 0; i < total_len && i < 22; i++) {
                Serial.printf(" %02X", buf[i]);
            }
            Serial.println();

            exi_spi_prepare_response(buf, total_len);
            free(buf);
            if (is_last) g_watchlist_pending = false;
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
            Serial.println("DEBUG=EXI: GAME_RESET");
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
    /* Defense-in-depth dedup via hash: skip any addr that is ALREADY in
     * watch_addresses. collect_missing_addresses already dedups via
     * hash_lookup, but if the hash got into a transient inconsistent state
     * (e.g. mid-eviction tombstones in earlier versions), duplicates could
     * slip in. Cheap (O(count) hash hits) and bullet-proof. */
    static uint32_t  filtered_addrs[ADDR_QUERY_MAX];
    static uint8_t   filtered_values[ADDR_QUERY_MAX];
    uint16_t filtered_count = 0;
    for (uint16_t i = 0; i < count && filtered_count < ADDR_QUERY_MAX; i++) {
        if (hash_lookup(addrs[i]) >= 0) continue;  /* already in watchlist */
        /* Also dedup within this batch (could be same addr twice). */
        bool dup_in_batch = false;
        for (uint16_t j = 0; j < filtered_count; j++) {
            if (filtered_addrs[j] == addrs[i]) { dup_in_batch = true; break; }
        }
        if (dup_in_batch) continue;
        filtered_addrs[filtered_count]  = addrs[i];
        filtered_values[filtered_count] = values[i];
        filtered_count++;
    }
    if (filtered_count == 0) return watch_count;
    count = filtered_count;
    addrs = filtered_addrs;
    values = filtered_values;

    uint32_t new_count = (uint32_t)old_count + count;
    /* Phase A.5 — HIGH WATER eviction: drop LRU dynamic entries that
     * AREN'T protected (statics + their byte-fanouts always stay). Queues
     * a REMOVE event for ra-module to defrag in lockstep. */
    if (new_count > WATCHLIST_HIGH_WATER) {
        Serial.printf("DEBUG=watchlist_append: HIGH WATER hit (%u + %u > %u) — running evict_lru\r\n",
                      (unsigned)old_count, (unsigned)count, (unsigned)WATCHLIST_HIGH_WATER);
        evict_lru();
        old_count = watch_count;
        new_count = (uint32_t)old_count + count;
    }
    if (new_count > RA_MAX_WATCH_ADDRS) {
        Serial.printf("DEBUG=watchlist_append: would overflow (%u + %u > %u)\r\n",
                      old_count, count, RA_MAX_WATCH_ADDRS);
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
    return (uint16_t)new_count;
}

/* Grow watch_addresses/memory_data to MAX capacity at game-load so
 * subsequent watchlist_append() calls never need to realloc (which
 * would race with Core 0's reads via peek_from_snapshot). */
static void ensure_watchlist_capacity() {
    if (!watch_addresses)
        watch_addresses = (uint32_t*)malloc(RA_MAX_WATCH_ADDRS * sizeof(uint32_t));
    if (!memory_data)
        memory_data = (uint8_t*)calloc(RA_MAX_WATCH_ADDRS, 1);
    if (!last_used_frame)
        last_used_frame = (uint32_t*)calloc(RA_MAX_WATCH_ADDRS, sizeof(uint32_t));
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
    ra_module_expected_count = expanded;  /* truth at boot */
    lru_clock = 1;

    // Publish into dedicated watchlist buffer for chunked delivery
    memcpy(g_watchlist_addrs, watch_addresses, expanded * sizeof(uint32_t));
    g_watchlist_count   = expanded;
    g_watchlist_pending = true;

    Serial.printf("DEBUG=Watchlist updated: %u addresses (%u chunks) [static_byte_count=%u, base_addrs=%u]\r\n",
                  (unsigned)expanded, RA_WATCHLIST_NUM_CHUNKS(expanded),
                  (unsigned)static_watch_count, (unsigned)new_count);
}

/* Phase A.5 — LRU eviction of dynamic entries.
 *
 * Selects up to EVICT_BATCH_SIZE entries from the dynamic region
 * [static_watch_count..watch_count-1] with the SMALLEST last_used_frame
 * values (= least recently touched). Removes them from the hash, defrags
 * watch_addresses/memory_data/last_used_frame (shifts survivors down),
 * queues their addresses (BE-encoded) in pending_remove_addrs[] for
 * emission via RA_EVT_WATCHLIST_REMOVE, and updates ra_module_expected_count
 * so subsequent canonical padding aligns with ra-module's post-REMOVE TX.
 *
 * Static entries (last_used_frame == LRU_PROTECTED) are NEVER picked,
 * preserving chain[0]/chain[1] byte-fanouts that the trigger evaluator
 * depends on. Insertion order of survivors is preserved → ra-module's
 * defrag (linear scan + shift on REMOVE parse) arrives at the same array. */
static void evict_lru(void) {
    if (watch_count <= static_watch_count) return;

    uint16_t dyn_start = static_watch_count;
    uint16_t dyn_count = watch_count - dyn_start;
    uint16_t target = (dyn_count > EVICT_BATCH_SIZE) ? EVICT_BATCH_SIZE : dyn_count;
    if (target == 0) return;

    /* Partial selection: walk dynamic region; mark the `target` slots with
     * the smallest last_used_frame values. We use a high-water-mark scan —
     * O(N×K) where K=target=256, but N≤1024, fine on ESP32-S3. */
    static bool evict_mark[RA_MAX_WATCH_ADDRS];
    memset(evict_mark + dyn_start, 0, dyn_count * sizeof(bool));

    for (uint16_t k = 0; k < target; k++) {
        uint32_t min_frame = LRU_PROTECTED;
        int32_t  min_idx   = -1;
        for (uint16_t i = dyn_start; i < watch_count; i++) {
            if (evict_mark[i]) continue;
            if (last_used_frame[i] < min_frame) {
                min_frame = last_used_frame[i];
                min_idx   = i;
            }
        }
        if (min_idx < 0) break;  // somehow all marked
        evict_mark[min_idx] = true;
    }

    /* Queue evicted addresses in BE wire order BEFORE defrag (otherwise
     * indices shift). Dedup as we go AND log any duplicates found —
     * the off-by-N forensic loop (v0.19.3-60hz showed exact off-by-2
     * even at 10Hz so it can't be a timing race; if duplicates exist in
     * watch_addresses they'd show up here). */
    pending_remove_count = 0;
    uint16_t dup_skipped = 0;
    for (uint16_t i = dyn_start; i < watch_count && pending_remove_count < EVICT_BATCH_SIZE; i++) {
        if (!evict_mark[i]) continue;
        uint32_t addr_host = watch_addresses[i];
        uint32_t addr_be   = ra_host_to_be32(addr_host);
        bool dup = false;
        for (uint16_t k = 0; k < pending_remove_count; k++) {
            if (pending_remove_addrs[k] == addr_be) { dup = true; break; }
        }
        if (dup) {
            dup_skipped++;
            Serial.printf("DEBUG=evict_lru DUP: addr=0x%08lX at idx=%u (already queued)\r\n",
                          (unsigned long)addr_host, (unsigned)i);
            continue;
        }
        pending_remove_addrs[pending_remove_count++] = addr_be;
    }
    if (dup_skipped > 0) {
        Serial.printf("DEBUG=evict_lru: %u duplicates skipped (watch_addresses had dups!)\r\n",
                      (unsigned)dup_skipped);
    }

    /* Sanity: every entry in pending_remove_addrs MUST be present in the hash
     * (since we read them from watch_addresses, which the hash tracks). If
     * the hash returns -1 for any of them, the hash is out of sync with
     * watch_addresses — that's a bug we want to know about immediately. */
    {
        uint16_t hash_missing = 0;
        for (uint16_t k = 0; k < pending_remove_count; k++) {
            uint32_t addr_host = ra_be32_to_host(pending_remove_addrs[k]);
            if (hash_lookup(addr_host) < 0) {
                hash_missing++;
                if (hash_missing <= 4) {
                    Serial.printf("DEBUG=evict_lru HASH_MISS: addr=0x%08lX not in hash\r\n",
                                  (unsigned long)addr_host);
                }
            }
        }
        if (hash_missing > 0) {
            Serial.printf("DEBUG=evict_lru: %u/%u REMOVE addrs missing from hash\r\n",
                          (unsigned)hash_missing, (unsigned)pending_remove_count);
        }
    }

    /* Log first 4 + last 4 addresses going into REMOVE so we can correlate
     * with any external trace of what ra-module actually receives. */
    if (pending_remove_count >= 8) {
        Serial.printf("DEBUG=evict_lru REMOVE-first4: 0x%08lX 0x%08lX 0x%08lX 0x%08lX\r\n",
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[0]),
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[1]),
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[2]),
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[3]));
        Serial.printf("DEBUG=evict_lru REMOVE-last4: 0x%08lX 0x%08lX 0x%08lX 0x%08lX\r\n",
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[pending_remove_count-4]),
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[pending_remove_count-3]),
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[pending_remove_count-2]),
                      (unsigned long)ra_be32_to_host(pending_remove_addrs[pending_remove_count-1]));
    }

    /* Defrag: shift survivors down to fill the holes. We DO NOT touch the
     * hash inline — open-addressing with linear probing tolerates inserts
     * but NOT deletes: a HASH_EMPTY tombstone breaks any probe chain that
     * passed through the now-empty slot, so subsequent hash_lookup misses
     * entries that are still in watch_addresses. Those false misses caused
     * watchlist_append to add duplicates of already-present addresses,
     * which then propagated into pending_remove_addrs as duplicates, then
     * into REMOVE events that ra-module could only partially honor →
     * silent off-by-N desync after eviction (observed v0.19.3-60hz 2026-06-01,
     * off-by-2 froze multi-pass). Instead, REBUILD the hash from scratch
     * after the defrag completes. O(N) but N≤1024 → ~50µs. */
    uint16_t write_idx = dyn_start;
    for (uint16_t read_idx = dyn_start; read_idx < watch_count; read_idx++) {
        if (evict_mark[read_idx]) continue;
        if (read_idx != write_idx) {
            watch_addresses[write_idx] = watch_addresses[read_idx];
            memory_data[write_idx]     = memory_data[read_idx];
            last_used_frame[write_idx] = last_used_frame[read_idx];
        }
        write_idx++;
    }

    uint16_t before = watch_count;
    __atomic_store_n(&watch_count, write_idx, __ATOMIC_RELEASE);

    /* Rebuild hash from scratch — eliminates tombstone-induced false misses. */
    hash_clear();
    for (uint16_t i = 0; i < write_idx; i++) {
        hash_insert(watch_addresses[i], i);
    }

    /* Update ra-module expected count — ra-module will shrink by the same N
     * when it parses REMOVE. */
    ra_module_expected_count -= pending_remove_count;

    /* Clear reject_set — addresses that hit reject before might be valid
     * now that there's room (not strictly needed in v0.18 since we no
     * longer mark_rejected in watchlist_append). */
    reject_count = 0;
    reject_write_idx = 0;

    Serial.printf("DEBUG=evict_lru: %u → %u (dropped %u dynamic, ra_expected=%u)\r\n",
                  (unsigned)before, (unsigned)write_idx,
                  (unsigned)pending_remove_count,
                  (unsigned)ra_module_expected_count);
}

// ============================================================================
// Game loading - runs on Core 0 when state == STATE_LOADING_GAME
// ============================================================================

static void on_game_loaded(int result, const char *error_message, rc_client_t *client, void *userdata) {
    if (result != RC_OK) {
        Serial.printf("ERROR=rc_client: game load failed: %s\r\n", error_message ? error_message : "?");
        state = STATE_ERROR;
        return;
    }
    Serial.println("DEBUG=Game loaded, building initial watchlist...");

    const rc_memrefs_t *memrefs = rc_client_get_memrefs(client);
    uint32_t n = 0;
    if (memrefs)
        n = rc_memrefs_get_addresses(memrefs, prefetch_addrs, prefetch_sizes, PREFETCH_MAX, NULL, NULL);

    watch_count = 0;
    free(watch_addresses); watch_addresses = NULL;
    free(memory_data);     memory_data     = NULL;
    watchlist_update_if_changed(n);

    rc_client_set_read_memory_function(client, read_memory_ingame);

    const rc_client_game_t *game = rc_client_get_game_info(client);
    if (game) {
        gameName = String(game->title);
        Serial.printf("DEBUG=Game: %s\r\n", game->title);
    }

    setSemaphoreLED(LED_ON, LED_GREEN);
    /* Transition to GAME_LOADED so WiiFlow sees status 0x06 (RA_STATUS_GAME_LOADED)
     * and knows it can proceed with the IOS reload + game boot.
     * STATE_ACTIVE (0x07) is set on the first SNAPSHOT received from ra-module. */
    state = STATE_GAME_LOADED;
    Serial.printf("DEBUG=Game loaded, watching %u addresses — waiting for first SNAPSHOT\r\n", watch_count);
}

void loadGame(const char *hash) {
    // Destroy previous client if any
    if (g_client) {
        rc_client_destroy(g_client);
        g_client = NULL;
    }

    g_client = rc_client_create(read_memory_nop, server_call);
    rc_client_enable_logging(g_client, RC_CLIENT_LOG_LEVEL_VERBOSE, log_message);
    rc_client_set_event_handler(g_client, event_handler);
    rc_client_set_hardcore_enabled(g_client, 0);
    rc_client_set_get_time_millisecs_function(g_client, get_millisecs);

    // rc_client needs its own login session with correct URLs via server_call
    String ra_user = read_ra_user_from_eeprom();
    String ra_pass = read_ra_pass_from_eeprom();
    rc_client_begin_login_with_password(g_client, ra_user.c_str(), ra_pass.c_str(),
                                        rc_callback, g_callback_userdata);

    Serial.printf("DEBUG=Loading game with hash: %s\r\n", hash);

    // on_game_loaded sets state = STATE_ACTIVE or STATE_ERROR.
    // server_call is synchronous so all HTTP calls complete before this returns.
    rc_client_begin_load_game(g_client, hash, on_game_loaded, NULL);
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
/* ===================================================================
 * Live cache dump for Kirby's Return to Dream Land "Kirby to the Past"
 * cheevo. Prints the values rcheevos would see when evaluating the chain:
 *   0x008c2760 (32-bit BE)  → player object pointer (mask 0x7FFFFFF)
 *   *(player & 0x7FFFFFF) + 0x1af4 → player_data pointer (mask 0x7FFFFFF)
 *   *(player_data & 0x7FFFFFF) + 0xa64 → CURRENT ABILITY (target: 0x01 = Sword)
 *   0x015ed0c0 (32-bit BE) → Game Mode (target: 0x00 Main or 0x01 Extra)
 *   0x015ee8ea (16-bit BE) → Level/Room ID (must NOT be in exclusion list)
 * If any address is missing from cache, prints "MISS" for that field.
 * Throttled to every 2 seconds. =================================== */
static uint32_t cache_read_u32_be(uint32_t addr, bool *ok) {
    int32_t i0 = hash_lookup(addr + 0);
    int32_t i1 = hash_lookup(addr + 1);
    int32_t i2 = hash_lookup(addr + 2);
    int32_t i3 = hash_lookup(addr + 3);
    if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0) { if (ok) *ok = false; return 0; }
    if (ok) *ok = true;
    return ((uint32_t)memory_data[i0] << 24)
         | ((uint32_t)memory_data[i1] << 16)
         | ((uint32_t)memory_data[i2] <<  8)
         | ((uint32_t)memory_data[i3]      );
}
static uint16_t cache_read_u16_be(uint32_t addr, bool *ok) {
    int32_t i0 = hash_lookup(addr + 0);
    int32_t i1 = hash_lookup(addr + 1);
    if (i0 < 0 || i1 < 0) { if (ok) *ok = false; return 0; }
    if (ok) *ok = true;
    return ((uint16_t)memory_data[i0] << 8) | memory_data[i1];
}
static void dump_kirby_chain() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 2000) return;
    last = now;

    bool ok_pp, ok_pd_masked, ok_pd_raw, ok_mode, ok_room;
    uint32_t player_ptr  = cache_read_u32_be(0x008c2760, &ok_pp);
    uint32_t game_mode   = cache_read_u32_be(0x015ed0c0, &ok_mode);
    uint16_t room_id     = cache_read_u16_be(0x015ee8ea, &ok_room);


    /* Try BOTH the masked address (what cheevo logic actually reads at)
     * AND the raw unmasked address (what rc_memrefs_get_addresses MAY be
     * outputting — without applying the mask modifier). If raw-addr hits
     * but masked-addr misses, we know the issue is rcheevos generating
     * addresses without mask application, and our cache is storing them
     * at the "wrong" key. */
    uint32_t pd_addr_masked = (player_ptr & 0x07FFFFFF) + 0x1af4;
    uint32_t pd_addr_raw    =  player_ptr               + 0x1af4;
    uint32_t pd_val_masked  = cache_read_u32_be(pd_addr_masked, &ok_pd_masked);
    uint32_t pd_val_raw     = cache_read_u32_be(pd_addr_raw,    &ok_pd_raw);

    Serial.printf("DEBUG=KIRBY pptr=%s%08X mode=%s%08X room=%s%04X\r\n",
                  ok_pp   ? "" : "MISS:", player_ptr,
                  ok_mode ? "" : "MISS:", game_mode,
                  ok_room ? "" : "MISS:", room_id);
    Serial.printf("DEBUG=KIRBY pd_masked@%08X=%s%08X pd_raw@%08X=%s%08X\r\n",
                  pd_addr_masked, ok_pd_masked ? "" : "MISS:", pd_val_masked,
                  pd_addr_raw,    ok_pd_raw    ? "" : "MISS:", pd_val_raw);

    /* Compute the FINAL ability address using the fully-masked chain. If
     * both pointer levels resolved to something other than 0x80000000
     * (= null pointer in Kirby's menu state) and we have the value
     * cached, this prints the live ability code: 0x01 = Sword, 0x00 =
     * none, 0x05 = Beam, etc. When ability transitions to 0x01 while
     * Kirby is in a normal level, "Kirby to the Past" should fire. */
    bool ok_ability;
    uint32_t pd_masked = pd_val_masked & 0x07FFFFFF;
    uint32_t ability_addr = pd_masked + 0x0a64;
    uint32_t ability_val = cache_read_u32_be(ability_addr, &ok_ability);
    bool valid_chain = (player_ptr != 0x80000000) && (player_ptr != 0)
                    && ok_pp && ok_pd_masked;
    Serial.printf("DEBUG=KIRBY ability@%08X=%s%08X chain_valid=%s\r\n",
                  ability_addr, ok_ability ? "" : "MISS:", ability_val,
                  valid_chain ? "YES" : "no(player_null)");
}

void processSnapshot() {
    if (!new_snapshot || state != STATE_ACTIVE || !g_client) return;
    new_snapshot = false;

    dump_kirby_chain();

    /* All required addresses are guaranteed cached by the time we get
     * here — the SNAPSHOT/ADDR_RESPONSE handler chain iterates
     * ADDR_QUERY rounds until rc_memrefs_get_addresses reports no
     * missing addresses before setting new_snapshot=true. So the very
     * first do_frame sees all REAL values; rcheevos's per-trigger
     * WAITING state naturally suppresses spurious triggers (see
     * trigger.c:243 — WAITING + condition TRUE → reset to WAITING). */
    rc_client_do_frame(g_client);
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
    // Poll for SPI transaction with 20ms timeout (short to stay responsive)
    if (exi_spi_poll(20)) {
        // Transaction completed - read the data
        if (exi_spi_has_data()) {
            static uint8_t buf[EXI_MAX_TRANSACTION_SIZE];  // STATIC to avoid stack overflow!
            size_t len = exi_spi_read(buf, sizeof(buf));
            if (len > 0) {
                if (DEBUG) {
                    /* Promiscuous log: print every distinct non-0xFF packet immediately.
                     * Dedup by (len, first 4 bytes) so repeated identical frames are
                     * suppressed but any content change always produces a log line. */
                    bool all_ff = true;
                    for (size_t i = 0; i < len && i < 8; i++) {
                        if (buf[i] != 0xFF) { all_ff = false; break; }
                    }
                    if (!all_ff) {
                        static uint8_t last_sig[4] = {0xFF,0xFF,0xFF,0xFF};
                        static size_t  last_sig_len = 0;
                        bool same = (len == last_sig_len &&
                                     buf[0] == last_sig[0] && buf[1] == last_sig[1] &&
                                     (len < 3 || buf[2] == last_sig[2]) &&
                                     (len < 4 || buf[3] == last_sig[3]));
                        if (!same) {
                            last_sig_len = len;
                            for (int _i = 0; _i < 4 && (size_t)_i < len; _i++) last_sig[_i] = buf[_i];
                            // Serial.printf("DEBUG=EXI RX %u bytes: ", (unsigned)len);
                            // for (size_t i = 0; i < len && i < 16; i++)
                            //     Serial.printf("%02X ", buf[i]);
                            // Serial.println();
                        }
                    }
                }
                handle_exi_command(buf, len);
            }
        }
        /* Re-arm the SPI slave AFTER handle_exi_command has had its chance
         * to call exi_spi_prepare_response. exi_spi_poll() intentionally
         * doesn't queue anymore — queueing snapshots tx_buf into the
         * peripheral's send path, so we must queue with the final response
         * already written. See exi_spi_slave.h for the longer rationale. */
        exi_spi_arm();
    }
    // Drain any queued markers AFTER the SPI slave has been re-armed.
    // Serial.printf here no longer blocks the receive path.
    drain_marker_queue();
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
            Serial.printf("DEBUG=state transition %02X -> %02X\r\n",
                          last_published_state, (uint8_t)state);
            last_published_state = (uint8_t)state;
            exi_spi_update_status(last_published_state);
        }

        processEXI();
        vTaskDelay(1);  // Yield to prevent watchdog
        
        // Heartbeat every 5 seconds
        uint32_t now = millis();
        if (now - last_heartbeat >= 5000) {
            last_heartbeat = now;
            int cs = gpio_get_level((gpio_num_t)EXI_PIN_CS);
            /* Echo firmware version + build stamp in every heartbeat so we can
             * confirm a flashed binary is current without needing the boot log. */
            Serial.printf("DEBUG=fw v0.19.3-60hz build=%s %s\n", __DATE__, __TIME__);
            Serial.printf("DEBUG=Core1 alive | CS=%d | transactions=%lu | rx_bytes=%lu | state=%d\n",
                          cs, (unsigned long)exi_spi_get_transaction_count(),
                          (unsigned long)exi_spi_get_total_bytes_rx(), (int)state);
            /* Capture the post-setup callback diagnostics. cb_invocations tells
             * us whether the callback ever fires (and how many times) and
             * cb_last_response_len / cb_last_tx6 capture the state seen by the
             * cb at the most recent invocation. If response_len is small (14
             * = default_ack) at every fire, the cb is running BEFORE main loop
             * sets the dynamic response — race we'll fix differently. */
            extern volatile uint32_t exi_cb_invocations;
            extern volatile uint16_t exi_cb_last_response_len;
            extern volatile uint8_t  exi_cb_last_response_b0;
            extern volatile uint8_t  exi_cb_last_tx_b6;
            extern volatile uint32_t exi_prepare_call_count;
            extern volatile uint16_t exi_prepare_last_len;
            extern volatile uint8_t  exi_prepare_post_tx_b6;
            Serial.printf("DEBUG=cb invocations=%lu last_response_len=%u resp[0]=%02X tx[6]=%02X\n",
                          (unsigned long)exi_cb_invocations,
                          (unsigned)exi_cb_last_response_len,
                          (unsigned)exi_cb_last_response_b0,
                          (unsigned)exi_cb_last_tx_b6);
            Serial.printf("DEBUG=prepare calls=%lu last_len=%u post_tx[6]=%02X\n",
                          (unsigned long)exi_prepare_call_count,
                          (unsigned)exi_prepare_last_len,
                          (unsigned)exi_prepare_post_tx_b6);

            /* Dump default_ack to confirm the dynamic-status update is in this
             * firmware build (byte 5 should match the current state value). */
            char ack_str[64] = {0};
            exi_spi_dump_default_ack(ack_str, sizeof(ack_str));
            Serial.printf("DEBUG=default_ack: %s\n", ack_str);

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
            Serial.println(line);
        }
    }
}

// ============================================================================
// Arduino setup() and loop()
// ============================================================================
void setup() {
    pinMode(RESET_PIN, INPUT);
    beginEEPROM(false);
    /* v0.18.10: bumped 115200 → 250000 (max of vscode serial monitor)*/
    Serial.begin(250000);
    delay(250);
    /* Mark all hash buckets as empty BEFORE any potential lookup.
     * Static .bss gives us all-zero which would alias to "addr=0 is here". */
    hash_clear();
    
    FastLED.addLeds<NEOPIXEL, PIN_RGB>(leds, NUM_LEDS);
    setSemaphoreLED(LED_BLINK_MEDIUM, LED_YELLOW);
    /* Bump on any meaningful change so the user can verify the flash actually
     * landed by looking at the serial log. Format: vMAJOR.MINOR.BUILDID where
     * BUILDID is __DATE__ __TIME__ from preprocessor. */
    Serial.printf("WII-RA-ADAPTER v0.19.3-60hz (build %s %s)\n", __DATE__, __TIME__);

    // Initialize SPI slave for EXI
    if (!exi_spi_init(NULL)) {
        Serial.println("ERROR=SPI init failed!");
        setSemaphoreLED(LED_BLINK_FAST, LED_RED);
        while(1) yield();
    }
    Serial.println("DEBUG=SPI slave initialized");

    // WiFi + RA login (same flow as fpga-ra-adapter)
    if (!isConfigured()) {
        wm.setDebugOutput(false);
        wm.setCustomHeadElement(head);
        wm.setDarkMode(true);
        wm.addParameter(&custom_p1);
        wm.addParameter(&custom_user);
        wm.addParameter(&custom_pass);
        wm.setAPStaticIPConfig(IPAddress(192,168,1,1), IPAddress(192,168,1,1), IPAddress(255,255,255,0));
        
        while (!isConfigured()) {
            setSemaphoreLED(LED_BLINK_MEDIUM, LED_YELLOW);
            Serial.println("DEBUG=Connect to 'WII_RA_ADAPTER' WiFi, open http://192.168.1.1");
            if (wm.startConfigPortal("WII_RA_ADAPTER", "12345678")) {
                String token = try_login_RA(custom_user.getValue(), custom_pass.getValue());
                if (token != "null") {
                    save_configuration_info_eeprom(custom_user.getValue(), custom_pass.getValue());
                    setSemaphoreLED(LED_ON, LED_GREEN);
                } else {
                    setSemaphoreLED(LED_BLINK_FAST, LED_RED);
                }
            }
        }
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.begin();
        while (WiFi.status() != WL_CONNECTED) yield();
        setSemaphoreLED(LED_BLINK_SLOW, LED_GREEN);
        Serial.println("DEBUG=WiFi OK");
        String token = try_login_RA(read_ra_user_from_eeprom(), read_ra_pass_from_eeprom());
        if (token != "null") {
            setSemaphoreLED(LED_BLINK_MEDIUM, LED_GREEN);
            Serial.println("DEBUG=RA login OK");
        }
    }

    state = STATE_WAIT_GAME_ID;
    Serial.println("DEBUG=Ready, waiting for GameCube...");

    // Start Core 1 task for EXI communication (16KB stack to avoid overflow)
    xTaskCreatePinnedToCore(taskCore1, "EXI_Task", 16384, nullptr, 1, &taskCore1Handle, 1);
}

void loop() {
    // Reconnect WiFi if needed
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin();
        while (WiFi.status() != WL_CONNECTED) yield();
    }
    
    // State machine
    if (state == STATE_LOADING_GAME) {
        loadGame(gameId.c_str());
        // state is now STATE_ACTIVE or STATE_ERROR (set by on_game_loaded callback)
    }
    
    processSnapshot();
    vTaskDelay(1);
}
