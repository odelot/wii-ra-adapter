/**
 * gc_ra_protocol.h
 * 
 * Binary protocol definition for GameCube RetroAchievements via ESP32-S3 (EXI/SPI)
 * 
 * This protocol replaces the text-based UART protocol from fpga-ra-adapter
 * with a compact binary protocol optimized for the GameCube's EXI bus (SPI).
 * 
 * Key differences from fpga-ra-adapter:
 * - Binary instead of ASCII text
 * - 32-bit addresses (GameCube) instead of 16-bit (NES)
 * - Snapshot-based (GC reads RAM and sends values) instead of write-notification-based
 * - SPI slave instead of UART serial
 * 
 * Communication flow:
 *   1. GameCube detects ESP32-RA on EXI bus via device ID query
 *   2. GameCube sends game identification (Game ID + optional hash)
 *   3. ESP32 loads game from RA, discovers memory addresses, sends watch list
 *   4. Every VBlank: GameCube reads watched addresses, sends snapshot to ESP32
 *   5. ESP32 runs rc_client_do_frame(), sends back events (achievements, etc.)
 */

#ifndef GC_RA_PROTOCOL_H
#define GC_RA_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Protocol Constants
 * ============================================================================
 */

/** Device ID returned by ESP32-RA when queried via EXI device detection */
#define RA_DEVICE_ID            0x52410001  /* "RA" + version 0.0.1 */

/** Protocol version */
#define RA_PROTOCOL_VERSION     0x01

/** Magic bytes for packet validation */
#define RA_MAGIC_GC_TO_ESP      0x52  /* 'R' - GameCube to ESP32 */
#define RA_MAGIC_ESP_TO_GC      0xAE  /* ESP32 to GameCube */

/** Maximum number of watched addresses.
 *  6144 since v0.24.7 — SMG's LIVE per-frame working set measured
 *  ~3700 on hw (119 active achievements x ~30 chain bytes each), which
 *  sat exactly ON the old 4096/3800 ceiling: eviction at capacity has
 *  no cold entries to pick, so it dropped live bytes, the evaluator
 *  re-requested them next frame, and the system livelocked
 *  (3698->3442->3570->3698 cycle, fires at ~10Hz) while masked-chain
 *  bytes read 0 on miss and mass-fired false unlocks. Capacity must
 *  sit comfortably above live demand; snapshot (12+6144=6156 B) still
 *  fits EXI_MAX_TRANSACTION_SIZE=8192.
 *  KEEP IN SYNC with the copy of this header in d2x-cios/source/ra-module/. */
#define RA_MAX_WATCH_ADDRS      6144

/** Number of addresses per watchlist chunk (fits in one EXI transaction) */
/* Each chunk: 6B resp header + 4B chunk_hdr + N*4B addrs <= EXI_MAX_TRANSACTION_SIZE */
/* 4 + (N*4) <= 8192  =>  N <= 2047. Use 1024 for safety. */
#define RA_WATCHLIST_CHUNK_ADDRS  1024

/** Maximum response data payload size */
#define RA_MAX_RESPONSE_DATA    256

/** Maximum achievement title length */
#define RA_MAX_TITLE_LEN        128

/** Game ID length (GameCube disc header) */
#define RA_GAME_ID_LEN          6

/** MD5 hash length */
#define RA_HASH_LEN             32

/*
 * ============================================================================
 * Command Types: GameCube → ESP32
 * ============================================================================
 */

typedef enum {
    /** Query device identity - ESP32 responds with RA_DEVICE_ID */
    RA_CMD_IDENTIFY         = 0x01,

    /** Send game identification data to ESP32 */
    RA_CMD_LOAD_GAME        = 0x02,

    /** Send memory snapshot (values of all watched addresses) */
    RA_CMD_SNAPSHOT          = 0x03,

    /** Poll for pending events/responses from ESP32 */
    RA_CMD_POLL              = 0x04,

    /** Notify ESP32 that the game was reset/changed */
    RA_CMD_GAME_RESET        = 0x05,

    /** Request current status from ESP32 */
    RA_CMD_STATUS            = 0x06,

    /**
     * Request one chunk of the pending watchlist update.
     * Payload: uint16_t chunk_index (big-endian)
     * Response: ra_watchlist_chunk_t header + up to RA_WATCHLIST_CHUNK_ADDRS addresses.
     * GC calls this repeatedly (chunk 0, 1, …) until chunk.is_last == 1.
     */
    RA_CMD_GET_WATCHLIST_CHUNK = 0x07,

    /**
     * Send queried memory values back to ESP32.
     * Payload: ra_addr_response_t header + N bytes (one per queried address).
     * Sent by GC in response to RA_EVT_ADDR_QUERY.
     */
    RA_CMD_ADDR_RESPONSE     = 0x08,

    /**
     * Send a debug message (ASCII text) to ESP32 — ESP32 logs it via Serial.
     * Payload format: u8 msg_len, then msg_len bytes of ASCII text.
     * Use for diagnostics from custom IOS modules where LED-encoding numbers
     * is fragile / impractical.
     */
    RA_CMD_DEBUG_LOG         = 0x09,

    /**
     * Wipe the WiFi + RetroAchievements credentials stored on the ESP32 by
     * WiFiManager / EEPROM, then reboot the ESP32 into its config portal.
     * No payload. The response in the same transaction is unreliable (the ESP
     * reboots shortly after), so the sender should not depend on it.
     * Replaces the physical "reset button on the memory card" used by the
     * nes-ra-adapter: WiiFlow triggers it from a Settings menu entry.
     */
    RA_CMD_RESET_CREDENTIALS = 0x0A,
} ra_gc_command_t;

/*
 * ============================================================================
 * Event Types: ESP32 → GameCube
 * ============================================================================
 */

typedef enum {
    /** No pending event */
    RA_EVT_NONE              = 0x00,

    /** Achievement triggered - data contains achievement info */
    RA_EVT_ACHIEVEMENT       = 0x01,

    /**
     * Watchlist changed - GC must fetch new list via RA_CMD_GET_WATCHLIST_CHUNK.
     * data_len == 4: uint16_t total_addr_count (BE) + uint16_t num_chunks (BE).
     * After receiving this event, GC calls RA_CMD_GET_WATCHLIST_CHUNK(0..n-1).
     */
    RA_EVT_WATCHLIST_UPDATE  = 0x02,

    /** Game info loaded - data contains game title */
    RA_EVT_GAME_INFO         = 0x03,

    /** Login status update */
    RA_EVT_LOGIN_STATUS      = 0x04,

    /** Error occurred */
    RA_EVT_ERROR             = 0x05,

    /** Leaderboard started */
    RA_EVT_LEADERBOARD_START = 0x06,

    /** Leaderboard submitted */
    RA_EVT_LEADERBOARD_SUBMIT = 0x07,

    /** Challenge indicator show/hide */
    RA_EVT_CHALLENGE         = 0x08,

    /** Rich presence update */
    RA_EVT_RICH_PRESENCE     = 0x09,

    /**
     * ESP32 asks GC to read specific addresses from RAM.
     * data: ra_addr_query_t header + N × uint32_t addresses (big-endian).
     * GC must read the byte at each address and send RA_CMD_ADDR_RESPONSE.
     * This enables two-pass processing: resolve indirect addresses with
     * fresh snapshot values, then fetch any missing bytes before do_frame.
     */
    RA_EVT_ADDR_QUERY        = 0x0A,

    /**
     * ESP32 announces NEW addresses to APPEND to the GC's local watchlist.
     * After receiving, ra-module must include these addresses in all future
     * SNAPSHOTs (watchlist grows additively — never shrinks via this event).
     * Payload: ra_watchlist_append_t followed by addr_count u32 addresses (BE).
     * This avoids a GET_CHUNK round-trip for incremental growth and keeps
     * ra-module/ESP32 in sync without wiping memory_data.
     */
    RA_EVT_WATCHLIST_APPEND  = 0x0B,

    /**
     * DEPRECATED in favor of RA_EVT_WATCHLIST_REMOVE. TRUNCATE-tail
     * eviction also drops byte-fanouts of static base addresses, breaking
     * chain[0] reads. See project_eviction_disabled_v0_17_2.md.
     */
    RA_EVT_WATCHLIST_TRUNCATE = 0x0C,

    /**
     * ESP32 evicted N specific addresses (LRU-selected, chain-root-safe)
     * and tells ra-module to remove them. Payload: ra_watchlist_remove_t
     * + N × u32 addresses (BE). ra-module finds each in ra_watchlist[],
     * removes by shifting subsequent entries down. Insertion order of
     * survivors preserved on both sides → SNAPSHOT positions stay aligned.
     */
    RA_EVT_WATCHLIST_REMOVE  = 0x0D,

    /* Watchlist removal by INDEX (v0.25.0 — replaces address-based
     * REMOVE on the hot path). Both replicas are identical before the
     * removal, so the ESP sends the u16 array indices of the entries to
     * drop (ascending order) instead of their u32 addresses. ra-module
     * marks by index directly — O(R) instead of the O(R*N) linear
     * address search that cost ~25-30ms (≈2 frames!) at N=6144 — then
     * compacts in one pass. Index identity also eliminates the entire
     * address-ambiguity bug class (duplicate addresses, hash-tombstone
     * off-by-N).
     *
     * Payload: ra_watchlist_remove_idx_t (u16 idx_count, BE) followed
     * by idx_count x u16 indices (BE, ascending, all < current count).
     *
     * Parse MUST be in both ra_send_snapshot AND ra_send_addr_response
     * (twin function divergence hazard). */
    RA_EVT_WATCHLIST_REMOVE_IDX = 0x0E,
} ra_esp_event_t;

/**
 * Internal ESP32 event structure for the pending events queue.
 */
typedef struct {
    uint8_t type;       /* ra_esp_event_t */
    uint8_t data[RA_MAX_RESPONSE_DATA];
    uint16_t data_len;
} PendingEvent;

/*
 * ============================================================================
 * Status Codes
 * ============================================================================
 */

typedef enum {
    RA_STATUS_INITIALIZING   = 0x00,
    RA_STATUS_WIFI_CONNECTING = 0x01,
    RA_STATUS_WIFI_CONNECTED = 0x02,
    RA_STATUS_LOGGING_IN     = 0x03,
    RA_STATUS_LOGGED_IN      = 0x04,
    RA_STATUS_LOADING_GAME   = 0x05,
    RA_STATUS_GAME_LOADED    = 0x06,
    RA_STATUS_ACTIVE         = 0x07,  /* Fully operational, processing frames */
    RA_STATUS_ERROR_WIFI     = 0xE0,
    RA_STATUS_ERROR_LOGIN    = 0xE1,
    RA_STATUS_ERROR_GAME     = 0xE2,
    RA_STATUS_ERROR_PROTOCOL = 0xE3,
} ra_status_t;

/*
 * ============================================================================
 * Packet Structures: GameCube → ESP32
 * ============================================================================
 * 
 * All multi-byte values are BIG-ENDIAN (native to PowerPC/GameCube)
 * ESP32 (little-endian) must byte-swap on receive.
 */

/**
 * Header for all GC→ESP32 packets
 * Sent at the start of every EXI transaction
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* RA_MAGIC_GC_TO_ESP (0x52) */
    uint8_t  command;        /* ra_gc_command_t */
    uint16_t payload_len;    /* Length of data following this header */
} ra_gc_header_t;

/**
 * RA_CMD_LOAD_GAME payload
 * Sent once when the game boots
 */
typedef struct __attribute__((packed)) {
    char     game_id[RA_GAME_ID_LEN];  /* 6-byte Game ID from disc header (e.g., "GALE01") */
    uint8_t  disc_number;              /* Disc number (0 for single-disc games) */
    uint8_t  has_hash;                 /* 1 if md5_hash is valid, 0 otherwise */
    char     md5_hash[RA_HASH_LEN];   /* MD5 hash of the ISO (optional, for direct RA lookup) */
} ra_load_game_t;

/**
 * RA_CMD_SNAPSHOT payload
 * Sent every VBlank (~60 times per second)
 * 
 * Contains the values of all currently watched addresses.
 * The GameCube reads these directly from RAM using the watch list
 * provided by the ESP32 in RA_EVT_WATCHLIST_UPDATE.
 * 
 * This is the HOT PATH - must be as small and fast as possible.
 */
typedef struct __attribute__((packed)) {
    uint32_t frame_counter;  /* Incrementing frame counter */
    uint16_t addr_count;     /* Number of values in this snapshot */
    uint16_t wl_seq;         /* v0.26.0 Phase D2: seq of the LAST watchlist
                              * mutation the Wii applied. Sync is VERIFIED
                              * against the ESP's emitted seq every frame —
                              * behind → resend exactly the next mutation
                              * (idempotent); impossible → full RESYNC. */
    /* Followed by addr_count bytes: values[0..addr_count-1] */
    /* Each byte corresponds to the address at the same index in the watch list */
} ra_snapshot_header_t;

/*
 * ============================================================================
 * Packet Structures: ESP32 → GameCube
 * ============================================================================
 */

/**
 * Header for all ESP32→GC responses
 * Always sent as the response in the same EXI transaction
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* RA_MAGIC_ESP_TO_GC (0xAE) */
    uint8_t  status;         /* ra_status_t - current ESP32 state */
    uint8_t  event_type;     /* ra_esp_event_t - pending event */
    uint8_t  event_count;    /* Number of additional pending events */
    uint16_t data_len;       /* Length of event data following this header */
} ra_esp_header_t;

/**
 * RA_EVT_WATCHLIST_UPDATE notification data (4 bytes)
 * Sent inline with the snapshot response to tell GC a new list is ready.
 * GC then calls RA_CMD_GET_WATCHLIST_CHUNK to fetch the actual addresses.
 */
typedef struct __attribute__((packed)) {
    uint16_t total_addr_count;  /* big-endian total number of addresses */
    uint16_t num_chunks;        /* big-endian number of chunks to fetch */
    uint16_t seq;               /* v0.26.0: mutation-seq base of this (re)load —
                                 * after the chunk fetch completes, BOTH sides
                                 * hold this seq and the list it names. */
} ra_watchlist_notify_t;

/**
 * RA_CMD_GET_WATCHLIST_CHUNK request payload (sent by GC)
 */
typedef struct __attribute__((packed)) {
    uint16_t chunk_index;       /* big-endian: 0-based index of chunk requested */
} ra_watchlist_chunk_req_t;

/**
 * RA_CMD_GET_WATCHLIST_CHUNK response header
 * Followed by up to RA_WATCHLIST_CHUNK_ADDRS × uint32_t addresses (big-endian)
 */
typedef struct __attribute__((packed)) {
    uint16_t chunk_index;       /* echoes the requested chunk index */
    uint16_t addr_count;        /* number of addresses in THIS chunk */
    uint8_t  is_last;           /* 1 if this is the final chunk */
    uint8_t  reserved;
} ra_watchlist_chunk_t;

/**
 * RA_EVT_WATCHLIST_UPDATE data (legacy, kept for compat — now used as notify only)
 * GC should use RA_CMD_GET_WATCHLIST_CHUNK instead of reading addresses inline.
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;     /* Number of addresses in the watch list */
    /* Followed by addr_count × uint32_t: addresses to monitor */
} ra_watchlist_t;

/**
 * RA_EVT_ADDR_QUERY data
 * ESP32 asks GC to read specific addresses and send back the values.
 * Followed by addr_count × uint32_t addresses (big-endian).
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;     /* Number of addresses to query (big-endian) */
} ra_addr_query_t;

/**
 * RA_EVT_WATCHLIST_APPEND data — same wire format as ADDR_QUERY:
 * count followed by `count` u32 addresses (BE). Distinct struct so the
 * intent is clear at call sites.
 */
typedef struct __attribute__((packed)) {
    uint16_t seq;            /* v0.26.0: mutation seq — Wii applies iff seq == ra_seq+1 */
    uint16_t addr_count;     /* Number of addresses to append (big-endian) */
} ra_watchlist_append_t;

/**
 * RA_EVT_WATCHLIST_TRUNCATE data — DEPRECATED, see WATCHLIST_REMOVE.
 */
typedef struct __attribute__((packed)) {
    uint16_t keep_count;     /* New watchlist length (big-endian) */
} ra_watchlist_truncate_t;

/**
 * RA_EVT_WATCHLIST_REMOVE data — addr_count followed by addr_count u32
 * addresses (BE). ra-module locates each in ra_watchlist[] (linear scan)
 * and removes by shift. Survivors keep insertion order so SNAPSHOT memcpy
 * stays position-aligned with ESP32's compacted watch_addresses[].
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;     /* Number of addresses to remove (big-endian) */
} ra_watchlist_remove_t;

/**
 * RA_EVT_WATCHLIST_REMOVE_IDX data — idx_count followed by idx_count u16
 * array indices (BE, ascending, all < current watchlist count). ra-module
 * marks the slots directly and compacts in a single pass.
 */
typedef struct __attribute__((packed)) {
    uint16_t seq;            /* v0.26.0: mutation seq — Wii applies iff seq == ra_seq+1 */
    uint16_t idx_count;
} ra_watchlist_remove_idx_t;

/**
 * RA_CMD_ADDR_RESPONSE data
 * GC sends back the values for previously queried addresses.
 * Followed by addr_count × uint8_t values (one byte per address, same order).
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;     /* Number of values (big-endian) */
} ra_addr_response_t;

/**
 * RA_EVT_ACHIEVEMENT data
 * Sent when an achievement is triggered
 */
typedef struct __attribute__((packed)) {
    uint32_t achievement_id;
    uint8_t  title_len;
    /* Followed by title_len bytes of UTF-8 title string (no null terminator) */
} ra_achievement_t;

/**
 * RA_EVT_GAME_INFO data
 * Sent after successful game load
 */
typedef struct __attribute__((packed)) {
    uint32_t game_id;         /* RA game database ID */
    uint16_t achievement_count;
    uint8_t  title_len;
    /* Followed by title_len bytes of UTF-8 game title */
} ra_game_info_t;

/**
 * RA_EVT_LEADERBOARD_START / RA_EVT_LEADERBOARD_SUBMIT data
 */
typedef struct __attribute__((packed)) {
    uint32_t leaderboard_id;
    uint8_t  title_len;
    /* Followed by title_len bytes of title, then formatted score string */
} ra_leaderboard_t;

/*
 * ============================================================================
 * Helper Macros
 * ============================================================================
 */

/** Calculate total snapshot packet size for N watched addresses */
#define RA_SNAPSHOT_PACKET_SIZE(n) \
    (sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t) + (n))

/** Calculate total watchlist chunk response size for N addresses */
#define RA_WATCHLIST_CHUNK_RESPONSE_SIZE(n) \
    (sizeof(ra_esp_header_t) + sizeof(ra_watchlist_chunk_t) + (n) * sizeof(uint32_t))

/** Number of chunks needed to transfer N addresses */
#define RA_WATCHLIST_NUM_CHUNKS(n) \
    (((n) + RA_WATCHLIST_CHUNK_ADDRS - 1) / RA_WATCHLIST_CHUNK_ADDRS)

/** Minimum response size (header only, no event) */
#define RA_MIN_RESPONSE_SIZE    sizeof(ra_esp_header_t)

/*
 * ============================================================================
 * Byte-order helpers (ESP32 is little-endian, GameCube is big-endian)
 * ============================================================================
 */

#ifdef ESP_PLATFORM
/* ESP32 side: convert from big-endian (network/GC) to native (little-endian) */
static inline uint16_t ra_be16_to_host(uint16_t v) {
    return __builtin_bswap16(v);
}
static inline uint32_t ra_be32_to_host(uint32_t v) {
    return __builtin_bswap32(v);
}
static inline uint16_t ra_host_to_be16(uint16_t v) {
    return __builtin_bswap16(v);
}
static inline uint32_t ra_host_to_be32(uint32_t v) {
    return __builtin_bswap32(v);
}
#else
/* GameCube side (PowerPC, big-endian): no conversion needed */
#define ra_be16_to_host(v) (v)
#define ra_be32_to_host(v) (v)
#define ra_host_to_be16(v) (v)
#define ra_host_to_be32(v) (v)
#endif

#ifdef __cplusplus
}
#endif

#endif /* GC_RA_PROTOCOL_H */
