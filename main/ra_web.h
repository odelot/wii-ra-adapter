#pragma once
/*
 * ra_web — LAN dashboard for the Wii RA adapter (Phase 1: mDNS + HTTP + /api/state).
 *
 * Everything here runs on Core 0 (the network core) so the Core 1 EXI + do_frame
 * realtime path is never touched. The rc_client is NOT thread-safe, so the HTTP
 * handler on Core 0 never reads it directly: the rc_client/loop task (Core 1)
 * publishes a compact snapshot via ra_web_publish() and the handler serves that.
 *
 * Later phases add a WebSocket (/ws) for live unlock/challenge/progress pushes and
 * a full achievement-list JSON built from a PSRAM shadow model.
 */
#include <stdint.h>

/* Feature flag — single source of truth. Define to 0 in the build to compile the
 * whole dashboard out (mirrors RA_TROPHY_OVERLAY / RA_DEBUG_SEND gating). */
#ifndef RA_WEB_DASHBOARD
#define RA_WEB_DASHBOARD 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Published cross-core snapshot. Written ONLY from the rc_client/loop task
 * (Core 1) under an internal spinlock; read by the HTTP handler (Core 0). */
typedef struct {
    uint32_t game_id;
    uint16_t total;            /* core achievements in the set        */
    uint16_t unlocked;         /* how many the user has unlocked      */
    uint32_t points_total;
    uint32_t points_unlocked;
    uint8_t  state;            /* firmware STATE_* enum value          */
    char     game_title[128];
} ra_web_pub_t;

/* Bring up mDNS (http://wii-ra.local/) + esp_http_server on Core 0.
 * No-op when RA_WEB_DASHBOARD == 0. Safe to call from setup() — it only spawns
 * its own Core-0 server task. Requires WiFi to already be connected. */
void ra_web_init(void);

/* Publish the small header snapshot (game + summary). Call ONLY from the
 * rc_client/loop task. Serves as the fallback for /api/state before the full
 * achievement JSON has been built (i.e. before a phone connects). */
void ra_web_publish(const ra_web_pub_t *pub);

/* Publish the full /api/state JSON (header + achievement array), pre-rendered on
 * the loop task. Copies into an internal PSRAM buffer under a mutex; the Core-0
 * handler serves that copy. Call ONLY from the rc_client/loop task. */
void ra_web_set_state_json(const char *json, unsigned len);

/* True if a browser fetched /api/state (or /) within the last window_ms, OR a
 * WebSocket client is connected — i.e. a phone is actually watching. The loop
 * task uses this to gate the (heavier) full-list build so there is ZERO cost
 * when nobody is connected. */
int ra_web_client_active(unsigned window_ms);

/* ---- Live event push (WebSocket /ws) --------------------------------------
 * All ra_web_push_* are called from the rc_client/loop task (Core 1). They only
 * enqueue a small fixed-size event into a FreeRTOS queue and return immediately
 * — the actual socket write happens on the Core-0 ws task, so the do_frame path
 * is never blocked by the network. No-op if no client is connected. */

/* Achievement unlocked — sent to the browser immediately (toast + sound). */
void ra_web_push_unlock(unsigned id, const char *title, const char *badge);

/* Challenge indicator primed/cleared (show=1/0). */
void ra_web_push_challenge(int show, unsigned id);

/* Measured-progress change for one achievement (coalesced to ~4 Hz on Core 0). */
void ra_web_push_progress(unsigned id, int pct, const char *measured);

/* Rich-presence line changed. */
void ra_web_push_rp(const char *msg);

/* Game (re)loaded / changed — tells connected clients to re-fetch /api/state. */
void ra_web_push_reload(void);

#ifdef __cplusplus
}
#endif
