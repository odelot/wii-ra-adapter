#include "ra_web.h"
#if RA_WEB_DASHBOARD

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mdns.h"

static const char *TAG = "ra_web";

/* Single-file dashboard, embedded from main/web/index.html (EMBED_TXTFILES). */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

/* Small header snapshot, guarded by a cross-core spinlock. Core 1 holds it only
 * for a ~150-byte struct copy — negligible for do_frame. Used as the /api/state
 * fallback until the full achievement JSON exists (before a phone connects). */
static portMUX_TYPE  s_pub_mux = portMUX_INITIALIZER_UNLOCKED;
static ra_web_pub_t  s_pub;
static httpd_handle_t s_server = NULL;

/* Full /api/state JSON, rendered on the loop task and copied here (PSRAM) under
 * a FreeRTOS mutex. The Core-0 handler copies it out under the same mutex and
 * sends the copy WITHOUT holding the lock (so a slow network write never blocks
 * the loop task's next publish). */
static SemaphoreHandle_t s_json_mux = NULL;
static char             *s_json     = NULL;   /* PSRAM */
static size_t            s_json_cap = 0;
static size_t            s_json_len = 0;

/* Last time a browser hit / or /api/state — the "is a phone watching?" signal. */
static volatile uint32_t s_last_fetch_ms = 0;

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

void ra_web_publish(const ra_web_pub_t *pub)
{
    portENTER_CRITICAL(&s_pub_mux);
    s_pub = *pub;
    portEXIT_CRITICAL(&s_pub_mux);
}

void ra_web_set_state_json(const char *json, unsigned len)
{
    if (!s_json_mux || !json) return;
    xSemaphoreTake(s_json_mux, portMAX_DELAY);
    if (len + 1 > s_json_cap) {
        size_t ncap = len + 1;
        char *nb = (char *)heap_caps_realloc(s_json, ncap, MALLOC_CAP_SPIRAM);
        if (nb) { s_json = nb; s_json_cap = ncap; }
        else { xSemaphoreGive(s_json_mux); return; }   /* keep old buffer */
    }
    memcpy(s_json, json, len);
    s_json[len] = '\0';
    s_json_len = len;
    xSemaphoreGive(s_json_mux);
}

/* ---- WebSocket /ws: live push --------------------------------------------- */

enum { WS_UNLOCK = 1, WS_CHALLENGE, WS_PROGRESS, WS_RP, WS_RELOAD };

typedef struct {
    uint8_t  type;
    uint8_t  show;   /* challenge */
    int16_t  pct;    /* progress  */
    uint32_t id;
    char     s1[64]; /* title / rp text */
    char     s2[24]; /* badge / measured */
} ws_evt_t;

#define WS_MAX_CLIENTS 3
static int              s_ws_fds[WS_MAX_CLIENTS];
static int              s_ws_nfd = 0;
static portMUX_TYPE     s_ws_fd_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t    s_ws_q = NULL;

static int ra_web_ws_count(void)
{
    portENTER_CRITICAL(&s_ws_fd_mux);
    int n = s_ws_nfd;
    portEXIT_CRITICAL(&s_ws_fd_mux);
    return n;
}

static void ws_fd_add(int fd)
{
    portENTER_CRITICAL(&s_ws_fd_mux);
    for (int i = 0; i < s_ws_nfd; i++) if (s_ws_fds[i] == fd) { portEXIT_CRITICAL(&s_ws_fd_mux); return; }
    if (s_ws_nfd < WS_MAX_CLIENTS) s_ws_fds[s_ws_nfd++] = fd;
    portEXIT_CRITICAL(&s_ws_fd_mux);
}

static void ws_fd_remove(int fd)
{
    portENTER_CRITICAL(&s_ws_fd_mux);
    for (int i = 0; i < s_ws_nfd; i++) {
        if (s_ws_fds[i] == fd) { s_ws_fds[i] = s_ws_fds[--s_ws_nfd]; break; }
    }
    portEXIT_CRITICAL(&s_ws_fd_mux);
}

int ra_web_client_active(unsigned window_ms)
{
    if (ra_web_ws_count() > 0) return 1;
    uint32_t last = s_last_fetch_ms;
    if (last == 0) return 0;
    return (now_ms() - last) < window_ms;
}

/* Marshal the actual socket write onto the httpd task (the documented-safe way
 * to push async WS frames). Payload is a heap copy freed after the send. */
typedef struct { int fd; char *payload; size_t len; } ws_async_t;

static void ws_async_send(void *arg)
{
    ws_async_t *m = (ws_async_t *)arg;
    httpd_ws_frame_t f;
    memset(&f, 0, sizeof(f));
    f.type    = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t *)m->payload;
    f.len     = m->len;
    if (httpd_ws_send_frame_async(s_server, m->fd, &f) != ESP_OK)
        ws_fd_remove(m->fd);   /* client gone → drop it */
    free(m->payload);
    free(m);
}

static void ws_broadcast(const char *json, size_t len)
{
    if (!s_server) return;
    int fds[WS_MAX_CLIENTS], n;
    portENTER_CRITICAL(&s_ws_fd_mux);
    n = s_ws_nfd;
    for (int i = 0; i < n; i++) fds[i] = s_ws_fds[i];
    portEXIT_CRITICAL(&s_ws_fd_mux);

    for (int i = 0; i < n; i++) {
        /* PSRAM for the marshalling scratch — internal SRAM is scarce (rcheevos
         * leaves ~18KB) and shared with the RA-server TLS pings. */
        ws_async_t *m = (ws_async_t *)heap_caps_malloc(sizeof(ws_async_t), MALLOC_CAP_SPIRAM);
        if (!m) continue;
        m->fd = fds[i];
        m->payload = (char *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
        if (!m->payload) { free(m); continue; }
        memcpy(m->payload, json, len);
        m->len = len;
        if (httpd_queue_work(s_server, ws_async_send, m) != ESP_OK) {
            free(m->payload); free(m);
        }
    }
}

static size_t ws_json_esc_append(char *dst, size_t cap, size_t o, const char *s)
{
    for (size_t i = 0; s && s[i] && o + 7 < cap; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') { dst[o++] = '\\'; dst[o++] = (char)c; }
        else if (c < 0x20)         { o += snprintf(dst + o, cap - o, "\\u%04x", c); }
        else                       { dst[o++] = (char)c; }
    }
    return o;
}

static void ws_format_and_send(const ws_evt_t *e)
{
    char buf[256];
    size_t o = 0;
    switch (e->type) {
        case WS_UNLOCK:
            o += snprintf(buf, sizeof(buf), "{\"e\":\"unlock\",\"id\":%lu,\"t\":\"",
                          (unsigned long)e->id);
            o = ws_json_esc_append(buf, sizeof(buf), o, e->s1);
            o += snprintf(buf + o, sizeof(buf) - o, "\",\"badge\":\"%s\"}", e->s2);
            break;
        case WS_CHALLENGE:
            o = snprintf(buf, sizeof(buf), "{\"e\":\"challenge\",\"show\":%u,\"id\":%lu}",
                         (unsigned)e->show, (unsigned long)e->id);
            break;
        case WS_PROGRESS:
            o += snprintf(buf, sizeof(buf), "{\"e\":\"progress\",\"id\":%lu,\"pct\":%d,\"m\":\"",
                          (unsigned long)e->id, (int)e->pct);
            o = ws_json_esc_append(buf, sizeof(buf), o, e->s2);
            o += snprintf(buf + o, sizeof(buf) - o, "\"}");
            break;
        case WS_RP:
            o += snprintf(buf, sizeof(buf), "{\"e\":\"rp\",\"m\":\"");
            o = ws_json_esc_append(buf, sizeof(buf), o, e->s1);
            o += snprintf(buf + o, sizeof(buf) - o, "\"}");
            break;
        case WS_RELOAD:
        default:
            o = snprintf(buf, sizeof(buf), "{\"e\":\"reload\"}");
            break;
    }
    ws_broadcast(buf, o);
}

/* Core-0 task: drains the event queue, sends immediately for unlock/challenge/
 * reload/rp, and COALESCES progress to ~4 Hz (only the newest per achievement)
 * so a game that bumps a measured value every frame can't flood lwIP / starve
 * httpTask. */
static void ws_task(void *arg)
{
    (void)arg;
    ws_evt_t pend_prog;
    int      have_prog = 0;
    uint32_t last_prog_ms = 0;
    for (;;) {
        ws_evt_t e;
        if (xQueueReceive(s_ws_q, &e, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (e.type == WS_PROGRESS) { pend_prog = e; have_prog = 1; }
            else                       { ws_format_and_send(&e); }
        }
        if (have_prog && (now_ms() - last_prog_ms) >= 250) {
            ws_format_and_send(&pend_prog);
            have_prog = 0;
            last_prog_ms = now_ms();
        }
    }
}

static void ws_enqueue(const ws_evt_t *e)
{
    if (!s_ws_q) return;
    if (ra_web_ws_count() == 0) return;       /* nobody listening → drop */
    xQueueSend(s_ws_q, e, 0);                  /* non-blocking: never stalls Core 1 */
}

void ra_web_push_unlock(unsigned id, const char *title, const char *badge)
{
    ws_evt_t e; memset(&e, 0, sizeof(e));
    e.type = WS_UNLOCK; e.id = id;
    if (title) strncpy(e.s1, title, sizeof(e.s1) - 1);
    if (badge) strncpy(e.s2, badge, sizeof(e.s2) - 1);
    ws_enqueue(&e);
}

void ra_web_push_challenge(int show, unsigned id)
{
    ws_evt_t e; memset(&e, 0, sizeof(e));
    e.type = WS_CHALLENGE; e.show = show ? 1 : 0; e.id = id;
    ws_enqueue(&e);
}

void ra_web_push_progress(unsigned id, int pct, const char *measured)
{
    ws_evt_t e; memset(&e, 0, sizeof(e));
    e.type = WS_PROGRESS; e.id = id; e.pct = (int16_t)pct;
    if (measured) strncpy(e.s2, measured, sizeof(e.s2) - 1);
    ws_enqueue(&e);
}

void ra_web_push_rp(const char *msg)
{
    ws_evt_t e; memset(&e, 0, sizeof(e));
    e.type = WS_RP;
    if (msg) strncpy(e.s1, msg, sizeof(e.s1) - 1);
    ws_enqueue(&e);
}

void ra_web_push_reload(void)
{
    ws_evt_t e; memset(&e, 0, sizeof(e));
    e.type = WS_RELOAD;
    ws_enqueue(&e);
}

static esp_err_t ws_get(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* Handshake completed by httpd. Register the client fd for pushes. */
        int fd = httpd_req_to_sockfd(req);
        ws_fd_add(fd);
        s_last_fetch_ms = now_ms();
        return ESP_OK;
    }
    /* Data/control frame from the client (e.g. a keepalive "ping").
     * A recv ERROR here means the peer is gone (phone slept / tab closed /
     * WiFi reset -> ECONNRESET then ENOTCONN): the error MUST propagate so
     * httpd closes the session. Swallowing it (the pre-2026-07-02 bug) left
     * the dead socket in the select set -> the handler spun at ~126 it/s
     * ("WS frame is not properly masked" + errno 128 storm, wii4.log) and
     * leaked one of the 4 fds per disconnect. A clean browser close sends
     * HTTPD_WS_TYPE_CLOSE -> same teardown path. */
    httpd_ws_frame_t f;
    memset(&f, 0, sizeof(f));
    f.type = HTTPD_WS_TYPE_TEXT;
    uint8_t tmp[16];
    f.payload = tmp;
    esp_err_t err = httpd_ws_recv_frame(req, &f, sizeof(tmp) - 1);
    if (err != ESP_OK || f.type == HTTPD_WS_TYPE_CLOSE) {
        ws_fd_remove(httpd_req_to_sockfd(req));
        return (err != ESP_OK) ? err : ESP_FAIL;   /* != ESP_OK => httpd closes the session */
    }
    s_last_fetch_ms = now_ms();
    return ESP_OK;
}

/* Minimal JSON string escaping (quotes, backslashes, control chars). Titles come
 * from the RA server and can contain quotes/backslashes. */
static void json_escape(const char *in, char *out, size_t out_sz)
{
    size_t o = 0;
    for (size_t i = 0; in && in[i] && o + 2 < out_sz; i++) {
        unsigned char c = (unsigned char)in[i];
        if (c == '"' || c == '\\') {
            out[o++] = '\\';
            out[o++] = c;
        } else if (c < 0x20) {
            if (o + 6 >= out_sz) break;
            o += snprintf(out + o, out_sz - o, "\\u%04x", c);
        } else {
            out[o++] = c;
        }
    }
    out[o] = '\0';
}

static esp_err_t root_get(httpd_req_t *req)
{
    s_last_fetch_ms = now_ms();   /* page loaded → a phone is watching */
    const size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, len);
}

static esp_err_t state_get(httpd_req_t *req)
{
    s_last_fetch_ms = now_ms();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    /* Preferred path: serve the full pre-rendered JSON (built on the loop task).
     * Copy it out of the shared buffer under the mutex, then send the copy so the
     * network write never holds the lock against the loop task's next publish. */
    if (s_json_mux) {
        xSemaphoreTake(s_json_mux, portMAX_DELAY);
        size_t len = s_json_len;
        char *copy = (len > 0) ? (char *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM) : NULL;
        if (copy) memcpy(copy, s_json, len);
        xSemaphoreGive(s_json_mux);
        if (copy) {
            esp_err_t r = httpd_resp_send(req, copy, len);
            free(copy);
            return r;
        }
    }

    /* Fallback (no full JSON yet): the tiny header snapshot so the page still
     * shows the game + progress ring immediately on first load. */
    ra_web_pub_t p;
    portENTER_CRITICAL(&s_pub_mux);
    p = s_pub;
    portEXIT_CRITICAL(&s_pub_mux);

    char title_esc[256];
    json_escape(p.game_title, title_esc, sizeof(title_esc));

    char buf[512];
    int n = snprintf(buf, sizeof(buf),
        "{\"state\":%u,\"game_id\":%lu,\"title\":\"%s\","
        "\"total\":%u,\"unlocked\":%u,"
        "\"points_total\":%lu,\"points_unlocked\":%lu,\"achievements\":[]}",
        (unsigned)p.state, (unsigned long)p.game_id, title_esc,
        (unsigned)p.total, (unsigned)p.unlocked,
        (unsigned long)p.points_total, (unsigned long)p.points_unlocked);

    return httpd_resp_send(req, buf, n);
}

void ra_web_init(void)
{
    memset(&s_pub, 0, sizeof(s_pub));
    if (!s_json_mux) s_json_mux = xSemaphoreCreateMutex();

    /* mDNS: browse to http://wii-ra.local/ */
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %d", (int)err);
    } else {
        mdns_hostname_set("wii-ra");
        mdns_instance_name_set("Wii RA Adapter");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id         = 0;    /* network core — NEVER Core 1 (do_frame)   */
    config.task_priority   = 1;    /* below lwIP/httpTask; never preempts RT    */
    config.stack_size      = 4096; /* trimmed: /api/state copies a PSRAM buffer  */
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.server_port     = 80;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return;
    }

    httpd_uri_t root;
    memset(&root, 0, sizeof(root));
    root.uri = "/";
    root.method = HTTP_GET;
    root.handler = root_get;

    httpd_uri_t st;
    memset(&st, 0, sizeof(st));
    st.uri = "/api/state";
    st.method = HTTP_GET;
    st.handler = state_get;

    httpd_uri_t ws;
    memset(&ws, 0, sizeof(ws));
    ws.uri = "/ws";
    ws.method = HTTP_GET;
    ws.handler = ws_get;
    ws.is_websocket = true;

    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &st);
    httpd_register_uri_handler(s_server, &ws);

    /* Live-push plumbing: event queue (Core 1 → Core 0) + the sender task on
     * Core 0, beside the WiFi/lwIP + httpTask stack (never Core 1). The task
     * stack lives in PSRAM (WS_Task never runs in ISR context and does no DMA),
     * keeping the scarce internal SRAM for the realtime path + TLS. Only the TCB
     * stays internal (touched by the scheduler). */
    if (!s_ws_q) s_ws_q = xQueueCreate(16, sizeof(ws_evt_t));
    static StaticTask_t s_ws_tcb;
    static StackType_t *s_ws_stack = NULL;
    const uint32_t ws_stack_words = 4096;
    if (!s_ws_stack)
        s_ws_stack = (StackType_t *)heap_caps_malloc(ws_stack_words * sizeof(StackType_t),
                                                     MALLOC_CAP_SPIRAM);
    if (s_ws_stack)
        xTaskCreateStaticPinnedToCore(ws_task, "WS_Task", ws_stack_words, nullptr, 1,
                                      s_ws_stack, &s_ws_tcb, 0);
    else
        xTaskCreatePinnedToCore(ws_task, "WS_Task", 4096, nullptr, 1, nullptr, 0);

    ESP_LOGI(TAG, "web dashboard up: http://wii-ra.local/");
}

#endif /* RA_WEB_DASHBOARD */
