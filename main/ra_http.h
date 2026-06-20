/**
 * ra_http.h
 *
 * Job/completion structs for the async HTTP worker (v0.24.0 core split).
 *
 * These typedefs live in a header (not the .ino) on purpose: the Arduino
 * preprocessor auto-generates function prototypes and hoists them to the
 * top of the sketch, BEFORE any typedef written in the .ino body — so a
 * function taking `const http_job_t *` fails with "does not name a type"
 * unless the type comes from an #include.
 *
 * Flow (see wii-ra-adapter.ino):
 *   server_call (Core 1)  → deep-copy request → http_req_q
 *   httpTask    (Core 0)  → server_call_blocking → http_finish → http_done_q
 *   loop        (Core 1)  → http_drain_done → rc_client callback
 */

#ifndef RA_HTTP_H
#define RA_HTTP_H

#include <stdint.h>
#include <stddef.h>
#include "rc_client.h"

typedef struct {
    char *url;
    char *post_data;            /* NULL → GET */
    rc_client_server_callback_t callback;
    void *callback_data;
    uint32_t gen;               /* rc_client generation — discard stale */
} http_job_t;

typedef struct {
    char *body;                 /* ps_malloc'd; freed by the drain */
    size_t body_len;
    int status;
    rc_client_server_callback_t callback;
    void *callback_data;
    uint32_t gen;
} http_done_t;

#endif /* RA_HTTP_H */
