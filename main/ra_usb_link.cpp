/**
 * ra_usb_link.cpp — console link over USB (CONFIG_RA_LINK_USB builds).
 *
 * Drop-in replacement for exi_spi_slave.cpp: implements the same API
 * (exi_spi_slave.h) over a TinyUSB vendor-class device, so main.cpp handles
 * the RA protocol exactly as on EXI. Plan and the measurements behind every
 * rule below: c:/dev/wii/PLAN-USB-SPIKE.md (§9-§11).
 *
 * Device: VID:PID 303A:4052, one vendor interface, bulk OUT 0x01 + bulk IN
 * 0x81, 64-byte packets (full speed) — identical to the spike firmware the
 * console side was validated against (c:/dev/wii/usb-spike-esp).
 *
 * Wire = the RA protocol itself, unchanged:
 *   - one bulk OUT message = one request (ra_gc_header_t + payload);
 *   - one bulk IN message = its response, byte for byte what the EXI build
 *     would put in tx_buf — including the 0xFF pads of the Phase-A commands
 *     (4 for IDENTIFY/POLL/..., 6 for the chunk fetches). The console strips
 *     them exactly like the EXI read phase skips them;
 *   - EVERY request gets exactly one reply. When a handler prepares none,
 *     the reply is default_ack — what an EXI read would have clocked out.
 * Framing: no message is ever a multiple of 64 bytes long (a 0x00 byte is
 * appended when it would be), so every message ends in a short packet and
 * that short packet is the end-of-message marker on both sides. The pad is
 * stripped here using the request header, so main.cpp sees the exact
 * request length EXI would have delivered. Replies are also zero-padded to
 * a multiple of 4 (usb_wire_len — IN messages arrive cut down to one).
 *
 * Threading (mirrors the EXI sole-owner servicer): the TinyUSB task (Core 1,
 * prio 20 — the EXI servicer's slot) assembles a request and hands it to the
 * worker (taskCore1, prio 19) through exi_spi_wait_request; the worker runs
 * handle_exi_command and exi_spi_submit_response starts the IN transfer.
 * The console never sends a request before it has the previous reply, so
 * one request buffer and one response buffer are enough.
 */

#include "exi_spi_slave.h"
#include "ra_log.h"
#include "gc_ra_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include <string.h>
#include <stdio.h>

#define RA_USB_VID              0x303A
#define RA_USB_PID              0x4052
#define RA_USB_MSOS20           1       /* WinUSB auto-bind on a PC (host tests); the Wii ignores the BOS */
#define EP_OUT                  0x01
#define EP_IN                   0x81
#define ITF_VENDOR              0
#define VENDOR_REQUEST_MSOS20   0x01

/* Request accumulator: the stack delivers an OUT message in pieces of up to
 * CFG_TUD_VENDOR_EPSIZE bytes. +64 = the framing pad byte, rounded. */
#define RX_ACC_MAX              (EXI_MAX_TRANSACTION_SIZE + 64)

/* ---- descriptors ----------------------------------------------------------- */

static const tusb_desc_device_t s_dev_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = RA_USB_MSOS20 ? 0x0210 : 0x0200,
    .bDeviceClass       = 0x00,   /* per interface */
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = RA_USB_VID,
    .idProduct          = RA_USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

/* 500 mA: Wi-Fi + TLS peaks, all drawn from the console's port. */
static const uint8_t s_cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 500),
    TUD_VENDOR_DESCRIPTOR(ITF_VENDOR, 4, EP_OUT, EP_IN, 64),
};

static const char *s_str_desc[] = {
    "\x09\x04",               /* 0: English (0x0409) */
    "wii-ra",                 /* 1: manufacturer */
    "Wii RA Adapter",         /* 2: product */
    "RA-USB-0001",            /* 3: serial */
    "RA link",                /* 4: interface */
};

#if RA_USB_MSOS20
/* Microsoft OS 2.0 descriptor set (same as the spike): compatible ID WINUSB +
 * DeviceInterfaceGUIDs, so a PC binds WinUSB with no driver install. */
#define MS_OS_20_DESC_LEN  0xA2

static const uint8_t s_desc_ms_os_20[] = {
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0, 'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    U16_TO_U8S_LE(0x0050),
    '{', 0, '8', 0, 'A', 0, '1', 0, 'C', 0, '7', 0, 'F', 0, '3', 0, 'E', 0, '-', 0,
    '5', 0, '2', 0, 'D', 0, '4', 0, '-', 0, '4', 0, 'B', 0, '9', 0, 'A', 0, '-', 0,
    '9', 0, 'E', 0, '6', 0, '1', 0, '-', 0, '3', 0, 'C', 0, '0', 0, 'D', 0, '2', 0,
    'F', 0, '7', 0, 'B', 0, '5', 0, 'A', 0, '1', 0, '1', 0, '}', 0, 0, 0, 0, 0,
};
static_assert(sizeof(s_desc_ms_os_20) == MS_OS_20_DESC_LEN, "MS OS 2.0 descriptor size");

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t s_desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MSOS20),
};
#endif /* RA_USB_MSOS20 */

/* ---- state ------------------------------------------------------------------ */

/* Request path: TinyUSB task -> worker. */
static uint8_t s_acc[RX_ACC_MAX];
static size_t  s_acc_len = 0;
static bool    s_acc_ovf = false;
static uint8_t g_request_buf[EXI_MAX_TRANSACTION_SIZE];
static volatile size_t g_request_len = 0;
static SemaphoreHandle_t g_req_sem = NULL;

/* Response path: worker -> IN endpoint. +8 = wire padding (usb_wire_len). */
static uint8_t s_resp[EXI_MAX_TRANSACTION_SIZE + 8];
static size_t  s_resp_len = 0;
static bool    s_resp_prepared = false;

/* What an EXI read clocks out when no handler prepared a response: 4 bytes of
 * 0xFF (the Phase-A write phase), then the esp header with the live status
 * byte and the device id. Same template as exi_spi_slave.cpp. */
static uint8_t default_ack[14] = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xAE, 0x01, 0x00, 0x00, 0x00, 0x04,
    0x52, 0x41, 0x00, 0x01
};

static bool s_installed = false;

/* Counters (written by the TinyUSB task / worker, read racily by the log). */
static volatile uint32_t st_msgs = 0, st_bytes_rx = 0, st_bytes_tx = 0;
static volatile uint32_t st_ovf = 0, st_tx_busy = 0, st_overrun = 0, st_default = 0;
static volatile uint32_t st_attach = 0, st_detach = 0;

/* Worker watchdog hooks read by main.cpp's exi_watchdog_task. Stage: 0 idle,
 * 1 request handed to the worker, 2 reply started. */
volatile uint32_t g_servicer_ticks = 0;
volatile uint8_t  g_servicer_stage = 0;
volatile uint8_t  g_servicer_prev_stage = 0;

/* EXI-API compatibility (exi_spi_has_data / exi_spi_read). */
static volatile bool data_ready = false;

static inline uint16_t rd_be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

/* ---- TinyUSB callbacks (TinyUSB task) -------------------------------------- */

extern "C" {

#if RA_USB_MSOS20
uint8_t const *tud_descriptor_bos_cb(void)
{
    return s_desc_bos;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP)
        return true;
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        request->bRequest == VENDOR_REQUEST_MSOS20 && request->wIndex == 7)
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_desc_ms_os_20,
                                sizeof(s_desc_ms_os_20));
    return false;   /* stall anything else */
}
#endif

/* A request arrives in pieces; the protocol says where it ends: messages are
 * never a multiple of 64 bytes, so a non-empty piece that IS a multiple of 64
 * is partial and only the terminating short packet completes the message
 * (spike finding: pieces are 8192 B regardless of EPSIZE). */
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf;

    if (s_acc_len + bufsize <= sizeof(s_acc))
        memcpy(s_acc + s_acc_len, buffer, bufsize);
    else
        s_acc_ovf = true;
    s_acc_len += bufsize;

    if (bufsize != 0 && (bufsize & 63u) == 0)
        return;

    st_msgs++;
    st_bytes_rx += (uint32_t)s_acc_len;

    if (s_acc_ovf || s_acc_len > EXI_MAX_TRANSACTION_SIZE + 1) {
        /* Never happens with a conforming console (requests <= 8192 B). Still
         * owes the console a reply, or it would wait on its IN forever. */
        static uint8_t ovf_reply[16];   /* default_ack + 2 zero bytes (usb_wire_len) */
        st_ovf++;
        memcpy(ovf_reply, default_ack, sizeof(default_ack));
        if (tud_vendor_write(ovf_reply, sizeof(ovf_reply)) != sizeof(ovf_reply))
            st_tx_busy++;
    } else {
        size_t n = s_acc_len;
        /* Strip the framing pad: the header says how long the request is. */
        if (n >= sizeof(ra_gc_header_t)) {
            size_t want = sizeof(ra_gc_header_t) + rd_be16(s_acc + 2);
            if (n == want + 1 && (want & 63u) == 0)
                n = want;
        }
        if (g_servicer_stage == 1)
            st_overrun++;   /* previous request not taken yet: console broke the alternation */
        memcpy(g_request_buf, s_acc, n);
        g_request_len = n;
        data_ready = true;
        g_servicer_prev_stage = g_servicer_stage;
        g_servicer_stage = 1;
        g_servicer_ticks++;
        xSemaphoreGive(g_req_sem);
    }
    s_acc_len = 0;
    s_acc_ovf = false;
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    (void)itf;
    st_bytes_tx += sent_bytes;
}

} /* extern "C" */

static void usb_event_cb(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        st_attach++;
        LOG_INFO("DEBUG=USB link: configured by the console\r\n");
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        st_detach++;
        s_acc_len = 0;
        s_acc_ovf = false;
        LOG_INFO("DEBUG=USB link: detached\r\n");
    }
}

/* ---- exi_spi_slave.h API ------------------------------------------------------ */

bool exi_spi_init(exi_transaction_cb_t callback)
{
    (void)callback;

    g_req_sem = xSemaphoreCreateBinary();
    if (!g_req_sem) {
        LOG_ERR("ERROR=USB link: semaphore alloc failed\r\n");
        return false;
    }

    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG(usb_event_cb);
    cfg.descriptor.device = &s_dev_desc;
    cfg.descriptor.full_speed_config = s_cfg_desc;
    cfg.descriptor.string = s_str_desc;
    cfg.descriptor.string_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]);
    /* Core 1, prio 20 = the EXI servicer's slot: the request hand-off must not
     * wait behind loop()'s do_frame, and the worker (19) runs right after. */
    cfg.task = TINYUSB_TASK_CUSTOM(4096, 20, 1);

    esp_err_t err = tinyusb_driver_install(&cfg);
    if (err != ESP_OK) {
        LOG_ERR("ERROR=USB link: tinyusb_driver_install failed: %d\r\n", (int)err);
        return false;
    }
    s_installed = true;
    LOG_INFO("DEBUG=USB link up: VID:PID %04X:%04X EPSIZE=%d (log on UART0 only)\r\n",
             RA_USB_VID, RA_USB_PID, (int)CFG_TUD_VENDOR_EPSIZE);
    return true;
}

bool exi_spi_wait_request(uint8_t *buf, size_t *len, uint32_t timeout_ms)
{
    if (!g_req_sem) return false;
    if (xSemaphoreTake(g_req_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return false;
    size_t n = g_request_len;
    if (n > EXI_MAX_TRANSACTION_SIZE) n = EXI_MAX_TRANSACTION_SIZE;
    memcpy(buf, g_request_buf, n);
    *len = n;
    data_ready = false;
    /* Fresh reply slot for this request: default_ack unless the handler
     * prepares one (EXI: the read before every request resets tx_buf). */
    s_resp_prepared = false;
    return true;
}

void exi_spi_prepare_response(const uint8_t *data, size_t len)
{
    if (len > EXI_MAX_TRANSACTION_SIZE) len = EXI_MAX_TRANSACTION_SIZE;
    memcpy(s_resp, data, len);
    s_resp_len = len;
    s_resp_prepared = true;
}

/* Reply length on the wire: rounded UP to a multiple of 4, plus 4 more when
 * that lands on a multiple of 64 (framing: end on a short packet). The 4-byte
 * rounding is a hardware finding, Wii round 1 (2026-10-13): a 14-byte
 * IDENTIFY reply reached WiiFlow with its last 2 bytes zeroed — IN messages
 * arrive cut DOWN to a multiple of 4 (the spike only ever sent 8-byte replies;
 * console->ESP messages of any length are fine). The zero tail is ignored by
 * every parser (they read the header lengths) and fits every console reply
 * buffer (largest reply 4120 B, a multiple of 4 already). */
static size_t usb_wire_len(size_t n)
{
    n = (n + 3u) & ~(size_t)3u;
    if ((n & 63u) == 0)
        n += 4u;
    return n;
}

void exi_spi_submit_response(void)
{
    const uint8_t *p;
    size_t n, w;

    if (s_resp_prepared) {
        n = s_resp_len;
        p = s_resp;
    } else {
        st_default++;
        memcpy(s_resp, default_ack, sizeof(default_ack));
        n = sizeof(default_ack);
        p = s_resp;
    }
    w = usb_wire_len(n);
    memset(s_resp + n, 0, w - n);
    n = w;
    /* No TX FIFO (CONFIG_TINYUSB_VENDOR_TX_BUFSIZE=0): this copies into the
     * endpoint buffer and arms the IN transfer at once. It fails only while a
     * previous reply is still unread — a console that broke the alternation. */
    if (tud_vendor_write(p, n) != n)
        st_tx_busy++;
    g_servicer_prev_stage = g_servicer_stage;
    g_servicer_stage = 2;
}

void exi_spi_update_status(uint8_t status)
{
    default_ack[5] = status;
}

void exi_spi_dump_default_ack(char *out, size_t out_size)
{
    size_t pos = 0;
    for (size_t i = 0; i < sizeof(default_ack) && pos + 4 < out_size; i++)
        pos += snprintf(out + pos, out_size - pos, "%02X ", default_ack[i]);
}

/* Heartbeat "rxdiag" line: on USB it reports the link counters instead of
 * the EXI MOSI-line diagnosis. */
void exi_spi_rxdiag(char *out, size_t out_size)
{
    if (!out || out_size == 0) return;
    snprintf(out, out_size,
             "usb mounted=%d msgs=%lu rx=%luB tx=%luB default=%lu ovf=%lu overrun=%lu "
             "tx_busy=%lu attach=%lu detach=%lu",
             s_installed ? (int)tud_mounted() : -1,
             (unsigned long)st_msgs, (unsigned long)st_bytes_rx, (unsigned long)st_bytes_tx,
             (unsigned long)st_default, (unsigned long)st_ovf, (unsigned long)st_overrun,
             (unsigned long)st_tx_busy, (unsigned long)st_attach, (unsigned long)st_detach);
}

bool exi_spi_has_data(void) { return data_ready; }

size_t exi_spi_read(uint8_t *buf, size_t buf_size)
{
    if (!data_ready) return 0;
    size_t n = g_request_len < buf_size ? g_request_len : buf_size;
    memcpy(buf, g_request_buf, n);
    data_ready = false;
    return n;
}

/* EXI-only concepts: no INT line, no slave to arm, no polling path. */
void exi_spi_assert_interrupt(void) {}
void exi_spi_clear_interrupt(void) {}
bool exi_spi_poll(uint32_t timeout_ms) { (void)timeout_ms; return false; }
void exi_spi_arm(void) {}

uint32_t exi_spi_get_transaction_count(void) { return st_msgs; }
uint32_t exi_spi_get_total_bytes_rx(void) { return st_bytes_rx; }
bool exi_spi_servicer_active(void) { return s_installed; }
