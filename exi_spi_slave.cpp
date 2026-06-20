/**
 * exi_spi_slave.cpp
 * 
 * ESP32-S3 SPI Slave implementation for GameCube EXI bus.
 * Uses ESP-IDF spi_slave driver for hardware SPI.
 * 
 * The EXI protocol works as follows:
 *   1. GC asserts CS (low)
 *   2. GC clocks out command bytes (we receive on MOSI)
 *   3. GC may clock in response bytes (we send on MISO) 
 *   4. GC deasserts CS (high)
 * 
 * We use ISR callbacks + polling for reliable operation.
 * IMPORTANT: spi_slave_transaction_t must be static/persistent
 * because the ESP-IDF driver stores a pointer, not a copy.
 */

#include "exi_spi_slave.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"      /* GPIO_OUT_W1TC_REG — ISR-safe INT assert */
#include <string.h>
#include <Arduino.h>

// DMA-aligned buffers for SPI transactions
WORD_ALIGNED_ATTR static uint8_t rx_buf[EXI_MAX_TRANSACTION_SIZE];
WORD_ALIGNED_ATTR static uint8_t tx_buf[EXI_MAX_TRANSACTION_SIZE];

// Pending response data
WORD_ALIGNED_ATTR static uint8_t response_buf[EXI_MAX_TRANSACTION_SIZE];
static size_t response_len = 0;

// Last received data
static uint8_t received_buf[EXI_MAX_TRANSACTION_SIZE];
static size_t received_len = 0;
static volatile bool data_ready = false;

static bool spi_initialized = false;

// MUST be static — ESP-IDF SPI driver stores a pointer, not a copy!
static spi_slave_transaction_t current_trans;

// Debug counters
static volatile uint32_t total_transactions = 0;
static volatile uint32_t total_bytes_rx = 0;

// Callback diagnostics — observable from main loop heartbeat in
// wii-ra-adapter.ino. cb_invocations counts how many times spi_pre_setup_cb
// has fired since boot. last_response_len + last_response_b0 + last_tx_b6
// snapshot the state visible to the cb at its most recent firing. Lets us
// answer: "does the cb fire per CS-low? does it see the dynamic response or
// the default_ack reset?". Volatile because read from a different core.
volatile uint32_t exi_cb_invocations       = 0;
volatile uint16_t exi_cb_last_response_len = 0;
volatile uint8_t  exi_cb_last_response_b0  = 0;
volatile uint8_t  exi_cb_last_tx_b6        = 0;

/* Counters/state for exi_spi_prepare_response itself — to confirm it's
 * actually being entered with the chunk len, separately from what the cb sees. */
volatile uint32_t exi_prepare_call_count = 0;
volatile uint16_t exi_prepare_last_len   = 0;
volatile uint8_t  exi_prepare_post_tx_b6 = 0;  /* tx_buf[6] right after we wrote it */

/* Phase B INT-handshake state. Set by exi_spi_prepare_response when a real
 * (non-default) response has been written into tx_buf. Consumed by
 * exi_spi_arm — after queueing the new transaction, if this flag is set,
 * the firmware asserts the INT pin (slot pin 2) so the Wii can observe it
 * via the CSR EXI_IRQ latch and open the response read CS-low.
 *
 * exi_spi_poll() clears the INT pin on every transaction completion. That
 * releases any prior assertion now that the Wii has either delivered a
 * request (in which case the prior INT — if any — was already consumed)
 * or read the response (in which case INT's job is done). Either way the
 * next assert (post-arm of the next response) becomes observable.
 *
 * Phase A backward compat: WiiFlow's PPC handshake doesn't read INT at all.
 * The assert/clear sequence is harmless (just GPIO wiggling) — Phase A's
 * 1-tx-delay flow still works because WiiFlow polls periodically. */
static volatile bool g_response_prepared = false;

// 14-byte default ACK: 4 leading 0xFF pad bytes (clocked out during GC write phase, ignored)
// followed by ra_esp_header_t(6) + RA_DEVICE_ID(4) so bytes[4..13] are valid when GC reads.
// NON-const so the status byte (offset 5) can be patched to reflect current state — see
// exi_spi_update_status(). Without this, POLL responses always returned the stale status
// hardcoded here, even though the firmware had transitioned to LOADING_GAME / GAME_LOADED.
static uint8_t default_ack[14] = {
    0xFF, 0xFF, 0xFF, 0xFF,             // padding — clocked out during 4-byte GC write, GC ignores
    0xAE, 0x04, 0x00, 0x00, 0x00, 0x04, // magic, status=LOGGED_IN(4) initially, evt=NONE, cnt=0, data_len=4
    0x52, 0x41, 0x00, 0x01              // RA_DEVICE_ID big-endian
};

// Pre-transaction callback: load response data into TX buffer
static void IRAM_ATTR spi_pre_setup_cb(spi_slave_transaction_t *trans)
{
    /* Capture diagnostics BEFORE the memcpy, so we see what the cb received
     * — not what it produced. Reading tx_buf[6] tells us what the previous
     * transaction (or default_ack reset) left in the byte the GC will actually
     * sample first during a chunk read. */
    exi_cb_invocations++;
    exi_cb_last_response_len = (uint16_t)response_len;
    exi_cb_last_response_b0  = response_buf[0];
    exi_cb_last_tx_b6        = tx_buf[6];

    if (response_len > 0) {
        memcpy(tx_buf, response_buf, response_len);
    }

    /* Phase B v2 (v0.23.0): assert INT HERE, not in exi_spi_arm. This
     * callback fires when the ESP-IDF driver has actually loaded the
     * transaction into the SPI peripheral (registers + DMA descriptors
     * armed), so "INT low" now means "the read CS-low may start right
     * now". This removes the race that forced the Wii-side 1ms SETTLE
     * sleep between wait_int and the read CS-low: previously the assert
     * happened right after spi_slave_queue_trans() returned, which is
     * before the driver task had armed the DMA — large responses
     * (>=520 B) clocked out idle 0xFF instead of tx_buf.
     *
     * ISR context: write the w1tc register directly (gpio_set_level is
     * not IRAM-safe). EXI_PIN_INT=14 < 32 → GPIO_OUT_W1TC_REG. */
    if (g_response_prepared) {
        g_response_prepared = false;
        REG_WRITE(GPIO_OUT_W1TC_REG, 1U << EXI_PIN_INT);
    }
}

// Post-transaction callback: capture received data (runs in ISR context)
static void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
    if (trans->trans_len > 0) {
        size_t bytes = trans->trans_len / 8;
        if (bytes > 0 && bytes <= EXI_MAX_TRANSACTION_SIZE) {
            memcpy(received_buf, rx_buf, bytes);
            received_len = bytes;
            data_ready = true;
            total_transactions++;
            total_bytes_rx += bytes;
        }
    }
}

bool exi_spi_init(exi_transaction_cb_t callback)
{
    (void)callback;

    // SPI bus configuration
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = EXI_PIN_MOSI;
    bus_cfg.miso_io_num = EXI_PIN_MISO;
    bus_cfg.sclk_io_num = EXI_PIN_CLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = EXI_MAX_TRANSACTION_SIZE;

    // SPI slave configuration
    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 0;  // SPI Mode 0 (CPOL=0, CPHA=0) - matches EXI
    slave_cfg.spics_io_num = EXI_PIN_CS;
    slave_cfg.queue_size = 1;
    slave_cfg.post_setup_cb = spi_pre_setup_cb;
    slave_cfg.post_trans_cb = spi_post_trans_cb;

    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("ERROR=spi_slave_initialize failed: %d\n", ret);
        return false;
    }

    // Configure INT pin
    gpio_config_t int_cfg = {};
    int_cfg.pin_bit_mask = (1ULL << EXI_PIN_INT);
    int_cfg.mode = GPIO_MODE_OUTPUT;
    int_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    int_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&int_cfg);
    gpio_set_level((gpio_num_t)EXI_PIN_INT, 1);

    spi_initialized = true;

    // Pre-load a valid IDENTIFY probe response in TX buffer.
    //
    // EXI IDENTIFY transaction is 14 bytes total (CS held throughout):
    //   Phase 1 — GC writes 4 bytes (52 01 00 00); SPI full-duplex clocks TX[0..3] to GC but GC discards them.
    //   Phase 2 — GC reads 10 bytes (sends 10×FF); SPI full-duplex clocks TX[4..13] to GC — GC keeps these.
    //
    // So the actual IDENTIFY response (ra_esp_header_t + RA_DEVICE_ID) must start at TX[4].
    // TX[0..3] are 0xFF (harmless padding that is never read by the GC).
    static const uint8_t default_response[14] = {
        0xFF, 0xFF, 0xFF, 0xFF,             // TX[0..3]: padding, clocked out during GC write phase
        0xAE,                               // TX[4]:  RA_MAGIC_ESP_TO_GC
        0x01,                               // TX[5]:  status: STATE_WAIT_GAME_ID
        0x00,                               // TX[6]:  event_type: RA_EVT_NONE
        0x00,                               // TX[7]:  event_count: 0
        0x00, 0x04,                         // TX[8..9]:  data_len: 4 (device ID follows)
        0x52, 0x41, 0x00, 0x01              // TX[10..13]: RA_DEVICE_ID = 0x52410001
    };
    memcpy(tx_buf, default_response, sizeof(default_response));
    memcpy(response_buf, default_response, sizeof(default_response));
    response_len = sizeof(default_response);

    // Queue first transaction (using persistent struct)
    memset(&current_trans, 0, sizeof(current_trans));
    current_trans.length = EXI_MAX_TRANSACTION_SIZE * 8;
    current_trans.tx_buffer = tx_buf;
    current_trans.rx_buffer = rx_buf;
    spi_slave_queue_trans(SPI2_HOST, &current_trans, portMAX_DELAY);

    Serial.printf("DEBUG=SPI Pins: MOSI=%d MISO=%d CLK=%d CS=%d INT=%d\n",
                   EXI_PIN_MOSI, EXI_PIN_MISO, EXI_PIN_CLK, EXI_PIN_CS, EXI_PIN_INT);
    int cs_state = gpio_get_level((gpio_num_t)EXI_PIN_CS);
    Serial.printf("DEBUG=CS pin state: %d (expect 1=HIGH when idle)\n", cs_state);

    return true;
}

bool exi_spi_poll(uint32_t timeout_ms)
{
    if (!spi_initialized) return false;

    // Wait for the queued transaction to complete
    spi_slave_transaction_t *completed = NULL;
    esp_err_t ret = spi_slave_get_trans_result(SPI2_HOST, &completed, pdMS_TO_TICKS(timeout_ms));

    if (ret == ESP_OK) {
        size_t bytes = completed->trans_len / 8;

        /* Phase B: release the INT pin. Either this transaction WAS the
         * Wii consuming a response (so the assertion has done its job) or
         * it was a fresh request (in which case no prior assert was active,
         * making this a no-op). In both cases, the NEXT assert — triggered
         * by the next exi_spi_arm with g_response_prepared=true — will be
         * a clean low-edge that latches EXI_IRQ in the Hollywood CSR. */
        gpio_set_level((gpio_num_t)EXI_PIN_INT, 1);

        /* Reset tx_buf / response_buf back to default_ack so the next
         * transaction has a sane fallback if main loop's handler doesn't
         * call prepare_response (idle, malformed request, etc.).
         * NOTE: we do NOT re-queue here. Queueing snapshots the tx_buf into
         * the SPI peripheral's send path; if we queue here before main loop
         * has had a chance to call prepare_response with the dynamic payload,
         * the peripheral would transmit stale default_ack on the next CS-low.
         * Caller MUST call exi_spi_arm() after handling the command. */
        memcpy(tx_buf, default_ack, sizeof(default_ack));
        memcpy(response_buf, default_ack, sizeof(default_ack));
        response_len = sizeof(default_ack);

        // Skip spurious 0-byte CS pulses (noise when GC is not yet fully powered).
        // We still return true so the caller arms for the next transaction.
        if (bytes == 0) {
            return true;
        }

        /* Throttled log — happens AFTER re-queue so it never blocks reception.
         * 5s throttle: per-tx logs at 30Hz would blast 30 lines/s × 80 chars
         * = ~24 KB/s of Serial output, more than 115200 baud can absorb
         * (~11.5 KB/s), so the writer blocks → Core 0's loop() starves →
         * lwIP timer misses ticks → HTTP/ping fails. */
        static uint32_t last_log_ms = 0;
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms - last_log_ms >= 5000) {
            last_log_ms = now_ms;
            // Identify command type for the log
            const char *cmd_name = "?";
            if (bytes >= 2 && rx_buf[0] == 0x52) {
                switch (rx_buf[1]) {
                    case 0x01: cmd_name = "IDENTIFY"; break;
                    case 0x02: cmd_name = "LOAD_GAME"; break;
                    case 0x03: cmd_name = "SNAPSHOT"; break;
                    case 0x04: cmd_name = "POLL"; break;
                    case 0x05: cmd_name = "GAME_RESET"; break;
                    case 0x07: cmd_name = "GET_CHUNK"; break;
                    case 0x08: cmd_name = "ADDR_RESP"; break;
                    default: cmd_name = "UNKNOWN"; break;
                }
            } else if (bytes > 0 && rx_buf[0] == 0xFF) {
                cmd_name = "ALL_FF(idle?)";
            }
            Serial.printf("DEBUG=EXI trans: %u bytes, cmd=%s, total=%lu\n",
                          bytes, cmd_name, (unsigned long)total_transactions);
        }

        return data_ready;
    }

    return false;
}

void exi_spi_arm(void)
{
    if (!spi_initialized) return;
    /* Queue the persistent transaction so the SPI peripheral is ready for
     * the next CS-low. Called from the main loop AFTER handle_exi_command
     * has had a chance to call exi_spi_prepare_response, so the queued
     * snapshot reflects the actual dynamic response (chunk, addr_query,
     * etc.) instead of the default_ack fallback that exi_spi_poll wrote. */
    spi_slave_queue_trans(SPI2_HOST, &current_trans, portMAX_DELAY);

    /* Phase B v2 (v0.23.0): the INT assert moved into spi_pre_setup_cb
     * (the driver's post_setup callback), which fires only when the
     * peripheral is genuinely armed. Asserting here — right after
     * queue_trans returns — was too early: the driver task arms the DMA
     * asynchronously, and the Wii (with the SETTLE sleep removed) would
     * open the read CS-low against an unarmed slave. g_response_prepared
     * set by prepare_response is consumed inside the callback. */
}

void exi_spi_prepare_response(const uint8_t *data, size_t len)
{
    if (len > EXI_MAX_TRANSACTION_SIZE) len = EXI_MAX_TRANSACTION_SIZE;
    memcpy(response_buf, data, len);
    /* Also update tx_buf directly. The spi_pre_setup_cb was supposed to
     * lazy-copy response_buf → tx_buf at the start of every transaction,
     * but ESP-IDF's post_setup_cb fires when spi_slave_queue_trans() is
     * called — i.e., back inside exi_spi_poll() right after the previous
     * transaction completed and BEFORE main loop has had a chance to
     * call prepare_response with the new data. Net effect: tx_buf stayed
     * frozen on default_ack and any dynamic response (most importantly
     * GET_CHUNK's 868-byte payload) never reached the wire. Updating
     * tx_buf here closes the gap: since the main loop only calls
     * prepare_response between transactions (CS is HIGH at that moment),
     * the memcpy completes before the next CS-low and DMA reads the
     * fresh data. Future scale-up to 60 Hz may need a double-buffered
     * tx_buf to avoid the residual race; for current 1 Hz cadence the
     * race window is comfortably wide. */
    memcpy(tx_buf, data, len);
    response_len = len;

    /* Diag — verify we actually wrote the new len and tx_buf[6] reflects the
     * dynamic payload (e.g., 0xAE for an esp_header start byte). */
    exi_prepare_call_count++;
    exi_prepare_last_len   = (uint16_t)len;
    exi_prepare_post_tx_b6 = tx_buf[6];

    /* Phase B: mark that we have a real response queued. exi_spi_arm()
     * will consume this flag and assert the INT pin AFTER queueing the
     * new transaction, signaling the Wii that the read CS-low can begin. */
    g_response_prepared = true;
}

void exi_spi_update_status(uint8_t status)
{
    /* Patches only the default_ack template's status byte. The new value
     * propagates to tx_buf naturally on the next exi_spi_poll() reset cycle
     * (which does a full memcpy from default_ack into tx_buf).
     *
     * Earlier version ALSO wrote tx_buf[5] directly to avoid the 1-transaction
     * latency. That introduced DMA hazards: SPI slave DMA may read tx_buf at
     * any time when CS is asserted, and a single-byte write race could corrupt
     * the transmission. With the direct write removed, the worst case is now
     * "next transaction sees up-to-date status", which is fine for WiiFlow's
     * 300ms POLL cadence. */
    default_ack[5] = status;
}

void exi_spi_dump_default_ack(char *out, size_t out_size)
{
    /* Diagnostic helper: serializes current default_ack into a printable string
     * so the main loop can include it in heartbeat logs. Confirms whether the
     * status-byte update is actually taking effect. */
    size_t pos = 0;
    for (size_t i = 0; i < sizeof(default_ack) && pos + 4 < out_size; i++) {
        pos += snprintf(out + pos, out_size - pos, "%02X ", default_ack[i]);
    }
}

bool exi_spi_has_data(void)
{
    return data_ready;
}

size_t exi_spi_read(uint8_t *buf, size_t buf_size)
{
    if (!data_ready) return 0;
    size_t copy_len = (received_len < buf_size) ? received_len : buf_size;
    memcpy(buf, received_buf, copy_len);
    data_ready = false;
    return copy_len;
}

void exi_spi_assert_interrupt(void)
{
    gpio_set_level((gpio_num_t)EXI_PIN_INT, 0);
}

void exi_spi_clear_interrupt(void)
{
    gpio_set_level((gpio_num_t)EXI_PIN_INT, 1);
}

uint32_t exi_spi_get_transaction_count(void) { return total_transactions; }
uint32_t exi_spi_get_total_bytes_rx(void) { return total_bytes_rx; }
