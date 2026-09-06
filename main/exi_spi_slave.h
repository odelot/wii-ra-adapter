/**
 * exi_spi_slave.h
 * 
 * ESP32-S3 SPI Slave driver compatible with GameCube EXI bus protocol.
 * Replaces the UART serial communication from fpga-ra-adapter.
 * 
 * The GameCube EXI bus is essentially SPI with:
 *   - Big-endian byte order (ESP32 is little-endian, must swap)
 *   - CS active low
 *   - Clock speeds from 1MHz to 32MHz
 *   - Half-duplex transactions (write then read)

            nop x x - nt
            di  x x - cs
            gnd x x - do
            clk x x - 3,3v
 */

#ifndef EXI_SPI_SLAVE_H
#define EXI_SPI_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* EXI_PIN_* (per-board pin map) come from here. */
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum transaction size in bytes */
#define EXI_MAX_TRANSACTION_SIZE  8192  /* must hold snapshot: 12 hdr + 4096 values */

/** Status of an EXI transaction */
typedef enum {
    EXI_STATUS_OK = 0,
    EXI_STATUS_TIMEOUT,
    EXI_STATUS_ERROR,
    EXI_STATUS_NO_DATA,
} exi_status_t;

/** Callback for when a complete transaction is received */
typedef void (*exi_transaction_cb_t)(const uint8_t *rx_data, size_t rx_len,
                                      uint8_t *tx_data, size_t *tx_len);

/**
 * Initialize the SPI slave for EXI communication.
 * Must be called once during setup().
 * 
 * @param callback  Function called when GC completes a transaction.
 *                  The callback receives the data from GC and must fill
 *                  tx_data/tx_len with the response.
 * @return true on success
 */
bool exi_spi_init(exi_transaction_cb_t callback);

/**
 * Prepare a response buffer that will be sent on the next GC read.
 * Call this to queue data for the GameCube to read.
 * 
 * @param data  Response data (will be copied internally)
 * @param len   Length in bytes
 */
void exi_spi_prepare_response(const uint8_t *data, size_t len);

/**
 * Patch the status byte in the default response template. Call frequently
 * from the main loop to keep WiiFlow's POLL responses reflecting current state.
 * See exi_spi_slave.cpp for the rationale (the default_ack template is what
 * WiiFlow reads on POLL transactions; without this, the status byte stays at
 * its compile-time value forever).
 */
void exi_spi_update_status(uint8_t status);

/**
 * Diagnostic: dump the current default_ack template contents into a string.
 * Used in heartbeat to confirm the firmware build has the dynamic-status
 * code (and that it's actually being called by the main loop).
 */
void exi_spi_dump_default_ack(char *out, size_t out_size);

/**
 * Check if a transaction was received since last check.
 * Non-blocking.
 * 
 * @return true if new data is available
 */
bool exi_spi_has_data(void);

/**
 * Get the last received transaction data.
 * 
 * @param buf      Output buffer
 * @param buf_size Size of output buffer
 * @return Number of bytes copied, 0 if no data
 */
size_t exi_spi_read(uint8_t *buf, size_t buf_size);

/**
 * Assert the INT line to signal the GameCube that we have pending data.
 * The GC can optionally poll this instead of periodic polling.
 */
void exi_spi_assert_interrupt(void);

/** Clear the INT line */
void exi_spi_clear_interrupt(void);

/**
 * Poll for a completed SPI transaction. Does NOT re-arm the slave —
 * the caller must explicitly call exi_spi_arm() after processing the
 * response (i.e. after calling exi_spi_prepare_response()).
 *
 * Why the split: ESP-IDF's spi_slave_queue_trans() appears to snapshot
 * the tx_buf contents into the SPI peripheral's send path at queue time.
 * If we queue before main loop has had a chance to call prepare_response
 * with the dynamic payload, the peripheral transmits stale default_ack
 * even though tx_buf was later updated. Arming AFTER prepare_response
 * guarantees the queued snapshot reflects the actual response.
 *
 * @param timeout_ms  Max time to wait for a transaction (0 = non-blocking)
 * @return true if a transaction completed (data or spurious 0-byte CS)
 */
bool exi_spi_poll(uint32_t timeout_ms);

/**
 * Re-arm the SPI slave: queue the persistent transaction so the peripheral
 * is ready to receive the next CS-low. MUST be called after every
 * exi_spi_poll() that returned true (data or spurious), otherwise the
 * hardware stays idle and subsequent GC transactions are lost.
 */
void exi_spi_arm(void);

/** Get total number of completed SPI transactions (for debug) */
uint32_t exi_spi_get_transaction_count(void);

/** Get total bytes received (for debug) */
uint32_t exi_spi_get_total_bytes_rx(void);

/**
 * Diagnostic: describe what the Wii->ESP data line (MOSI) actually delivered on
 * the transactions that were NOT recognised as requests. Distinguishes a dead
 * wire (and == or, i.e. the line never changes) from a live but mis-sampled one
 * (and == 00 with or == FF). See the comment block in exi_spi_slave.cpp.
 */
void exi_spi_rxdiag(char *out, size_t out_size);

/** True if the robust re-arm servicer task is running (project_exi_robust_
 * handshake). When false, the caller must arm unconditionally as a fallback. */
bool exi_spi_servicer_active(void);

/** Sole-owner handoff (project_exi_robust_handshake): the worker (taskCore1)
 * calls exi_spi_wait_request to block until the servicer hands a REQUEST (returns
 * false on timeout), processes it (handle_exi_command -> exi_spi_prepare_response),
 * then calls exi_spi_submit_response to tell the servicer to arm the response. */
bool exi_spi_wait_request(uint8_t *buf, size_t *len, uint32_t timeout_ms);
void exi_spi_submit_response(void);

#ifdef __cplusplus
}
#endif

#endif /* EXI_SPI_SLAVE_H */
