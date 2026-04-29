#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"   /* TickType_t */

/* Core affinity policy for the LoRa subsystem.
 * PRO_CPU (0) is what WiFi/BT defaults to; keep LoRa off it. */
#define APP_PINNED_CORE     0
#define SX1262_PINNED_CORE  1

typedef enum {
    SX1262_OK = 0,
    SX1262_ERR_TIMEOUT,
    SX1262_ERR_BUSY,
    SX1262_ERR_PARAM,
    SX1262_ERR_HW,
    SX1262_ERR_QUEUE_FULL,
} sx1262_status_t;

/* Called from the LoRa task on each successful RX (CRC-clean) packet. */
typedef void (*sx1262_rx_cb_t)(const uint8_t *data, size_t len,
                               int8_t rssi_dbm, int8_t snr_db);

typedef struct {
    uint32_t        rf_freq_hz;        /* e.g. 915000000 (US), 868000000 (EU) */
    int8_t          tx_power_dbm;      /* -9 .. +22 (HP PA) */
    uint8_t         spreading_factor;  /* SF5..SF12;  default 7 */
    uint32_t        bandwidth_hz;      /* 7800 .. 500000 */
    uint8_t         coding_rate;       /* 1..4  → 4/5..4/8 */
    uint16_t        preamble_symbols;  /* default 8 */
    uint16_t        sync_word;         /* 0x3444 public, 0x1424 private */
    bool            crc_on;
    bool            iq_inverted;
    sx1262_rx_cb_t  rx_callback;
} sx1262_config_t;

sx1262_config_t sx1262_default_config(void);

/* Reset chip, run init sequence (TCXO, calibration, modem params, IRQ map),
 * install DIO1 ISR, allocate the TX queue. */
sx1262_status_t sx1262_init(const sx1262_config_t *cfg);

/* Enqueue a TX request. Returns once queued; the actual TX happens later
 * inside the LoRa task. `timeout` bounds the queue-wait (NOT the air time). */
sx1262_status_t sx1262_send(const uint8_t *data, size_t len, TickType_t timeout);

/* The LoRa task body. Blocks forever, servicing the TX queue and dispatching
 * RX_DONE events to the configured callback. Call from the LoRa-pinned task. */
void sx1262_run(void);
