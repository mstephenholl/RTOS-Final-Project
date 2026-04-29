/*
 * sx1262.c — bare-metal LoRa driver for the Semtech SX1262 on Heltec V3.
 *
 * Threading model: a single FreeRTOS task (`sx1262_run`) owns the chip.
 * Producers call sx1262_send() to enqueue TX requests; the task drains
 * the queue and runs an interlocked TX/RX state machine, blocking on
 * the DIO1 semaphore between events.
 *
 * No LoRaWAN, no ChirpSpread library, no LoRaMac-node — every command
 * is a hand-rolled SPI transaction against DS_SX1261-2_V2.1.
 */

#include "sx1262.h"
#include "sx1262_hal.h"
#include "sx1262_pins.h"
#include "sx1262_cmds.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "sx1262";

#define TX_QUEUE_DEPTH      4
#define MAX_LORA_PAYLOAD    255

typedef struct {
    uint8_t  data[MAX_LORA_PAYLOAD];
    uint8_t  len;
} tx_request_t;

static QueueHandle_t   s_tx_queue;
static sx1262_config_t s_cfg;

/* ---------------------------------------------------------------------- */
/*  Defaults                                                              */
/* ---------------------------------------------------------------------- */

sx1262_config_t sx1262_default_config(void)
{
    sx1262_config_t c = {
        .rf_freq_hz       = 915000000,    /* US ISM band; change for EU/AS */
        .tx_power_dbm     = 14,           /* +14 dBm via HP PA */
        .spreading_factor = 7,
        .bandwidth_hz     = 125000,
        .coding_rate      = 1,            /* 4/5 */
        .preamble_symbols = 8,
        .sync_word        = 0x3444,       /* "public" — also used for p2p */
        .crc_on           = true,
        .iq_inverted      = false,
        .rx_callback      = NULL,
    };
    return c;
}

/* ---------------------------------------------------------------------- */
/*  Static helpers — every function below is a hand-rolled SPI command    */
/* ---------------------------------------------------------------------- */

/* DS table 13-49: bandwidth-code mapping. */
static uint8_t bw_to_code(uint32_t hz)
{
    switch (hz) {
        case   7800: return 0x00;
        case  10400: return 0x08;
        case  15600: return 0x01;
        case  20800: return 0x09;
        case  31200: return 0x02;
        case  41700: return 0x0A;
        case  62500: return 0x03;
        case 125000: return 0x04;
        case 250000: return 0x05;
        case 500000: return 0x06;
        default:     return 0x04;          /* fall back to 125 kHz */
    }
}

static sx1262_status_t set_standby(uint8_t mode)
{
    return sx1262_hal_cmd(SX_CMD_SET_STANDBY, &mode, 1);
}

static sx1262_status_t set_packet_type_lora(void)
{
    uint8_t v = SX_PACKET_TYPE_LORA;
    return sx1262_hal_cmd(SX_CMD_SET_PACKET_TYPE, &v, 1);
}

static sx1262_status_t set_rf_frequency(uint32_t hz)
{
    /* freq_reg = freq * 2^25 / 32_000_000.  Use 64-bit math: at 915 MHz
     * the intermediate (hz << 25) overflows uint32_t. */
    uint64_t freq_reg = ((uint64_t)hz << 25) / 32000000ULL;
    uint8_t  p[4] = {
        (uint8_t)(freq_reg >> 24),
        (uint8_t)(freq_reg >> 16),
        (uint8_t)(freq_reg >>  8),
        (uint8_t)(freq_reg      ),
    };
    return sx1262_hal_cmd(SX_CMD_SET_RF_FREQUENCY, p, 4);
}

static sx1262_status_t set_dio3_tcxo_ctrl(void)
{
    /* Heltec V3 has a TCXO whose enable line is wired to the SX1262's DIO3.
     * Voltage = 1.7 V (param 0x02), startup timeout ≈ 5 ms (0x000140 in
     * 15.625 µs units = 320 -> 5 ms). */
    uint8_t p[4] = { 0x02, 0x00, 0x01, 0x40 };
    return sx1262_hal_cmd(SX_CMD_SET_DIO3_AS_TCXO_CTRL, p, 4);
}

static sx1262_status_t set_dio2_as_rf_switch(void)
{
    /* DIO2 = 1 in TX, 0 in RX — drives the antenna switch. */
    uint8_t enable = 0x01;
    return sx1262_hal_cmd(SX_CMD_SET_DIO2_AS_RF_SWITCH, &enable, 1);
}

static sx1262_status_t calibrate_image(uint32_t hz)
{
    /* Band-specific image rejection cal — DS §13.1.6. */
    uint8_t cal[2];
    if      (hz >= 902000000 && hz <= 928000000) { cal[0] = 0xE1; cal[1] = 0xE9; }
    else if (hz >= 863000000 && hz <= 870000000) { cal[0] = 0xD7; cal[1] = 0xDB; }
    else if (hz >= 779000000 && hz <= 787000000) { cal[0] = 0xC1; cal[1] = 0xC5; }
    else if (hz >= 470000000 && hz <= 510000000) { cal[0] = 0x75; cal[1] = 0x81; }
    else if (hz >= 430000000 && hz <= 440000000) { cal[0] = 0x6B; cal[1] = 0x6F; }
    else { cal[0] = 0xE1; cal[1] = 0xE9; }                 /* default to US */
    return sx1262_hal_cmd(SX_CMD_CALIBRATE_IMAGE, cal, 2);
}

static sx1262_status_t set_pa_config(int8_t tx_dbm)
{
    /* DS table 13-21 — recommended (paDutyCycle, hpMax) by output power. */
    uint8_t dc, hp;
    if      (tx_dbm >= 22) { dc = 0x04; hp = 0x07; }
    else if (tx_dbm >= 20) { dc = 0x03; hp = 0x05; }
    else if (tx_dbm >= 17) { dc = 0x02; hp = 0x03; }
    else                   { dc = 0x02; hp = 0x02; }       /* +14 dBm and below */

    uint8_t p[4] = { dc, hp, 0x00 /* deviceSel = SX1262 */, 0x01 /* paLut */ };
    return sx1262_hal_cmd(SX_CMD_SET_PA_CONFIG, p, 4);
}

static sx1262_status_t set_tx_params(int8_t tx_dbm)
{
    /* RampTime = 0x04 = 200 µs.  Fine for narrow-BW LoRa. */
    uint8_t p[2] = { (uint8_t)tx_dbm, 0x04 };
    return sx1262_hal_cmd(SX_CMD_SET_TX_PARAMS, p, 2);
}

static sx1262_status_t set_modulation_params(uint8_t sf, uint32_t bw_hz, uint8_t cr)
{
    /* LowDataRateOptimize must be on when symbol time > 16 ms.
     * Symbol time = 2^SF / BW. */
    uint8_t ldro = ((sf >= 11 && bw_hz <= 125000) ||
                    (sf == 12 && bw_hz <= 250000)) ? 1 : 0;

    uint8_t p[4] = { sf, bw_to_code(bw_hz), cr, ldro };
    return sx1262_hal_cmd(SX_CMD_SET_MODULATION_PARAMS, p, 4);
}

static sx1262_status_t set_packet_params(uint16_t preamble, uint8_t payload_len,
                                         bool crc_on, bool iq_inv)
{
    uint8_t p[6] = {
        (uint8_t)(preamble >> 8),
        (uint8_t)(preamble & 0xFF),
        0x00,                                  /* explicit (variable) header */
        payload_len,
        (uint8_t)(crc_on ? 0x01 : 0x00),
        (uint8_t)(iq_inv ? 0x01 : 0x00),
    };
    return sx1262_hal_cmd(SX_CMD_SET_PACKET_PARAMS, p, 6);
}

static sx1262_status_t set_buffer_base(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t p[2] = { tx_base, rx_base };
    return sx1262_hal_cmd(SX_CMD_SET_BUFFER_BASE_ADDRESS, p, 2);
}

static sx1262_status_t set_dio_irq_params(uint16_t mask,
                                          uint16_t dio1, uint16_t dio2, uint16_t dio3)
{
    uint8_t p[8] = {
        (uint8_t)(mask >> 8), (uint8_t)mask,
        (uint8_t)(dio1 >> 8), (uint8_t)dio1,
        (uint8_t)(dio2 >> 8), (uint8_t)dio2,
        (uint8_t)(dio3 >> 8), (uint8_t)dio3,
    };
    return sx1262_hal_cmd(SX_CMD_SET_DIO_IRQ_PARAMS, p, 8);
}

static sx1262_status_t get_irq_status(uint16_t *out)
{
    uint8_t rx[2];
    sx1262_status_t s = sx1262_hal_cmd_read(SX_CMD_GET_IRQ_STATUS, NULL, 0, rx, 2);
    if (s == SX1262_OK) *out = ((uint16_t)rx[0] << 8) | rx[1];
    return s;
}

static sx1262_status_t clear_irq_status(uint16_t mask)
{
    uint8_t p[2] = { (uint8_t)(mask >> 8), (uint8_t)mask };
    return sx1262_hal_cmd(SX_CMD_CLEAR_IRQ_STATUS, p, 2);
}

static sx1262_status_t set_sync_word(uint16_t sw)
{
    uint8_t v[2] = { (uint8_t)((sw >> 8) & 0xFF), (uint8_t)(sw & 0xFF) };
    return sx1262_hal_write_reg(SX_REG_LSYNCRH, v, 2);
}

/* DS-recommended cadDetPeak per SF (Semtech AN1200.48). */
static uint8_t cad_det_peak_for_sf(uint8_t sf)
{
    if (sf <= 8)  return 22;
    if (sf == 9)  return 23;
    if (sf == 10) return 24;
    if (sf == 11) return 25;
    return 28;  /* SF12 */
}

static sx1262_status_t set_cad_params(uint8_t sf)
{
    /* SetCadParams (DS table 13-44):
     *   [cadSymbolNum] [cadDetPeak] [cadDetMin] [cadExitMode] [cadTimeout(24-bit)]
     * cadSymbolNum = 0x02 → 4-symbol listen window (good balance for SF7-12).
     * cadExitMode  = 0x00 → CAD_ONLY: return to STDBY_RC after CAD completes.
     * cadTimeout   = 0   → unused in CAD_ONLY. */
    uint8_t p[7] = {
        0x02,
        cad_det_peak_for_sf(sf),
        10,
        0x00,
        0x00, 0x00, 0x00,
    };
    return sx1262_hal_cmd(SX_CMD_SET_CAD_PARAMS, p, 7);
}

static sx1262_status_t enter_cad(void)
{
    return sx1262_hal_cmd(SX_CMD_SET_CAD, NULL, 0);
}

static sx1262_status_t enter_rx_continuous(void)
{
    /* 0xFFFFFF in the timeout field = continuous RX. */
    uint8_t p[3] = { 0xFF, 0xFF, 0xFF };
    return sx1262_hal_cmd(SX_CMD_SET_RX, p, 3);
}

static sx1262_status_t enter_tx(uint32_t timeout_ticks)
{
    uint8_t p[3] = {
        (uint8_t)(timeout_ticks >> 16),
        (uint8_t)(timeout_ticks >>  8),
        (uint8_t)(timeout_ticks      ),
    };
    return sx1262_hal_cmd(SX_CMD_SET_TX, p, 3);
}

/* ---------------------------------------------------------------------- */
/*  Public API                                                            */
/* ---------------------------------------------------------------------- */

sx1262_status_t sx1262_init(const sx1262_config_t *cfg)
{
    if (!cfg) return SX1262_ERR_PARAM;
    s_cfg = *cfg;

    sx1262_status_t s = sx1262_hal_init();
    if (s != SX1262_OK) return s;

    sx1262_hal_reset();
    s = sx1262_hal_wait_busy(100);                        if (s != SX1262_OK) return s;

    /* 1. Standby on internal RC; safe place to issue config commands. */
    s = set_standby(SX_STANDBY_RC);                       if (s != SX1262_OK) return s;

    /* 2. TCXO via DIO3.  Clear the resulting XOSC_START error flag. */
    s = set_dio3_tcxo_ctrl();                             if (s != SX1262_OK) return s;
    {
        uint8_t z[2] = { 0, 0 };
        sx1262_hal_cmd(SX_CMD_CLEAR_DEVICE_ERRORS, z, 2);
    }

    /* 3. Image-rejection calibration *for this band*. */
    s = calibrate_image(s_cfg.rf_freq_hz);                if (s != SX1262_OK) return s;

    /* 4. Antenna RF switch via DIO2. */
    s = set_dio2_as_rf_switch();                          if (s != SX1262_OK) return s;

    /* 5. Packet type → LoRa, then frequency, PA, TX params. */
    s = set_packet_type_lora();                           if (s != SX1262_OK) return s;
    s = set_rf_frequency(s_cfg.rf_freq_hz);               if (s != SX1262_OK) return s;
    s = set_pa_config(s_cfg.tx_power_dbm);                if (s != SX1262_OK) return s;
    s = set_tx_params(s_cfg.tx_power_dbm);                if (s != SX1262_OK) return s;

    /* 6. Modulation + packet parameters, sync word, FIFO base addresses. */
    s = set_modulation_params(s_cfg.spreading_factor,
                              s_cfg.bandwidth_hz,
                              s_cfg.coding_rate);          if (s != SX1262_OK) return s;
    s = set_packet_params(s_cfg.preamble_symbols, MAX_LORA_PAYLOAD,
                          s_cfg.crc_on, s_cfg.iq_inverted); if (s != SX1262_OK) return s;
    s = set_sync_word(s_cfg.sync_word);                    if (s != SX1262_OK) return s;
    s = set_buffer_base(0, 0);                             if (s != SX1262_OK) return s;

    /* 6.5. CAD params (used by do_tx as listen-before-talk). */
    s = set_cad_params(s_cfg.spreading_factor);            if (s != SX1262_OK) return s;

    /* 7. Route the relevant LoRa events to DIO1. */
    uint16_t mask = SX_IRQ_TX_DONE | SX_IRQ_RX_DONE | SX_IRQ_TIMEOUT |
                    SX_IRQ_CRC_ERR | SX_IRQ_CAD_DONE | SX_IRQ_CAD_DETECTED;
    s = set_dio_irq_params(mask, mask, 0, 0);              if (s != SX1262_OK) return s;

    /* 8. ISR + queue. */
    s = sx1262_hal_install_dio1_isr();                     if (s != SX1262_OK) return s;
    s_tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_request_t));
    if (!s_tx_queue) return SX1262_ERR_HW;

    ESP_LOGI(TAG, "init OK: %lu Hz  SF%u  BW%lu  CR4/%u  %d dBm",
             (unsigned long)s_cfg.rf_freq_hz, s_cfg.spreading_factor,
             (unsigned long)s_cfg.bandwidth_hz, s_cfg.coding_rate + 4,
             s_cfg.tx_power_dbm);
    return SX1262_OK;
}

sx1262_status_t sx1262_send(const uint8_t *data, size_t len, TickType_t timeout)
{
    if (!data || len == 0 || len > MAX_LORA_PAYLOAD) return SX1262_ERR_PARAM;

    tx_request_t req;
    req.len = (uint8_t)len;
    memcpy(req.data, data, len);

    return (xQueueSend(s_tx_queue, &req, timeout) == pdTRUE)
            ? SX1262_OK : SX1262_ERR_QUEUE_FULL;
}

/* ---------------------------------------------------------------------- */
/*  TX / RX execution (runs only inside sx1262_run)                       */
/* ---------------------------------------------------------------------- */

#define MAX_CAD_RETRIES 3

/* Returns true if CAD detected a LoRa preamble on our channel. Leaves the
 * chip in STDBY_RC regardless of outcome (cadExitMode = CAD_ONLY). */
static bool channel_busy(void)
{
    set_standby(SX_STANDBY_RC);
    clear_irq_status(SX_IRQ_ALL);

    /* Drain any spurious DIO1 give from prior events. */
    (void)sx1262_hal_wait_dio1(0);

    enter_cad();

    /* CAD with 4 symbols at SF7/BW125 ≈ 4 ms; longer at higher SF.
     * 50 ms covers SF12 (32 ms/symbol × 4 = 128 ms… revisit if SF>10). */
    if (!sx1262_hal_wait_dio1(pdMS_TO_TICKS(50))) {
        ESP_LOGW(TAG, "CAD: DIO1 never fired");
        return false;  /* assume clear so we don't permanently defer */
    }

    uint16_t irq = 0;
    get_irq_status(&irq);
    clear_irq_status(SX_IRQ_ALL);

    return (irq & SX_IRQ_CAD_DETECTED) != 0;
}

static void do_tx(const tx_request_t *req)
{
    /* CAD-before-TX: listen for an in-progress LoRa transmission; on
     * detection, back off with random jitter and retry. After
     * MAX_CAD_RETRIES we transmit anyway — the channel may be persistently
     * busy and the upper layer's cadence is what gates retry-bombing. */
    for (int attempt = 1; attempt <= MAX_CAD_RETRIES; attempt++) {
        if (!channel_busy()) break;

        if (attempt == MAX_CAD_RETRIES) {
            ESP_LOGW(TAG, "CAD: channel busy after %d attempts, TXing anyway",
                     MAX_CAD_RETRIES);
            break;
        }

        uint32_t jitter_ms = 5 + (esp_random() % 46);  /* 5..50 ms */
        ESP_LOGI(TAG, "CAD: busy, backoff %lu ms (attempt %d/%d)",
                 (unsigned long)jitter_ms, attempt, MAX_CAD_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(jitter_ms));
    }

    /* Re-set packet params with the actual payload length so the chip's
     * implicit-mode optimizations (if ever enabled) get the right value. */
    set_packet_params(s_cfg.preamble_symbols, req->len,
                      s_cfg.crc_on, s_cfg.iq_inverted);
    sx1262_hal_write_buf(0, req->data, req->len);
    clear_irq_status(SX_IRQ_ALL);

    /* Drain any spurious pending DIO1 give. */
    (void)sx1262_hal_wait_dio1(0);

    enter_tx(0);                          /* 0 = no chip-side timeout */

    /* Bound the wait at 5 s — far longer than any sub-1 KB SF7 packet. */
    if (!sx1262_hal_wait_dio1(pdMS_TO_TICKS(5000))) {
        ESP_LOGW(TAG, "TX wait timed out — DIO1 never fired");
    }

    uint16_t irq = 0;
    get_irq_status(&irq);
    clear_irq_status(SX_IRQ_ALL);

    if (irq & SX_IRQ_TX_DONE) {
        ESP_LOGI(TAG, "TX done (%u B)", req->len);
    } else {
        ESP_LOGW(TAG, "TX irq=0x%04x (no TX_DONE bit)", irq);
    }
}

static void do_rx_packet(void)
{
    /* Freeze the FIFO during read-out: STDBY_RC pauses the receiver so a
     * back-to-back RX can't overwrite the bytes we're about to read. The
     * 256-byte data buffer is preserved across STDBY (DS §6.1). The cost
     * is missing any packet whose preamble starts during this ~100 µs
     * window; for beacon-rate traffic that is effectively zero. */
    set_standby(SX_STANDBY_RC);

    uint8_t st[2] = {0};
    sx1262_hal_cmd_read(SX_CMD_GET_RX_BUFFER_STATUS, NULL, 0, st, 2);
    uint8_t payload_len = st[0];
    uint8_t buf_start   = st[1];

    if (payload_len == 0) {
        enter_rx_continuous();
        return;
    }

    static uint8_t rxbuf[MAX_LORA_PAYLOAD];
    sx1262_hal_read_buf(buf_start, rxbuf, payload_len);

    /* GetPacketStatus (LoRa) returns: rssiPkt, snrPkt, signalRssiPkt.
     *   rssi_dbm = -rssiPkt / 2          (rssiPkt unsigned)
     *   snr_db   =  snrPkt   / 4          (snrPkt is two's complement) */
    uint8_t ps[3] = {0};
    sx1262_hal_cmd_read(SX_CMD_GET_PACKET_STATUS, NULL, 0, ps, 3);
    int8_t rssi = -(int8_t)(ps[0] / 2);
    int8_t snr  =  (int8_t)ps[1] / 4;

    /* Resume continuous RX before the callback runs. */
    enter_rx_continuous();

    if (s_cfg.rx_callback) {
        s_cfg.rx_callback(rxbuf, payload_len, rssi, snr);
    }
}

void sx1262_run(void)
{
    /* Start in continuous RX so we can hear traffic when idle. */
    set_packet_params(s_cfg.preamble_symbols, MAX_LORA_PAYLOAD,
                      s_cfg.crc_on, s_cfg.iq_inverted);
    clear_irq_status(SX_IRQ_ALL);
    enter_rx_continuous();

    for (;;) {
        tx_request_t req;

        /* Prefer outbound traffic: drain a TX request if one is pending. */
        if (xQueueReceive(s_tx_queue, &req, pdMS_TO_TICKS(100)) == pdTRUE) {
            do_tx(&req);
            clear_irq_status(SX_IRQ_ALL);
            enter_rx_continuous();
            continue;
        }

        /* No TX queued: poll DIO1 briefly for an inbound packet. */
        if (sx1262_hal_wait_dio1(pdMS_TO_TICKS(50))) {
            uint16_t irq = 0;
            get_irq_status(&irq);
            clear_irq_status(SX_IRQ_ALL);

            if (irq & SX_IRQ_RX_DONE) {
                if (irq & SX_IRQ_CRC_ERR) {
                    /* Continuous-mode auto-rearm covers the drop. */
                    ESP_LOGW(TAG, "RX CRC error — packet dropped");
                } else {
                    /* do_rx_packet handles the STDBY/RX flip itself. */
                    do_rx_packet();
                }
            } else if (irq & SX_IRQ_TIMEOUT) {
                /* Shouldn't happen with timeout=0xFFFFFF, but rearm just in case. */
                enter_rx_continuous();
            }
        }
    }
}
