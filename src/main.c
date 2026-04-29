/*
 * Heltec WiFi LoRa 32 V3.2 — FreeRTOS + bare-metal SX1262 driver + OLED.
 *
 * Topology:
 *   CPU0 (PRO_CPU): app_task   — drives the OLED, enqueues TX requests,
 *                                drains the rx_event_t queue between cadence ticks.
 *   CPU1 (APP_CPU): lora_task  — owns the SX1262 state machine; on RX,
 *                                copies the packet into an rx_event_t and
 *                                posts it to s_rx_events. No I²C in the
 *                                radio hot path.
 *
 * Frame format (over-the-air):
 *   [src_id : u8][seq : u8][body bytes ...]
 * src_id is derived from the LSB of the factory MAC, so two boards flashed
 * with the same firmware auto-differentiate.
 *
 * Display layout (text grid, 21 cols × 8 rows, rendered with 5x7 font):
 *
 *   Row 0:  LORA 915 MHZ           <- band header
 *   Row 1:  SF7 BW125 CR4/5        <- modulation summary
 *   Row 2:  ----                   <- separator
 *   Row 3:  TX: NNNN  ID:XX        <- send counter and our node ID
 *   Row 4:  from #YY seq Q         <- last RX sender + sequence number
 *   Row 5:  RX: MMMM               <- receive counter
 *   Row 6:  <last RX payload>      <- truncated to 21 chars
 *   Row 7:  RSSI -dd  SNR sdd      <- last RX link metrics
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "sx1262.h"
#include "ssd1306.h"

static const char *TAG = "app";

#define FRAME_HEADER_LEN 2

/* Snapshot of one received LoRa packet (post-header parse), handed from
 * lora_task -> app_task so the radio task never blocks on I²C. */
typedef struct {
    uint8_t  data[253];      /* SX1262 max payload (255) minus header */
    uint8_t  len;            /* body length, post-header */
    uint8_t  src_id;
    uint8_t  seq;
    int8_t   rssi;
    int8_t   snr;
    uint32_t count;          /* 1-based RX index assigned at reception time */
} rx_event_t;

#define RX_EVENT_QUEUE_DEPTH 4
static QueueHandle_t s_rx_events;

/* Single-producer (lora_task via on_rx_packet) — no atomic needed. */
static uint32_t s_rx_count = 0;

/* Identity & TX sequencing — only touched by app_task. */
static uint8_t s_node_id = 0;
static uint8_t s_tx_seq  = 0;

static void init_node_id(void)
{
    /* MAC LSB: stable per board, distinct between boards. Lets us flash
     * one firmware to N boards and have them auto-differentiate. */
    uint8_t mac[6] = {0};
    esp_err_t err = esp_read_mac(mac, ESP_MAC_BASE);
    if (err == ESP_OK) {
        s_node_id = mac[5];
    } else {
        s_node_id = 0xA1;
        ESP_LOGW(TAG, "esp_read_mac failed (%s); fallback ID 0x%02X",
                 esp_err_to_name(err), s_node_id);
    }
    ESP_LOGI(TAG, "node ID: 0x%02X", s_node_id);
}

static void on_rx_packet(const uint8_t *data, size_t len, int8_t rssi, int8_t snr)
{
    s_rx_count++;

    if (len < FRAME_HEADER_LEN) {
        ESP_LOGW(TAG, "RX %u B  rssi=%d  snr=%d  (no frame header — dropped)",
                 (unsigned)len, rssi, snr);
        return;
    }

    uint8_t src_id = data[0];
    uint8_t seq    = data[1];
    const uint8_t *body = &data[FRAME_HEADER_LEN];
    size_t body_len = len - FRAME_HEADER_LEN;

    ESP_LOGI(TAG, "RX from #%02X seq %u  %u B  rssi=%d  snr=%d  : %.*s",
             src_id, seq, (unsigned)body_len, rssi, snr,
             (int)body_len, (const char *)body);

    rx_event_t evt = {
        .len    = (uint8_t)body_len,
        .src_id = src_id,
        .seq    = seq,
        .rssi   = rssi,
        .snr    = snr,
        .count  = s_rx_count,
    };
    if (body_len > 0) memcpy(evt.data, body, body_len);

    /* Drop on full: a missed display update is preferable to back-pressuring
     * the radio task into an I²C-paced wait. */
    if (xQueueSend(s_rx_events, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "rx event queue full — dropping display update for #%lu",
                 (unsigned long)s_rx_count);
    }
}

static void render_rx_event(const rx_event_t *evt)
{
    ssd1306_printf(0, 4, "from #%02X seq %u", evt->src_id, evt->seq);
    ssd1306_printf(0, 5, "RX: %04lu", (unsigned long)evt->count);
    ssd1306_printf(0, 6, "%.*s", (int)evt->len, (const char *)evt->data);
    ssd1306_printf(0, 7, "RSSI %d  SNR %+d", evt->rssi, evt->snr);
    ssd1306_flush();
}

static void lora_task(void *arg)
{
    ESP_LOGI(TAG, "lora_task on CPU%d", xPortGetCoreID());

    sx1262_config_t cfg = sx1262_default_config();
    cfg.rx_callback = on_rx_packet;

    if (sx1262_init(&cfg) != SX1262_OK) {
        ESP_LOGE(TAG, "sx1262_init failed — radio will not run");
        vTaskDelete(NULL);
    }

    sx1262_run();   /* never returns */
}

static void app_task(void *arg)
{
    ESP_LOGI(TAG, "app_task on CPU%d", xPortGetCoreID());

    if (ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "ssd1306_init failed — running headless");
    } else {
        ssd1306_clear();
        ssd1306_print(0, 0, "LORA 915 MHZ");
        ssd1306_print(0, 1, "SF7 BW125 CR4/5");
        ssd1306_print(0, 2, "---------------------");
        ssd1306_printf(0, 3, "TX: 0000  ID:%02X", s_node_id);
        ssd1306_printf(0, 5, "RX: 0000");
        ssd1306_print(0, 6, "(no rx yet)");
        ssd1306_flush();
    }

    /* Brief pause so the LoRa task finishes init before the first send. */
    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t n = 0;
    char     text[40];
    uint8_t  frame[FRAME_HEADER_LEN + sizeof(text)];

    /* TX every TX_INTERVAL; service RX events in between via a
     * deadline-bounded queue wait. */
    const TickType_t TX_INTERVAL = pdMS_TO_TICKS(10000);
    TickType_t next_tx = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        if ((int32_t)(next_tx - now) <= 0) {
            int text_len = snprintf(text, sizeof(text),
                                    "hello from heltec v3 #%lu", (unsigned long)n);
            if (text_len < 0) text_len = 0;

            frame[0] = s_node_id;
            frame[1] = s_tx_seq;
            memcpy(&frame[FRAME_HEADER_LEN], text, (size_t)text_len);
            size_t frame_len = FRAME_HEADER_LEN + (size_t)text_len;

            sx1262_status_t s = sx1262_send(frame, frame_len, pdMS_TO_TICKS(2000));
            if (s == SX1262_OK) {
                ESP_LOGI(TAG, "TX queued: src=%02X seq=%u \"%s\"",
                         s_node_id, s_tx_seq, text);
                s_tx_seq++;
                ssd1306_printf(0, 3, "TX: %04lu  ID:%02X",
                               (unsigned long)(n + 1), s_node_id);
                ssd1306_flush();
            } else {
                ESP_LOGW(TAG, "TX enqueue failed (status %d)", s);
            }
            n++;
            next_tx = xTaskGetTickCount() + TX_INTERVAL;
            continue;
        }

        rx_event_t evt;
        TickType_t wait = next_tx - now;
        if (xQueueReceive(s_rx_events, &evt, wait) == pdTRUE) {
            render_rx_event(&evt);
        }
    }
}

void app_main(void)
{
    init_node_id();

    s_rx_events = xQueueCreate(RX_EVENT_QUEUE_DEPTH, sizeof(rx_event_t));
    configASSERT(s_rx_events != NULL);

    ESP_LOGI(TAG, "boot — pinning lora_task to CPU%d, app_task to CPU%d",
             SX1262_PINNED_CORE, APP_PINNED_CORE);

    BaseType_t ok;

    ok = xTaskCreatePinnedToCore(
            lora_task, "lora",
            4096, NULL,
            configMAX_PRIORITIES - 2,
            NULL,
            SX1262_PINNED_CORE);
    configASSERT(ok == pdPASS);

    ok = xTaskCreatePinnedToCore(
            app_task, "app",
            4096, NULL,
            tskIDLE_PRIORITY + 2,
            NULL,
            APP_PINNED_CORE);
    configASSERT(ok == pdPASS);
}
