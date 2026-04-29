/*
 * Heltec WiFi LoRa 32 V3.2 — FreeRTOS + bare-metal SX1262 driver + OLED.
 *
 * Topology:
 *   CPU0 (PRO_CPU): app_task   — drives the OLED, runs the mesh state
 *                                machine (dedup + forward scheduler), and
 *                                originates periodic broadcasts.
 *   CPU1 (APP_CPU): lora_task  — owns the SX1262 state machine; on RX,
 *                                copies the packet into an rx_event_t and
 *                                posts it to s_rx_events. No I²C in the
 *                                radio hot path.
 *
 * Frame format (over-the-air):
 *   [src_id : u8][seq : u8][dst_id : u8][hop_flags : u8][body bytes ...]
 *
 *   src_id     - originating node (LSB of factory MAC)
 *   seq        - per-source monotonic counter, used as the dedup key
 *   dst_id     - target node, or MESH_BROADCAST (0xFF) for "everyone"
 *   hop_flags  - bits 7..4: TTL (0..15)
 *                bit 3:     wants_ack    (reserved for Tier 2)
 *                bit 2:     is_ack       (reserved for Tier 2)
 *                bit 1:     is_forwarded (set by relay nodes)
 *                bit 0:     reserved
 *
 * Mesh behavior (Meshtastic-style flooding):
 *   - Each RX is dedup'd on (src_id, seq); duplicates are dropped silently.
 *   - Unique packets are delivered locally if dst matches us or is
 *     broadcast, and forwarded (TTL decremented) if not solely addressed
 *     to us. Forwards are scheduled with random 50–500 ms jitter to
 *     desynchronize neighbors that all hear the same broadcast at once;
 *     CAD-before-TX provides a second collision-avoidance layer.
 *   - We pre-record our own outgoing (s_node_id, s_tx_seq) into the dedup
 *     cache so neighbor forwards of our own packets get dropped on
 *     return.
 *
 * Display layout (text grid, 21 cols × 8 rows, rendered with 5x7 font):
 *
 *   Row 0:  LORA 915 MHZ           <- band header
 *   Row 1:  SF7 BW125 CR4/5        <- modulation summary
 *   Row 2:  ----                   <- separator
 *   Row 3:  TX: NNNN  ID:XX        <- send counter and our node ID
 *   Row 4:  #YY>ZZZ sNNN tTT[*]    <- last RX: src, dst, seq, TTL (*=fwd)
 *   Row 5:  RX: MMMM               <- delivered-to-us count (post-dedup)
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
#include "esp_random.h"

#include "sx1262.h"
#include "ssd1306.h"

static const char *TAG = "app";

/* ---------------- Frame format ---------------- */

#define FRAME_HEADER_LEN          4
#define FRAME_BODY_MAX            (255 - FRAME_HEADER_LEN)   /* 251 */

#define HOP_FLAGS_TTL_SHIFT       4
#define HOP_FLAGS_TTL_MASK        0xF0u
#define HOP_FLAGS_WANTS_ACK       0x08u
#define HOP_FLAGS_IS_ACK          0x04u
#define HOP_FLAGS_IS_FORWARDED    0x02u

#define MESH_BROADCAST            0xFFu
#define MESH_TTL_DEFAULT          4

/* ---------------- Cross-task RX event ---------------- */

typedef struct {
    uint8_t  data[FRAME_BODY_MAX];
    uint8_t  len;            /* body length, post-header */
    uint8_t  src_id;
    uint8_t  seq;
    uint8_t  dst_id;
    uint8_t  hop_flags;
    int8_t   rssi;
    int8_t   snr;
    uint32_t count;          /* 1-based RX index assigned at reception time */
} rx_event_t;

#define RX_EVENT_QUEUE_DEPTH 4
static QueueHandle_t s_rx_events;

/* Single-producer (lora_task via on_rx_packet) — no atomic needed. */
static uint32_t s_rx_count = 0;

/* ---------------- Identity ---------------- */

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
    /* Reserve 0xFF for broadcast addressing. */
    if (s_node_id == MESH_BROADCAST) s_node_id = 0xFE;
    ESP_LOGI(TAG, "node ID: 0x%02X", s_node_id);
}

/* ---------------- Dedup cache ---------------- */

#define MESH_DEDUP_ENTRIES   32
#define MESH_DEDUP_TTL_MS    30000   /* entries expire 30 s after insertion */

typedef struct {
    uint8_t    src_id;
    uint8_t    seq;
    uint8_t    valid;        /* 0 = empty slot */
    TickType_t expiry;       /* tick at which this entry becomes stale */
} dedup_entry_t;

static dedup_entry_t s_dedup[MESH_DEDUP_ENTRIES];

/* If (src_id, seq) is already cached and unexpired, return true (duplicate).
 * Otherwise insert it and return false. Touched only by app_task. */
static bool dedup_check_and_record(uint8_t src_id, uint8_t seq)
{
    TickType_t now = xTaskGetTickCount();
    int free_slot = -1;
    int oldest_slot = 0;
    TickType_t oldest_exp = s_dedup[0].expiry;

    for (int i = 0; i < MESH_DEDUP_ENTRIES; i++) {
        dedup_entry_t *e = &s_dedup[i];
        bool expired = (!e->valid) || (int32_t)(now - e->expiry) >= 0;

        if (expired) {
            if (free_slot < 0) free_slot = i;
            continue;
        }
        if (e->src_id == src_id && e->seq == seq) {
            return true;
        }
        /* Track the slot whose expiry is earliest, in case we need eviction. */
        if ((int32_t)(e->expiry - oldest_exp) < 0) {
            oldest_exp  = e->expiry;
            oldest_slot = i;
        }
    }

    int slot = (free_slot >= 0) ? free_slot : oldest_slot;
    s_dedup[slot].src_id = src_id;
    s_dedup[slot].seq    = seq;
    s_dedup[slot].valid  = 1;
    s_dedup[slot].expiry = now + pdMS_TO_TICKS(MESH_DEDUP_TTL_MS);
    return false;
}

/* ---------------- Forward queue ---------------- */

#define MESH_FORWARD_QUEUE_DEPTH  4
#define MESH_FORWARD_JITTER_MIN_MS 50
#define MESH_FORWARD_JITTER_MAX_MS 500

typedef struct {
    uint8_t    frame[FRAME_HEADER_LEN + FRAME_BODY_MAX];
    uint8_t    len;
    uint8_t    valid;
    TickType_t deadline;
} forward_t;

static forward_t s_fwd_queue[MESH_FORWARD_QUEUE_DEPTH];

static bool forward_schedule(const rx_event_t *evt)
{
    uint8_t ttl = (evt->hop_flags & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;
    if (ttl == 0) return false;

    int slot = -1;
    for (int i = 0; i < MESH_FORWARD_QUEUE_DEPTH; i++) {
        if (!s_fwd_queue[i].valid) { slot = i; break; }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "fwd queue full — dropping forward of #%02X seq %u",
                 evt->src_id, evt->seq);
        return false;
    }

    forward_t *f = &s_fwd_queue[slot];
    uint8_t new_ttl = ttl - 1;
    uint8_t new_flags = (evt->hop_flags & ~HOP_FLAGS_TTL_MASK)
                      | (new_ttl << HOP_FLAGS_TTL_SHIFT)
                      | HOP_FLAGS_IS_FORWARDED;

    f->frame[0] = evt->src_id;
    f->frame[1] = evt->seq;
    f->frame[2] = evt->dst_id;
    f->frame[3] = new_flags;
    if (evt->len > 0) memcpy(&f->frame[FRAME_HEADER_LEN], evt->data, evt->len);
    f->len   = FRAME_HEADER_LEN + evt->len;
    f->valid = 1;

    uint32_t span = MESH_FORWARD_JITTER_MAX_MS - MESH_FORWARD_JITTER_MIN_MS + 1;
    uint32_t jitter_ms = MESH_FORWARD_JITTER_MIN_MS + (esp_random() % span);
    f->deadline = xTaskGetTickCount() + pdMS_TO_TICKS(jitter_ms);

    ESP_LOGI(TAG, "fwd scheduled: #%02X seq %u dst=%02X ttl %u (in %lu ms)",
             evt->src_id, evt->seq, evt->dst_id, new_ttl,
             (unsigned long)jitter_ms);
    return true;
}

/* If a forward is due, copy it out and clear its slot. */
static bool forward_pop_due(forward_t *out, TickType_t now)
{
    int earliest = -1;
    TickType_t earliest_dl = 0;
    for (int i = 0; i < MESH_FORWARD_QUEUE_DEPTH; i++) {
        if (!s_fwd_queue[i].valid) continue;
        if (earliest < 0 || (int32_t)(s_fwd_queue[i].deadline - earliest_dl) < 0) {
            earliest    = i;
            earliest_dl = s_fwd_queue[i].deadline;
        }
    }
    if (earliest < 0) return false;
    if ((int32_t)(earliest_dl - now) > 0) return false;

    *out = s_fwd_queue[earliest];
    s_fwd_queue[earliest].valid = 0;
    return true;
}

/* Returns the earliest forward deadline, or portMAX_DELAY-equivalent
 * if the queue is empty. Caller computes wait time via signed subtraction. */
static TickType_t forward_next_deadline(TickType_t fallback)
{
    TickType_t earliest = fallback;
    bool found = false;
    for (int i = 0; i < MESH_FORWARD_QUEUE_DEPTH; i++) {
        if (!s_fwd_queue[i].valid) continue;
        if (!found || (int32_t)(s_fwd_queue[i].deadline - earliest) < 0) {
            earliest = s_fwd_queue[i].deadline;
            found = true;
        }
    }
    return earliest;
}

/* ---------------- TX completion (still just logs in Tier 1) ---------------- */

static void on_tx_complete(sx1262_status_t status, size_t len)
{
    if (status == SX1262_OK) {
        ESP_LOGI(TAG, "TX completed on air (%u B)", (unsigned)len);
    } else {
        ESP_LOGW(TAG, "TX air-side failure (status %d, %u B)", status, (unsigned)len);
    }
}

/* ---------------- RX path ---------------- */

static void on_rx_packet(const uint8_t *data, size_t len, int8_t rssi, int8_t snr)
{
    s_rx_count++;

    if (len < FRAME_HEADER_LEN) {
        ESP_LOGW(TAG, "RX %u B  rssi=%d  snr=%d  (no frame header — dropped)",
                 (unsigned)len, rssi, snr);
        return;
    }

    uint8_t src_id    = data[0];
    uint8_t seq       = data[1];
    uint8_t dst_id    = data[2];
    uint8_t hop_flags = data[3];
    const uint8_t *body = &data[FRAME_HEADER_LEN];
    size_t body_len = len - FRAME_HEADER_LEN;

    uint8_t ttl = (hop_flags & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;
    ESP_LOGI(TAG, "RX #%02X>%02X seq %u ttl %u  %u B  rssi=%d snr=%d  : %.*s",
             src_id, dst_id, seq, ttl, (unsigned)body_len, rssi, snr,
             (int)body_len, (const char *)body);

    rx_event_t evt = {
        .len       = (uint8_t)body_len,
        .src_id    = src_id,
        .seq       = seq,
        .dst_id    = dst_id,
        .hop_flags = hop_flags,
        .rssi      = rssi,
        .snr       = snr,
        .count     = s_rx_count,
    };
    if (body_len > 0) memcpy(evt.data, body, body_len);

    /* Drop on full: a missed display update is preferable to back-pressuring
     * the radio task into an I²C-paced wait. */
    if (xQueueSend(s_rx_events, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "rx event queue full — dropping #%lu",
                 (unsigned long)s_rx_count);
    }
}

/* ---------------- OLED render ---------------- */

static void render_rx_event(const rx_event_t *evt)
{
    uint8_t ttl = (evt->hop_flags & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;
    const char *fwd_marker = (evt->hop_flags & HOP_FLAGS_IS_FORWARDED) ? "*" : "";

    if (evt->dst_id == MESH_BROADCAST) {
        ssd1306_printf(0, 4, "#%02X>ALL s%u t%u%s",
                       evt->src_id, evt->seq, ttl, fwd_marker);
    } else {
        ssd1306_printf(0, 4, "#%02X>#%02X s%u t%u%s",
                       evt->src_id, evt->dst_id, evt->seq, ttl, fwd_marker);
    }
    ssd1306_printf(0, 5, "RX: %04lu", (unsigned long)evt->count);
    ssd1306_printf(0, 6, "%.*s", (int)evt->len, (const char *)evt->data);
    ssd1306_printf(0, 7, "RSSI %d  SNR %+d", evt->rssi, evt->snr);
    ssd1306_flush();
}

/* ---------------- Mesh state machine (app_task context) ---------------- */

static void handle_rx_event(const rx_event_t *evt)
{
    if (dedup_check_and_record(evt->src_id, evt->seq)) {
        ESP_LOGI(TAG, "dedup drop: #%02X seq %u", evt->src_id, evt->seq);
        return;
    }

    bool for_us = (evt->dst_id == s_node_id) || (evt->dst_id == MESH_BROADCAST);
    if (for_us) render_rx_event(evt);

    /* Forward unless solely addressed to us. Broadcasts get forwarded too;
     * dedup at neighbors handles the resulting redundancy. */
    bool should_forward = (evt->dst_id != s_node_id);
    uint8_t ttl = (evt->hop_flags & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;
    if (should_forward && ttl > 0) {
        forward_schedule(evt);
    }
}

/* ---------------- Tasks ---------------- */

static void lora_task(void *arg)
{
    ESP_LOGI(TAG, "lora_task on CPU%d", xPortGetCoreID());

    sx1262_config_t cfg = sx1262_default_config();
    cfg.rx_callback = on_rx_packet;
    cfg.tx_callback = on_tx_complete;

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

    /* TX every TX_INTERVAL; service RX events and forward deadlines in
     * between via a deadline-bounded queue wait. */
    const TickType_t TX_INTERVAL = pdMS_TO_TICKS(10000);
    TickType_t next_tx = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* 1. Fire forwards whose deadlines have arrived. */
        forward_t fwd;
        if (forward_pop_due(&fwd, now)) {
            sx1262_status_t s = sx1262_send(fwd.frame, fwd.len, pdMS_TO_TICKS(2000));
            if (s == SX1262_OK) {
                uint8_t f_ttl = (fwd.frame[3] & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;
                ESP_LOGI(TAG, "fwd TX queued: #%02X>%02X seq %u ttl %u (%u B)",
                         fwd.frame[0], fwd.frame[2], fwd.frame[1], f_ttl,
                         (unsigned)fwd.len);
            } else {
                ESP_LOGW(TAG, "fwd TX enqueue failed (status %d)", s);
            }
            continue;
        }

        /* 2. Fire periodic origination if it's our turn. */
        if ((int32_t)(next_tx - now) <= 0) {
            int text_len = snprintf(text, sizeof(text),
                                    "hello from heltec v3 #%lu", (unsigned long)n);
            if (text_len < 0) text_len = 0;

            frame[0] = s_node_id;
            frame[1] = s_tx_seq;
            frame[2] = MESH_BROADCAST;
            frame[3] = (MESH_TTL_DEFAULT << HOP_FLAGS_TTL_SHIFT);
            memcpy(&frame[FRAME_HEADER_LEN], text, (size_t)text_len);
            size_t frame_len = FRAME_HEADER_LEN + (size_t)text_len;

            /* Pre-record so neighbor forwards of our own packet are dropped
             * when they come back to us. */
            (void)dedup_check_and_record(s_node_id, s_tx_seq);

            sx1262_status_t s = sx1262_send(frame, frame_len, pdMS_TO_TICKS(2000));
            if (s == SX1262_OK) {
                ESP_LOGI(TAG, "TX queued: src=%02X seq=%u dst=ALL ttl=%u \"%s\"",
                         s_node_id, s_tx_seq, MESH_TTL_DEFAULT, text);
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

        /* 3. Wait for the next event: either an RX or the nearest deadline
         * (next TX, or next forward). */
        TickType_t deadline = forward_next_deadline(next_tx);
        if ((int32_t)(deadline - next_tx) > 0) deadline = next_tx;
        TickType_t wait = (int32_t)(deadline - now) > 0 ? (deadline - now) : 0;

        rx_event_t evt;
        if (xQueueReceive(s_rx_events, &evt, wait) == pdTRUE) {
            handle_rx_event(&evt);
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
