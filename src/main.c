/*
 * Heltec WiFi LoRa 32 V3.2 — FreeRTOS + bare-metal SX1262 driver + OLED.
 *
 * Topology:
 *   CPU0 (PRO_CPU): app_task   — drives the OLED, runs the mesh state
 *                                machine (dedup + forward scheduler +
 *                                ACK retry tracker), and originates
 *                                periodic broadcasts and unicasts.
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
 *                bit 3:     wants_ack    (request an ACK from dst)
 *                bit 2:     is_ack       (this frame is itself an ACK)
 *                bit 1:     is_forwarded (set by relay nodes)
 *                bit 0:     reserved
 *
 *   ACK frames carry one body byte = original_seq being acknowledged.
 *
 * Mesh behavior (Meshtastic-style flooding):
 *   - Each RX is dedup'd on (src_id, seq); duplicates are dropped silently
 *     for routing/UI purposes — but ACK generation runs BEFORE dedup, so a
 *     duplicate wants_ack frame triggers a re-ACK (loss recovery for the
 *     return path).
 *   - Unique packets are delivered locally if dst matches us or is
 *     broadcast, and forwarded (TTL decremented) if not solely addressed
 *     to us. Forwards are scheduled with random 50–500 ms jitter.
 *
 * ACK protocol (Tier 2):
 *   - Unicast TXs set wants_ack=1; broadcasts never do (would cause storms).
 *   - Recipient generates ACK back to original sender. ACK src/seq are the
 *     ACK sender's; ACK body is the original_seq being acknowledged.
 *   - Originator tracks (dst_id, original_seq) in a pending table with
 *     exponential-backoff retries (2 s, 4 s, 8 s; cap 12 s; up to 3 retries).
 *   - Retry timing starts at sx1262_send enqueue, not air-side TX completion;
 *     queue dwell is microseconds so the slop is invisible at our timescale.
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
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"

#ifdef ENABLE_LIGHT_SLEEP
#include "esp_pm.h"
#endif

#ifdef ENABLE_DEEP_SLEEP
#include "esp_sleep.h"
#endif

#include "sx1262.h"
#include "ssd1306.h"
#include "instrumentation.h"

static const char *TAG = "app";

/* ---------------- Role ---------------- */

/* Each board takes one of three roles. Set via -DAPP_ROLE=APP_ROLE_LEAF
 * (or _ROUTER / _GATEWAY) in build_flags, or override the #define below
 * per-board before flashing.
 *
 *   LEAF    : battery-style. Chip sleeps between TX events; opens a brief
 *             post-TX RX window for ACKs. Originates traffic. Does NOT
 *             forward — a sleeping node can't reliably relay.
 *   ROUTER  : always-on relay. Continuous RX, originates broadcasts +
 *             unicasts, forwards mesh traffic. The default.
 *   GATEWAY : always-on observer. Continuous RX, forwards, but never
 *             originates broadcasts/unicasts. Useful as a passive logger
 *             that doesn't pollute the network with its own traffic.
 *             (It still emits ACKs for unicasts addressed to it.) */
typedef enum {
    APP_ROLE_LEAF    = 0,
    APP_ROLE_ROUTER  = 1,
    APP_ROLE_GATEWAY = 2,
} app_role_t;

#ifndef APP_ROLE
#define APP_ROLE APP_ROLE_ROUTER
#endif

/* Compile-time policy bits — folded out of the binary for unused branches. */
#define ROLE_LOW_POWER    (APP_ROLE == APP_ROLE_LEAF)
#define ROLE_FORWARDS     (APP_ROLE != APP_ROLE_LEAF)
#define ROLE_ORIGINATES   (APP_ROLE != APP_ROLE_GATEWAY)

static const char *role_name(void)
{
    switch (APP_ROLE) {
        case APP_ROLE_LEAF:    return "LEAF";
        case APP_ROLE_ROUTER:  return "ROUTER";
        case APP_ROLE_GATEWAY: return "GATEWAY";
        default:               return "?";
    }
}

/* ---------------- Frame format ----------------
 *
 * Wire layout:
 *   byte 0   : channel_hash (low byte of SHA-256(CHANNEL_NAME))
 *   byte 1   : src_id
 *   byte 2   : seq
 *   byte 3   : dst_id
 *   byte 4   : hop_flags
 *   bytes 5-8: enc_counter (u32 LE) — plaintext, used as AES nonce input
 *   bytes 9+ : AES-CTR-encrypted body
 */

#define FRAME_HEADER_LEN          5    /* chan_hash + 4-byte routing header */
#define FRAME_ENC_PREFIX_LEN      4    /* enc_counter (u32 LE), plaintext */
#define FRAME_WIRE_MAX            255  /* SX1262 max LoRa payload */
#define FRAME_BODY_MAX            (FRAME_WIRE_MAX - FRAME_HEADER_LEN - FRAME_ENC_PREFIX_LEN)
                                  /* = 246: max plaintext body length */

#define HOP_FLAGS_TTL_SHIFT       4
#define HOP_FLAGS_TTL_MASK        0xF0u
#define HOP_FLAGS_WANTS_ACK       0x08u
#define HOP_FLAGS_IS_ACK          0x04u
#define HOP_FLAGS_IS_FORWARDED    0x02u

#define MESH_BROADCAST            0xFFu
#define MESH_TTL_DEFAULT          4

/* ---------------- Cross-task RX event ---------------- */

typedef struct {
    uint8_t  data[FRAME_BODY_MAX];   /* decrypted plaintext */
    uint8_t  len;                    /* plaintext length */
    uint8_t  src_id;
    uint8_t  seq;
    uint8_t  dst_id;
    uint8_t  hop_flags;
    int8_t   rssi;
    int8_t   snr;
    uint32_t count;                  /* 1-based RX index assigned at reception */
    uint32_t enc_counter;            /* needed to re-encrypt for forwarding */
    int64_t  ts_enqueued_us;         /* esp_timer_get_time() at enqueue;
                                      * dequeue side computes parse latency */
} rx_event_t;

#define RX_EVENT_QUEUE_DEPTH 4
static QueueHandle_t s_rx_events;

/* Single-producer (lora_task via on_rx_packet) — no atomic needed. */
static uint32_t s_rx_count = 0;

/* ---------------- Identity ---------------- */

/* s_tx_seq is uint32_t internally so persistence math doesn't wrap; the
 * wire format remains uint8_t (cast at frame-write time), which is what
 * the dedup layer expects. */
static uint8_t  s_node_id      = 0;
static uint32_t s_tx_seq       = 0;
static uint8_t  s_last_seen_id = 0;   /* for choosing a unicast peer */

/* ---------------- NVS-persisted seq counter ---------------- */

#define SEQ_PERSIST_INTERVAL  32        /* commit a "future" value every N TXs */
#define NVS_NS                "mesh"
#define NVS_KEY_TX_SEQ        "tx_seq"

static uint32_t s_seq_persistent = 0;

static void nvs_init_safe(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init: erasing partition (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/* "Future commit" pattern: NVS always contains a value > any seq we've sent.
 * On boot we restore that and reserve another batch. Cuts flash writes by
 * SEQ_PERSIST_INTERVAL× compared to write-per-TX, at the cost of "burning"
 * up to N-1 seqs across each reboot (no replay risk). */
static void seq_persist_init(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed (%s); seq starts at 0",
                 esp_err_to_name(err));
        return;
    }

    uint32_t restored = 0;
    err = nvs_get_u32(h, NVS_KEY_TX_SEQ, &restored);
    if (err == ESP_OK) {
        s_tx_seq = restored;
        ESP_LOGI(TAG, "restored s_tx_seq=%lu from NVS", (unsigned long)restored);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "nvs_get_u32: %s", esp_err_to_name(err));
    }

    /* Reserve next batch. */
    s_seq_persistent = s_tx_seq + SEQ_PERSIST_INTERVAL;
    nvs_set_u32(h, NVS_KEY_TX_SEQ, s_seq_persistent);
    nvs_commit(h);
    nvs_close(h);
}

static void seq_persist_advance(void)
{
    s_tx_seq++;
    if (s_tx_seq < s_seq_persistent) return;

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    s_seq_persistent = s_tx_seq + SEQ_PERSIST_INTERVAL;
    nvs_set_u32(h, NVS_KEY_TX_SEQ, s_seq_persistent);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "seq batch reserved up to %lu",
             (unsigned long)s_seq_persistent);
}

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

/* ---------------- Channel hash (early filter) ---------------- */

#define CHANNEL_NAME "rtos-final-project"

static uint8_t s_channel_hash;

static void init_channel_hash(void)
{
    uint8_t digest[32];
    int ret = mbedtls_sha256((const uint8_t *)CHANNEL_NAME,
                             strlen(CHANNEL_NAME), digest, 0);
    if (ret != 0) {
        ESP_LOGW(TAG, "mbedtls_sha256 failed (%d); using fallback hash", ret);
        s_channel_hash = 0xC0;
        return;
    }
    s_channel_hash = digest[0];
    ESP_LOGI(TAG, "channel hash: 0x%02X (\"%s\")", s_channel_hash, CHANNEL_NAME);
}

/* ---------------- Channel encryption (AES-128-CTR) ---------------- */

/* Hardcoded channel key. Production deployments would derive this from a
 * passphrase via PBKDF2 or similar; for the educational artifact it's a
 * compile-time constant. Anyone with this firmware can read the channel.
 *
 * SECURITY NOTE: AES-CTR provides confidentiality only — an attacker can
 * still flip ciphertext bits to flip the same plaintext bits (malleability).
 * Real production needs AES-GCM or AES-CTR + HMAC. Replay is mitigated by
 * dedup, not by this layer. */
static const uint8_t s_channel_key[16] = {
    0x4D, 0x65, 0x73, 0x68, 0x4C, 0x6F, 0x52, 0x61,
    0x52, 0x54, 0x4F, 0x53, 0x4C, 0x65, 0x61, 0x66,
};
static mbedtls_aes_context s_aes_ctx;

static void aes_init_safe(void)
{
    mbedtls_aes_init(&s_aes_ctx);
    int ret = mbedtls_aes_setkey_enc(&s_aes_ctx, s_channel_key, 128);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_aes_setkey_enc failed (%d)", ret);
    }
}

/* Build a 16-byte CTR nonce_counter block. Bytes 0..4 = (src_id, enc_ctr_LE);
 * bytes 5..15 = 0 and serve as the per-block counter that AES-CTR increments
 * internally. 40 bits of nonce uniqueness covers 2^40 distinct frames per
 * channel — astronomical at our TX rate. */
static void aes_make_nonce(uint8_t out[16], uint8_t src_id, uint32_t enc_ctr)
{
    memset(out, 0, 16);
    out[0] = src_id;
    out[1] = (uint8_t)(enc_ctr      );
    out[2] = (uint8_t)(enc_ctr >>  8);
    out[3] = (uint8_t)(enc_ctr >> 16);
    out[4] = (uint8_t)(enc_ctr >> 24);
}

/* Symmetric: same call encrypts and decrypts (CTR mode). Safe for concurrent
 * calls from lora_task and app_task — after setkey, s_aes_ctx is read-only
 * during crypt operations. */
static void aes_crypt(uint8_t *output, const uint8_t *input, size_t len,
                      uint8_t src_id, uint32_t enc_ctr)
{
    if (len == 0) return;
    uint8_t nonce_counter[16];
    aes_make_nonce(nonce_counter, src_id, enc_ctr);
    uint8_t stream_block[16] = {0};
    size_t  nc_off = 0;
    int ret = mbedtls_aes_crypt_ctr(&s_aes_ctx, len, &nc_off, nonce_counter,
                                    stream_block, input, output);
    if (ret != 0) {
        ESP_LOGW(TAG, "mbedtls_aes_crypt_ctr failed (%d)", ret);
    }
}

/* ---------------- Dedup cache ---------------- */

#define MESH_DEDUP_ENTRIES   32
#define MESH_DEDUP_TTL_MS    30000

typedef struct {
    uint8_t    src_id;
    uint8_t    seq;
    uint8_t    valid;
    TickType_t expiry;
} dedup_entry_t;

static dedup_entry_t s_dedup[MESH_DEDUP_ENTRIES];

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

#define MESH_FORWARD_QUEUE_DEPTH   4
#define MESH_FORWARD_JITTER_MIN_MS 50
#define MESH_FORWARD_JITTER_MAX_MS 500

typedef struct {
    uint8_t    frame[FRAME_WIRE_MAX];   /* encrypted wire bytes ready to TX */
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
        instr_radio_log_fwd_drop();
        ESP_LOGW(TAG, "fwd queue full — dropping forward of #%02X seq %u",
                 evt->src_id, evt->seq);
        return false;
    }

    forward_t *f = &s_fwd_queue[slot];
    uint8_t new_ttl = ttl - 1;
    uint8_t new_flags = (evt->hop_flags & ~HOP_FLAGS_TTL_MASK)
                      | (new_ttl << HOP_FLAGS_TTL_SHIFT)
                      | HOP_FLAGS_IS_FORWARDED;

    /* Header — chan_hash, then TTL-decremented routing fields. */
    f->frame[0] = s_channel_hash;
    f->frame[1] = evt->src_id;
    f->frame[2] = evt->seq;
    f->frame[3] = evt->dst_id;
    f->frame[4] = new_flags;

    /* Plaintext enc_counter prefix — same value the originator used so any
     * receiver can reproduce the nonce. */
    f->frame[FRAME_HEADER_LEN + 0] = (uint8_t)(evt->enc_counter      );
    f->frame[FRAME_HEADER_LEN + 1] = (uint8_t)(evt->enc_counter >>  8);
    f->frame[FRAME_HEADER_LEN + 2] = (uint8_t)(evt->enc_counter >> 16);
    f->frame[FRAME_HEADER_LEN + 3] = (uint8_t)(evt->enc_counter >> 24);

    /* Re-encrypt the body with the originator's (src_id, enc_counter). The
     * resulting ciphertext bytes are bit-identical to what we received. */
    if (evt->len > 0) {
        aes_crypt(&f->frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN],
                  evt->data, evt->len, evt->src_id, evt->enc_counter);
    }
    f->len   = FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + evt->len;
    f->valid = 1;

    uint32_t span = MESH_FORWARD_JITTER_MAX_MS - MESH_FORWARD_JITTER_MIN_MS + 1;
    uint32_t jitter_ms = MESH_FORWARD_JITTER_MIN_MS + (esp_random() % span);
    f->deadline = xTaskGetTickCount() + pdMS_TO_TICKS(jitter_ms);

    ESP_LOGI(TAG, "fwd scheduled: #%02X seq %u dst=%02X ttl %u (in %lu ms)",
             evt->src_id, evt->seq, evt->dst_id, new_ttl,
             (unsigned long)jitter_ms);
    return true;
}

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

/* Returns the earliest forward deadline, clamped to `fallback` from above. */
static TickType_t forward_next_deadline(TickType_t fallback)
{
    TickType_t earliest = fallback;
    for (int i = 0; i < MESH_FORWARD_QUEUE_DEPTH; i++) {
        if (!s_fwd_queue[i].valid) continue;
        if ((int32_t)(s_fwd_queue[i].deadline - earliest) < 0) {
            earliest = s_fwd_queue[i].deadline;
        }
    }
    return earliest;
}

/* ---------------- ACK pending table ---------------- */

#define ACK_PENDING_DEPTH       4
#define ACK_TIMEOUT_INITIAL_MS  2000
#define ACK_TIMEOUT_CAP_MS      12000
#define ACK_MAX_RETRIES         3       /* total sends = 1 initial + 3 retries */

typedef struct {
    uint8_t    frame[FRAME_WIRE_MAX];   /* encrypted wire bytes for retry */
    uint8_t    len;
    uint8_t    dst_id;
    uint8_t    orig_seq;
    uint8_t    attempt;       /* 0 = initial only, ≥1 = N retries done */
    uint8_t    valid;
    TickType_t next_retry;
} ack_pending_t;

static ack_pending_t s_ack_pending[ACK_PENDING_DEPTH];

static void ack_pending_add(const uint8_t *frame, size_t len,
                            uint8_t dst_id, uint8_t orig_seq)
{
    int slot = -1;
    for (int i = 0; i < ACK_PENDING_DEPTH; i++) {
        if (!s_ack_pending[i].valid) { slot = i; break; }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "ack pending table full; not tracking #%02X seq %u",
                 dst_id, orig_seq);
        return;
    }

    ack_pending_t *p = &s_ack_pending[slot];
    memcpy(p->frame, frame, len);
    p->len        = (uint8_t)len;
    p->dst_id     = dst_id;
    p->orig_seq   = orig_seq;
    p->attempt    = 0;
    p->valid      = 1;
    p->next_retry = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_INITIAL_MS);
}

static void ack_clear_pending(uint8_t src_id, uint8_t orig_seq)
{
    for (int i = 0; i < ACK_PENDING_DEPTH; i++) {
        ack_pending_t *p = &s_ack_pending[i];
        if (p->valid && p->dst_id == src_id && p->orig_seq == orig_seq) {
            ESP_LOGI(TAG, "ACK received from #%02X for seq %u (after %u %s)",
                     src_id, orig_seq, p->attempt + 1,
                     p->attempt == 0 ? "send" : "sends");
            p->valid = 0;
            return;
        }
    }
}

static int ack_due_index(TickType_t now)
{
    int earliest = -1;
    TickType_t earliest_dl = 0;
    for (int i = 0; i < ACK_PENDING_DEPTH; i++) {
        if (!s_ack_pending[i].valid) continue;
        if (earliest < 0 || (int32_t)(s_ack_pending[i].next_retry - earliest_dl) < 0) {
            earliest    = i;
            earliest_dl = s_ack_pending[i].next_retry;
        }
    }
    if (earliest < 0) return -1;
    if ((int32_t)(earliest_dl - now) > 0) return -1;
    return earliest;
}

static TickType_t ack_next_deadline(TickType_t fallback)
{
    TickType_t earliest = fallback;
    for (int i = 0; i < ACK_PENDING_DEPTH; i++) {
        if (!s_ack_pending[i].valid) continue;
        if ((int32_t)(s_ack_pending[i].next_retry - earliest) < 0) {
            earliest = s_ack_pending[i].next_retry;
        }
    }
    return earliest;
}

/* Send an ACK frame back to `orig_src` referring to its `orig_seq`. */
static void ack_send(uint8_t orig_src, uint8_t orig_seq)
{
    uint8_t frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + 1];
    frame[0] = s_channel_hash;
    frame[1] = s_node_id;
    frame[2] = (uint8_t)s_tx_seq;
    frame[3] = orig_src;
    frame[4] = (MESH_TTL_DEFAULT << HOP_FLAGS_TTL_SHIFT) | HOP_FLAGS_IS_ACK;

    uint32_t enc_ctr = s_tx_seq;
    frame[FRAME_HEADER_LEN + 0] = (uint8_t)(enc_ctr      );
    frame[FRAME_HEADER_LEN + 1] = (uint8_t)(enc_ctr >>  8);
    frame[FRAME_HEADER_LEN + 2] = (uint8_t)(enc_ctr >> 16);
    frame[FRAME_HEADER_LEN + 3] = (uint8_t)(enc_ctr >> 24);

    /* Encrypt the 1-byte body in place. */
    uint8_t plaintext_body = orig_seq;
    aes_crypt(&frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN],
              &plaintext_body, 1, s_node_id, enc_ctr);

    /* Pre-record so neighbor forwards of our own ACK get dropped on return. */
    (void)dedup_check_and_record(s_node_id, (uint8_t)s_tx_seq);

    sx1262_status_t s = sx1262_send(frame, sizeof(frame), pdMS_TO_TICKS(2000));
    if (s == SX1262_OK) {
        ESP_LOGI(TAG, "ACK TX: src=%02X seq=%u dst=%02X (acks #%02X seq %u)",
                 s_node_id, (unsigned)(s_tx_seq & 0xFFu), orig_src,
                 orig_src, orig_seq);
        seq_persist_advance();
    } else {
        ESP_LOGW(TAG, "ACK enqueue failed (status %d)", s);
    }
}

/* ---------------- TX completion (still just logs) ---------------- */

static void on_tx_complete(sx1262_status_t status, size_t len)
{
    if (status == SX1262_OK) {
        ESP_LOGI(TAG, "TX completed on air (%u B)", (unsigned)len);
    } else {
        ESP_LOGW(TAG, "TX air-side failure (status %d, %u B)", status, (unsigned)len);
    }
    instr_log(INSTR_EVT_TX_DONE, INSTR_TASK_LORA, (uint32_t)len);
    instr_gpio_pulse(INSTR_GPIO_LORA_TX);
}

/* ---------------- RX path ---------------- */

static void on_rx_packet(const uint8_t *data, size_t len, int8_t rssi, int8_t snr)
{
    s_rx_count++;

    if (len < FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN) {
        ESP_LOGW(TAG, "RX %u B  rssi=%d  snr=%d  (too short — dropped)",
                 (unsigned)len, rssi, snr);
        return;
    }

    /* Channel-hash filter: drop packets that aren't on our channel before
     * doing any AES work. Also catches accidental noise frames from CRC
     * collisions, which would have bogus first bytes. */
    if (data[0] != s_channel_hash) {
        ESP_LOGD(TAG, "wrong channel: got 0x%02X, want 0x%02X — dropped",
                 data[0], s_channel_hash);
        return;
    }

    uint8_t src_id    = data[1];
    uint8_t seq       = data[2];
    uint8_t dst_id    = data[3];
    uint8_t hop_flags = data[4];

    uint32_t enc_counter = (uint32_t)data[5]
                         | ((uint32_t)data[6] <<  8)
                         | ((uint32_t)data[7] << 16)
                         | ((uint32_t)data[8] << 24);

    const uint8_t *enc_body = &data[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN];
    size_t body_len = len - FRAME_HEADER_LEN - FRAME_ENC_PREFIX_LEN;

    if (body_len > FRAME_BODY_MAX) {
        ESP_LOGW(TAG, "RX body too large (%u > %u)",
                 (unsigned)body_len, (unsigned)FRAME_BODY_MAX);
        return;
    }

    /* Decrypt body in place into a staging buffer. */
    uint8_t plaintext[FRAME_BODY_MAX];
    if (body_len > 0) {
        aes_crypt(plaintext, enc_body, body_len, src_id, enc_counter);
    }

    uint8_t ttl = (hop_flags & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;
    bool is_ack    = (hop_flags & HOP_FLAGS_IS_ACK)    != 0;
    bool wants_ack = (hop_flags & HOP_FLAGS_WANTS_ACK) != 0;
    ESP_LOGI(TAG, "RX #%02X>%02X seq %u ttl %u%s%s  %u B  rssi=%d snr=%d  : %.*s",
             src_id, dst_id, seq, ttl,
             is_ack    ? " ACK"  : "",
             wants_ack ? " WACK" : "",
             (unsigned)body_len, rssi, snr,
             (int)body_len, (const char *)plaintext);

    rx_event_t evt = {
        .len            = (uint8_t)body_len,
        .src_id         = src_id,
        .seq            = seq,
        .dst_id         = dst_id,
        .hop_flags      = hop_flags,
        .rssi           = rssi,
        .snr            = snr,
        .count          = s_rx_count,
        .enc_counter    = enc_counter,
        .ts_enqueued_us = esp_timer_get_time(),
    };
    if (body_len > 0) memcpy(evt.data, plaintext, body_len);

    instr_radio_log_rx_received();
    if (xQueueSend(s_rx_events, &evt, 0) != pdTRUE) {
        instr_radio_log_rx_drop();
        ESP_LOGW(TAG, "rx event queue full — dropping #%lu",
                 (unsigned long)s_rx_count);
    }
    instr_log(INSTR_EVT_RX, INSTR_TASK_LORA, ((uint32_t)src_id << 8) | seq);
    instr_gpio_pulse(INSTR_GPIO_LORA_RX);
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
    /* Parse latency = time spent sitting in s_rx_events while app_task was
     * preempted or busy. Grows under load — that's the metric we're after. */
    instr_radio_log_parse_latency(esp_timer_get_time() - evt->ts_enqueued_us);

    bool is_ack    = (evt->hop_flags & HOP_FLAGS_IS_ACK)    != 0;
    bool wants_ack = (evt->hop_flags & HOP_FLAGS_WANTS_ACK) != 0;
    uint8_t ttl = (evt->hop_flags & HOP_FLAGS_TTL_MASK) >> HOP_FLAGS_TTL_SHIFT;

    /* Track the most recent peer we heard a non-ACK from; the unicast
     * cadence sends to whoever was last to broadcast/unicast at us. */
    if (!is_ack && evt->src_id != s_node_id) {
        s_last_seen_id = evt->src_id;
    }

    /* (1) ACK reception: clear pending whether or not dedup catches a
     *     duplicate ACK. Idempotent — re-clearing already-cleared is a no-op. */
    if (is_ack && evt->dst_id == s_node_id && evt->len >= 1) {
        ack_clear_pending(evt->src_id, evt->data[0]);
    }

    /* (2) ACK generation: respond to wants_ack addressed to us, even on
     *     duplicates — the previous ACK may have been lost. */
    if (wants_ack && !is_ack && evt->dst_id == s_node_id) {
        ack_send(evt->src_id, evt->seq);
    }

    /* (3) Dedup for routing/UI decisions. */
    if (dedup_check_and_record(evt->src_id, evt->seq)) {
        ESP_LOGI(TAG, "dedup drop: #%02X seq %u", evt->src_id, evt->seq);
        return;
    }

    /* (4) Local delivery (don't render ACKs — they're protocol traffic). */
    bool for_us = (evt->dst_id == s_node_id) || (evt->dst_id == MESH_BROADCAST);
    if (for_us && !is_ack) render_rx_event(evt);

    /* (5) Forwarding for anything not solely addressed to us. LEAF role
     *     never forwards — sleeping nodes can't reliably relay, and the
     *     compile-time constant lets the compiler elide this branch. */
    bool should_forward = (evt->dst_id != s_node_id);
    if (ROLE_FORWARDS && should_forward && ttl > 0) forward_schedule(evt);
}

/* ---------------- Tasks ---------------- */

static void lora_task(void *arg)
{
    ESP_LOGI(TAG, "lora_task on CPU%d", xPortGetCoreID());

    sx1262_config_t cfg = sx1262_default_config();
    cfg.rx_callback = on_rx_packet;
    cfg.tx_callback = on_tx_complete;
    cfg.low_power   = ROLE_LOW_POWER;

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
        ssd1306_printf(0, 4, "ROLE: %s", role_name());
        ssd1306_printf(0, 5, "RX: 0000");
        ssd1306_print(0, 6, "(no rx yet)");
        ssd1306_flush();
    }

    /* Brief pause so the LoRa task finishes init before the first send. */
    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t n = 0;             /* broadcast counter */
    uint32_t ping_n = 0;        /* unicast/ping counter */
    char     text[40];
    uint8_t  frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + sizeof(text)];

    const TickType_t TX_INTERVAL      = pdMS_TO_TICKS(10000);
    const TickType_t UNICAST_INTERVAL = pdMS_TO_TICKS(30000);
    TickType_t next_tx      = xTaskGetTickCount();
    TickType_t next_unicast = xTaskGetTickCount() + pdMS_TO_TICKS(15000);

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* 1. Service due ACK retries (highest priority — time-sensitive). */
        int idx = ack_due_index(now);
        if (idx >= 0) {
            ack_pending_t *p = &s_ack_pending[idx];
            if (p->attempt >= ACK_MAX_RETRIES) {
                ESP_LOGW(TAG, "ACK timeout: gave up on #%02X seq %u after %u sends",
                         p->dst_id, p->orig_seq, p->attempt + 1);
                p->valid = 0;
            } else {
                p->attempt++;
                ESP_LOGI(TAG, "ACK retry %u/%u: #%02X seq %u",
                         p->attempt, ACK_MAX_RETRIES, p->dst_id, p->orig_seq);
                sx1262_status_t s = sx1262_send(p->frame, p->len, pdMS_TO_TICKS(2000));
                if (s != SX1262_OK) {
                    ESP_LOGW(TAG, "ACK retry enqueue failed (status %d)", s);
                }
                uint32_t timeout_ms = (uint32_t)ACK_TIMEOUT_INITIAL_MS << p->attempt;
                if (timeout_ms > ACK_TIMEOUT_CAP_MS) timeout_ms = ACK_TIMEOUT_CAP_MS;
                p->next_retry = now + pdMS_TO_TICKS(timeout_ms);
            }
            continue;
        }

        /* 2. Service due forwards. */
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

        /* 3. Periodic broadcast (skipped if our role doesn't originate). */
        if (ROLE_ORIGINATES && (int32_t)(next_tx - now) <= 0) {
            int text_len = snprintf(text, sizeof(text),
                                    "hello from heltec v3 #%lu", (unsigned long)n);
            if (text_len < 0) text_len = 0;

            frame[0] = s_channel_hash;
            frame[1] = s_node_id;
            frame[2] = (uint8_t)s_tx_seq;
            frame[3] = MESH_BROADCAST;
            frame[4] = (MESH_TTL_DEFAULT << HOP_FLAGS_TTL_SHIFT);

            uint32_t enc_ctr = s_tx_seq;
            frame[FRAME_HEADER_LEN + 0] = (uint8_t)(enc_ctr      );
            frame[FRAME_HEADER_LEN + 1] = (uint8_t)(enc_ctr >>  8);
            frame[FRAME_HEADER_LEN + 2] = (uint8_t)(enc_ctr >> 16);
            frame[FRAME_HEADER_LEN + 3] = (uint8_t)(enc_ctr >> 24);

            aes_crypt(&frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN],
                      (const uint8_t *)text, (size_t)text_len,
                      s_node_id, enc_ctr);
            size_t frame_len = FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + (size_t)text_len;

            (void)dedup_check_and_record(s_node_id, (uint8_t)s_tx_seq);

            instr_radio_log_tx_originated();
            sx1262_status_t s = sx1262_send(frame, frame_len, pdMS_TO_TICKS(2000));
            if (s == SX1262_OK) {
                ESP_LOGI(TAG, "TX bcast: src=%02X seq=%u dst=ALL ttl=%u \"%s\"",
                         s_node_id, (unsigned)(s_tx_seq & 0xFFu),
                         MESH_TTL_DEFAULT, text);
                seq_persist_advance();
                ssd1306_printf(0, 3, "TX: %04lu  ID:%02X",
                               (unsigned long)(n + 1), s_node_id);
                ssd1306_flush();
            } else {
                instr_radio_log_tx_fail();
                ESP_LOGW(TAG, "TX enqueue failed (status %d)", s);
            }
            n++;
            next_tx = xTaskGetTickCount() + TX_INTERVAL;
            continue;
        }

        /* 4. Periodic unicast w/ wants_ack — only if originating and we've
         *    heard a peer. */
        if (ROLE_ORIGINATES && (int32_t)(next_unicast - now) <= 0) {
            if (s_last_seen_id != 0) {
                int text_len = snprintf(text, sizeof(text),
                                        "ping #%lu to %02X",
                                        (unsigned long)ping_n, s_last_seen_id);
                if (text_len < 0) text_len = 0;

                frame[0] = s_channel_hash;
                frame[1] = s_node_id;
                frame[2] = (uint8_t)s_tx_seq;
                frame[3] = s_last_seen_id;
                frame[4] = (MESH_TTL_DEFAULT << HOP_FLAGS_TTL_SHIFT)
                         | HOP_FLAGS_WANTS_ACK;

                uint32_t u_enc_ctr = s_tx_seq;
                frame[FRAME_HEADER_LEN + 0] = (uint8_t)(u_enc_ctr      );
                frame[FRAME_HEADER_LEN + 1] = (uint8_t)(u_enc_ctr >>  8);
                frame[FRAME_HEADER_LEN + 2] = (uint8_t)(u_enc_ctr >> 16);
                frame[FRAME_HEADER_LEN + 3] = (uint8_t)(u_enc_ctr >> 24);

                aes_crypt(&frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN],
                          (const uint8_t *)text, (size_t)text_len,
                          s_node_id, u_enc_ctr);
                size_t frame_len = FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + (size_t)text_len;

                (void)dedup_check_and_record(s_node_id, (uint8_t)s_tx_seq);

                sx1262_status_t s = sx1262_send(frame, frame_len, pdMS_TO_TICKS(2000));
                if (s == SX1262_OK) {
                    ESP_LOGI(TAG, "TX unicast: src=%02X seq=%u dst=%02X "
                             "wants_ack \"%s\"",
                             s_node_id, (unsigned)(s_tx_seq & 0xFFu),
                             s_last_seen_id, text);
                    ack_pending_add(frame, frame_len, s_last_seen_id,
                                    (uint8_t)s_tx_seq);
                    seq_persist_advance();
                    ping_n++;
                } else {
                    ESP_LOGW(TAG, "Unicast TX enqueue failed (status %d)", s);
                }
            }
            next_unicast = xTaskGetTickCount() + UNICAST_INTERVAL;
            continue;
        }

        /* 5. Wait until the soonest deadline (TX, unicast, forward, or ACK
         *    retry), or until an RX event arrives. For non-originating roles
         *    (GATEWAY), origination deadlines are skipped; we use a 60 s
         *    fallback so the loop wakes occasionally even without traffic. */
        TickType_t deadline = now + pdMS_TO_TICKS(60000);
        if (ROLE_ORIGINATES) {
            if ((int32_t)(next_tx - deadline) < 0) deadline = next_tx;
            if ((int32_t)(next_unicast - deadline) < 0) deadline = next_unicast;
        }
        deadline = forward_next_deadline(deadline);
        deadline = ack_next_deadline(deadline);
        TickType_t wait = (int32_t)(deadline - now) > 0 ? (deadline - now) : 0;

        rx_event_t evt;
        if (xQueueReceive(s_rx_events, &evt, wait) == pdTRUE) {
            handle_rx_event(&evt);
        }
    }
}

#ifdef ENABLE_DEEP_SLEEP

/* Time the chip stays in deep sleep between cycles. */
#ifndef DEEP_SLEEP_INTERVAL_S
#define DEEP_SLEEP_INTERVAL_S  60
#endif
/* Post-TX RX window in deep-sleep mode (allows brief listen for unsolicited
 * traffic; not used for ACKs since deep-sleep nodes don't track them across
 * sleeps anyway — RAM is lost). */
#ifndef DEEP_SLEEP_RX_WINDOW_MS
#define DEEP_SLEEP_RX_WINDOW_MS 2000
#endif

/* Deep-sleep one-shot: boot, transmit one beacon, brief listen, sleep.
 * The whole active phase takes a couple hundred ms. The chip wakes via
 * RTC timer and reboots from scratch — there's no FreeRTOS task that
 * survives across cycles, so persistent state lives in NVS. */
static void deep_sleep_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "deep-sleep wake (cause=%d), role=%s seq=%lu",
             (int)cause, role_name(), (unsigned long)s_tx_seq);

    /* Radio init — low_power so the chip starts asleep. send_now wakes
     * it for the TX cycle, listen_for keeps it awake for the window. */
    sx1262_config_t cfg = sx1262_default_config();
    cfg.rx_callback = on_rx_packet;
    cfg.tx_callback = on_tx_complete;
    cfg.low_power   = true;
    if (sx1262_init(&cfg) != SX1262_OK) {
        ESP_LOGE(TAG, "sx1262_init failed; sleeping anyway to avoid burn loop");
        goto sleep;
    }

    /* Compose a broadcast beacon. We deliberately don't set wants_ack —
     * a deep-sleep node can't track pending ACKs across sleep cycles. */
    char    text[40];
    int     text_len = snprintf(text, sizeof(text),
                                "deep-sleep beacon seq=%lu",
                                (unsigned long)(s_tx_seq & 0xFFu));
    if (text_len < 0) text_len = 0;

    uint8_t frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + sizeof(text)];
    frame[0] = s_channel_hash;
    frame[1] = s_node_id;
    frame[2] = (uint8_t)s_tx_seq;
    frame[3] = MESH_BROADCAST;
    frame[4] = (MESH_TTL_DEFAULT << HOP_FLAGS_TTL_SHIFT);

    uint32_t enc_ctr = s_tx_seq;
    frame[FRAME_HEADER_LEN + 0] = (uint8_t)(enc_ctr      );
    frame[FRAME_HEADER_LEN + 1] = (uint8_t)(enc_ctr >>  8);
    frame[FRAME_HEADER_LEN + 2] = (uint8_t)(enc_ctr >> 16);
    frame[FRAME_HEADER_LEN + 3] = (uint8_t)(enc_ctr >> 24);

    aes_crypt(&frame[FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN],
              (const uint8_t *)text, (size_t)text_len, s_node_id, enc_ctr);
    size_t frame_len = FRAME_HEADER_LEN + FRAME_ENC_PREFIX_LEN + (size_t)text_len;

    ESP_LOGI(TAG, "TX beacon: src=%02X seq=%u dst=ALL ttl=%u \"%s\"",
             s_node_id, (unsigned)(s_tx_seq & 0xFFu),
             MESH_TTL_DEFAULT, text);

    if (sx1262_send_now(frame, frame_len) == SX1262_OK) {
        seq_persist_advance();
    }

    /* Brief listen for unsolicited traffic (mostly for educational
     * visibility — the rx_callback fires and queues to s_rx_events, but
     * no one drains it before we sleep). */
    sx1262_listen_for(DEEP_SLEEP_RX_WINDOW_MS);

    /* Drop the chip into WARM sleep (~160 nA) before the MCU does. */
    sx1262_sleep(true);

sleep:
    {
        uint64_t sleep_us = (uint64_t)DEEP_SLEEP_INTERVAL_S * 1000000ULL;
        esp_sleep_enable_timer_wakeup(sleep_us);
        ESP_LOGI(TAG, "entering deep sleep for %u s", DEEP_SLEEP_INTERVAL_S);
        /* Brief delay so the log line completes over UART before we cut. */
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_deep_sleep_start();
    }
    /* Never returns. */
}

#endif  /* ENABLE_DEEP_SLEEP */

/* ---------------- Synthetic RX injector (instrumentation-only) ----------------
 *
 * Posts fake rx_event_t to s_rx_events at a fixed rate so we can stress the
 * RX -> parse path on a single board, without a peer. Frames look exactly
 * like what on_rx_packet would produce: same struct, same queue, same
 * downstream dispatch via handle_rx_event. The mesh state machine cannot
 * tell injected events from radio events.
 *
 * Knobs (compile-time, edit and reflash):
 *   INJECTOR_RATE_HZ  - 0 disables; 1..50 sweeps the interesting region.
 *   INJECTOR_BODY_LEN - bytes of body. Mostly affects render + memcpy cost
 *                       (we skip AES, since these are pre-decrypted plaintext).
 *   INJECTOR_TTL      - 0 skips forward_schedule; >0 exercises fwd_drop AND
 *                       puts real TX on air (forward task will retransmit).
 *
 * Priority tradeoff: runs at priority 6 (above load_a/b/c) so the actual
 * inject rate stays close to INJECTOR_RATE_HZ even under heavy CPU load.
 * The cost is ~50us/sec of CPU at 50 Hz — negligible vs. the load tasks.
 *
 * src_id is fixed at 0x42 with an 8-bit seq counter that wraps every 256
 * events. With a 30 s dedup TTL, sustained rates >= 9 Hz will eventually
 * trigger dedup-suppression of forward_schedule / render. The rx_drop and
 * parse_latency metrics are unaffected (those gate before dedup); the
 * forward path becomes a less reliable signal at high rates. */

#ifdef ENABLE_INSTRUMENTATION

#define INJECTOR_RATE_HZ      0     /* 0 disables. Try 5, 10, 20, 50. */
#define INJECTOR_BODY_LEN     23
#define INJECTOR_TTL          0     /* 0 = no forward; >0 to stress fwd queue */
#define INJECTOR_SRC_ID       0x42
#define INJECTOR_RSSI         -80
#define INJECTOR_SNR          5

static void injector_task(void *arg)
{
    (void)arg;

    if (INJECTOR_RATE_HZ == 0) {
        ESP_LOGI(TAG, "injector disabled (INJECTOR_RATE_HZ=0)");
        vTaskDelete(NULL);
    }

    TickType_t period_ticks = pdMS_TO_TICKS(1000 / INJECTOR_RATE_HZ);
    if (period_ticks < 1) period_ticks = 1;

    uint8_t  inj_seq   = 0;
    uint32_t inj_count = 0;
    TickType_t next = xTaskGetTickCount();

    ESP_LOGI(TAG, "injector running: %d Hz, src=0x%02X, ttl=%d, body=%d B",
             INJECTOR_RATE_HZ, INJECTOR_SRC_ID, INJECTOR_TTL, INJECTOR_BODY_LEN);

    for (;;) {
        rx_event_t evt = {
            .len            = INJECTOR_BODY_LEN,
            .src_id         = INJECTOR_SRC_ID,
            .seq            = inj_seq++,
            .dst_id         = MESH_BROADCAST,
            .hop_flags      = (INJECTOR_TTL << HOP_FLAGS_TTL_SHIFT),
            .rssi           = INJECTOR_RSSI,
            .snr            = INJECTOR_SNR,
            .count          = ++inj_count,
            .enc_counter    = inj_count,
            .ts_enqueued_us = esp_timer_get_time(),
        };

        /* Body: "inj NNNNNN" padded out to INJECTOR_BODY_LEN. */
        int written = snprintf((char *)evt.data, sizeof(evt.data),
                               "inj %lu", (unsigned long)inj_count);
        if (written < 0) written = 0;
        for (int i = written; i < INJECTOR_BODY_LEN && i < (int)sizeof(evt.data); i++) {
            evt.data[i] = ' ';
        }

        instr_radio_log_rx_received();
        if (xQueueSend(s_rx_events, &evt, 0) != pdTRUE) {
            instr_radio_log_rx_drop();
        }

        vTaskDelayUntil(&next, period_ticks);
    }
}

#endif  /* ENABLE_INSTRUMENTATION */

void app_main(void)
{
    nvs_init_safe();
    init_node_id();
    seq_persist_init();
    init_channel_hash();
    aes_init_safe();
    instr_init();

#ifdef ENABLE_DEEP_SLEEP
    /* One-shot path. Never returns. OLED is intentionally NOT initialized
     * (would otherwise dominate the sleep-state power budget at ~3 mA). */
    deep_sleep_main();
#endif

#ifdef ENABLE_LIGHT_SLEEP
    /* Light sleep via the IDF PM framework. The kernel automatically
     * light-sleeps the CPU when no task is runnable AND no driver holds
     * a no-sleep lock. DIO1 wakes the CPU on RX/TX events. */
    {
        esp_pm_config_t pm_cfg = {
            .max_freq_mhz = 240,
            .min_freq_mhz = 80,
            .light_sleep_enable = true,
        };
        esp_err_t err = esp_pm_configure(&pm_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_pm_configure failed (%s)", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "light sleep enabled (CPU %d-%d MHz)",
                     pm_cfg.min_freq_mhz, pm_cfg.max_freq_mhz);
        }
        if (sx1262_enable_dio1_wake() != SX1262_OK) {
            ESP_LOGW(TAG, "DIO1 wake setup failed; CPU may not wake on RX");
        }
    }
#endif
    ESP_LOGI(TAG, "role: %s (low_power=%d, forwards=%d, originates=%d)",
             role_name(), ROLE_LOW_POWER, ROLE_FORWARDS, ROLE_ORIGINATES);

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

    instr_start_load_tasks();

#ifdef ENABLE_INSTRUMENTATION
    BaseType_t inj_ok = xTaskCreate(injector_task, "injector",
                                    3072, NULL, 6, NULL);
    configASSERT(inj_ok == pdPASS);
#endif
}
