/*
 * Heltec WiFi LoRa 32 V3.2 — FreeRTOS + bare-metal SX1262 driver + OLED.
 *
 * Topology:
 *   CPU0 (PRO_CPU): app_task   — drives the OLED, enqueues TX requests
 *   CPU1 (APP_CPU): lora_task  — owns the SX1262 state machine
 *
 * Display layout (text grid, 21 cols × 8 rows, rendered with 5x7 font):
 *
 *   Row 0:  LORA 915 MHZ           <- band header
 *   Row 1:  SF7 BW125 CR4/5        <- modulation summary
 *   Row 2:  ----                   <- separator
 *   Row 3:  TX: NNNN               <- send counter
 *   Row 4:  (blank)
 *   Row 5:  RX: MMMM               <- receive counter
 *   Row 6:  <last RX payload>      <- truncated to 21 chars
 *   Row 7:  RSSI -dd  SNR sdd      <- last RX link metrics
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sx1262.h"
#include "ssd1306.h"

static const char *TAG = "app";

static atomic_uint g_rx_count = 0;

static void on_rx_packet(const uint8_t *data, size_t len, int8_t rssi, int8_t snr)
{
    unsigned n = atomic_fetch_add(&g_rx_count, 1) + 1;

    ESP_LOGI(TAG, "RX %u B  rssi=%d dBm  snr=%d dB  : %.*s",
             (unsigned)len, rssi, snr, (int)len, (const char *)data);

    /* Update display from the LoRa task; ssd1306's mutex serializes us
     * against the app_task's writes. */
    ssd1306_printf(0, 5, "RX: %04u", n);
    ssd1306_printf(0, 6, "%.*s", (int)len, (const char *)data);
    ssd1306_printf(0, 7, "RSSI %d  SNR %+d", rssi, snr);
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
        ssd1306_printf(0, 3, "TX: 0000");
        ssd1306_printf(0, 5, "RX: 0000");
        ssd1306_print(0, 6, "(no rx yet)");
        ssd1306_flush();
    }

    /* Brief pause so the LoRa task finishes init before the first send. */
    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t n = 0;
    char     payload[40];

    for (;;) {
        int len = snprintf(payload, sizeof(payload),
                           "hello from heltec v3 #%lu", (unsigned long)n);

        sx1262_status_t s = sx1262_send((const uint8_t *)payload,
                                        (size_t)len, pdMS_TO_TICKS(2000));
        if (s == SX1262_OK) {
            ESP_LOGI(TAG, "TX queued: \"%s\"", payload);
            ssd1306_printf(0, 3, "TX: %04lu", (unsigned long)(n + 1));
            ssd1306_flush();
        } else {
            ESP_LOGW(TAG, "TX enqueue failed (status %d)", s);
        }

        n++;
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
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
