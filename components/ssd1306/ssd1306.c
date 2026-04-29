/*
 * ssd1306.c — SSD1306 OLED driver for the Heltec WiFi LoRa 32 V3.2.
 *
 * Wire protocol (DS_SSD1306, Solomon Systech):
 *   Every I2C transaction is [addr][control][bytes...].
 *     control = 0x00 → all following bytes are commands
 *     control = 0x40 → all following bytes are display RAM (GDDRAM) data
 *
 * Framebuffer organization:
 *   The chip stores 128 cols × 8 pages, each page = 8 vertical pixels in
 *   one byte (LSB = top). With horizontal addressing mode, after each
 *   byte the column auto-increments; at end of row it wraps to col 0 of
 *   the next page. So a single 1024-byte burst paints the whole display
 *   in one shot.
 */

#include "ssd1306.h"
#include "ssd1306_pins.h"
#include "ssd1306_font.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ssd1306";

#define SSD1306_I2C_ADDR     0x3C
#define SSD1306_I2C_FREQ_HZ  400000
#define SSD1306_FB_BYTES     (SSD1306_WIDTH * SSD1306_PAGES)   /* 1024 */

/* SSD1306 command opcodes (subset). */
#define SSD_DISPLAY_OFF             0xAE
#define SSD_DISPLAY_ON              0xAF
#define SSD_SET_CONTRAST            0x81
#define SSD_NORMAL_DISPLAY          0xA6
#define SSD_ENTIRE_DISPLAY_RESUME   0xA4
#define SSD_SET_DISPLAY_OFFSET      0xD3
#define SSD_SET_COM_PINS            0xDA
#define SSD_SET_VCOM_DETECT         0xDB
#define SSD_SET_DISPLAY_CLOCK_DIV   0xD5
#define SSD_SET_PRECHARGE           0xD9
#define SSD_SET_MULTIPLEX           0xA8
#define SSD_SET_START_LINE          0x40   /* OR with line 0..63 */
#define SSD_MEMORY_MODE             0x20
#define SSD_COLUMN_ADDR             0x21
#define SSD_PAGE_ADDR               0x22
#define SSD_COM_SCAN_DEC            0xC8
#define SSD_SEG_REMAP_INV           0xA1
#define SSD_CHARGE_PUMP             0x8D

static i2c_master_bus_handle_t  s_bus;
static i2c_master_dev_handle_t  s_dev;
static SemaphoreHandle_t        s_mutex;
static uint8_t                  s_fb[SSD1306_FB_BYTES];

/* I2C transmit buffer for the framebuffer flush — lives in BSS so we
 * don't put 1 KB on the task stack every refresh. */
static uint8_t s_flush_buf[1 + SSD1306_FB_BYTES];

/* Send a sequence of commands in a single I2C transaction. */
static esp_err_t ssd_cmds(const uint8_t *cmds, size_t n)
{
    uint8_t buf[16];
    if (n + 1 > sizeof(buf)) return ESP_ERR_INVALID_ARG;
    buf[0] = 0x00;                   /* Co=0, D/C#=0 → command stream */
    memcpy(&buf[1], cmds, n);
    return i2c_master_transmit(s_dev, buf, n + 1, 100);
}

esp_err_t ssd1306_init(void)
{
    /* 1. Power the OLED rail via the VEXT control MOSFET (active low). */
    gpio_config_t vext_cfg = {
        .pin_bit_mask = (1ULL << SSD1306_PIN_VEXT),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&vext_cfg));
    gpio_set_level(SSD1306_PIN_VEXT, 0);
    vTaskDelay(pdMS_TO_TICKS(50));   /* let the rail come up */

    /* 2. RST line: pulse low. */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << SSD1306_PIN_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));
    gpio_set_level(SSD1306_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(SSD1306_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 3. I2C bus + device handle. */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port           = 0,
        .sda_io_num         = SSD1306_PIN_SDA,
        .scl_io_num         = SSD1306_PIN_SCL,
        .clk_source         = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt  = 7,
        .flags = { .enable_internal_pullup = 1 },
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_new_master_bus: %d", err); return err; }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SSD1306_I2C_ADDR,
        .scl_speed_hz    = SSD1306_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_master_bus_add_device: %d", err); return err; }

    /* 4. Panel init — single transaction worth of commands.
     * Order matters: charge pump must be turned on before DISPLAY_ON, and
     * the orientation flips (SEG_REMAP_INV + COM_SCAN_DEC) make the panel
     * render right-side-up given the Heltec V3 board layout. */
    static const uint8_t init_seq[] = {
        SSD_DISPLAY_OFF,
        SSD_SET_DISPLAY_CLOCK_DIV, 0x80,
        SSD_SET_MULTIPLEX,          0x3F,    /* 64-row panel */
        SSD_SET_DISPLAY_OFFSET,     0x00,
        SSD_SET_START_LINE | 0x00,
        SSD_CHARGE_PUMP,            0x14,    /* internal VCC -> 7.5 V */
        SSD_MEMORY_MODE,            0x00,    /* horizontal addressing */
        SSD_SEG_REMAP_INV,                    /* col 127 -> SEG0 */
        SSD_COM_SCAN_DEC,                     /* COM63 -> top */
        SSD_SET_COM_PINS,           0x12,
        SSD_SET_CONTRAST,           0xCF,
        SSD_SET_PRECHARGE,          0xF1,
        SSD_SET_VCOM_DETECT,        0x40,
        SSD_ENTIRE_DISPLAY_RESUME,
        SSD_NORMAL_DISPLAY,
        SSD_DISPLAY_ON,
    };
    /* Send the init burst as one transaction. We can't use ssd_cmds()
     * directly because its 16-byte buffer is too small. */
    static uint8_t init_buf[1 + sizeof(init_seq)];
    init_buf[0] = 0x00;
    memcpy(&init_buf[1], init_seq, sizeof(init_seq));
    err = i2c_master_transmit(s_dev, init_buf, sizeof(init_buf), 100);
    if (err != ESP_OK) { ESP_LOGE(TAG, "init burst failed: %d", err); return err; }

    /* 5. Concurrency mutex + initial blank display. */
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    s_flush_buf[0] = 0x40;       /* Co=0, D/C#=1 → data stream */

    ssd1306_clear();
    ssd1306_flush();

    ESP_LOGI(TAG, "init OK (128x64, I2C@%d kHz, addr 0x%02X)",
             SSD1306_I2C_FREQ_HZ / 1000, SSD1306_I2C_ADDR);
    return ESP_OK;
}

void ssd1306_clear(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_fb, 0, sizeof(s_fb));
    xSemaphoreGive(s_mutex);
}

/* Caller must hold s_mutex. */
static void putc_locked(uint8_t col, uint8_t row, char c)
{
    if (col >= SSD1306_COLS_TEXT || row >= SSD1306_ROWS_TEXT) return;
    if (c < 0x20 || c > 0x7E) c = '?';

    const uint8_t *glyph = ssd1306_font5x7[c - 0x20];
    uint8_t *dst = &s_fb[row * SSD1306_WIDTH + col * SSD1306_FONT_W];
    dst[0] = glyph[0];
    dst[1] = glyph[1];
    dst[2] = glyph[2];
    dst[3] = glyph[3];
    dst[4] = glyph[4];
    dst[5] = 0x00;               /* 1-pixel inter-character gap */
}

void ssd1306_putc(uint8_t col, uint8_t row, char c)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    putc_locked(col, row, c);
    xSemaphoreGive(s_mutex);
}

void ssd1306_print(uint8_t col, uint8_t row, const char *s)
{
    if (!s) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    while (*s && col < SSD1306_COLS_TEXT) {
        putc_locked(col++, row, *s++);
    }
    xSemaphoreGive(s_mutex);
}

void ssd1306_printf(uint8_t col, uint8_t row, const char *fmt, ...)
{
    char tmp[SSD1306_COLS_TEXT + 1];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    /* Erase the line from `col` rightward so leftover chars from a longer
     * previous write don't ghost behind shorter new content. */
    for (uint8_t c = col; c < SSD1306_COLS_TEXT; c++) {
        putc_locked(c, row, ' ');
    }
    for (size_t i = 0; tmp[i] && (col + i) < SSD1306_COLS_TEXT; i++) {
        putc_locked(col + (uint8_t)i, row, tmp[i]);
    }
    xSemaphoreGive(s_mutex);
}

esp_err_t ssd1306_flush(void)
{
    /* Set the display window to the whole panel (col 0..127, page 0..7). */
    static const uint8_t window[] = {
        SSD_COLUMN_ADDR, 0,   127,
        SSD_PAGE_ADDR,   0,   7,
    };
    esp_err_t err = ssd_cmds(window, sizeof(window));
    if (err != ESP_OK) return err;

    /* Snapshot the framebuffer under the lock, transmit unlocked. */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(&s_flush_buf[1], s_fb, SSD1306_FB_BYTES);
    xSemaphoreGive(s_mutex);

    return i2c_master_transmit(s_dev, s_flush_buf, sizeof(s_flush_buf), 1000);
}
