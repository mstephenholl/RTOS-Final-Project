#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* Geometry */
#define SSD1306_WIDTH        128
#define SSD1306_HEIGHT       64
#define SSD1306_PAGES        8     /* HEIGHT / 8 */

/* Text grid — 5x7 glyph + 1px gap = 6 px wide, 1 page tall. */
#define SSD1306_FONT_W       6
#define SSD1306_COLS_TEXT    21    /* 128 / 6 */
#define SSD1306_ROWS_TEXT    8     /* one row per page */

/* Bring up VEXT, RST, the I2C bus, and the panel itself.
 * Must be called once from a task (not from app_main directly — uses
 * vTaskDelay for power-rail settle). */
esp_err_t ssd1306_init(void);

/* Zero the framebuffer (display goes blank after the next flush). */
void ssd1306_clear(void);

/* Draw a single ASCII char into the framebuffer at text-grid (col,row).
 * Non-printable bytes render as '?'. Off-grid coordinates are no-ops. */
void ssd1306_putc(uint8_t col, uint8_t row, char c);

/* Draw a string starting at (col,row); does not erase the rest of the row. */
void ssd1306_print(uint8_t col, uint8_t row, const char *s);

/* printf-style draw into the framebuffer. The line from `col` to the right
 * edge is *erased first*, so the new text fully replaces whatever was there. */
void ssd1306_printf(uint8_t col, uint8_t row, const char *fmt, ...);

/* Push the framebuffer to the panel over I2C (~21 ms at 400 kHz). */
esp_err_t ssd1306_flush(void);
