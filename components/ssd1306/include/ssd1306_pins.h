#pragma once

/* Heltec WiFi LoRa 32 V3.2 — SSD1306 OLED + power-gate pin map.
 *
 * The OLED's VDD comes through a P-channel MOSFET whose gate sits on
 * GPIO36 (silkscreen "Vext_Ctrl"). Drive VEXT *low* to enable the rail. */

#define SSD1306_PIN_SDA    17
#define SSD1306_PIN_SCL    18
#define SSD1306_PIN_RST    21
#define SSD1306_PIN_VEXT   36   /* active-low: 0 = OLED powered, 1/Hi-Z = off */
