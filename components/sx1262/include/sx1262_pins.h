#pragma once

/* Heltec WiFi LoRa 32 V3 — SX1262 pin assignments
 * Source: Heltec V3 schematic (rev 1.1) and the official board pinout
 * diagram. If you see "init OK" but no RF, sanity-check these against
 * your unit's silkscreen/schematic before assuming a software bug. */

#define SX1262_PIN_NSS    8     /* SPI chip-select, active low */
#define SX1262_PIN_SCK    9
#define SX1262_PIN_MOSI   10
#define SX1262_PIN_MISO   11
#define SX1262_PIN_RST    12    /* Reset, active low */
#define SX1262_PIN_BUSY   13    /* Chip-busy indicator, active high */
#define SX1262_PIN_DIO1   14    /* General-purpose IRQ line (TX_DONE/RX_DONE/...) */

/* DIO2 and DIO3 are *not* routed to MCU pins on this board — they stay
 * inside the SX1262 module:
 *   DIO2 → drives the antenna RF switch (TX/RX path select).
 *   DIO3 → powers the on-module TCXO.
 * Both are programmed via SetDIO2AsRfSwitchCtrl / SetDIO3AsTcxoCtrl. */
