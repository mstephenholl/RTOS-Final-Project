#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "sx1262.h"   /* sx1262_status_t, TickType_t */

/* One-time setup of the SPI bus, GPIOs, and DIO1 binary semaphore. */
sx1262_status_t sx1262_hal_init(void);

/* Pulse RST low for >100 us, release, wait for the chip to come up. */
void sx1262_hal_reset(void);

/* Block until BUSY is low or `timeout_ms` elapses. The SX1262 *requires*
 * BUSY to be low before each new SPI command, otherwise commands are
 * silently dropped. Always call this on entry to a HAL transaction. */
sx1262_status_t sx1262_hal_wait_busy(uint32_t timeout_ms);

/* Write-only command:  [opcode][tx_len bytes].  No status read-back. */
sx1262_status_t sx1262_hal_cmd(uint8_t opcode,
                               const uint8_t *tx, size_t tx_len);

/* Read-back command: clocks `tx_len` bytes of params after the opcode,
 * then receives one NOP_status byte and `rx_len` bytes of payload.
 * Only the payload is copied to `rx`; the status byte is discarded. */
sx1262_status_t sx1262_hal_cmd_read(uint8_t opcode,
                                    const uint8_t *tx, size_t tx_len,
                                    uint8_t *rx, size_t rx_len);

/* Direct register access (uses opcodes 0x0D / 0x1D). Address is 16-bit. */
sx1262_status_t sx1262_hal_write_reg(uint16_t addr,
                                     const uint8_t *data, size_t len);
sx1262_status_t sx1262_hal_read_reg(uint16_t addr,
                                    uint8_t *data, size_t len);

/* Tx/Rx FIFO (256 B internal buffer), addressed by 8-bit offset. */
sx1262_status_t sx1262_hal_write_buf(uint8_t offset,
                                     const uint8_t *data, size_t len);
sx1262_status_t sx1262_hal_read_buf(uint8_t offset,
                                    uint8_t *data, size_t len);

/* Install the DIO1 rising-edge ISR. The ISR posts to a binary semaphore
 * the LoRa task can wait on. */
sx1262_status_t sx1262_hal_install_dio1_isr(void);

/* Block up to `timeout` ticks for the next DIO1 IRQ. Returns true if one
 * arrived, false on timeout. */
bool sx1262_hal_wait_dio1(TickType_t timeout);
