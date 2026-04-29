#pragma once

/* SX1262 command opcodes — Semtech DS_SX1261-2_V2.1, table 11-1.
 * "Command" here means a single SPI transaction:
 *     [opcode][param0][param1]...[paramN]
 * Read-style commands are clocked as:
 *     [opcode][NOP_status][NOP_data0][NOP_data1]...
 * with the chip status returned in the slot following the opcode. */

/* ---- Operating mode --------------------------------------------------- */
#define SX_CMD_SET_SLEEP                 0x84
#define SX_CMD_SET_STANDBY               0x80
#define SX_CMD_SET_FS                    0xC1
#define SX_CMD_SET_TX                    0x83
#define SX_CMD_SET_RX                    0x82
#define SX_CMD_STOP_TIMER_ON_PREAMBLE    0x9F
#define SX_CMD_SET_RX_DUTY_CYCLE         0x94
#define SX_CMD_SET_CAD                   0xC5
#define SX_CMD_SET_TX_CONTINUOUS_WAVE    0xD1
#define SX_CMD_SET_TX_INFINITE_PREAMBLE  0xD2
#define SX_CMD_SET_REGULATOR_MODE        0x96
#define SX_CMD_CALIBRATE                 0x89
#define SX_CMD_CALIBRATE_IMAGE           0x98
#define SX_CMD_SET_PA_CONFIG             0x95
#define SX_CMD_SET_RX_TX_FALLBACK_MODE   0x93

/* ---- Register / FIFO access ------------------------------------------ */
#define SX_CMD_WRITE_REGISTER            0x0D
#define SX_CMD_READ_REGISTER             0x1D
#define SX_CMD_WRITE_BUFFER              0x0E
#define SX_CMD_READ_BUFFER               0x1E

/* ---- DIO and IRQ ----------------------------------------------------- */
#define SX_CMD_SET_DIO_IRQ_PARAMS        0x08
#define SX_CMD_GET_IRQ_STATUS            0x12
#define SX_CMD_CLEAR_IRQ_STATUS          0x02
#define SX_CMD_SET_DIO2_AS_RF_SWITCH     0x9D
#define SX_CMD_SET_DIO3_AS_TCXO_CTRL     0x97

/* ---- RF / packet ----------------------------------------------------- */
#define SX_CMD_SET_RF_FREQUENCY          0x86
#define SX_CMD_SET_PACKET_TYPE           0x8A
#define SX_CMD_GET_PACKET_TYPE           0x11
#define SX_CMD_SET_TX_PARAMS             0x8E
#define SX_CMD_SET_MODULATION_PARAMS     0x8B
#define SX_CMD_SET_PACKET_PARAMS         0x8C
#define SX_CMD_SET_CAD_PARAMS            0x88
#define SX_CMD_SET_BUFFER_BASE_ADDRESS   0x8F
#define SX_CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

/* ---- Status / diagnostics -------------------------------------------- */
#define SX_CMD_GET_STATUS                0xC0
#define SX_CMD_GET_RX_BUFFER_STATUS      0x13
#define SX_CMD_GET_PACKET_STATUS         0x14
#define SX_CMD_GET_RSSI_INST             0x15
#define SX_CMD_GET_STATS                 0x10
#define SX_CMD_RESET_STATS               0x00
#define SX_CMD_GET_DEVICE_ERRORS         0x17
#define SX_CMD_CLEAR_DEVICE_ERRORS       0x07

/* ---- IRQ status bits (16-bit) ---------------------------------------- */
#define SX_IRQ_TX_DONE                   (1u << 0)
#define SX_IRQ_RX_DONE                   (1u << 1)
#define SX_IRQ_PREAMBLE_DETECTED         (1u << 2)
#define SX_IRQ_SYNC_WORD_VALID           (1u << 3)
#define SX_IRQ_HEADER_VALID              (1u << 4)
#define SX_IRQ_HEADER_ERR                (1u << 5)
#define SX_IRQ_CRC_ERR                   (1u << 6)
#define SX_IRQ_CAD_DONE                  (1u << 7)
#define SX_IRQ_CAD_DETECTED              (1u << 8)
#define SX_IRQ_TIMEOUT                   (1u << 9)
#define SX_IRQ_LR_FHSS_HOP               (1u << 14)
#define SX_IRQ_ALL                       0x7FFFu

/* ---- Packet type ----------------------------------------------------- */
#define SX_PACKET_TYPE_GFSK              0x00
#define SX_PACKET_TYPE_LORA              0x01
#define SX_PACKET_TYPE_LR_FHSS           0x03

/* ---- Standby submode ------------------------------------------------- */
#define SX_STANDBY_RC                    0x00   /* 13 MHz RC oscillator */
#define SX_STANDBY_XOSC                  0x01   /* 32 MHz TCXO */

/* ---- Selected registers --------------------------------------------- */
#define SX_REG_LSYNCRH                   0x0740   /* LoRa sync word, MSB */
#define SX_REG_LSYNCRL                   0x0741   /* LoRa sync word, LSB */
#define SX_REG_RX_GAIN                   0x08AC
#define SX_REG_TX_CLAMP_CONFIG           0x08D8
#define SX_REG_OCP                       0x08E7
