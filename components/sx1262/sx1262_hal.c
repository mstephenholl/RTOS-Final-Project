/*
 * sx1262_hal.c — thin SPI/GPIO/IRQ layer for the SX1262.
 *
 * Everything here is mechanical wire-protocol: what goes on the SPI bus,
 * when to assert NSS, when to wait for BUSY, how to receive DIO1 IRQs.
 * No knowledge of LoRa modulation parameters lives here — that belongs
 * one level up in sx1262.c.
 */

#include "sx1262_hal.h"
#include "sx1262_pins.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"

static const char *TAG = "sx1262_hal";

#define SX1262_SPI_HOST       SPI2_HOST
#define SX1262_SPI_CLOCK_HZ   8000000   /* DS allows up to 16 MHz; 8 is conservative */
#define SX1262_HAL_BUFSZ      256       /* matches the SX1262's internal FIFO size */

static spi_device_handle_t s_spi;
static SemaphoreHandle_t   s_dio1_sem;

/* ---------- ISR ------------------------------------------------------- */

static void IRAM_ATTR dio1_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_dio1_sem, &hp);
    if (hp) portYIELD_FROM_ISR();
}

/* ---------- Bring-up -------------------------------------------------- */

sx1262_status_t sx1262_hal_init(void)
{
    /* RST and NSS as outputs, idle high. */
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << SX1262_PIN_RST) | (1ULL << SX1262_PIN_NSS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(SX1262_PIN_NSS, 1);
    gpio_set_level(SX1262_PIN_RST, 1);

    /* BUSY as input, no pull (chip drives it actively). */
    gpio_config_t busy = {
        .pin_bit_mask = (1ULL << SX1262_PIN_BUSY),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&busy));

    /* DIO1 as input with rising-edge IRQ. */
    gpio_config_t dio1 = {
        .pin_bit_mask = (1ULL << SX1262_PIN_DIO1),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&dio1));

    /* SPI bus on SPI2_HOST. */
    spi_bus_config_t bus = {
        .mosi_io_num     = SX1262_PIN_MOSI,
        .miso_io_num     = SX1262_PIN_MISO,
        .sclk_io_num     = SX1262_PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = SX1262_HAL_BUFSZ,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SX1262_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = SX1262_SPI_CLOCK_HZ,
        .mode           = 0,                /* CPOL=0 CPHA=0 per DS */
        .spics_io_num   = -1,               /* manual CS — we own NSS */
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SX1262_SPI_HOST, &dev, &s_spi));

    s_dio1_sem = xSemaphoreCreateBinary();
    if (!s_dio1_sem) return SX1262_ERR_HW;

    return SX1262_OK;
}

void sx1262_hal_reset(void)
{
    gpio_set_level(SX1262_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));     /* DS: tRESET ≥ 100 µs; 2 ms is plenty */
    gpio_set_level(SX1262_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));    /* allow the chip to come up to STDBY_RC */
}

/* ---------- BUSY polling --------------------------------------------- */

sx1262_status_t sx1262_hal_wait_busy(uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (gpio_get_level(SX1262_PIN_BUSY)) {
        if (esp_timer_get_time() > deadline) {
            ESP_LOGW(TAG, "BUSY high for >%lu ms — chip wedged?", (unsigned long)timeout_ms);
            return SX1262_ERR_TIMEOUT;
        }
        /* yield rather than spin — most BUSY phases finish in <1 ms */
        vTaskDelay(1);
    }
    return SX1262_OK;
}

/* ---------- Core SPI primitive --------------------------------------- */

static sx1262_status_t spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    gpio_set_level(SX1262_PIN_NSS, 0);
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    gpio_set_level(SX1262_PIN_NSS, 1);
    return (err == ESP_OK) ? SX1262_OK : SX1262_ERR_HW;
}

/* ---------- Public command primitives -------------------------------- */

sx1262_status_t sx1262_hal_cmd(uint8_t opcode, const uint8_t *tx, size_t tx_len)
{
    if (tx_len + 1 > SX1262_HAL_BUFSZ) return SX1262_ERR_PARAM;

    sx1262_status_t s = sx1262_hal_wait_busy(100);
    if (s != SX1262_OK) return s;

    uint8_t buf[SX1262_HAL_BUFSZ];
    buf[0] = opcode;
    if (tx_len) memcpy(&buf[1], tx, tx_len);
    return spi_xfer(buf, NULL, tx_len + 1);
}

sx1262_status_t sx1262_hal_cmd_read(uint8_t opcode,
                                    const uint8_t *tx, size_t tx_len,
                                    uint8_t *rx, size_t rx_len)
{
    /* Wire format: [opcode][tx params...][NOP_status][NOP_data0]...[NOP_dataN-1] */
    size_t total = 1 + tx_len + 1 + rx_len;
    if (total > SX1262_HAL_BUFSZ) return SX1262_ERR_PARAM;

    sx1262_status_t s = sx1262_hal_wait_busy(100);
    if (s != SX1262_OK) return s;

    uint8_t txbuf[SX1262_HAL_BUFSZ];
    uint8_t rxbuf[SX1262_HAL_BUFSZ];
    memset(txbuf, 0, total);
    txbuf[0] = opcode;
    if (tx_len) memcpy(&txbuf[1], tx, tx_len);

    s = spi_xfer(txbuf, rxbuf, total);
    if (s == SX1262_OK && rx_len) {
        /* Skip the opcode echo, the param echoes, and the status byte. */
        memcpy(rx, &rxbuf[1 + tx_len + 1], rx_len);
    }
    return s;
}

sx1262_status_t sx1262_hal_write_reg(uint16_t addr, const uint8_t *data, size_t len)
{
    if (len + 3 > SX1262_HAL_BUFSZ) return SX1262_ERR_PARAM;
    sx1262_status_t s = sx1262_hal_wait_busy(100);
    if (s != SX1262_OK) return s;

    uint8_t buf[SX1262_HAL_BUFSZ];
    buf[0] = 0x0D;                                /* WriteRegister */
    buf[1] = (uint8_t)(addr >> 8);
    buf[2] = (uint8_t)(addr & 0xFF);
    memcpy(&buf[3], data, len);
    return spi_xfer(buf, NULL, len + 3);
}

sx1262_status_t sx1262_hal_read_reg(uint16_t addr, uint8_t *data, size_t len)
{
    /* Wire format: [0x1D][addrH][addrL][NOP_status][NOP_data0]... */
    size_t total = 4 + len;
    if (total > SX1262_HAL_BUFSZ) return SX1262_ERR_PARAM;

    sx1262_status_t s = sx1262_hal_wait_busy(100);
    if (s != SX1262_OK) return s;

    uint8_t txbuf[SX1262_HAL_BUFSZ];
    uint8_t rxbuf[SX1262_HAL_BUFSZ];
    memset(txbuf, 0, total);
    txbuf[0] = 0x1D;
    txbuf[1] = (uint8_t)(addr >> 8);
    txbuf[2] = (uint8_t)(addr & 0xFF);

    s = spi_xfer(txbuf, rxbuf, total);
    if (s == SX1262_OK) memcpy(data, &rxbuf[4], len);
    return s;
}

sx1262_status_t sx1262_hal_write_buf(uint8_t offset, const uint8_t *data, size_t len)
{
    if (len + 2 > SX1262_HAL_BUFSZ) return SX1262_ERR_PARAM;
    sx1262_status_t s = sx1262_hal_wait_busy(100);
    if (s != SX1262_OK) return s;

    uint8_t buf[SX1262_HAL_BUFSZ];
    buf[0] = 0x0E;                                /* WriteBuffer */
    buf[1] = offset;
    memcpy(&buf[2], data, len);
    return spi_xfer(buf, NULL, len + 2);
}

sx1262_status_t sx1262_hal_read_buf(uint8_t offset, uint8_t *data, size_t len)
{
    /* Wire format: [0x1E][offset][NOP_status][NOP_data0]... */
    size_t total = 3 + len;
    if (total > SX1262_HAL_BUFSZ) return SX1262_ERR_PARAM;

    sx1262_status_t s = sx1262_hal_wait_busy(100);
    if (s != SX1262_OK) return s;

    uint8_t txbuf[SX1262_HAL_BUFSZ];
    uint8_t rxbuf[SX1262_HAL_BUFSZ];
    memset(txbuf, 0, total);
    txbuf[0] = 0x1E;
    txbuf[1] = offset;

    s = spi_xfer(txbuf, rxbuf, total);
    if (s == SX1262_OK) memcpy(data, &rxbuf[3], len);
    return s;
}

/* ---------- DIO1 IRQ wiring ----------------------------------------- */

sx1262_status_t sx1262_hal_install_dio1_isr(void)
{
    static bool service_installed = false;
    if (!service_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        service_installed = true;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(SX1262_PIN_DIO1, dio1_isr, NULL));
    return SX1262_OK;
}

bool sx1262_hal_wait_dio1(TickType_t timeout)
{
    return xSemaphoreTake(s_dio1_sem, timeout) == pdTRUE;
}

/* ---------- Wake pulse for Sleep -> STDBY_RC ------------------------ */

sx1262_status_t sx1262_hal_wake_pulse(void)
{
    /* DS §13.1.4: any NSS falling edge wakes the chip. We don't send SPI
     * bytes during the wake — the chip's wakeup state machine isn't ready
     * to interpret them yet. The pulse width isn't tightly specified;
     * 20 µs is well above any reasonable threshold. */
    gpio_set_level(SX1262_PIN_NSS, 0);
    esp_rom_delay_us(20);
    gpio_set_level(SX1262_PIN_NSS, 1);
    /* tWAKE up to ~5 ms; allow margin. */
    return sx1262_hal_wait_busy(20);
}

sx1262_status_t sx1262_hal_enable_dio1_wake(void)
{
    /* GPIO wake from light sleep is level-based (the edge ISR is separate
     * and continues to fire on rising-edge — the GPIO peripheral remembers
     * the edge across the sleep boundary so the ISR runs after wake). */
    esp_err_t err = gpio_wakeup_enable(SX1262_PIN_DIO1, GPIO_INTR_HIGH_LEVEL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "gpio_wakeup_enable failed (%d)", err);
        return SX1262_ERR_HW;
    }
    err = esp_sleep_enable_gpio_wakeup();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_sleep_enable_gpio_wakeup failed (%d)", err);
        return SX1262_ERR_HW;
    }
    return SX1262_OK;
}
