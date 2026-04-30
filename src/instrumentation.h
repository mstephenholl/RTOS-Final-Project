#pragma once

/*
 * Real-time instrumentation harness for scheduler / load characterization.
 *
 * Two output channels with very different bandwidth budgets:
 *   1. GPIO toggles  — high-frequency "is task X running" trace, captured
 *      by an external logic analyzer. Cost per toggle: ~150 ns.
 *   2. Ring buffer    — sparse anomaly events (deadline misses, TX/RX),
 *      dumped to UART by a low-priority task. Cost per append: ~330 ns.
 *
 * Logic-analyzer pin map (verify against your Heltec V3 edge connector;
 * these are non-strapping GPIOs not used by the SX1262 / SSD1306 drivers):
 *
 *   GPIO 1: APP_TASK active
 *   GPIO 2: LOAD_A    active
 *   GPIO 4: LOAD_B    active   (skip GPIO 3 — JTAG strapping pin)
 *   GPIO 5: LOAD_C    active
 *
 * Probe DIO1 (GPIO 14) externally to get TX_DONE / RX_DONE timing for free
 * — the SX1262 already drives that pin on every radio event.
 *
 * All API is no-op when ENABLE_INSTRUMENTATION is undefined; the default
 * router/leaf/gateway/litesleep/deepsleep builds pay zero cost.
 */

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* ---- Event IDs (ring buffer / UART) ---- */
#define INSTR_EVT_DEADLINE_MISS  1
#define INSTR_EVT_TX_QUEUED      2
#define INSTR_EVT_TX_DONE        3
#define INSTR_EVT_RX             4
#define INSTR_EVT_USER           100

/* ---- Task IDs (ring buffer / UART) ---- */
#define INSTR_TASK_LORA    1
#define INSTR_TASK_APP     2
#define INSTR_TASK_LOAD_A  3
#define INSTR_TASK_LOAD_B  4
#define INSTR_TASK_LOAD_C  5

/* ---- Logic-analyzer GPIOs ---- */
#define INSTR_GPIO_APP     1
#define INSTR_GPIO_LOAD_A  2
#define INSTR_GPIO_LOAD_B  4
#define INSTR_GPIO_LOAD_C  5

#ifdef ENABLE_INSTRUMENTATION

/* Configure GPIOs, spawn the timeline-dumper and stats-reporter tasks.
 * Call once at boot, before any other instr_* function. */
esp_err_t instr_init(void);

/* Append a sparse event to the ring buffer. ISR-safe, lock-free.
 * UART dumper picks it up later. */
void instr_log(uint16_t event_id, uint16_t task_id, uint32_t param);

/* Drive a logic-analyzer GPIO. Use HIGH at task entry, LOW at task exit. */
void instr_gpio_set(int pin, int level);

/* Spawn the synthetic load tasks. Edit the s_load_cfgs[] array in
 * instrumentation.c to retune the workload for your experiment. */
void instr_start_load_tasks(void);

#else  /* !ENABLE_INSTRUMENTATION */

static inline esp_err_t instr_init(void) { return 0; }
static inline void instr_log(uint16_t event_id, uint16_t task_id, uint32_t param)
    { (void)event_id; (void)task_id; (void)param; }
static inline void instr_gpio_set(int pin, int level)
    { (void)pin; (void)level; }
static inline void instr_start_load_tasks(void) { }

#endif  /* ENABLE_INSTRUMENTATION */
