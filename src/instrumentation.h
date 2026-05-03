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
 * Logic-analyzer pin map. All pins are non-strapping, header-accessible
 * GPIOs not used by the SX1262 or SSD1306 drivers (those pins —
 * GPIO 8-14, 17-18, 21 — are routed internally on the V3 PCB and aren't
 * brought out to the headers).
 *
 *   GPIO 1: APP_TASK active        (currently unused; reserved)
 *   GPIO 2: LOAD_A    active
 *   GPIO 4: LOAD_B    active       (skip GPIO 3 — JTAG strapping pin)
 *   GPIO 5: LOAD_C    active
 *   GPIO 6: LORA TX_DONE pulse     (firmware-mirrored from sx1262 tx_callback)
 *   GPIO 7: LORA RX pulse          (firmware-mirrored from on_rx_packet)
 *
 * GPIO 6 and 7 emit a ~5 µs pulse on every radio event — the SX1262's
 * DIO1 (internally GPIO 14) isn't accessible on the header, so we
 * mirror its rising edge in firmware via instr_gpio_pulse(). Resolves
 * cleanly at 24 MHz sampling.
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
#define INSTR_EVT_PREEMPTION     5     /* logged only for "unusual" gaps; see
                                        * PREEMPTION_LOG_THRESHOLD_US */
#define INSTR_EVT_USER           100

/* ---- Task IDs (ring buffer / UART) ---- */
#define INSTR_TASK_LORA    1
#define INSTR_TASK_APP     2
#define INSTR_TASK_LOAD_A  3
#define INSTR_TASK_LOAD_B  4
#define INSTR_TASK_LOAD_C  5

/* ---- Logic-analyzer GPIOs ---- */
#define INSTR_GPIO_APP      1
#define INSTR_GPIO_LOAD_A   2
#define INSTR_GPIO_LOAD_B   4
#define INSTR_GPIO_LOAD_C   5
#define INSTR_GPIO_LORA_TX  6
#define INSTR_GPIO_LORA_RX  7

/* ---- Synthetic injector identification ----
 * Synthetic frames carry src_id = INSTR_INJECTOR_SRC_ID. handle_rx_event
 * uses this to skip OLED render (which would otherwise dominate at high
 * inject rates) — the injector is purely a stress driver, not real traffic
 * worth displaying. */
#define INSTR_INJECTOR_SRC_ID  0x42

#ifdef ENABLE_INSTRUMENTATION

/* Configure GPIOs, spawn the timeline-dumper and stats-reporter tasks.
 * Call once at boot, before any other instr_* function. */
esp_err_t instr_init(void);

/* Append a sparse event to the ring buffer. ISR-safe, lock-free.
 * UART dumper picks it up later. */
void instr_log(uint16_t event_id, uint16_t task_id, uint32_t param);

/* Drive a logic-analyzer GPIO. Use HIGH at task entry, LOW at task exit. */
void instr_gpio_set(int pin, int level);

/* Emit a brief (~5 µs) HIGH pulse — long enough for an LA at 24 MHz
 * (~120 samples) to capture cleanly. Use for point-in-time markers
 * (TX done, RX received) rather than active/idle intervals. */
void instr_gpio_pulse(int pin);

/* Spawn the synthetic load tasks. Edit the s_load_cfgs[] array in
 * instrumentation.c to retune the workload for your experiment. */
void instr_start_load_tasks(void);

/* ---- Radio path counters ----
 *
 * Track how often the radio path drops frames vs. successfully delivers them
 * across increasing CPU load. The drop sites live in main.c (rx queue full,
 * forward queue full, sx1262_send enqueue fail); the counters and reporter
 * live here so the same 5 s stats line shows both load and radio outcomes.
 *
 * All increments are single-producer per counter:
 *   rx_received / rx_drop:    incremented from lora_task (on_rx_packet)
 *   tx_originated / tx_fail:  incremented from app_task (broadcast send path)
 *   fwd_drop:                 incremented from app_task (forward_schedule)
 *   parse_latency:            incremented from app_task (handle_rx_event)
 *
 * 32-bit aligned writes are atomic on Xtensa, and stats_task reads are
 * eventually consistent — no locking needed. */
void instr_radio_log_rx_received(void);
void instr_radio_log_rx_drop(void);
void instr_radio_log_tx_originated(void);
void instr_radio_log_tx_fail(void);
void instr_radio_log_fwd_drop(void);
void instr_radio_log_parse_latency(int64_t latency_us);

/* Time spent inside render_rx_event() — the OLED I2C flush is the main
 * suspect for parse-latency spikes. Compare avg render duration against
 * avg parse latency to see how much of the latter is blocking-on-OLED. */
void instr_radio_log_render_us(int64_t duration_us);

#else  /* !ENABLE_INSTRUMENTATION */

static inline esp_err_t instr_init(void) { return 0; }
static inline void instr_log(uint16_t event_id, uint16_t task_id, uint32_t param)
    { (void)event_id; (void)task_id; (void)param; }
static inline void instr_gpio_set(int pin, int level)
    { (void)pin; (void)level; }
static inline void instr_gpio_pulse(int pin) { (void)pin; }
static inline void instr_start_load_tasks(void) { }
static inline void instr_radio_log_rx_received(void) { }
static inline void instr_radio_log_rx_drop(void) { }
static inline void instr_radio_log_tx_originated(void) { }
static inline void instr_radio_log_tx_fail(void) { }
static inline void instr_radio_log_fwd_drop(void) { }
static inline void instr_radio_log_parse_latency(int64_t latency_us) { (void)latency_us; }
static inline void instr_radio_log_render_us(int64_t duration_us) { (void)duration_us; }

#endif  /* ENABLE_INSTRUMENTATION */
