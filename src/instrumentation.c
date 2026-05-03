#ifdef ENABLE_INSTRUMENTATION

/*
 * See instrumentation.h for design rationale.
 *
 * The synthetic load tasks busy-wait against esp_timer_get_time() rather than
 * counting NOPs. This is "wallclock-bounded" — under preemption, a load task's
 * effective CPU time is less than its configured wcet_us, because the deadline
 * check exits the spin early. The deadline-miss metric still captures what we
 * want for baseline characterization: "did the system keep up with the
 * demand?" For fair scheduler-vs-scheduler comparison we'd want CPU-time-
 * accurate load (calibrated NOP loops) so each scheduler sees identical work.
 */

#include "instrumentation.h"

#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

static const char *TAG = "instr";

/* ---- Sparse-event ring buffer ---- */

#define INSTR_TIMELINE_ENTRIES  256       /* power of two for fast modulo */

typedef struct {
    int64_t  ts_us;
    uint16_t event_id;
    uint16_t task_id;
    uint32_t param;
} timeline_entry_t;

static timeline_entry_t s_timeline[INSTR_TIMELINE_ENTRIES];
static atomic_uint      s_timeline_head;

void instr_log(uint16_t event_id, uint16_t task_id, uint32_t param)
{
    /* Lock-free MPMC append. Slots can interleave by timestamp under
     * concurrent producers; downstream analysis sorts by ts_us. */
    uint32_t idx = atomic_fetch_add_explicit(&s_timeline_head, 1,
                                             memory_order_relaxed);
    timeline_entry_t *e = &s_timeline[idx & (INSTR_TIMELINE_ENTRIES - 1)];
    e->ts_us    = esp_timer_get_time();
    e->event_id = event_id;
    e->task_id  = task_id;
    e->param    = param;
}

void instr_gpio_set(int pin, int level)
{
    gpio_set_level(pin, level);
}

void instr_gpio_pulse(int pin)
{
    gpio_set_level(pin, 1);
    esp_rom_delay_us(5);
    gpio_set_level(pin, 0);
}

/* ---- Synthetic load tasks ----
 *
 * Edit s_load_cfgs[] to change the workload. Each task busy-waits for wcet_us
 * every period_ms, then deadlines until the next period.  Reasonable starting
 * point: total demanded CPU ~= sum(wcet_us / period_ms) across all load tasks
 * (here ~75% if not preempted by lora_task or app_task).
 *
 * Priority numbering (FreeRTOS): higher number = higher priority. The lora
 * task runs at configMAX_PRIORITIES - 2, app_task at tskIDLE_PRIORITY + 2.
 * Load priorities here sit between, so load preempts app but not lora. */

typedef struct {
    const char *name;
    uint16_t    task_id;
    int         instr_gpio;
    UBaseType_t priority;
    uint32_t    period_ms;
    uint32_t    wcet_us;

    /* Stats — touched only by the task itself; reads from stats_task are
     * eventually consistent (snapshot every 5 s, no locking). */
    uint32_t    activations;
    uint32_t    deadline_misses;
    int64_t     max_late_us;
} load_cfg_t;

static load_cfg_t s_load_cfgs[] = {
    /* name      task_id              gpio              prio period(ms) wcet(us) */
    { "load_a",  INSTR_TASK_LOAD_A,   INSTR_GPIO_LOAD_A, 5,  5,         1000  },
    { "load_b",  INSTR_TASK_LOAD_B,   INSTR_GPIO_LOAD_B, 4,  20,        5000  },
    { "load_c",  INSTR_TASK_LOAD_C,   INSTR_GPIO_LOAD_C, 3,  100,       30000 },
};
#define NUM_LOAD_TASKS  (sizeof(s_load_cfgs) / sizeof(s_load_cfgs[0]))

static void burn_cpu_us(uint32_t us)
{
    int64_t target = esp_timer_get_time() + us;
    while (esp_timer_get_time() < target) {
        /* tight spin; the volatile in esp_timer_get_time prevents hoisting */
    }
}

static void load_task_body(void *arg)
{
    load_cfg_t *cfg = arg;
    int64_t period_us = (int64_t)cfg->period_ms * 1000;
    TickType_t next_wake = xTaskGetTickCount();
    int64_t   next_deadline_us = esp_timer_get_time() + period_us;

    for (;;) {
        instr_gpio_set(cfg->instr_gpio, 1);
        burn_cpu_us(cfg->wcet_us);
        instr_gpio_set(cfg->instr_gpio, 0);

        cfg->activations++;
        int64_t end_us = esp_timer_get_time();
        if (end_us > next_deadline_us) {
            cfg->deadline_misses++;
            int64_t late = end_us - next_deadline_us;
            if (late > cfg->max_late_us) cfg->max_late_us = late;
            instr_log(INSTR_EVT_DEADLINE_MISS, cfg->task_id, (uint32_t)late);
        }
        next_deadline_us += period_us;

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(cfg->period_ms));
    }
}

void instr_start_load_tasks(void)
{
    for (size_t i = 0; i < NUM_LOAD_TASKS; i++) {
        load_cfg_t *cfg = &s_load_cfgs[i];
        BaseType_t ok = xTaskCreate(load_task_body, cfg->name,
                                    2048, cfg, cfg->priority, NULL);
        configASSERT(ok == pdPASS);
        ESP_LOGI(TAG, "spawned %s: period=%ums wcet=%uus prio=%u gpio=%d",
                 cfg->name, (unsigned)cfg->period_ms,
                 (unsigned)cfg->wcet_us, (unsigned)cfg->priority,
                 cfg->instr_gpio);
    }
}

/* ---- Stats reporter ----
 *
 * Logs an aggregate every 5 s. Per-task: activations, deadline misses, miss
 * percentage, max lateness. Eyeball at-a-glance whether the system is
 * keeping up. */

static void stats_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "---- 5 s stats ----");
        for (size_t i = 0; i < NUM_LOAD_TASKS; i++) {
            load_cfg_t *cfg = &s_load_cfgs[i];
            uint32_t miss_x10 = cfg->activations > 0
                ? (cfg->deadline_misses * 1000U) / cfg->activations
                : 0;
            ESP_LOGI(TAG, "%s: %u acts, %u misses (%u.%u%%), max late=%lldus",
                     cfg->name,
                     (unsigned)cfg->activations,
                     (unsigned)cfg->deadline_misses,
                     (unsigned)(miss_x10 / 10),
                     (unsigned)(miss_x10 % 10),
                     cfg->max_late_us);
        }
    }
}

/* ---- Timeline dumper ----
 *
 * Logs new ring-buffer entries to UART every 2 s. At sparse-event rates
 * (~10/s under heavy stress) this comfortably fits in 115200-baud bandwidth.
 * Runs at low priority so it doesn't perturb timing of the things being
 * measured. */

static void timeline_dumper_task(void *arg)
{
    (void)arg;
    uint32_t last = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t head = atomic_load_explicit(&s_timeline_head,
                                             memory_order_relaxed);
        if (head == last) continue;

        for (uint32_t i = last; i < head; i++) {
            const timeline_entry_t *e =
                &s_timeline[i & (INSTR_TIMELINE_ENTRIES - 1)];
            ESP_LOGI(TAG, "ts=%lld evt=%u task=%u param=%lu",
                     e->ts_us, (unsigned)e->event_id,
                     (unsigned)e->task_id, (unsigned long)e->param);
        }
        last = head;
    }
}

/* ---- Init ---- */

esp_err_t instr_init(void)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << INSTR_GPIO_APP)
                      | (1ULL << INSTR_GPIO_LOAD_A)
                      | (1ULL << INSTR_GPIO_LOAD_B)
                      | (1ULL << INSTR_GPIO_LOAD_C)
                      | (1ULL << INSTR_GPIO_LORA_TX)
                      | (1ULL << INSTR_GPIO_LORA_RX),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));

    BaseType_t ok = xTaskCreate(timeline_dumper_task, "instr_dump",
                                4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    configASSERT(ok == pdPASS);

    ok = xTaskCreate(stats_task, "instr_stats",
                     3072, NULL, tskIDLE_PRIORITY + 1, NULL);
    configASSERT(ok == pdPASS);

    ESP_LOGI(TAG, "instrumentation enabled (timeline=%u entries, %u load tasks)",
             INSTR_TIMELINE_ENTRIES, (unsigned)NUM_LOAD_TASKS);
    return ESP_OK;
}

#endif  /* ENABLE_INSTRUMENTATION */
