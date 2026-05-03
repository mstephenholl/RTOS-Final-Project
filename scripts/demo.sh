#!/usr/bin/env bash
#
# demo.sh — Live demo of the RTOS-Final-Project radio breakdown findings.
#
# Walks one board through six configurations to demonstrate:
#   - Phase 2 vs 3:  the 26 Hz OLED-bound cliff
#   - Phase 3 vs 4:  the OLED is the cause (bypass removes the cliff)
#   - Phase 5:       post-bypass capacity (200 Hz easy)
#   - Phase 6:       subsystem isolation under CPU overload
#
# Single-board demo. For richer presentation, open a SECOND shell window
# alongside this one and run:
#
#     pio device monitor --port /dev/ttyUSB0
#
# That window shows the full unfiltered serial output; this controller
# narrates each phase and shows a filtered view of the same stream.
#
# Usage:
#   ./scripts/demo.sh                   # uses /dev/ttyUSB0, 30 s per phase
#   PORT=/dev/ttyUSB1 ./scripts/demo.sh
#   WATCH_SECONDS=20 ./scripts/demo.sh

set -euo pipefail

# ─── Config ──────────────────────────────────────────────────────────────
PORT="${PORT:-/dev/ttyUSB0}"
ENV="loadtest"
WATCH_SECONDS="${WATCH_SECONDS:-30}"

# Move to project root so sed paths resolve regardless of where this is run.
cd "$(dirname "$0")/.."

# ─── Sanity checks ───────────────────────────────────────────────────────
preflight() {
    command -v pio >/dev/null \
        || { echo "ERROR: PlatformIO 'pio' not in PATH" >&2; exit 1; }
    [[ -e "$PORT" ]] \
        || { echo "ERROR: serial port $PORT not present" >&2; exit 1; }

    if ! git diff --quiet src/main.c src/instrumentation.c 2>/dev/null; then
        cat >&2 <<EOF
ERROR: src/main.c or src/instrumentation.c has uncommitted changes.

The demo edits these files in place between phases. To proceed safely:
  - commit your changes, or
  - 'git checkout src/main.c src/instrumentation.c' to discard them.
EOF
        exit 1
    fi
}

# ─── Knob editors ────────────────────────────────────────────────────────
# Each function rewrites one #define / one s_load_cfgs[] entry in place.
# All three knobs have unique anchors in the source so sed is unambiguous.

set_inj_rate() {
    sed -i -E "s|^(#define INJECTOR_RATE_HZ +)[0-9]+|\\1${1}|" src/main.c
}

set_bypass() {
    sed -i -E "s|^(#define INJECTOR_BYPASS_OLED +)[01]|\\1${1}|" src/main.c
}

set_load_c() {
    sed -i -E "s|(\"load_c\".*100, +)[0-9]+( +},)|\\1${1}\\2|" src/instrumentation.c
}

# ─── Cleanup on exit ─────────────────────────────────────────────────────
DEMO_STARTED=0
cleanup() {
    [[ "$DEMO_STARTED" -eq 0 ]] && return
    echo
    echo "─── Restoring source defaults ──────────────────────────────"
    set_inj_rate 0
    set_bypass 1
    set_load_c 30000
    echo "    Re-flashing board with default config (silent)..."
    pio run -e "$ENV" -t upload --silent > /dev/null 2>&1 || true
    echo "    Done. Working tree is back to its committed state."
}
trap cleanup EXIT INT TERM

# ─── One demo phase: edit, build, flash, stream ──────────────────────────
phase() {
    local n="$1" title="$2" description="$3" rate="$4" bypass="$5" load_c_us="$6"
    local watch="${7:-$WATCH_SECONDS}"

    cat <<EOF

═══════════════════════════════════════════════════════════════════════════
  PHASE $n: $title
═══════════════════════════════════════════════════════════════════════════

  $description

  Config:
    INJECTOR_RATE_HZ      $rate
    INJECTOR_BYPASS_OLED  $bypass
    load_c WCET (us)      $load_c_us

EOF

    set_inj_rate "$rate"
    set_bypass "$bypass"
    set_load_c "$load_c_us"
    DEMO_STARTED=1

    echo "  Building & flashing..."
    if ! pio run -e "$ENV" -t upload --silent >/tmp/demo-flash.log 2>&1; then
        echo "  ERROR: build/flash failed; see /tmp/demo-flash.log" >&2
        cat /tmp/demo-flash.log >&2
        return 1
    fi

    echo "  Streaming serial for ${watch} s..."
    echo "  ─────────────────────────────────────────────────────────"
    # Stream pio device monitor for `watch` seconds, filter for the
    # interesting lines, drop the high-frequency timeline-ring dumps.
    timeout "${watch}s" pio device monitor --port "$PORT" 2>/dev/null \
        | awk '
            /ts=[0-9]+ evt=/ { next }
            /instr:.*(stats|load_[abc]:|radio:|parse|render|misses)/ { print; fflush(); next }
            /task_wdt/ { print; fflush(); next }
            /app: TX (bcast|unicast)/ { print; fflush(); next }
        ' \
        || true
    echo "  ─────────────────────────────────────────────────────────"
    echo

    read -rp "  Press Enter for next phase (Ctrl+C to abort)..." _
}

# ─── Main flow ───────────────────────────────────────────────────────────
preflight

cat <<'EOF'

═══════════════════════════════════════════════════════════════════════════
  RTOS Final Project — Live Demo
═══════════════════════════════════════════════════════════════════════════

  Focal question:  at what loads does radio transmission start to break?

  This demo walks the board through six configurations. Each phase
  shows: a description of what is being tested, the configuration
  applied, and a filtered ~30 s serial stream from the board.

  Tip: open a second shell window and run

      pio device monitor --port /dev/ttyUSB0

  to see the full unfiltered output alongside the filtered narration
  in this window.

EOF

read -rp "  Press Enter to begin..." _

phase 1 "Baseline (rate 0, default load)" \
"The system at rest. Periodic broadcasts every 10 s; no synthetic RX events;
  load tasks at default 75% CPU demand. Reference frame for what stable looks
  like. Expect: 0 misses, 0 drops, 0 renders, parse-latency stats empty (n=0)." \
        0 1 30000 15

phase 2 "Below the cliff (25 Hz, OLED render active)" \
"Synthetic RX at 25 Hz. Each event renders the OLED (~39 ms blocking I2C).
  Inter-arrival 40 ms vs render 39 ms = 1 ms of slack per event.
  Predicted: stable, 0% drops, parse latency a few ms avg, max ~60 ms (slack
  edge cases). The system runs RIGHT at the cliff but doesn't fall." \
        25 0 30000

phase 3 "Above the cliff (30 Hz, OLED render active)" \
"Same as phase 2 but +5 Hz. Inter-arrival 33 ms now BELOW render 39 ms.
  Predicted: ~15% drops, parse-latency avg JUMPS to ~138 ms (queue saturated).
  This is the dramatic phase change. Watch the radio: drop count starts
  climbing immediately, by ~5 per stats window." \
        30 0 30000

phase 4 "Bypass enabled (30 Hz, OLED skipped for synthetic)" \
"Same RX rate as phase 3, but synthetic events bypass render_rx_event.
  The OLED bottleneck is removed.
  Predicted: drops back to 0%, latency back to sub-ms avg, oled_render n=0
  in stats. Confirms the OLED was the entire cause of phase 3's failures." \
        30 1 30000

phase 5 "High rate with bypass (200 Hz)" \
"10x the rate of phase 3. With OLED out of the path, the system handles it.
  Predicted: ~0.1% drops (residual TX-path OLED render on app_task);
  latency ~0.7 ms avg, ~40 ms max during broadcast TX events." \
        200 1 30000

phase 6 "CPU overload (load_c saturated)" \
"Same 200 Hz RX, but load_c WCET is now 100 ms on a 100 ms period =
  100% utilization for that single task.
  Predicted:
    - load_c misses 100% of deadlines (every single one)
    - task_wdt: IDLE1 fires repeatedly (CPU 1's idle starves)
    - load_a sees small misses (~0.1%), load_b stays clean
    - RADIO drops STAY at ~0.3-0.4% — failure isolated to load_c." \
        200 1 100000

cat <<'EOF'

═══════════════════════════════════════════════════════════════════════════
  Demo complete. What you saw:

    Phase 2 → 3:  the 26 Hz cliff in the OLED-bound regime
                  (0% drops at 25 Hz → 15% at 30 Hz with one rebuild)
    Phase 3 → 4:  the OLED is the cause; bypass removes the cliff
                  (15% → 0% drops, no other change)
    Phase 5:      post-bypass capacity is much higher (200 Hz easy)
    Phase 6:      single-task saturation breaks only that task;
                  radio remains protected (~0.4% drops while load_c
                  misses 100% and the watchdog fires)

  The closed-form takeaway:
    cliff_rate = 1000 / blocking_io_ms
    drop_rate  = (rate − cliff_rate) / rate    (above the cliff)

  Generalizes to any sensor with read time T ms: ceiling = 1000/T Hz.

EOF
