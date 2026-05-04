# RTOS Final Project

A clean-room SX1262 LoRa driver, FreeRTOS-integrated mesh stack, and
load-characterization harness for the Heltec WiFi LoRa 32 V3 board.
The headline experiment characterizes the conditions under which radio
transmission breaks down — combined RX rate and CPU contention.
Full project writeup in [NARRATIVE.adoc](NARRATIVE.adoc).

## Hardware

- 1× or 2× **Heltec WiFi LoRa 32 V3** board (ESP32-S3 + SX1262 + SSD1306 OLED).
- USB-C cable (data, not power-only).
- For two-board validation, a second board on a separate USB port.
- 915 MHz wire-whip antenna (US ISM band — the radio is configured for
  915 MHz; rebuild with a different `SX1262_FREQ` for EU 868 MHz or
  other regions).

## Quick start

```bash
git clone <repo-url>
cd RTOS-Final-Project
pio run -t upload -t monitor
```

PlatformIO will fetch ESP-IDF 5.5.3 and the ESP32-S3 toolchain on
the first build (~5-10 minutes, one time).

## Prerequisites (Ubuntu 24.04)

### 1. System packages

```bash
sudo apt update
sudo apt install -y git python3 python3-venv pipx
```

### 2. Serial port access

The board appears as `/dev/ttyUSB0` (or higher). Add yourself to the
`dialout` group so non-root processes can open it:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in (or run `newgrp dialout`) for the group to take effect.
```

### 3. PlatformIO

Install via `pipx` to isolate it from system Python:

```bash
pipx install platformio
pipx ensurepath
# Restart your shell, or: source ~/.bashrc
```

Verify:

```bash
pio --version
# PlatformIO Core, version 6.x
```

The first `pio run` automatically downloads ESP-IDF, the
`xtensa-esp-elf-gcc` toolchain, and project dependencies. No manual
install of ESP-IDF or the cross-compiler is required.

### 4. Verify hardware

Plug in a Heltec V3 over USB-C and check enumeration:

```bash
pio device list
# Should list:
#   /dev/ttyUSB0
#   ------------
#   Hardware ID: USB VID:PID=10C4:EA60 ...
#   Description: CP2102 USB to UART Bridge Controller
```

If the device doesn't appear:

- Confirm the USB-C cable carries data (some are charge-only).
- Confirm you've logged out / back in after `usermod`.
- Run `dmesg | tail` after plugging in to see the kernel's
  enumeration messages.
- ModemManager occasionally grabs `/dev/ttyUSB*` and probes it
  (you'd see the connection drop after ~5 s). If that happens:
  `sudo systemctl mask ModemManager.service`.

### 5. Optional — AsciiDoc rendering

Required only to regenerate `NARRATIVE.html` from the source:

```bash
sudo apt install -y ruby-asciidoctor ruby-rouge
asciidoctor NARRATIVE.adoc -o NARRATIVE.html
```

GitHub renders `NARRATIVE.adoc` natively; this step is for local
preview.

## Build environments

Six distinct binaries from one source tree, selected via PlatformIO
environments:

| Env | Description | Use |
|---|---|---|
| `router` | always-on relay (default) | development, interop |
| `leaf` | sleeps between TX, no forwarding | battery endpoint nodes |
| `gateway` | always-on observer, no origination | data sink |
| `litesleep` | router + CPU light sleep | low-power relay |
| `deepsleep` | beacon-then-RTC-sleep | battery sensor nodes |
| `loadtest` | router + instrumentation harness | the experiment |

Build and flash any of them:

```bash
pio run -e <env> -t upload -t monitor
```

The `loadtest` environment is what `NARRATIVE.adoc` documents.

## Running the live demo

`scripts/demo.sh` walks one board through six configurations,
demonstrating the experiment's findings (cliff at 26 Hz, OLED bypass
removes it, post-bypass capacity, failure isolation under CPU
overload). Single-board only; takes about 3-4 minutes:

```bash
./scripts/demo.sh
```

Useful overrides:

```bash
WATCH_SECONDS=15 ./scripts/demo.sh    # longer stats window per phase
PORT=/dev/ttyUSB1 ./scripts/demo.sh   # target a specific board
```

The demo edits `src/main.c` and `src/instrumentation.c` in place
between phases and restores them on exit (including on Ctrl+C). The
working tree must be clean before the demo runs.

## Two-board test

For the airtime confirmation experiment, flash both boards with the
same firmware:

```bash
pio run -e loadtest -t upload --upload-port /dev/ttyUSB0
pio run -e loadtest -t upload --upload-port /dev/ttyUSB1
```

Then open two shell windows:

```bash
# Window A
pio device monitor --port /dev/ttyUSB0

# Window B
pio device monitor --port /dev/ttyUSB1
```

Each board should hear the other's broadcasts, render OLED for them
(~37-39 ms per render — the validation data point), and exchange ACKs
end-to-end.

## Project layout

```
src/
  main.c                    mesh + role policy + sleep variants + injector
  instrumentation.{c,h}     load-test harness (loadtest env only)
components/
  sx1262/                   hand-rolled SX1262 LoRa driver
  ssd1306/                  hand-rolled SSD1306 OLED driver
sdkconfig.defaults          ESP-IDF Kconfig baseline
sdkconfig.<env>             per-env overrides
platformio.ini              six build envs
scripts/
  demo.sh                   live-demo controller
NARRATIVE.adoc              project narrative + experimental findings
README.md                   this file
```

## Further reading

- **[NARRATIVE.adoc](NARRATIVE.adoc)** — full narrative with problem
  statement, background concepts, the seven-phase experiment evolution,
  findings, generalization to sensor-driven applications, limitations,
  and reference URLs.
- **[scripts/demo.sh](scripts/demo.sh)** — the live-demo script with
  phase-by-phase predictions and configuration knobs.
