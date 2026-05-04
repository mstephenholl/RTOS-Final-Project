#!/usr/bin/env bash
#
# mesh-demo.sh — Two-board mesh networking demo.
#
# Captures serial output from both boards for ${DURATION} seconds,
# tags each line by source board ([A] / [B]), filters to mesh-protocol
# events, and prefixes a relative timestamp [t+SSS]. Demonstrates the
# features the project implements end-to-end on real airtime.
#
# Prerequisites:
#   - Two Heltec WiFi LoRa 32 V3 boards, both flashed with a router-
#     family firmware (router / loadtest / litesleep / etc.). The
#     'leaf' env disables forwarding so this demo is less interesting
#     against a leaf board.
#   - Both boards reachable on serial ports (default /dev/ttyUSB0 +
#     /dev/ttyUSB1).
#   - User in the 'dialout' group (see README.md).
#
# Usage:
#   ./scripts/mesh-demo.sh                           # default 90 s
#   DURATION=180 ./scripts/mesh-demo.sh              # 3 min capture
#   PORT_A=/dev/ttyUSB1 PORT_B=/dev/ttyUSB0 ./scripts/mesh-demo.sh
#
# Interactive demos to try while this script is running:
#   - Press the RST button on either board to see the boot banner +
#     "restored s_tx_seq=N from NVS" line (proves NVS persistence).
#   - Briefly unplug one board to see the other's ACK retries with
#     exponential back-off (2 s → 4 s → 8 s → timeout).

set -euo pipefail

PORT_A="${PORT_A:-/dev/ttyUSB0}"
PORT_B="${PORT_B:-/dev/ttyUSB1}"
DURATION="${DURATION:-90}"

cd "$(dirname "$0")/.."

# ─── Preflight ───────────────────────────────────────────────────────────
[[ -e "$PORT_A" ]] || { echo "ERROR: $PORT_A not present" >&2; exit 1; }
[[ -e "$PORT_B" ]] || { echo "ERROR: $PORT_B not present" >&2; exit 1; }

for P in "$PORT_A" "$PORT_B"; do
    if ! stty -F "$P" 115200 cs8 -cstopb -parenb raw -echo 2>/dev/null; then
        echo "ERROR: cannot configure $P (in use? close any monitor first)" >&2
        exit 1
    fi
done

# ─── Header ──────────────────────────────────────────────────────────────
cat <<EOF

═══════════════════════════════════════════════════════════════════════════
  Mesh Networking Demo — Two-Board Capture
═══════════════════════════════════════════════════════════════════════════

  Capturing serial output from both boards for ${DURATION} s.
  [A] = ${PORT_A}    [B] = ${PORT_B}

  Features demonstrated by real airtime traffic:

  * Identity         each board's MAC + node_id appears in every
                     "Hello from XX:XX:..." broadcast and in the
                     boot banner ("node ID: 0xXX (MAC: ...)").

  * Broadcast        periodic broadcasts every 10 s from each board:
                     "TX bcast: src=XX dst=ALL ttl=4 \"Hello from ...\""

  * Mesh forwarding  each board forwards the other's frames with TTL
                     decremented: "fwd scheduled: ... ttl 3" then
                     "fwd TX queued: ...".

  * Dedup cache      a frame returning to its originator via the
                     forwarder is caught: "dedup drop: #XX seq N".
                     Prevents infinite loops in the flood mesh.

  * Channel hash     out-of-channel frames are rejected at <2 µs by
                     the 1-byte SHA prefix filter; not visible in
                     this demo unless a different-channel transmitter
                     is in range.

  * AES-128-CTR      bodies are encrypted on the wire; plaintext
                     recovery on RX ("RX #XX>YY ... : Hello from ...")
                     is implicit proof. To see ciphertext, use an
                     SDR sniffer.

  * Unicast + ACK    "TX unicast: ... wants_ack" → "ACK TX: ..." →
                     "ACK received from #XX (after 1 send)". With
                     exponential back-off on loss (2/4/8 s, max 3
                     retries) — see the unplug suggestion below.

  * NVS-persisted    "restored s_tx_seq=N from NVS" on boot proves
    sequence         seq survives reboot (replay-safety).

  ─────────────────────────────────────────────────────────
  Try while this is running:
   * Press a board's RST button → fresh boot banner + NVS restore.
   * Briefly unplug one board → see the other's ACK retries fire.
  ─────────────────────────────────────────────────────────

  Each captured line is prefixed [A] or [B] by source board, and
  [t+SSS] by relative seconds since capture start.

EOF

# ─── Capture ─────────────────────────────────────────────────────────────
START=$(date +%s)

# The pipeline:
#   1. Two `cat` processes read the serial ports concurrently. Each is
#      time-bounded by `timeout` so the demo terminates after $DURATION.
#   2. `sed -u` line-tags each board's stream with [A] or [B] and forces
#      line-buffered output (no awaiting block-buffer fill).
#   3. The outer subshell merges both tagged streams.
#   4. `grep --line-buffered` filters to mesh-protocol events.
#   5. `while read` loop computes a fresh per-line timestamp.
#
# We use a bash `while read` loop rather than awk for the timestamp
# step because awk's stdin is stdio block-buffered when reading from
# a pipe — even with sed -u upstream and fflush() in awk, all events
# get the same systime() because they're processed in one burst at
# upstream EOF. Bash `read -r` is line-buffered and gets a fresh
# timestamp per line.
(
    timeout "${DURATION}s" cat "$PORT_A" 2>/dev/null | sed -u 's/^/[A] /' &
    timeout "${DURATION}s" cat "$PORT_B" 2>/dev/null | sed -u 's/^/[B] /' &
    wait
) | grep --line-buffered -E \
        'node ID|TX bcast|RX #|fwd scheduled|fwd TX queued|dedup drop|ACK TX|ACK received|ACK retry|ACK timeout|TX unicast|seq batch|restored s_tx_seq' \
  | while IFS= read -r line; do
        printf '[t+%03d] %s\n' "$(( $(date +%s) - START ))" "$line"
    done

# ─── Footer ──────────────────────────────────────────────────────────────
cat <<'EOF'

═══════════════════════════════════════════════════════════════════════════
  Demo complete. Map of the lines you just saw:

   "TX bcast: src=XX dst=ALL ttl=4 ..."
                       └─ this board originated a broadcast
   "RX #XX>FF seq N ttl T ..."
                       └─ this board received src XX's broadcast,
                          remaining TTL = T
   "fwd scheduled: #XX seq N ... ttl T-1 (in J ms)"
                       └─ this board will forward the frame after
                          J ms of jitter
   "fwd TX queued: ..."
                       └─ forward TX has been handed to the SX1262 driver
   "dedup drop: #XX seq N"
                       └─ this frame's (src, seq) was already in the
                          dedup cache — silently drop, prevents loops
   "TX unicast: src=XX dst=YY wants_ack ..."
                       └─ unicast send with the WACK bit set
   "ACK TX: src=XX seq=M dst=YY (acks #YY seq N)"
                       └─ this board sent an ACK back to YY for seq N
   "ACK received from #XX for seq N (after K send)"
                       └─ the unicast originator got the ACK on send K
                          (K=1 means no retries)
   "ACK retry K/3: #XX seq N"
                       └─ no ACK arrived in time; trying again
                          (back-off doubles each time)
   "ACK timeout: gave up on #XX seq N after K sends"
                       └─ no ACK after retries; give up

  Not exercised by this demo (require extra setup):

   * Encryption visibility — to see ciphertext bytes on the wire,
     instrument sx1262_send to hex-dump the encrypted body, or use
     an SDR + LoRa decoder.
   * Channel-hash isolation — rebuild one board with a different
     CHANNEL_NAME (in src/main.c) and re-run; that board's frames
     will be dropped at <2 µs by the other's hash filter.
   * Role policies — flash one board as `-e leaf` to see it RX
     normally but never forward; flash as `-e gateway` to see no
     origination but full forwarding.

  See NARRATIVE.adoc for the full description of each feature.
═══════════════════════════════════════════════════════════════════════════
EOF
