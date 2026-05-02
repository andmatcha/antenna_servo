#!/usr/bin/env zsh
set -euo pipefail

# Show printf output emitted on USART2 TX through the ST-LINK VCP.
# Pass a port as the first argument, or set PORT/BAUD in the environment.

PIO="${PIO:-pio}"
BAUD="${BAUD:-115200}"
PORT="${1:-${PORT:-}}"

if [[ -z "$PORT" ]]; then
  for candidate in /dev/cu.usbmodem*(N) /dev/tty.usbmodem*(N) /dev/ttyACM*(N); do
    PORT="$candidate"
    break
  done
fi

if [[ -z "$PORT" ]]; then
  echo "[serial_log] ST-LINK VCP serial port was not found." >&2
  echo "[serial_log] Available ports:" >&2
  "$PIO" device list >&2 || true
  exit 1
fi

echo "[serial_log] reading USART2/VCP logs on $PORT at ${BAUD} baud" >&2
echo "[serial_log] use make upload-debug first so DEBUG_LOG_ENABLED is set" >&2
"$PIO" device monitor --port "$PORT" --baud "$BAUD" --filter direct
