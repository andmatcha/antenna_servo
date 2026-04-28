#!/usr/bin/env zsh
set -euo pipefail

# Show printf output emitted through ITM stimulus port 0.

OPENOCD_CMD=(
  openocd -f interface/stlink.cfg -f target/stm32f3x.cfg \
    -c "init" \
    -c "stm32f3x.tpiu configure -protocol uart -traceclk 64000000 -pin-freq 1000000 -output :3344 -formatter off" \
    -c "itm ports on" \
    -c "stm32f3x.tpiu enable" \
    -c "reset run"
)

LOG_FILE="/tmp/openocd_itm_$(date +%Y%m%d_%H%M%S).log"

echo "[run_itm_nc] starting OpenOCD in background... (log: $LOG_FILE)" >&2
"${OPENOCD_CMD[@]}" >"$LOG_FILE" 2>&1 &
OPENOCD_PID=$!

cleanup() {
  if kill -0 "$OPENOCD_PID" 2>/dev/null; then
    echo "[run_itm_nc] stopping OpenOCD (PID: $OPENOCD_PID)" >&2
    kill "$OPENOCD_PID" 2>/dev/null || true
    wait "$OPENOCD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[run_itm_nc] waiting for port 3344..." >&2
for i in {1..100}; do
  if nc -z localhost 3344 2>/dev/null; then
    break
  fi
  sleep 0.1
done

echo "[run_itm_nc] connected: nc localhost 3344" >&2
nc localhost 3344

echo "[run_itm_nc] nc finished; cleaning up." >&2
