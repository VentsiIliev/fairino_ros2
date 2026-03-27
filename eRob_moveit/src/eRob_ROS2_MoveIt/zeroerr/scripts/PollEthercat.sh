#!/usr/bin/env bash
set -euo pipefail

MONITOR_PID_FILE="${MONITOR_PID_FILE:-}"
LOOP='trap "exit 0" INT TERM; while true; do clear; ethercat slaves; sleep 1; done'

record_pid() {
  local pid="$1"
  if [[ -n "${MONITOR_PID_FILE}" ]]; then
    printf '%s\n' "$pid" > "${MONITOR_PID_FILE}"
  fi
}

if command -v gnome-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  gnome-terminal -- bash -c "$LOOP" &
  record_pid $!
elif command -v konsole >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  konsole --noclose -e bash -c "$LOOP" &
  record_pid $!
elif command -v xfce4-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  xfce4-terminal --hold -e "bash -c '$LOOP'" &
  record_pid $!
elif command -v mate-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  mate-terminal -- bash -c "$LOOP" &
  record_pid $!
elif command -v xterm >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  xterm -hold -e bash -c "$LOOP" &
  record_pid $!
else
  echo "No supported graphical terminal found - slave monitor not opened."
fi
