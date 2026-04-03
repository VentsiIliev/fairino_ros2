#!/usr/bin/env bash
set -euo pipefail

MONITOR_PID_FILE="${MONITOR_PID_FILE:-}"

# Write bash's own PID into the file so the parent can kill the right process.
# gnome-terminal's $! is the transient client launcher (exits immediately), so
# recording $! is unreliable for that terminal family.
LOOP_BODY='trap "exit 0" INT TERM; while true; do clear; ethercat slaves; sleep 1; done'
if [[ -n "${MONITOR_PID_FILE}" ]]; then
  LOOP="echo \$\$ > $(printf '%q' "${MONITOR_PID_FILE}"); ${LOOP_BODY}"
else
  LOOP="${LOOP_BODY}"
fi

if command -v gnome-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  # --wait keeps gnome-terminal alive until the window is closed, but we rely
  # on the inner bash writing its own PID, so killing the bash process is
  # enough — the terminal will close when its child exits.
  gnome-terminal --wait -- bash -c "$LOOP" &
elif command -v konsole >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  konsole --noclose -e bash -c "$LOOP" &
elif command -v xfce4-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  xfce4-terminal --hold -e "bash -c '$LOOP'" &
elif command -v mate-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  mate-terminal -- bash -c "$LOOP" &
elif command -v xterm >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  xterm -hold -e bash -c "$LOOP" &
else
  echo "No supported graphical terminal found - slave monitor not opened."
fi
