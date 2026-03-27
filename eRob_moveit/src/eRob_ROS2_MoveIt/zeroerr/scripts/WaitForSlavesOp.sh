#!/usr/bin/env bash
set -euo pipefail

EXPECTED_SLAVES="${EXPECTED_SLAVES:-6}"
REQUIRED_STABLE_POLLS="${REQUIRED_STABLE_POLLS:-3}"
POLL_INTERVAL="${POLL_INTERVAL:-1}"
ETHERCAT_BIN="${ETHERCAT_BIN:-ethercat}"

stable_polls=0

echo "Waiting for ${EXPECTED_SLAVES} EtherCAT slaves to reach OP..."

while true; do
  if ! output="$(${ETHERCAT_BIN} slaves 2>/dev/null)"; then
    stable_polls=0
    sleep "${POLL_INTERVAL}"
    continue
  fi

  slave_lines="$(printf '%s\n' "${output}" | grep -E '^[[:space:]]*[0-9]+[[:space:]]' || true)"
  total_count="$(printf '%s\n' "${slave_lines}" | sed '/^$/d' | wc -l | tr -d ' ')"
  op_count="$(printf '%s\n' "${slave_lines}" | grep -c ' OP ' || true)"

  if [[ "${total_count}" -eq "${EXPECTED_SLAVES}" && "${op_count}" -eq "${EXPECTED_SLAVES}" ]]; then
    stable_polls=$((stable_polls + 1))
    if [[ "${stable_polls}" -ge "${REQUIRED_STABLE_POLLS}" ]]; then
      echo "All ${EXPECTED_SLAVES} EtherCAT slaves are in OP."
      exit 0
    fi
  else
    stable_polls=0
  fi

  sleep "${POLL_INTERVAL}"
done
