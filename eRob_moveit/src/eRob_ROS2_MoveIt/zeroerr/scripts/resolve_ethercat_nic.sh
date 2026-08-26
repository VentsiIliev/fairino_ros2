#!/usr/bin/env bash
# Resolve the Linux interface used by the EtherLab master.
# The EtherLab MAC configuration is the portable source of truth; interface
# names are allowed as an explicit override or when MASTER0_DEVICE names one.

set -euo pipefail

ETHERLAB_CONFIG="${ZEROERR_ETHERCAT_CONFIG:-/usr/local/etherlab/etc/sysconfig/ethercat}"
requested="${ZEROERR_ETHERCAT_IFACE:-${ZEROERR_NIC:-}}"

if [[ -n "$requested" ]]; then
  [[ -e "/sys/class/net/$requested" ]] || {
    echo "EtherCAT interface '$requested' does not exist" >&2
    exit 1
  }
  printf '%s\n' "$requested"
  exit 0
fi

[[ -r "$ETHERLAB_CONFIG" ]] || {
  echo "EtherLab configuration not readable: $ETHERLAB_CONFIG" >&2
  exit 1
}

master_device="$(awk -F= '
  /^[[:space:]]*MASTER0_DEVICE[[:space:]]*=/ {
    value=$2
    sub(/[[:space:]]*#.*/, "", value)
    gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
    gsub(/^\"|\"$/, "", value)
    gsub(/^\047|\047$/, "", value)
    print value
    exit
  }
' "$ETHERLAB_CONFIG")"

[[ -n "$master_device" ]] || {
  echo "MASTER0_DEVICE is not configured in $ETHERLAB_CONFIG" >&2
  exit 1
}

if [[ -e "/sys/class/net/$master_device" ]]; then
  printf '%s\n' "$master_device"
  exit 0
fi

needle="${master_device,,}"
for address_file in /sys/class/net/*/address; do
  [[ -r "$address_file" ]] || continue
  address="$(tr '[:upper:]' '[:lower:]' < "$address_file")"
  address="${address//$'\n'/}"
  if [[ "$address" == "$needle" ]]; then
    basename "$(dirname "$address_file")"
    exit 0
  fi
done

echo "No Linux interface matches EtherCAT MASTER0_DEVICE=$master_device" >&2
exit 1
