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
    gsub(/^\047|\047$/, "", value)
    print value
    exit
  }
' "$ETHERLAB_CONFIG")"

# Strip optional double quotes without relying on awk's non-portable \" regex
# escape (mawk warns about it).
master_device="${master_device#\"}"
master_device="${master_device%\"}"

[[ -n "$master_device" ]] || {
  echo "MASTER0_DEVICE is not configured in $ETHERLAB_CONFIG" >&2
  exit 1
}

if [[ -e "/sys/class/net/$master_device" ]]; then
  printf '%s\n' "$master_device"
  exit 0
fi

# EtherLab's broadcast address means "accept the first device offered by a
# driver". It cannot be matched to a NIC MAC, so infer the interface from
# physical, non-wireless Linux devices. Prefer the sole link with carrier;
# otherwise accept the sole wired device. Ambiguous machines fail closed.
if [[ "${master_device,,}" == "ff:ff:ff:ff:ff:ff" ]]; then
  wired=()
  wired_with_carrier=()
  for net_path in /sys/class/net/*; do
    iface="${net_path##*/}"
    [[ "$iface" != "lo" ]] || continue
    [[ -e "$net_path/device" ]] || continue
    [[ ! -d "$net_path/wireless" ]] || continue
    wired+=("$iface")
    if [[ -r "$net_path/carrier" ]] && [[ "$(<"$net_path/carrier")" == "1" ]]; then
      wired_with_carrier+=("$iface")
    fi
  done

  if [[ ${#wired_with_carrier[@]} -eq 1 ]]; then
    printf '%s\n' "${wired_with_carrier[0]}"
    exit 0
  fi
  if [[ ${#wired[@]} -eq 1 ]]; then
    printf '%s\n' "${wired[0]}"
    exit 0
  fi

  echo "Cannot uniquely resolve EtherCAT interface from MASTER0_DEVICE=$master_device" >&2
  echo "Physical wired candidates: ${wired[*]:-none}; with carrier: ${wired_with_carrier[*]:-none}" >&2
  echo "Set MASTER0_DEVICE to an interface name/MAC, or set ZEROERR_ETHERCAT_IFACE explicitly." >&2
  exit 1
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
