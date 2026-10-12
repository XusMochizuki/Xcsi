#!/usr/bin/env bash
# Build csi_recv once and flash it to every receiver (or the MACs given).
#   tools/flash_rx.sh
#   tools/flash_rx.sh 98:88:E0:76:4D:04
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/tools/devices.sh"

macs=("$@")
[[ ${#macs[@]} -eq 0 ]] && macs=("${RX_MACS[@]}")

cd "$ROOT/firmware/csi_recv"
idf build >/dev/null
for mac in "${macs[@]}"; do
    port="$(dev_path "$mac")"
    echo "== flash $mac ($port)"
    # USB re-enumerates on reset; usb_attach.sh brings the port back.
    idf -p "$port" flash >/dev/null
done
echo "done"
