#!/usr/bin/env bash
# Keep ESP32-C6 USB-Serial-JTAG devices attached to WSL across resets.
# Every flash/reset re-enumerates the device; usbipd --auto-attach re-attaches it.
#
#   tools/usb_attach.sh            # default: 1-2 (TX, COM9)
#   tools/usb_attach.sh 2-1 2-2 2-3
#   tools/usb_attach.sh stop       # stop all auto-attach loops started here
#
# Each busid must already be shared once from an admin shell on Windows:
#   usbipd bind --busid <busid>     (or: usbipd bind --force --busid <busid>)
set -euo pipefail

LOG_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/esp-csi"
mkdir -p "$LOG_DIR"

if [[ "${1:-}" == "stop" ]]; then
    pkill -f "usbipd.exe attach --wsl --busid .* --auto-attach" && echo "stopped" || echo "none running"
    exit 0
fi

busids=("$@")
[[ ${#busids[@]} -eq 0 ]] && busids=(1-2)

for b in "${busids[@]}"; do
    if pgrep -f "usbipd.exe attach --wsl --busid $b --auto-attach" >/dev/null; then
        echo "$b: auto-attach already running"
        continue
    fi
    nohup usbipd.exe attach --wsl --busid "$b" --auto-attach >"$LOG_DIR/usbipd-$b.log" 2>&1 &
    echo "$b: auto-attach started (log: $LOG_DIR/usbipd-$b.log)"
done
