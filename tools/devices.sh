# Bench device table (sourced by other scripts). USB serial number = chip MAC.
# busid/COM as of 2026-10-12; MAC is the stable identity.
TX_MAC="98:88:E0:71:A6:44"          # busid 1-2, COM9
RX_MACS=(
    "98:88:E0:76:4D:04"             # busid 2-3, COM15, position C (was B before chip swap)
    "98:88:E0:76:5D:68"             # busid 2-2, COM17, antenna A
    "98:88:E0:76:07:C4"             # busid 2-1, COM16, position B (weak PCB path)
)

dev_path() { echo "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_$1-if00"; }

# idf.py wrapper. The IDF activate script refuses to be sourced from another
# script (it checks $0), so run it in a child bash whose $0 is "bash".
idf() { bash -c 'source ~/.espressif/tools/activate_idf_v6.0.2.sh >/dev/null 2>&1 && idf.py "$@"' bash "$@"; }
