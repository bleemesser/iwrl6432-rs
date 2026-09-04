#!/usr/bin/env bash
# Grant non-root access to the EVM's USB debug/data devices.
#
# The dialout group covers the CDC/UART COM ports, but the raw USB nodes
# (/dev/bus/usb/*) that TI's DSS uses to drive the XDS110 JTAG probe, and
# that pyftdi uses for the FT232H SPI capture, stay root:root by default.
# This installs a udev rule assigning them to the `dialout` group so members
# can read/write without sudo.
#
# Run once: sudo ./scripts/install-udev-rules.sh   (then replug the EVM)

set -euo pipefail

RULES=/etc/udev/rules.d/99-iwrl6432.rules
GROUP=dialout

if [[ $EUID -ne 0 ]]; then
  echo "run as root: sudo $0" >&2
  exit 1
fi

cat > "$RULES" <<EOF
# IWRL6432 EVM USB debug/data devices -> $GROUP group (installed by
# scripts/install-udev-rules.sh)
# XDS110 JTAG debug probe (TI DSS)
SUBSYSTEM=="usb", ATTR{idVendor}=="0451", ATTR{idProduct}=="bef3", MODE="0660", GROUP="$GROUP"
# CP2105 dual USB-UART (console / backchannel)
SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea70", MODE="0660", GROUP="$GROUP"
# FT232H (C232HM SPI capture cable, pyftdi)
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", MODE="0660", GROUP="$GROUP"
EOF

udevadm control --reload-rules
udevadm trigger

echo "installed $RULES"
echo "replug the EVM (and ensure your user is in the '$GROUP' group)"
