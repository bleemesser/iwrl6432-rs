#!/usr/bin/env bash
# Build a flashable multicore .appimage from the pure-Rust M4 firmware + TI's RFS
# patch. Mirrors the SDK's non-auth bootimage recipe
# (out2rprc -> MulticoreImageGen -> crc -> appendBinCrc). Flash it with
# dss/flash_appimage.js.
#
# Usage:  bash build_appimage.sh [path/to/elf]
set -euo pipefail

FW="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TI="$FW/ti"   # SDK-sourced tools + RFS patch
ELF="${1:-$FW/target/thumbv7em-none-eabihf/release/iwrl6432-fw}"
OUT="$FW/appimage"
RFS="$TI/rfs/mmwave_rfs_patch_rprc.bin"

# out2rprc.exe is a .NET assembly; run under mono on Linux.
OUT2RPRC="mono $TI/boot/out2rprc/out2rprc.exe"
MCIG="$TI/boot/multicoreImageGen/MulticoreImageGen"
CRCGEN="$TI/boot/crcMulticoreImageGen/crcMulticoreimage"
APPENDCRC="$TI/boot/appendBinCrc/appendBinCrc"

# APPSS/M4 core id, FECSS/RFS core id, M4 vector-table base (see linker layout).
APPS_FLAGS=0x35510000
RFS_FLAGS=0xb5510000
BOOT_VECTOR=0x400000

RPRC="$OUT/fw_rprc.bin"
IMG="$OUT/fw.appimage"
TMP="$OUT/fw_temp.appimage"

mkdir -p "$OUT"
echo "[1/4] out2rprc: $(basename "$ELF") -> $(basename "$RPRC")"
$OUT2RPRC "$ELF" "$RPRC" >/dev/null
echo "[2/4] MulticoreImageGen: + RFS patch -> $(basename "$IMG")"
"$MCIG" LE 0 "$BOOT_VECTOR" 0 "$IMG" "$APPS_FLAGS" "$RPRC" "$RFS_FLAGS" "$RFS" >/dev/null
echo "[3/4] crcMulticoreimage"
"$CRCGEN" "$IMG" "$TMP" >/dev/null
echo "[4/4] appendBinCrc"
"$APPENDCRC" "$IMG" | grep -iE "crc|bytes" || true
rm -f "$TMP" "$RPRC"
echo "built: $IMG ($(stat -c%s "$IMG") bytes)"
