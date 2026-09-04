# TI-licensed files

Flashing and JTAG need a handful of TI-licensed files that cannot be
redistributed, so everything in this directory except this file is gitignored
and you populate it yourself from your own downloads.

## Download

From [ti.com](https://www.ti.com):

- **MMWAVE-L-SDK** (05.05.03.00 used here)
- **Radar Toolbox** (3.00.00.05)
- **UniFlash** or **Code Composer Studio** - only needed for the DSS JTAG
  scripting engine (the `ti/ccs_base` step below).

## Copy into place

Adjust the source paths to match where you installed each package.

```bash
SDK=~/ti/MMWAVE_L_SDK_05_05_03_00
TB=~/ti/radar_toolbox_3_00_00_05

mkdir -p ti/jtag ti/boot ti/rfs

# JTAG: target config + GEL (board init on connect) + TI's QSPI flasher stub
cp $TB/tools/JTAG_Flasher/tool/xwrl6432/{IWRL6432.ccxml,xwrl64xx.gel} ti/jtag/
cp $TB/tools/JTAG_Flasher/prebuilt_binaries/jtag_flasher_xwrL64xx-aop_m4fss0-0_freertos_ti-arm-clang.out ti/jtag/

# appimage build chain + the UART ROM-bootloader flasher (arprog)
cp -r $SDK/tools/boot/{out2rprc,multicoreImageGen,crcMulticoreImageGen,appendBinCrc} ti/boot/
cp $SDK/tools/boot/arprog{,_cmdline}.py ti/boot/

# RFS (radar front-end M3) patch firmware - REQUIRED, bundled into the appimage
cp $SDK/firmware/mmwave_dfp/rfsxWRL6432/mmwave_rfs_patch_rprc.bin ti/rfs/

# DSS JTAG scripting engine: the whole ccs_base tree (~1.1 GB - the debug
# server plus the XDS110/DEBUGSSM drivers the .ccxml needs, and dss.sh).
# From UniFlash it lives under deskdb/content/TICloudAgent/linux/ccs_base;
# a CCS install has an equivalent ccs_base directory.
cp -r ~/ti/uniflash_9.0.0/deskdb/content/TICloudAgent/linux/ccs_base ti/
```

## Expected layout

| Path | Contents |
|------|----------|
| `ti/jtag/` | `.ccxml` target config, GEL, QSPI flasher stub |
| `ti/boot/` | appimage build tools + `arprog` UART flasher |
| `ti/rfs/`  | RFS M3 patch firmware |
| `ti/ccs_base/` | DSS debug server + XDS110/DEBUGSSM drivers |

## Notes

- `dss/run.sh` runs DSS scripts against `ti/ccs_base` (override with `DSS_SH`).
  DSS needs a JRE (`java` on PATH).
- `out2rprc.exe` is a .NET assembly, so `build_appimage.sh` needs `mono`.
- `dss/probe.js` demo mode wants the SDK's prebuilt motion-and-presence demo
  ELF (path at the top of the script).
