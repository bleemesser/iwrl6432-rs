# IWRL6432 pure-Rust firmware

Pure-Rust firmware for the Texas Instruments IWRL6432 (xWRL6432) mmWave radar that doesn't link any TI C code. The register and mailbox sequences that bring up the RF front-end are
reproduced from the MMWAVE-L-SDK and used through a modified version of
[abeanater/xwrl64xx-pac](https://github.com/abeanater/xwrl64xx-pac).

The firmware boots the radar, captures raw ADC samples
for every chirp, and streams them out an SPI port in real time. A serial console exposes a CLI for selecting radar profiles and starting/stopping capture.

The goal of this project is to make it easy to extract the raw samples for real-time or deferred processing on the host machine.

## Hardware

You need an **IWRL6432BOOST** EVM and an FTDI
[C232HM-DDHSL-0-2](https://ftdichip.com/products/c232hm-ddhsl-0-2/) MPSSE cable. There is currently NO SUPPORT for streaming anything over UART or performing any signal processing on device.

### Switches

| Switch | Setting | Meaning |
|--------|---------|---------|
| S1 | `100011` | Functional mode + SPI streaming |
| S1 | `100010` | Functional mode, SPI off (USER_LED enabled instead) |
| S1 | `000011` | Flashing mode (set S1.1 to 0 to flash, then back to 1 to boot) |
| S4 | `0010` | All modes |
| S5 | `00` | All modes |

S1 bits are S1.1 through S1.6, left to right. S1.6 selects SPI streaming; when it
is on, the SPI port takes over the USER_LED pad.

### Connections

- Connect the EVM to the computer with a micro-USB cable. Add your user to the `dialout` group for COM port access:

  ```bash
  sudo usermod -aG dialout "$USER"   # log out/in after
  ```

  The `dialout` group covers the COM ports, but the raw USB nodes used by
  the XDS110 JTAG probe (TI DSS) and the FT232H SPI capture cable stay
  `root:root` by default. Install the udev rule to give the `dialout` group
  access to them as well (needed for `dss/run.sh` and `tools/spi_capture.py`):

  ```bash
  sudo ./scripts/install-udev-rules.sh   # then replug the EVM
  ```

- For SPI streaming, wire the C232HM cable to the **J2** 7-pin header. From
  top to bottom (pin 1 through pin 7):

  | Pin | Wire |
  |-----|------|
  | 1 | (unconnected) |
  | 2 | yellow |
  | 3 | green |
  | 4 | brown |
  | 5 | orange |
  | 6 | (unconnected) |
  | 7 | black |

  **Pin 6 must stay unconnected** - any pull-up on it at
  reset seems to prevent the chip from booting on this firmware.

## Prerequisites

- Rust toolchain with the Cortex-M4F target:

  ```bash
  rustup target add thumbv7em-none-eabihf
  ```

- `mono` (to run TI's `out2rprc` when building a flashable image).

  ```bash
  sudo dnf install mono-core # fedora
  sudo apt install mono-complete # debian (probably)
  ```

- A Java runtime on PATH for the DSS JTAG scripting engine, only if you
  use the JTAG dev-load workflow.
- [`uv`](https://docs.astral.sh/uv/) for the Python capture tools in `tools/`.
- TI-licensed files copied into `ti/` - see [`ti/README.md`](ti/README.md).

## Build

```bash
cargo build --release -p iwrl6432-fw
```

The unpackaged ELF lands at `target/thumbv7em-none-eabihf/release/iwrl6432-fw`.

## Flash (standalone boot)

Build a flashable multicore image and write it to QSPI over the UART ROM
bootloader:

Set S1 to flashing mode (S1.1 = 0) and power-cycle before flashing. Make sure to disconnect both UART and SPI cables from the host machine, and reconnect just the UART cable for flashing.

The process is a little finnicky. You may want to stop/disable `ModemManager.service` if you get
'port busy' errors. If arprog asks you to restart the device, Ctrl+C the program, unplug/replug, and try again.

When flashing is done set S1.1 back to 1 and power-cycle to boot from flash.

```bash
bash build_appimage.sh

python3 ti/boot/arprog_cmdline.py -p /dev/ttyACM0 -f appimage/fw.appimage -s SFLASH -t META_IMAGE1
```

## JTAG dev load (optional)

For a fast dev loop the firmware can be loaded straight into RAM over
JTAG (gone on power cycle). This needs TI's DSS and the files under `ti/jtag/`:

```bash
dss/run.sh load_fw.js # will automatically grab the binary from target/ -- build first
```

## Usage

### Console / CLI

The serial console appears as `/dev/ttyACM0` at 115200 8N1. Open it with any
terminal, for example:

```bash
picocom -b 115200 -f n /dev/ttyACM0 # exit with Ctrl+A-X
```

Type `help` for the command list: inspect preset radar profiles and their
derived metrics, start/stop capture, live-reconfigure between profiles, and
build custom configs with `set`. Which profile boots (and whether capture
auto-starts) is selected in `crates/iwrl6432-fw/src/profiles.rs`.

### SPI streaming

Raw ADC chirps stream out the SPI port to the J2 header. Capture them from the host with the FTDI cable:

```bash
cd tools
uv sync --extra gpu # for cuda
uv run spi_capture.py
```

Some profiles subsample to fit within the link bandwidth. Reconstruct full range spectra with:

```bash
cd tools
uv run cs_reconstruct.py
```

The protocol is implemented in `crates/iwrl6432-fw/src/stream.rs`. The stream is
self-describing: an `A55E` metadata frame (~1/s) carries the sample count, RX,
format and RF params, so the host tools label axes in meters/m·s without being
told the profile.

### Live radar scope

`radar_scope.py` turns the stream into live radar products: a range-azimuth PPI
with detected blips (TDMA-MIMO angle off the 2 TX × 3 RX virtual array),
range-Doppler, range profile, range-time waterfall, micro-Doppler spectrogram
and a birds-eye point cloud. Sub12 (CS) profiles are L1-reconstructed on the
fly, using a CUDA GPU (CuPy) automatically if present.

```bash
cd tools
uv run radar_scope.py                 # live over the FTDI SPI link
uv run radar_scope.py --replay capture.bin   # offline, from a saved capture
uv run radar_scope.py --cpu           # force NumPy
```

`radar_dsp.py` holds the pipeline and can be run standalone over a capture for a
text detection dump (`uv run radar_dsp.py capture.bin`). Live Sub12 profiles
want a GPU; on CPU they run best via `--replay` or the smaller Packed12 profiles
stream in real time.

## License

MIT, see [LICENSE](LICENSE).

`crates/xwrl64xx-pac` is a fork of
[abeanater/xwrl64xx-pac](https://github.com/abeanater/xwrl64xx-pac) and keeps
its own MIT license and copyright notice at
[`crates/xwrl64xx-pac/LICENSE`](crates/xwrl64xx-pac/LICENSE).

No TI source or binaries are redistributed here. The register writes and
mailbox sequences that bring up the RF front-end were derived by reading the
MMWAVE-L-SDK and its documentation; the register addresses and field layouts
themselves come from TI's public targetdb XML by way of the PAC. Everything TI
ships that this project needs at flash time is fetched from ti.com by you under
TI's own license, into the gitignored `ti/` directory (see
[`ti/README.md`](ti/README.md)).

This project is not affiliated with or endorsed by Texas Instruments.
