"""Capture the raw-ADC SPI stream from the IWRL6432 firmware.

Poll protocol (no handshake GPIO - J2 pin 6 / HOST_INTR is on the SYNC_OUT boot
strap and must stay unwired):
  1. master writes the 4-byte poll magic C0 FF EE 01
  2. master reads an 8-byte status header:  5A A5 | total (u16 BE) | first_seq (u32 BE)
  3. if total > 0, master reads `total` bytes: a batch of per-chirp chunks,
     each  magic (u16 BE) | len (u16 BE) | seq (u32 BE)  + len payload bytes
     (payload = RX0..RXn contiguous). The magic names the sample encoding:
       A5 5C  12-bit packed, 2 samples -> 3 bytes (see unpack12)
       A5 5B  Bfp16 block floating point: 17-byte blocks of 1 shift
              exponent + 16 int8 mantissas (see unpack_bfp), semi-lossless
       A5 6k  Sub12: k of every 16 sample pairs kept (random subsample for
              CS reconstruction), kept pairs 12-bit packed; pair indices are
              regenerated from seq (mask_indices, mirrors the firmware)

Wiring: only MOSI (yellow), MISO (green), CS (brown), CLK (orange), GND (black).

Clock ceiling: 20 MHz is the working maximum (default). 25 and 30 MHz fail
outright (every status header bad, 0 chirps; sensor on the host, flying
leads, tested 2026-07-11) - not marginal corruption, the slave never frames.

Usage (deps come from the uv env in this directory):
  uv run spi_capture.py --seconds 5 [--out capture.bin] [--freq 15e6]
                        [--poll-ms 5] [--stats]
"""

import argparse
import struct
import sys
import time

import numpy as np

FTDI_URL = "ftdi://ftdi:232h/1"
CMD_POLL = bytes.fromhex("C0FFEE01")
STATUS_MAGIC = 0x5AA5


class SpiLink:
    """Batched-MPSSE poll transport, shared by this tool and radar_scope.

    pyftdi's SpiController costs one USB round trip per write/read call, so
    the poll cycle (poll write + response-prime wait + header read + batch
    read) burned 3+ round trips and a fixed sleep - most of the cycle at 9 KB
    batches. The MPSSE engine executes a command stream sequentially from its
    buffer, so poll + spacing + header read are queued in ONE bulk write:

      [CS low, clock out C0FFEE01, CS high]     the poll
      [0x8F: ~60 us of dummy clocks, CS high]   spacing while the firmware SPI
                                                ISR resets + primes (~25 us);
                                                CS is high, the slave ignores
      [CS low, clock in 8 bytes, CS high, 0x87] status header

    Wire pins (ADBUS): 0=SCLK out, 1=MOSI out, 2=MISO in, 3=CS out; SPI
    mode 0, MSB first (out on -ve edge 0x11, in on +ve edge 0x20).
    """

    _CS_LO = bytes([0x80, 0x00, 0x0B])
    _CS_HI = bytes([0x80, 0x08, 0x0B])

    def __init__(self, freq=20e6, url=FTDI_URL):
        from pyftdi.ftdi import Ftdi

        self.ftdi = Ftdi()
        # latency=1 ms: the FTDI flushes partial USB packets to the host at
        # this cadence; the default 16 ms dominates small-read latency.
        self.freq = self.ftdi.open_mpsse_from_url(
            url, direction=0x0B, initial=0x08, frequency=freq, latency=1
        )
        self.ftdi.write_data(bytes([0x97, 0x8D, 0x85]))  # no adaptive/3phase/loopback
        self.ftdi.purge_buffers()
        # 0x8F clocks (n+1)*8 cycles without data; size for ~60 us of spacing.
        n = max(0, int(self.freq * 60e-6 / 8) - 1)
        spacing = bytes([0x8F, n & 0xFF, n >> 8])
        self._poll_cmd = (
            self._CS_LO + bytes([0x11, 3, 0]) + CMD_POLL + self._CS_HI
            + spacing
            + self._CS_LO + bytes([0x20, 7, 0]) + self._CS_HI + b"\x87"
        )

    def poll(self):
        """One poll cycle -> (batch, first_seq). batch is b"" when the
        firmware had nothing, or None on a bad/missed status header."""
        self.ftdi.write_data(self._poll_cmd)
        hdr = self.ftdi.read_data_bytes(8, attempt=8)
        if len(hdr) != 8:
            return None, 0
        magic, total, first_seq = struct.unpack(">HHI", hdr)
        if magic != STATUS_MAGIC:
            self.ftdi.purge_buffers()
            return None, 0
        if total == 0:
            return b"", first_seq
        n = total - 1
        self.ftdi.write_data(
            self._CS_LO + bytes([0x20, n & 0xFF, n >> 8]) + self._CS_HI + b"\x87"
        )
        batch = self.ftdi.read_data_bytes(total, attempt=16)
        return (bytes(batch) if len(batch) == total else None), first_seq

    def close(self):
        self.ftdi.close()
CHIRP_MAGIC_PACKED12 = 0xA55C
CHIRP_MAGIC_BFP16 = 0xA55B  # block floating point (see unpack_bfp)
META_MAGIC = 0xA55E  # self-describing config descriptor (see parse_meta)
SUB12_BLOCK_PAIRS = 16


def is_sub12(magic):
    """A5 6k = Sub12 with k of 16 pairs kept; returns k or None."""
    if magic & 0xFFF0 == 0xA560 and 1 <= (magic & 0xF) <= 15:
        return magic & 0xF
    return None


def is_bfp(magic):
    return magic == CHIRP_MAGIC_BFP16


def is_meta(magic):
    return magic == META_MAGIC


def valid_magic(magic):
    return (
        magic == CHIRP_MAGIC_PACKED12
        or magic == CHIRP_MAGIC_BFP16
        or magic == META_MAGIC
        or is_sub12(magic) is not None
    )


# Config-descriptor payload layout, mirrors stream.rs build_meta (32 B, BE).
META_STRUCT = ">BBBBHHHBBhHHxxIII"
META_FIELDS = (
    "version", "rx", "format_code", "sample_rate_code",
    "num_adc_samples", "chirps_per_burst", "bursts_per_frame",
    "tx_en", "mimo_pattern", "freq_slope", "ramp_end_time", "idle_time",
    "freq_start", "burst_periodicity", "frame_periodicity",
)


def parse_meta(payload):
    """Decode an A55E descriptor payload to a dict (see stream.rs build_meta).
    format_code 0 = packed12, 16 = bfp16, else Sub12 keep value."""
    return dict(zip(META_FIELDS, struct.unpack(META_STRUCT, payload)))


def mask_indices(seq, pairs, keep):
    """Kept-pair indices for chirp `seq` - bit-exact mirror of the firmware's
    capture::mask_fill (xorshift32 + per-16-block partial Fisher-Yates)."""
    s = ((seq ^ 0xA5A55A5A) * 0x9E3779B9 | 1) & 0xFFFFFFFF

    def rng():
        nonlocal s
        s = (s ^ s << 13) & 0xFFFFFFFF
        s = (s ^ s >> 17) & 0xFFFFFFFF
        s = (s ^ s << 5) & 0xFFFFFFFF
        return s

    out = []
    for block in range(pairs // SUB12_BLOCK_PAIRS):
        idx = list(range(SUB12_BLOCK_PAIRS))
        for j in range(keep):
            r = j + rng() % (SUB12_BLOCK_PAIRS - j)
            idx[j], idx[r] = idx[r], idx[j]
        out.extend(block * SUB12_BLOCK_PAIRS + i for i in sorted(idx[:keep]))
    return out


def unpack12(payload):
    """12-bit packed -> int16 ndarray. 3 bytes carry samples (s0, s1):
    b0 = s0[7:0], b1 = s0[11:8] | s1[3:0] << 4, b2 = s1[11:4]."""
    b = np.frombuffer(bytes(payload), dtype=np.uint8)
    b = b[: len(b) - len(b) % 3].reshape(-1, 3).astype(np.uint16)
    out = np.empty(2 * len(b), dtype=np.int16)
    out[0::2] = (b[:, 0] | (b[:, 1] & 0xF) << 8).astype(np.int16)
    out[1::2] = (b[:, 1] >> 4 | b[:, 2] << 4).astype(np.int16)
    out[out >= 2048] -= 4096
    return out


def unpack_bfp(payload):
    """Bfp16 -> int16 ndarray. 17-byte blocks: shift exponent (0..4) then
    16 int8 mantissas; sample = mantissa << exponent (see capture.rs
    pack_block_bfp)."""
    b = np.frombuffer(bytes(payload), dtype=np.uint8)
    b = b[: len(b) - len(b) % 17].reshape(-1, 17)
    exp = b[:, :1].astype(np.int16)
    mant = b[:, 1:].astype(np.int8).astype(np.int16)
    return (mant << exp).ravel()


def decode_payload(magic, payload):
    # Sub12 payloads are also 12-bit packed; index placement is up to the
    # consumer (see tools/cs_reconstruct.py).
    return unpack_bfp(payload) if is_bfp(magic) else unpack12(payload)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0, help="capture duration")
    ap.add_argument("--out", default="capture.bin")
    ap.add_argument("--freq", type=float, default=20e6, help="SPI clock Hz")
    ap.add_argument("--poll-ms", type=float, default=5.0, help="poll interval")
    ap.add_argument("--stats", action="store_true", help="per-batch prints")
    args = ap.parse_args()

    link = SpiLink(freq=args.freq)

    chirps, batches, gaps, bad = 0, 0, 0, 0
    bad_status, bad_chunk = 0, 0
    bad_dump = None
    last_seq = None
    payload_bytes = 0
    t0 = time.time()
    with open(args.out, "wb") as f:
        while time.time() - t0 < args.seconds:
            batch, first_seq = link.poll()
            if batch is None:
                bad += 1
                bad_status += 1
                time.sleep(args.poll_ms / 1000)
                continue
            if not batch:
                time.sleep(args.poll_ms / 1000)
                continue
            total = len(batch)
            batches += 1
            # split the batch into chirp chunks
            off = 0
            while off + 8 <= len(batch):
                m, ln, seq = struct.unpack(">HHI", batch[off:off + 8])
                if not valid_magic(m) or off + 8 + ln > len(batch):
                    bad += 1
                    bad_chunk += 1
                    if bad_dump is None:
                        bad_dump = (bytes(batch), off)
                    break
                f.write(batch[off:off + 8 + ln])
                if is_meta(m):
                    # Descriptor (seq field carries generation); archive it but
                    # don't treat it as a chirp or a seq gap.
                    if args.stats:
                        print(f"  meta gen={seq}: {parse_meta(batch[off + 8:off + 8 + ln])}")
                    off += 8 + ln
                    continue
                payload_bytes += ln
                if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
                    gaps += 1
                    if args.stats:
                        print(f"  seq gap: {last_seq} -> {seq}")
                last_seq = seq
                if args.stats and chirps < 3:
                    vals = decode_payload(m, batch[off + 8:off + 8 + ln])
                    k = is_sub12(m)
                    fmt = ("bfp16" if is_bfp(m) else
                           "packed12" if k is None else f"sub12 {k}/16")
                    print(f"  chirp seq={seq} len={ln} fmt={fmt} "
                          f"first8={vals[:8]} min={min(vals)} max={max(vals)}")
                chirps += 1
                off += 8 + ln
            if args.stats:
                print(f"batch: {total} B, first_seq={first_seq}")
    dt = time.time() - t0
    print(f"{chirps} chirps / {batches} batches in {dt:.2f}s "
          f"({payload_bytes / dt / 1024:.1f} KiB/s payload), "
          f"{gaps} seq gaps, {bad} bad headers "
          f"({bad_status} status, {bad_chunk} chunk) -> {args.out}")
    if bad_dump is not None:
        batch, off = bad_dump
        with open(args.out + ".bad", "wb") as f:
            f.write(batch)
        print(f"first bad batch: parse broke at offset {off}/{len(batch)}, "
              f"saved -> {args.out}.bad")
    link.close()


if __name__ == "__main__":
    main()
