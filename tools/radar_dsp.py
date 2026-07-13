"""Radar signal processing for the IWRL6432 raw-ADC stream.

Pure array math (NumPy, transparently CuPy) so it can be validated offline
against a capture.bin before driving the live dashboard (radar_scope.py).

Pipeline per frame (chirps_per_burst * bursts_per_frame chirps):
  1. range profile per chirp per RX  (Packed12 -> FFT; Sub12 -> FISTA L1)
  2. assemble the TDMA virtual array: even chirp = TX0, odd = TX1; each carries
     RX0..2, giving 6 virtual antennas per (burst, range) - see GEOMETRY.
  3. MTI clutter removal (subtract slow-time mean), Doppler FFT over bursts
  4. range-Doppler magnitude, range-azimuth map (the PPI "arc")
  5. CA-CFAR detections -> per-detection azimuth/elevation via the 2D virtual
     grid -> (x, y, z, velocity, snr) point cloud

Config + physical units are read from the firmware's self-describing A55E
metadata frame (spi_capture.parse_meta); metrics() mirrors profiles.rs.
"""

import struct

import numpy as np

from spi_capture import (SUB12_BLOCK_PAIRS, is_bfp, is_meta, is_sub12,
                         mask_indices, parse_meta, unpack12, unpack_bfp,
                         valid_magic)
from cs_reconstruct import fista

C_LIGHT = 3.0e8

# --- BOOST (xwrL64xx-evm) virtual-array geometry -----------------------------
# antGeometryCfg 0 0 1 1 0 2 0 1 1 2 0 3 2.418 2.418 - confirmed identical in
# the SDK xwrL64xx-evm profiles, the demo's compiled-in default
# (motion_detect.c gDefaultAntGeometry), sysconfig's per-board defaults, and
# the user's own ProfileSwitchIWRL6432.cfg. Pair semantics from the demo's CLI
# parser + dpc.c: (row, col) per virtual antenna (index = tx*numRx + rx),
# columns = azimuth (spacing antDistanceXdim), rows = ELEVATION (antDistanceZdim),
# both 2.418 mm = lambda/2. So the array is a 4-element azimuth line (row 0,
# virt 0/3/2/5 at cols 0..3) plus TWO elevated elements (row 1: virt 1 at
# col 1, virt 4 at col 2) - a lambda/2 elevation interferometer. 2D angle
# (az + coarse el) IS available on this board.
GEOMETRY = [(0, 0), (1, 1), (0, 2), (0, 1), (1, 2), (0, 3)]
ANT_DIST_MM = 2.418  # lambda/2 at ~62 GHz
N_ROWS = 2
N_COLS = 4
AZ_ROW_VIRT = [0, 3, 2, 5]     # row-0 elements in col order 0..3
EL_PAIRS = [(1, 3), (4, 2)]    # (row-1 virt, row-0 virt) at the same column
# HW-measured static phase cal (walk test 2026-07-12): the TX1 virtual elements
# read ~180 deg offset from TX0 (a boresight target measures [0,180,0,180]
# across the 4 azimuth columns), so the azimuth FFT dumps a broadside target
# into two +-90 deg edge blips and exaggerates every angle. Flip the TX1 azimuth
# columns to realign the manifold. Applied to AZIMUTH ONLY - the elevation
# interferometer pairs each mix one TX0 + one TX1 element and were tuned WITH
# this offset (see EL_REVERSED), so a global flip would corrupt elevation.
TX1_AZ_SIGN = -1.0

AZ_FFT = 64
EL_FFT = 16


def backend(force_cpu=False):
    """Return (xp, to_np, name). CuPy if importable and not forced off."""
    if not force_cpu:
        try:
            import cupy as xp  # type: ignore[import-not-found]
            xp.zeros(1) + 1  # touch the device so a broken CUDA falls back
            return xp, xp.asnumpy, "cupy"
        except Exception:
            pass
    return np, (lambda a: a), "numpy"


# ---------------------------------------------------------------------------
# Config + derived metrics (mirror of crates/.../profiles.rs metrics())
# ---------------------------------------------------------------------------

def freq_start_ghz(freq_start):
    """Decode the 16-bit RF start word to GHz (0xBE00=57 .. 0xD555=64)."""
    return 57.0 + (freq_start - 0xBE00) / (0xD555 - 0xBE00) * 7.0


class Config:
    """Radar geometry + RF params from an A55E descriptor dict."""

    def __init__(self, meta):
        self.raw = dict(meta)
        self.rx = meta["rx"]
        self.samples = meta["num_adc_samples"]
        # keep: None = exact packed12, "bfp" = bfp16 (also exact after
        # dequantization), 1..15 = Sub12 kept pairs (needs L1 recon).
        fc = meta["format_code"]
        self.keep = None if fc == 0 else ("bfp" if fc == 16 else fc)
        self.sample_rate_code = meta["sample_rate_code"]
        self.chirps_per_burst = meta["chirps_per_burst"]
        self.bursts_per_frame = meta["bursts_per_frame"]
        self.tx_en = meta["tx_en"]
        self.mimo = meta["mimo_pattern"]
        self.freq_slope = meta["freq_slope"]
        self.ramp_end_time = meta["ramp_end_time"]
        self.idle_time = meta["idle_time"]
        self.freq_start = meta["freq_start"]
        self.burst_periodicity = meta["burst_periodicity"]
        self.frame_periodicity = meta["frame_periodicity"]
        # TDMA (mimo=1) gives 2 TX; else a single effective TX (no angle sep).
        self.num_tx = 2 if self.mimo == 1 else 1

    @property
    def chirps_per_frame(self):
        return self.chirps_per_burst * self.bursts_per_frame

    @property
    def n_virtual(self):
        return self.num_tx * self.rx

    def metrics(self):
        m = {}
        fs = 100.0 / self.sample_rate_code               # MSPS
        slope = self.freq_slope * 0.02861                # MHz/us
        sampling_us = self.samples / fs
        sweep = slope * sampling_us                      # MHz
        lam = C_LIGHT / (freq_start_ghz(self.freq_start) * 1e9)
        t_rep = self.burst_periodicity * 100e-9          # per-TX repetition
        coherent = self.bursts_per_frame * t_rep
        m["fs_msps"] = fs
        m["slope_mhz_us"] = slope
        m["sweep_mhz"] = sweep
        m["lambda_m"] = lam
        m["range_res_m"] = C_LIGHT / (2 * sweep * 1e6)
        m["max_range_m"] = (fs / 2) * C_LIGHT / (2 * slope * 1e12) * 1e6
        m["v_max_ms"] = lam / (4 * t_rep)
        m["v_res_ms"] = lam / (2 * coherent)
        m["fps"] = 40e6 / self.frame_periodicity
        m["chirps_per_sec"] = self.chirps_per_frame * m["fps"]
        return m

    def range_axis(self, n_bins):
        return np.arange(n_bins) * self.metrics()["range_res_m"]

    def velocity_axis(self):
        vmax = self.metrics()["v_max_ms"]
        d = self.bursts_per_frame
        return np.linspace(-vmax, vmax, d, endpoint=False)

    def __repr__(self):
        mo = "TDM-2TX" if self.mimo == 1 else ("BPM" if self.mimo == 4 else "1TX")
        fmt = ("packed12" if self.keep is None else
               "bfp16" if self.keep == "bfp" else f"sub12 {self.keep}/16")
        return (f"Config({self.samples} samp, rx={self.rx}, {fmt}, {mo}, "
                f"{self.chirps_per_burst}x{self.bursts_per_frame} bursts)")


# ---------------------------------------------------------------------------
# Range processing
# ---------------------------------------------------------------------------

def range_profiles(xp, seq, keep, payload, cfg, iters=60, lam=0.02):
    """Decode one chirp payload to complex range profiles, shape (rx, N/2).
    The ADC is real, so the range FFT is conjugate-symmetric; only the first
    half (positive ranges) is kept. Both paths return the HANNING-windowed
    spectrum: for Sub12 the kept samples are windowed (w[idx]*y) and FISTA
    solves for fft(w*s) - the unwindowed spectrum's leakage skirts are not
    sparse and reconstruct incoherently across chirp masks."""
    vals = unpack_bfp(payload) if keep == "bfp" else unpack12(payload)
    per_rx = len(vals) // cfg.rx
    rx = xp.asarray(vals, dtype=float).reshape(cfg.rx, per_rx)
    if keep is None or keep == "bfp":
        # Per-chirp DC removal: the ADC carries a per-RX bias (tens of LSB on a
        # ~60-LSB signal here) that otherwise dumps into range bin 0 and leaks
        # into bins 1-2 through the window, swamping the near range and the
        # micro-Doppler peak-bin pick. Subtract each chirp's fast-time mean.
        rx = rx - rx.mean(axis=-1, keepdims=True)
        win = xp.hanning(per_rx)
        spec = xp.fft.fft(rx * win, axis=-1)
        return spec[:, : per_rx // 2]
    n = per_rx * SUB12_BLOCK_PAIRS // keep
    idx_pairs = mask_indices(seq, n // 2, keep)
    idx = xp.asarray([i for p in idx_pairs for i in (2 * p, 2 * p + 1)])
    y = rx.astype(complex) * xp.hanning(n)[idx]
    return fista(xp, y, idx, n, iters, lam, group=cfg.rx)[:, : n // 2]


def _mask_row(seq, per_rx, keep):
    """Kept-sample (not pair) indices for one chirp, as an int64 row."""
    n = per_rx * SUB12_BLOCK_PAIRS // keep
    pairs = np.asarray(mask_indices(seq, n // 2, keep), dtype=np.int64)
    row = np.repeat(pairs * 2, 2)
    row[1::2] += 1
    return row


def range_profiles_batch(xp, chirps, cfg, iters=60, lam=0.02):
    """Whole-frame variant of range_profiles: chirps is a list of
    (seq, payload); returns [nchirp, rx, N/2] complex. Sub12 chirps are
    reconstructed in ONE batched FISTA call (per-row masks) - per-chirp calls
    cannot keep up with the live stream (see radar_scope.py)."""
    nc = len(chirps)
    dec = unpack_bfp if cfg.keep == "bfp" else unpack12
    mats = np.stack([np.asarray(dec(p)).reshape(cfg.rx, -1)
                     for _, p in chirps])
    rx = xp.asarray(mats, dtype=float)          # [nc, rx, per_rx]
    per_rx = rx.shape[-1]
    if cfg.keep is None or cfg.keep == "bfp":
        # Per-chirp DC removal (see range_profiles): kill the ADC bias before
        # the range FFT so it doesn't pile into range bin 0.
        rx = rx - rx.mean(axis=-1, keepdims=True)
        spec = xp.fft.fft(rx * xp.hanning(per_rx), axis=-1)
        return spec[..., : per_rx // 2]
    n = per_rx * SUB12_BLOCK_PAIRS // cfg.keep
    idx = np.stack([_mask_row(seq, per_rx, cfg.keep) for seq, _ in chirps])
    idx2 = xp.asarray(np.repeat(idx, cfg.rx, axis=0))
    # windowed measurements (see range_profiles)
    y = rx.reshape(nc * cfg.rx, per_rx).astype(complex) * xp.hanning(n)[idx2]

    scale = 1.0 / xp.sqrt(n)

    def a_op(x):
        return xp.take_along_axis(xp.fft.ifft(x, axis=-1), idx2, axis=-1) / scale

    def at_op(r):
        z = xp.zeros((r.shape[0], n), dtype=complex)
        xp.put_along_axis(z, idx2, r, axis=-1)
        return xp.fft.fft(z, axis=-1) * scale

    # Static/moving decomposition: a dense scene (a room at cm bins) blows the
    # sparsity budget and its per-chirp recon error raises the MTI floor,
    # burying weak movers. The static profile is OVERDETERMINED across the
    # frame (nc*m samples for n unknowns per RX) - plain LS (Landweber), no
    # sparsity needed. FISTA then only reconstructs the residual (the movers),
    # which is genuinely sparse; static-estimate error is common to all
    # chirps and cancels in MTI.
    x_dc = xp.zeros((cfg.rx, n), dtype=complex)
    for _ in range(25):
        g = at_op(a_op(xp.tile(x_dc, (nc, 1))) - y)
        x_dc = x_dc - g.reshape(nc, cfg.rx, n).mean(axis=0)
    resid = y - a_op(xp.tile(x_dc, (nc, 1)))
    # one support shared by the whole frame (group = all rows)
    x_mv = fista(xp, resid, idx2, n, iters, lam, group=nc * cfg.rx)
    x = x_mv + xp.tile(x_dc, (nc, 1))
    return x.reshape(nc, cfg.rx, n)[..., : n // 2]


# ---------------------------------------------------------------------------
# Frame assembly + Doppler + angle
# ---------------------------------------------------------------------------

def assemble_frame(xp, chirps, cfg):
    """chirps: list of (seq, rx_profiles[rx, N]) for one frame, in seq order.
    Returns cube X[virtual_ant, burst, N]. TDMA: even seq -> TX0, odd -> TX1."""
    n = chirps[0][1].shape[-1]
    nb = cfg.bursts_per_frame
    nv = cfg.n_virtual
    X = xp.zeros((nv, nb, n), dtype=complex)
    for seq, prof in chirps:
        rel = seq - chirps[0][0]
        burst = rel // cfg.chirps_per_burst
        if burst >= nb:
            continue
        tx = (seq % cfg.chirps_per_burst) if cfg.num_tx > 1 else 0
        for r in range(cfg.rx):
            X[tx * cfg.rx + r, burst] = prof[r]
    return X


def doppler_process(xp, X, mti=True):
    """X[vant, burst, N] -> (rd_mag[burst, N], Xd[vant, burst, N] complex).
    MTI removes the slow-time mean (static clutter) before the Doppler FFT."""
    if mti:
        X = X - X.mean(axis=1, keepdims=True)
    win = xp.hanning(X.shape[1])[None, :, None]
    Xd = xp.fft.fftshift(xp.fft.fft(X * win, axis=1), axes=1)
    rd = xp.sqrt((xp.abs(Xd) ** 2).sum(axis=0))  # non-coherent over vant
    return rd, Xd


# Display convention: birds-eye/PPI viewed from BEHIND the sensor looking
# outward, so a target on the sensor's LEFT renders at negative x (screen
# left). The FFT-bin -> angle sign is set by antenna column order vs FFT
# direction; verified on HW 2026-07-11 (person standing on the sensor's left)
# that the spectrum needs this reversal to match the convention.
AZ_REVERSED = True


def tdm_compensate(xp, Xd, cfg, sign=1):
    """Remove the TDM-MIMO motion phase from the TX1 virtual elements.

    TX1 fires one chirp period T_c after TX0, so a target moving with
    Doppler f_d adds e^{j 2 pi f_d T_c} to every TX1 element. The azimuth
    row alternates TX0/TX1 elements, so that common phase masquerades as a
    SPATIAL frequency and scatters the angle estimate of anything moving
    (static targets are unaffected). Compensate per Doppler bin d (fftshifted:
    f_d = (d - N/2) / (N * t_rep)): TX1 elements *= e^{-j 2 pi (d-N/2) T_c /
    (N t_rep)}. `sign` flips the correction (+1/-1; 0 disables) - the phase
    convention is empirically determined (see --tdm-sign in radar_scope)."""
    if Xd.shape[0] < 4 or sign == 0:
        return Xd  # single TX / disabled: nothing to compensate
    n = Xd.shape[1]
    t_c = (cfg.ramp_end_time + cfg.idle_time) * 100e-9
    t_rep = cfg.burst_periodicity * 100e-9
    d = xp.arange(n) - n // 2
    ph = xp.exp(-2j * sign * np.pi * d * (t_c / (n * t_rep)))[None, :, None]
    Xd = Xd.copy()
    Xd[3:6] = Xd[3:6] * ph
    return Xd


def range_azimuth(xp, Xd):
    """Xd[vant, burst, N] -> RA[N, AZ_FFT] magnitude (the PPI arc). Azimuth FFT
    over the 4 row-0 elements, summed (non-coherent) over Doppler bins.
    Single-TX configs (3 vant) degrade to the 2 available row-0 elements."""
    sel = [v for v in AZ_ROW_VIRT if v < Xd.shape[0]]
    A = Xd[sel]
    if Xd.shape[0] >= 4:                       # 2-TX TDMA: flip TX1 columns
        A = A.copy()
        half = Xd.shape[0] // 2
        for i, v in enumerate(sel):
            if v >= half:
                A[i] = A[i] * TX1_AZ_SIGN
    sp = xp.fft.fftshift(xp.fft.fft(A, n=AZ_FFT, axis=0), axes=0)
    ra = xp.abs(sp).sum(axis=1)               # [AZ_FFT, N]
    if AZ_REVERSED:
        ra = ra[::-1]
    return ra.T                               # [N, AZ_FFT]


# Elevation sign HW-verified 2026-07-11 (squat test: dot must go DOWN):
# row 1 leads in phase for a target BELOW boresight, so the raw
# interferometer angle needs this reversal.
EL_REVERSED = True


def angle_2d(xp, vec6):
    """Virtual-antenna complex samples -> (az_rad, el_rad).

    Azimuth: FFT over the 4-element row-0 line (cols 0..3).
    Elevation: lambda/2 interferometer - for each column with both rows
    present, row1 * conj(row0) cancels the azimuth phase; the argument of the
    coherent sum is the elevation phase, el = arcsin(phase / pi). Each pair
    mixes a TX0 and a TX1 element, so tdm_compensate must run first for
    movers. Tolerates fewer channels (single-TX: 2 az elements, no el pair)."""
    line = xp.zeros(N_COLS, dtype=complex)
    half = len(vec6) // 2
    for col, v in enumerate(AZ_ROW_VIRT):
        if v < len(vec6):
            # flip the TX1 azimuth elements (static 180 deg cal, see TX1_AZ_SIGN)
            line[col] = vec6[v] * (TX1_AZ_SIGN if len(vec6) >= 4 and v >= half else 1.0)
    sp = xp.abs(xp.fft.fftshift(xp.fft.fft(line, n=AZ_FFT)))
    ai = int(xp.argmax(sp))
    # Parabolic peak interpolation: raw argmax quantizes azimuth to the
    # FFT grid (~15 cm lateral steps at 3 m), which makes a slowly walking
    # target hop between fixed x positions instead of moving smoothly.
    l, c, r = (float(sp[(ai - 1) % AZ_FFT]), float(sp[ai]),
               float(sp[(ai + 1) % AZ_FFT]))
    den = l - 2 * c + r
    frac = 0.5 * (l - r) / den if den != 0 else 0.0
    az = np.arcsin(np.clip(2 * ((ai + frac) / AZ_FFT - 0.5), -1, 1))
    if AZ_REVERSED:
        az = -az
    acc = 0j
    for up, lo in EL_PAIRS:
        if up < len(vec6) and lo < len(vec6):
            acc += complex(vec6[up]) * np.conj(complex(vec6[lo]))
    el = 0.0
    if acc != 0:
        el = float(np.arcsin(np.clip(np.angle(acc) / np.pi, -1, 1)))
        if EL_REVERSED:
            el = -el
    return az, el


AZ_ANGLE_AXIS = np.degrees(np.arcsin(np.clip(2 * (np.arange(AZ_FFT) / AZ_FFT - 0.5), -1, 1)))


# ---------------------------------------------------------------------------
# CFAR detection
# ---------------------------------------------------------------------------

def denoise_mag(mag, k=2.0, axis=0):
    """Robust spectral-subtraction denoise of a magnitude map (numpy).

    Noise floor is estimated per slice along `axis` as median + 1.4826*MAD
    (the median absolute deviation is immune to sparse targets, unlike a mean
    or percentile), then subtracted in the POWER domain with soft clamping:
    out = sqrt(max(mag^2 - (k*floor)^2, 0)). Noise cells collapse to zero,
    signal cells lose only the floor's energy - the display equivalent of a
    per-bin Wiener gate. k scales aggressiveness (~2 = ~6 dB over the floor)."""
    m = np.asarray(mag)
    med = np.median(m, axis=axis, keepdims=True)
    mad = np.median(np.abs(m - med), axis=axis, keepdims=True)
    floor = med + 1.4826 * mad
    return np.sqrt(np.maximum(m ** 2 - (k * floor) ** 2, 0.0))


def cfar_2d(mag, guard=2, train=4, thresh_db=11.0, min_range_bin=2):
    """CA-CFAR on a range-Doppler magnitude map (numpy). Returns a list of
    (doppler_idx, range_idx, snr_db) local maxima above threshold."""
    from scipy.ndimage import maximum_filter, uniform_filter
    p = np.asarray(mag) ** 2
    win = guard + train
    nf = (2 * win + 1) ** 2
    ng = (2 * guard + 1) ** 2
    full = uniform_filter(p, size=2 * win + 1, mode="nearest")
    grd = uniform_filter(p, size=2 * guard + 1, mode="nearest")
    noise = (full * nf - grd * ng) / (nf - ng)
    snr = 10 * np.log10(np.maximum(p, 1e-12) / np.maximum(noise, 1e-12))
    peak = (p == maximum_filter(p, size=3)) & (snr > thresh_db)
    peak[:, :min_range_bin] = False
    dets = []
    for d, r in zip(*np.nonzero(peak)):
        dets.append((int(d), int(r), float(snr[d, r])))
    return dets


def detections_pointcloud(xp, Xd, dets, cfg):
    """Turn CFAR (doppler, range) cells into (x, y, z, range_m, vel_ms, az_deg,
    el_deg, snr_db) rows using the per-cell virtual-array angle estimate."""
    m = cfg.metrics()
    rr = m["range_res_m"]
    vax = cfg.velocity_axis()
    out = []
    for d, r, snr in dets:
        vec6 = Xd[:, d, r]
        az, el = angle_2d(xp, vec6)
        rng = r * rr
        vel = float(vax[d]) if d < len(vax) else 0.0
        x = rng * np.sin(az) * np.cos(el)
        y = rng * np.cos(az) * np.cos(el)
        z = rng * np.sin(el)
        out.append((x, y, z, rng, vel, np.degrees(az), np.degrees(el), snr))
    return out


# ---------------------------------------------------------------------------
# Stream reader (file or bytes) + offline frame iterator
# ---------------------------------------------------------------------------

def iter_chunks(data):
    """Yield ('meta', dict) / ('chirp', seq, keep, payload) from a chunk blob.
    keep: None = packed12, "bfp" = bfp16, 1..15 = Sub12 kept pairs."""
    off = 0
    while off + 8 <= len(data):
        m, ln, seq = struct.unpack(">HHI", data[off:off + 8])
        if not valid_magic(m) or off + 8 + ln > len(data):
            break
        body = data[off + 8:off + 8 + ln]
        if is_meta(m):
            yield "meta", parse_meta(body)
        else:
            yield "chirp", seq, "bfp" if is_bfp(m) else is_sub12(m), body
        off += 8 + ln


def iter_frames(xp, data, cfg_box, iters=60):
    """Group chirps into frames; yield (cfg, X[vant, burst, N]). cfg_box is a
    one-element list holding the current Config (updated by meta frames)."""
    buf = []
    for item in iter_chunks(data):
        if item[0] == "meta":
            cfg_box[0] = Config(item[1])
            buf.clear()
            continue
        cfg = cfg_box[0]
        if cfg is None:
            continue
        _, seq, keep, payload = item
        prof = range_profiles(xp, seq, keep, payload, cfg, iters=iters)
        if buf and seq != buf[-1][0] + 1:
            buf.clear()  # gap: drop the partial frame
        buf.append((seq, prof))
        if len(buf) == cfg.chirps_per_frame:
            yield cfg, assemble_frame(xp, buf, cfg)
            buf = []


# ---------------------------------------------------------------------------
# Offline runner: validate the pipeline on a capture.bin (+ its A55E frames)
# ---------------------------------------------------------------------------

def _main():
    import argparse
    ap = argparse.ArgumentParser(description="offline radar DSP over a capture.bin")
    ap.add_argument("capture")
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--iters", type=int, default=60)
    ap.add_argument("--max-frames", type=int, default=10)
    ap.add_argument("--thresh-db", type=float, default=11.0)
    args = ap.parse_args()

    xp, to_np, name = backend(args.cpu)
    data = open(args.capture, "rb").read()
    cfg_box = [None]
    nframes = 0
    for cfg, X in iter_frames(xp, data, cfg_box, iters=args.iters):
        if nframes == 0:
            m = cfg.metrics()
            print(f"backend={name}  {cfg}")
            print(f"  range_res={m['range_res_m']*100:.1f}cm  max_range={m['max_range_m']:.0f}m"
                  f"  v_max=+-{m['v_max_ms']:.2f}m/s  v_res={m['v_res_ms']*100:.1f}cm/s"
                  f"  fps={m['fps']:.0f}  virt_ant={cfg.n_virtual}")
        rd, Xd = doppler_process(xp, X)
        dets = cfar_2d(to_np(rd), thresh_db=args.thresh_db)
        pc = detections_pointcloud(xp, Xd, dets, cfg)
        pc.sort(key=lambda p: -p[7])
        print(f"frame {nframes}: {len(dets)} detections")
        for x, y, z, rng, vel, az, el, snr in pc[:6]:
            print(f"    r={rng:5.2f}m  v={vel:+5.2f}m/s  az={az:+5.1f}deg"
                  f"  el={el:+5.1f}deg  snr={snr:4.1f}dB  (x={x:+.2f} y={y:+.2f} z={z:+.2f})")
        nframes += 1
        if nframes >= args.max_frames:
            break
    if nframes == 0:
        print("no complete frames (need an A55E descriptor + a full frame of chirps)")


if __name__ == "__main__":
    _main()
