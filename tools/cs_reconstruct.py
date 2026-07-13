"""L1 (compressed-sensing) reconstruction of Sub12-subsampled captures.

Input: the chunk stream written by spi_capture.py (magic|len|seq headers).
For Sub12 chunks (magic A5 6k) the kept-pair indices are regenerated from
seq (bit-exact mirror of the firmware mask, see spi_capture.mask_indices)
and each chirp/RX is reconstructed to a full N-point range spectrum by
FISTA on the partial-Fourier problem:

    min_x  0.5 * || S F^H x - y ||^2  +  lam * ||x||_1

where x is the (complex) range spectrum, F^H the unitary inverse DFT and
S the row-subsampling operator. Packed12 chunks are just FFT'd; with
--simulate-keep K they are first masked offline (same PRNG) and the
reconstruction is scored against the full-data FFT (ground truth) - the
recommended validation path before trusting live subsampled captures.

Uses NumPy; drops in CuPy transparently with --gpu (same API).

Usage (deps come from the uv env in this directory; --gpu needs
`uv sync --extra gpu`):
  uv run cs_reconstruct.py capture.bin --out spectra.npy [--rx 3]
                           [--iters 100] [--lam 0.02] [--gpu]
                           [--simulate-keep 8] [--max-chirps N]
"""

import argparse
import struct
import sys

from spi_capture import (SUB12_BLOCK_PAIRS, is_bfp, is_meta, is_sub12,
                         mask_indices, unpack12, unpack_bfp, valid_magic)


def load_chunks(path):
    """Yield (seq, keep, payload) per chirp; keep is None (packed12), "bfp"
    (bfp16, exact after dequantization) or the Sub12 kept-pair count. A55E
    config-descriptor chunks are skipped (they are not sample data)."""
    data = open(path, "rb").read()
    off = 0
    while off + 8 <= len(data):
        m, ln, seq = struct.unpack(">HHI", data[off:off + 8])
        if not valid_magic(m) or off + 8 + ln > len(data):
            print(f"bad chunk at offset {off}, stopping", file=sys.stderr)
            break
        if not is_meta(m):
            yield seq, "bfp" if is_bfp(m) else is_sub12(m), data[off + 8:off + 8 + ln]
        off += 8 + ln


def fista(xp, y, idx, n, iters, lam_rel, group=None, debias=8):
    """min 0.5*||S F^H x - y||^2 + lam*||x||_1, unitary F (L = 1).
    y: [batch, m] complex; idx: [m] shared mask or [batch, m] per-row masks
    (per-chirp seeded masks batched in one call); returns x: [batch, n].

    group: rows-per-group count g (batch = g*k, group-major). Shrinkage then
    uses the rms across each group's rows (group lasso / joint sparsity):
    real targets sit at FRACTIONAL bins, and with independent per-chirp
    recovery the few bins L1 picks for the leakage skirt flicker with each
    chirp's mask, destroying slow-time coherence (Doppler / waterfall). A
    frame-shared support keeps the same bins in every chirp.

    debias: after the L1 loop, this many plain gradient steps restricted to
    the recovered support - removes the shrinkage amplitude bias.

    Hermitian symmetry (y is real, so the true x is conjugate-symmetric) is
    automatic, not enforced: a_op maps Hermitian spectra to real samples and
    at_op maps real residuals back to Hermitian spectra, so every iterate
    stays in the subspace to machine precision (verified ~5e-16). Projecting
    each step would be a no-op."""
    scale = 1.0 / xp.sqrt(n)

    if idx.ndim == 1:
        def a_op(x):  # spectrum -> kept time samples
            return xp.fft.ifft(x, axis=-1)[:, idx] / scale

        def at_op(r):  # kept residual -> spectrum gradient space
            z = xp.zeros((r.shape[0], n), dtype=complex)
            z[:, idx] = r
            return xp.fft.fft(z, axis=-1) * scale
    else:
        def a_op(x):
            return xp.take_along_axis(xp.fft.ifft(x, axis=-1), idx, axis=-1) / scale

        def at_op(r):
            z = xp.zeros((r.shape[0], n), dtype=complex)
            xp.put_along_axis(z, idx, r.astype(complex), axis=-1)
            return xp.fft.fft(z, axis=-1) * scale

    def shrink_mag(g):
        if group is None:
            return xp.abs(g)
        G = (xp.abs(g) ** 2).reshape(group, -1, n)
        rms = xp.sqrt(G.mean(axis=0, keepdims=True))
        return xp.broadcast_to(rms, G.shape).reshape(g.shape)

    x = at_op(y)
    lam = lam_rel * xp.abs(x).max()
    z, t = x.copy(), 1.0
    for _ in range(iters):
        g = z - at_op(a_op(z) - y)
        mag = shrink_mag(g)
        x_new = g * xp.maximum(1 - lam / xp.maximum(mag, 1e-12), 0)
        t_new = (1 + (1 + 4 * t * t) ** 0.5) / 2
        z = x_new + ((t - 1) / t_new) * (x_new - x)
        x, t = x_new, t_new
    if debias:
        sup = shrink_mag(x) > 0
        for _ in range(debias):
            x = (x - at_op(a_op(x) - y)) * sup
    # One unrestricted step: off-support bins pick up the matched-filter
    # residual, giving dB plots a real noise floor instead of exact zeros.
    return x - at_op(a_op(x) - y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--out", default="spectra.npy")
    ap.add_argument("--rx", type=int, default=3)
    ap.add_argument("--iters", type=int, default=100)
    ap.add_argument("--lam", type=float, default=0.02,
                    help="l1 weight, relative to max |A^H y|")
    ap.add_argument("--gpu", action="store_true", help="use CuPy")
    ap.add_argument("--simulate-keep", type=int, default=None,
                    help="mask full Packed12 chirps offline at K/16 and "
                         "report PSNR vs the full-data FFT")
    ap.add_argument("--max-chirps", type=int, default=None)
    args = ap.parse_args()

    import numpy as np
    if args.gpu:
        import cupy as xp  # type: ignore[import-not-found]

        def to_np(a):
            return xp.asnumpy(a)
    else:
        xp = np

        def to_np(a):
            return a

    spectra, psnrs = [], []
    for count, (seq, keep, payload) in enumerate(load_chunks(args.capture)):
        if args.max_chirps and count >= args.max_chirps:
            break
        vals = unpack_bfp(payload) if keep == "bfp" else unpack12(payload)
        if keep == "bfp":
            keep = None  # exact after dequantization; treat like packed12
        per_rx = len(vals) // args.rx
        rx = xp.asarray(vals, dtype=float).reshape(args.rx, per_rx)

        # All paths produce the HANNING-windowed spectrum (fft(w*s)): the
        # unwindowed spectrum's leakage skirts are not sparse and break the
        # reconstruction (see radar_dsp.range_profiles).
        if keep is None and args.simulate_keep is None:
            # exact capture: plain windowed FFT range profile
            spectra.append(np.asarray(to_np(
                xp.fft.fft(rx * xp.hanning(per_rx), axis=-1))))
            continue

        if keep is None:  # offline masking of an exact capture
            keep = args.simulate_keep
            n = per_rx
            idx_pairs = mask_indices(seq, n // 2, keep)
            idx = [i for p in idx_pairs for i in (2 * p, 2 * p + 1)]
            w = xp.hanning(n)
            truth = xp.fft.fft(rx * w, axis=-1)
            y = (rx * w)[:, idx]
        else:  # live Sub12 capture: per_rx holds only the kept samples
            n = per_rx * SUB12_BLOCK_PAIRS // keep
            idx_pairs = mask_indices(seq, n // 2, keep)
            idx = [i for p in idx_pairs for i in (2 * p, 2 * p + 1)]
            truth = None
            y = rx * xp.hanning(n)[xp.asarray(idx)]

        x = fista(xp, y.astype(complex), xp.asarray(idx), n, args.iters,
                  args.lam, group=args.rx)
        if truth is not None:
            err = float(xp.mean(xp.abs(x - truth) ** 2))
            peak = float(xp.abs(truth).max())
            psnrs.append(10 * np.log10(peak * peak / err) if err else np.inf)
        spectra.append(np.asarray(to_np(x)))

    out = np.stack(spectra) if spectra else np.zeros((0,))
    np.save(args.out, out)
    msg = f"{len(spectra)} chirps -> {args.out} {out.shape}"
    if psnrs:
        msg += f" | simulated keep {args.simulate_keep}/16: " \
               f"PSNR median {np.median(psnrs):.1f} dB min {min(psnrs):.1f} dB"
    print(msg)


if __name__ == "__main__":
    main()
