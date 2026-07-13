"""Quantify what each lossy streaming format costs, on a real capture.

Input: a full-rate Packed12 capture.bin (exact samples = ground truth).
Each variant re-encodes the raw samples the way the firmware would, runs the
FULL production pipeline (range_profiles_batch -> assemble_frame ->
doppler_process -> cfar_2d) and is scored against the exact-sample pipeline:

  - per-chirp range-spectrum PSNR (median / min over chirps)
  - range-Doppler map error (dB, whole map) + strongest-cell level error
  - MTI noise-floor rise (median RD magnitude over non-target cells)
  - CFAR detection match vs truth (+-1 bin): hits / misses / false alarms,
    mean SNR delta on hits

Variants:
  bfp        block-floating-point quantization (16-sample blocks, shared
             exponent 0..4, round-to-nearest int8 mantissa) - simulates the
             proposed Format::Bfp16 BEFORE it exists in firmware
  sub12 k    random subsampling + FISTA L1 (the existing Sub12 path); the
             samples are actually 12-bit packed and fed through
             range_profiles_batch so the production decode/recon runs

Usage:
  uv run cs_diagnostics.py capture.bin [--frames 10] [--iters 60]
                           [--keep 4 6 8 12] [--lam 0.005 0.01 0.02 0.05]
                           [--sweep] [--out diag.npz] [--cpu]
  (without --sweep: one sub12 row per --keep value at the first --lam)
"""

import argparse

import numpy as np

from radar_dsp import (Config, _mask_row, assemble_frame, backend, cfar_2d,
                       doppler_process, iter_chunks, range_profiles_batch,
                       unpack12)


# ---------------------------------------------------------------------------
# Capture loading: frames of raw int16 samples
# ---------------------------------------------------------------------------

def load_frames(path, max_frames):
    """Return (cfg, frames): frames = list of [(seq, samples[rx, per_rx])],
    each a complete, gap-free, frame-aligned run of chirps_per_frame chirps.
    The capture must be Packed12 (exact samples are the ground truth)."""
    data = open(path, "rb").read()
    cfg, frames, buf = None, [], []
    for item in iter_chunks(data):
        if item[0] == "meta":
            cfg = Config(item[1])
            if cfg.keep is not None:
                raise SystemExit("capture is Sub12 - ground truth needs a "
                                 "full Packed12 capture (set keep 16)")
            buf.clear()
            continue
        if cfg is None:
            continue
        _, seq, keep, payload = item
        if keep is not None:
            raise SystemExit("Sub12 chunk in capture - need pure Packed12")
        if buf and seq != buf[-1][0] + 1:
            buf.clear()
        if not buf and seq % cfg.chirps_per_frame != 0:
            continue  # align to a true frame boundary (seq counts from 0)
        buf.append((seq, unpack12(payload).reshape(cfg.rx, -1)))
        if len(buf) == cfg.chirps_per_frame:
            frames.append(buf)
            buf = []
            if len(frames) >= max_frames:
                break
    if cfg is None or not frames:
        raise SystemExit("no complete frames (need an A55E descriptor + a "
                         "full gap-free frame)")
    return cfg, frames


# ---------------------------------------------------------------------------
# Firmware-mirroring encoders
# ---------------------------------------------------------------------------

def pack12(samples):
    """int16 samples -> 12-bit packed bytes (inverse of spi_capture.unpack12)."""
    s = np.asarray(samples, dtype=np.int16).ravel().astype(np.uint16) & 0xFFF
    s = s.reshape(-1, 2)
    b = np.empty((len(s), 3), dtype=np.uint8)
    b[:, 0] = s[:, 0] & 0xFF
    b[:, 1] = (s[:, 0] >> 8) | ((s[:, 1] & 0xF) << 4)
    b[:, 2] = s[:, 1] >> 4
    return b.tobytes()


def bfp_quantize(samples):
    """Round-trip int16 samples through the proposed Bfp16 encoding: blocks
    of 16 share exp = clamp(bitlen(max|s|) - 7, 0, 4); mantissa = round-to-
    nearest arithmetic shift, clamped to [-128, 127]; value = mant << exp.
    Mirrors the planned firmware pack_block_bfp exactly."""
    s = np.asarray(samples, dtype=np.int32)
    blocks = s.reshape(*s.shape[:-1], -1, 16)
    maxabs = np.abs(blocks).max(axis=-1, keepdims=True)
    bitlen = np.zeros_like(maxabs)
    nz = maxabs > 0
    bitlen[nz] = np.floor(np.log2(maxabs[nz])).astype(np.int32) + 1
    exp = np.clip(bitlen - 7, 0, 4)
    half = np.where(exp > 0, 1 << np.maximum(exp - 1, 0), 0)
    mant = np.clip((blocks + half) >> exp, -128, 127)
    return ((mant << exp).astype(np.int16)).reshape(s.shape)


def sub12_payload(seq, samples, keep):
    """Keep the firmware-masked samples of one chirp and 12-bit pack them,
    producing exactly the payload a live Sub12(keep) chunk would carry."""
    per_rx = samples.shape[-1]
    idx = _mask_row(seq, per_rx * keep // 16, keep)  # n == per_rx here
    return pack12(samples[:, idx])


# ---------------------------------------------------------------------------
# Pipeline + metrics
# ---------------------------------------------------------------------------

def run_pipeline(xp, to_np, frames_chirps, cfg, iters, lam):
    """(seq, payload) frames -> per-frame dict of spectra/RD/detections via
    the production path."""
    out = []
    for chirps in frames_chirps:
        spec = range_profiles_batch(xp, chirps, cfg, iters=iters, lam=lam)
        profs = [(seq, spec[i]) for i, (seq, _) in enumerate(chirps)]
        X = assemble_frame(xp, profs, cfg)
        rd, _ = doppler_process(xp, X)
        rd = to_np(rd)
        out.append({"spec": to_np(spec), "rd": rd, "dets": cfar_2d(rd)})
    return out


def match_dets(truth, var):
    """+-1-bin matching -> (hits, misses, false_alarms, mean SNR delta)."""
    used = [False] * len(var)
    hits, dsnr = 0, []
    for d, r, snr in truth:
        best = None
        for i, (dv, rv, sv) in enumerate(var):
            if not used[i] and abs(dv - d) <= 1 and abs(rv - r) <= 1:
                best = i
                break
        if best is not None:
            used[best] = True
            hits += 1
            dsnr.append(var[best][2] - snr)
    misses = len(truth) - hits
    fas = used.count(False)
    return hits, misses, fas, (float(np.mean(dsnr)) if dsnr else 0.0)


def floor_mask(rd_shape, dets):
    """True on cells farther than 1 bin from every truth detection."""
    m = np.ones(rd_shape, dtype=bool)
    for d, r, _ in dets:
        m[max(d - 1, 0):d + 2, max(r - 1, 0):r + 2] = False
    return m


def score(truth_frames, var_frames):
    """Aggregate all metrics of a variant against the truth frames."""
    psnr, rd_err, peak_err, floor_rise = [], [], [], []
    hits = misses = fas = 0
    dsnr = []
    for t, v in zip(truth_frames, var_frames):
        ts, vs = t["spec"], v["spec"]
        peak = np.abs(ts).max()
        err = np.mean(np.abs(vs - ts) ** 2, axis=(1, 2))  # per chirp
        psnr.extend(10 * np.log10(peak ** 2 / np.maximum(err, 1e-30)))
        rd_err.append(20 * np.log10(np.linalg.norm(v["rd"] - t["rd"])
                                    / np.linalg.norm(t["rd"])))
        pk = np.unravel_index(np.argmax(t["rd"]), t["rd"].shape)
        peak_err.append(20 * np.log10(max(v["rd"][pk], 1e-30)
                                      / max(t["rd"][pk], 1e-30)))
        m = floor_mask(t["rd"].shape, t["dets"])
        floor_rise.append(20 * np.log10(np.median(v["rd"][m])
                                        / max(np.median(t["rd"][m]), 1e-30)))
        h, mi, fa, ds = match_dets(t["dets"], v["dets"])
        hits, misses, fas = hits + h, misses + mi, fas + fa
        if h:
            dsnr.append(ds)
    return {
        "psnr_med": float(np.median(psnr)), "psnr_min": float(np.min(psnr)),
        "rd_err_db": float(np.median(rd_err)),
        "peak_err_db": float(np.median(peak_err)),
        "floor_rise_db": float(np.median(floor_rise)),
        "hits": hits, "misses": misses, "false_alarms": fas,
        "snr_delta_db": float(np.mean(dsnr)) if dsnr else 0.0,
    }


def inject_movers(frames, cfg, specs):
    """Add synthetic moving targets to the real samples (clutter and noise
    stay real). specs: list of "range_bin:amp:doppler_hz" strings; each adds
    amp*cos(2*pi*bin*t/N + 2*pi*fd*t_slow + rx phase) to every chirp."""
    t_rep = cfg.burst_periodicity * 100e-9
    n = cfg.samples
    t = np.arange(n)
    out = []
    for fr in frames:
        nf = []
        for seq, s in fr:
            burst = (seq // cfg.chirps_per_burst) % cfg.bursts_per_frame
            add = np.zeros((cfg.rx, n))
            for spec in specs:
                b, a, fd = (float(v) for v in spec.split(":"))
                for r in range(cfg.rx):
                    add[r] += a * np.cos(2 * np.pi * b * t / n
                                         + 2 * np.pi * fd * burst * t_rep
                                         + 0.4 * r)
            s2 = np.clip(s.astype(np.int32) + np.round(add).astype(np.int32),
                         -2048, 2047).astype(np.int16)
            nf.append((seq, s2))
        out.append(nf)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--iters", type=int, default=60)
    ap.add_argument("--keep", type=int, nargs="+", default=[4, 6, 8, 12])
    ap.add_argument("--lam", type=float, nargs="+",
                    default=[0.005, 0.01, 0.02, 0.05])
    ap.add_argument("--sweep", action="store_true",
                    help="full keep x lam grid (default: first lam only)")
    ap.add_argument("--out", default=None, help="save metrics table as .npz")
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--inject", nargs="+", default=None,
                    metavar="BIN:AMP:HZ",
                    help="add synthetic movers to the real samples, e.g. "
                         "40:30:8 (range bin 40, amplitude 30 LSB, 8 Hz)")
    args = ap.parse_args()

    xp, to_np, name = backend(args.cpu)
    cfg, frames = load_frames(args.capture, args.frames)
    if args.inject:
        frames = inject_movers(frames, cfg, args.inject)
    print(f"backend={name}  {cfg}  frames={len(frames)}")

    # Ground truth: the exact samples through the Packed12 path.
    def payload_frames(encode):
        return [[(seq, encode(seq, s)) for seq, s in fr] for fr in frames]

    truth = run_pipeline(
        xp, to_np, payload_frames(lambda _s, s: pack12(s)), cfg,
        args.iters, args.lam[0])
    ndets = sum(len(t["dets"]) for t in truth)
    print(f"truth: {ndets} detections over {len(frames)} frames")

    rows = []

    def add(label, var_frames):
        rows.append((label, score(truth, var_frames)))

    # BFP: quantize, then the exact-FFT path (cfg.keep stays None).
    add("bfp16", run_pipeline(
        xp, to_np, payload_frames(lambda _s, s: pack12(bfp_quantize(s))),
        cfg, args.iters, args.lam[0]))

    lams = args.lam if args.sweep else args.lam[:1]
    for keep in args.keep:
        if cfg.samples % 32:
            print(f"skip sub12 {keep}: samples % 32 != 0")
            continue
        pf = payload_frames(lambda seq, s, k=keep: sub12_payload(seq, s, k))
        for lam in lams:
            kcfg = Config(cfg.raw)
            kcfg.keep = keep
            add(f"sub12 {keep}/16 lam={lam}",
                run_pipeline(xp, to_np, pf, kcfg, args.iters, lam))

    hdr = (f"{'variant':<24}{'psnr med/min':>14}{'rd_err':>8}{'peak':>7}"
           f"{'floor':>7}{'hit':>5}{'miss':>6}{'fa':>5}{'dsnr':>7}")
    print(hdr)
    print("-" * len(hdr))
    for label, s in rows:
        print(f"{label:<24}{s['psnr_med']:7.1f}/{s['psnr_min']:5.1f} "
              f"{s['rd_err_db']:>8.1f}{s['peak_err_db']:>7.2f}"
              f"{s['floor_rise_db']:>7.2f}{s['hits']:>5}{s['misses']:>6}"
              f"{s['false_alarms']:>5}{s['snr_delta_db']:>7.2f}")

    if args.out:
        np.savez(args.out,
                 labels=np.array([r[0] for r in rows]),
                 metrics=np.array([[r[1][k] for k in sorted(r[1])]
                                   for r in rows]),
                 keys=np.array(sorted(rows[0][1])))
        print(f"saved -> {args.out}")


if __name__ == "__main__":
    main()
