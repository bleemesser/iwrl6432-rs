"""Vital signs (breathing + heart rate) from slow-time phase at a range bin.

TI's demo computes this in firmware and only streams waveforms; we have the
raw ADC so it's done here from first principles: the complex range-profile
value at the target's bin, one sample per frame, carries the chest's radial
displacement as phase (d = lambda * phi / (4 pi)). Breathing is 0.1-0.6 Hz,
heartbeat 0.8-2.2 Hz - both far below the frame rate, so slow time at
10-16 fps is plenty.

The monitored range bin is chosen by the caller (nearest confirmed track)
but LOCKED here: an unwrapped phase history is only meaningful on one bin,
so the lock moves only when the candidate disagrees persistently.
"""

import numpy as np
from scipy import signal


BREATH_BAND = (0.1, 0.6)   # Hz
HEART_BAND = (0.8, 2.2)    # Hz


class VitalsMonitor:
    def __init__(self, fps, lambda_m, window_s=30.0):
        self.fps = fps
        self.lam = lambda_m
        self.n = max(int(window_s * fps), 16)
        self.ring = np.zeros(self.n, dtype=complex)
        self.fill = 0
        self.bin = None
        self._cand = None
        self._cand_count = 0

    def _lock_bin(self, candidate):
        """Sticky bin lock: move only after ~3 s of persistent disagreement
        (> 2 bins), and reset the phase history when it moves."""
        if self.bin is None:
            self.bin = candidate
            return
        if abs(candidate - self.bin) <= 2:
            self._cand_count = 0
            return
        if self._cand is not None and abs(candidate - self._cand) <= 2:
            self._cand_count += 1
        else:
            self._cand = candidate
            self._cand_count = 1
        if self._cand_count >= int(3 * self.fps):
            self.bin = self._cand
            self._cand_count = 0
            self.fill = 0

    def feed(self, profile, candidate_bin):
        """profile: complex range profile (one per frame, e.g. burst-mean of
        virtual antenna 0). Returns the vitals dict or None while warming."""
        self._lock_bin(int(candidate_bin))
        self.ring = np.roll(self.ring, -1)
        self.ring[-1] = profile[self.bin]
        self.fill = min(self.fill + 1, self.n)
        if self.fill < int(8 * self.fps):  # need ~8 s before rates mean much
            return None
        return self._analyze()

    def _analyze(self):
        z = self.ring[-self.fill:]
        # displacement (mm) from unwrapped phase
        disp = np.unwrap(np.angle(z)) * self.lam / (4 * np.pi) * 1e3
        disp = disp - np.polyval(np.polyfit(np.arange(len(disp)), disp, 1),
                                 np.arange(len(disp)))  # detrend drift
        nyq = self.fps / 2
        out = {"bin": self.bin}
        for name, (lo, hi) in (("breath", BREATH_BAND), ("heart", HEART_BAND)):
            hi_c = min(hi, 0.95 * nyq)
            if hi_c <= lo:
                out[name + "_wave"] = np.zeros(0)
                out[name + "_bpm"] = 0.0
                continue
            sos = signal.butter(3, [lo, hi_c], "bandpass", fs=self.fps,
                                output="sos")
            w = signal.sosfiltfilt(sos, disp)
            out[name + "_wave"] = w[-int(12 * self.fps):]
            # rate = spectral peak in band, parabolic-interpolated
            nfft = max(4096, len(w))
            sp = np.abs(np.fft.rfft(w * np.hanning(len(w)), nfft))
            f = np.fft.rfftfreq(nfft, 1 / self.fps)
            band = (f >= lo) & (f <= hi_c)
            if not band.any():
                out[name + "_bpm"] = 0.0
                continue
            i = np.nonzero(band)[0][np.argmax(sp[band])]
            if 0 < i < len(sp) - 1 and sp[i] > 0:
                den = sp[i - 1] - 2 * sp[i] + sp[i + 1]
                di = 0.5 * (sp[i - 1] - sp[i + 1]) / den if den else 0.0
            else:
                di = 0.0
            out[name + "_bpm"] = float((f[i] + di * (f[1] - f[0])) * 60)
        # presence gate: breathing displacement of a live chest is >~0.1 mm
        out["deviation_mm"] = float(np.std(out["breath_wave"])) \
            if len(out["breath_wave"]) else 0.0
        if out["deviation_mm"] < 0.05:
            out["status"] = "no presence"
        elif out["deviation_mm"] < 0.12:
            out["status"] = "holding breath?"
        else:
            out["status"] = "presence"
        return out
