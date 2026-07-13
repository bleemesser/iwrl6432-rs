"""Live radar scope for the IWRL6432 raw-ADC SPI stream.

Polls the firmware over the FTDI SPI link (like spi_capture.py), runs the full
DSP pipeline (radar_dsp.py) in a worker thread, and shows a pyqtgraph
dashboard: a range-azimuth PPI "arc" with detections, a range-Doppler map, the
live range profile, a range-time waterfall, a micro-Doppler spectrogram, a
birds-eye point cloud, and a health readout.

The stream is self-describing (the firmware A55E metadata frame), so no profile
name is needed - axes are labelled in meters / m.s from the live config. Sub12
(CS) profiles are L1-reconstructed; a CUDA GPU (CuPy) is used automatically if
present, else NumPy on the CPU.

Usage (deps from the uv env; GPU via `uv sync --extra gpu`):
  uv run radar_scope.py [--cpu] [--freq 15e6] [--iters 60] [--thresh-db 11]
  uv run radar_scope.py --replay capture.bin        # offline, no hardware
"""

import argparse
import struct
import threading
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

import radar_dsp as dsp
import radar_track
import radar_vitals


HIST = 200  # waterfall / spectrogram history depth (frames)
NEAR_BIN = 3  # range bins near DC skipped for micro-Doppler peak-bin selection


class Products:
    """One processed frame's display products (all NumPy)."""
    __slots__ = ("cfg", "rd", "ra", "rprof", "pc", "range_axis", "vel_axis",
                 "peak_doppler", "stats", "tracks", "vitals")


class Reader(threading.Thread):
    """Poll (or replay) and decode; publish the latest Products.

    The link poll loop and the DSP run on SEPARATE threads: Sub12 (FISTA)
    reconstruction can be slower than real time, and doing it inline starves
    the FTDI polls, overflows the firmware ring and no frame ever completes
    (the "scope freezes on CS profiles" failure). The poll thread hands the
    newest COMPLETE frame of raw payloads to the DSP thread through a 1-deep
    mailbox; when the DSP is behind, older frames are dropped (counted)."""

    def __init__(self, args):
        super().__init__(daemon=True)
        self.args = args
        self.xp, self.to_np, self.backend = dsp.backend(args.cpu)
        self._lock = threading.Lock()
        self._latest = None
        self._pending = None            # (cfg, [(seq, payload), ...])
        self._pending_ev = threading.Event()
        self._dsp = None
        self._rd_ema = None
        self._ra_ema = None
        self.running = True
        self.denoise = False  # toggled by the scope's Denoise button
        self.declutter = True  # display profile uses the MTI (static-removed) map
        self._prev_dets = set()  # last frame's (doppler, range) cells
        self._tracker = None  # radar_track.Tracker, rebuilt on fps change
        self._tracker_fps = None
        self._vitals = None   # radar_vitals.VitalsMonitor, rebuilt with tracker
        # floor z in point-cloud coords: 0 when --overhead (z = height above
        # floor), else -sensor_height (wall/tripod mount).
        self._floor_z = 0.0 if args.overhead is not None else -args.sensor_height
        self.stats = dict(chirps=0, frames=0, dropped=0, gaps=0, bad=0,
                          aborts=0, backend=self.backend, cps=0.0)
        self._t0 = time.time()

    def latest(self):
        with self._lock:
            return self._latest

    def stop(self):
        """Shut down both threads BEFORE interpreter exit: a daemon thread
        killed mid-libusb call trips libusb_ref_device's refcount assertion
        and hangs the process on window close."""
        self.running = False
        self._pending_ev.set()  # wake the DSP thread so it sees running=False
        self.join(timeout=3)
        if self._dsp is not None:
            self._dsp.join(timeout=3)

    # -- data sources ------------------------------------------------------
    def _poll_source(self):
        """Yield raw chunk bytes batches from the live FTDI link."""
        from spi_capture import SpiLink
        link = SpiLink(freq=self.args.freq)
        try:
            while self.running:
                batch, _ = link.poll()
                if batch is None:
                    self.stats["bad"] += 1
                    time.sleep(0.005)
                    continue
                if not batch:
                    time.sleep(0.005)
                    continue
                yield batch
        finally:
            link.close()

    def _replay_source(self):
        """Yield the whole capture file once, then loop."""
        data = open(self.args.replay, "rb").read()
        while self.running:
            yield data
            time.sleep(0.2)

    # -- poll thread: decode fast, never block on DSP ------------------------
    def run(self):
        self._dsp = threading.Thread(target=self._dsp_loop, daemon=True)
        self._dsp.start()
        src = self._replay_source() if self.args.replay else self._poll_source()
        try:
            self._poll_loop(src)
        finally:
            src.close()  # runs _poll_source's finally -> ctrl.terminate()

    def _poll_loop(self, src):
        # Frames are aligned by absolute seq (the firmware resets seq at
        # capture init, so frame index = seq // chirps_per_frame). A chirp
        # lost to a link gap is ZERO-FILLED rather than the whole partial
        # frame discarded - on marginal chunk sizes (hires 2160 B) gaps come
        # every second or two and clearing stalled the display for hundreds
        # of ms; a zeroed chirp is invisible after MTI. Frames missing more
        # than half their chirps are still dropped (counted as gaps).
        cfg = None
        fidx = None
        slots = []
        for blob in src:
            if not self.running:
                break
            for item in dsp.iter_chunks(blob):
                if item[0] == "meta":
                    new = dsp.Config(item[1])
                    if cfg is None or new.raw != cfg.raw:
                        cfg = new
                        fidx = None
                    continue
                if cfg is None:
                    continue
                _, seq, _, payload = item
                self.stats["chirps"] += 1
                nf = cfg.chirps_per_frame
                if seq // nf != fidx:
                    if fidx is not None:
                        self._finish_frame(cfg, fidx, slots)
                    fidx = seq // nf
                    slots = [None] * nf
                slots[seq % nf] = bytes(payload)

    def _finish_frame(self, cfg, fidx, slots):
        nf = cfg.chirps_per_frame
        have = [p for p in slots if p is not None]
        if not have:
            return
        if len(have) < nf:
            self.stats["gaps"] += 1
            if len(have) < nf // 2:
                return
        base = fidx * nf
        frame = [(base + i, p) for i, p in enumerate(slots)]  # p None = hole
        with self._lock:
            if self._pending is not None:
                self.stats["dropped"] += 1
            self._pending = (cfg, frame)
        self._pending_ev.set()

    # -- DSP thread: newest complete frame only ------------------------------
    def _dsp_loop(self):
        while self.running:
            if not self._pending_ev.wait(timeout=0.2):
                continue
            with self._lock:
                item = self._pending
                self._pending = None
                self._pending_ev.clear()
            if item is None:
                continue
            cfg, raw = item
            # Reconstruct present chirps; a hole gets the frame's MEAN
            # profile so its MTI residual is ~zero. (Zero-filling instead
            # makes the hole a slow-time impulse: bright full-height
            # velocity stripes across the range-Doppler map.)
            present = [(seq, p) for seq, p in raw if p is not None]
            profs = dsp.range_profiles_batch(self.xp, present, cfg,
                                             iters=self.args.iters)
            # TDMA: even/odd chirps are different TX antennas - fill a hole
            # with the mean of its own parity only.
            cpb = cfg.chirps_per_burst
            means = {}
            for tx in {seq % cpb for seq, _ in present}:
                rows = [i for i, (seq, _) in enumerate(present) if seq % cpb == tx]
                means[tx] = profs[rows].mean(axis=0)
            # A hole filled from the WRONG parity's mean leaves an MTI
            # residual impulse (TX phase centers differ) - full-height
            # Doppler stripes. If any hole's parity has no mean, drop the
            # frame instead.
            if any(p is None and seq % cpb not in means for seq, p in raw):
                self.stats["gaps"] += 1
                continue
            out, k = [], 0
            for seq, p in raw:
                if p is None:
                    out.append((seq, means[seq % cpb]))
                else:
                    out.append((seq, profs[k]))
                    k += 1
            self._process(cfg, out)

    def _process(self, cfg, chirps):
        xp, to_np = self.xp, self.to_np
        X = dsp.assemble_frame(xp, chirps, cfg)
        rd, Xd = dsp.doppler_process(xp, X)
        # Remove the TDM motion phase BEFORE any angle use: without this a
        # moving target's angle estimate scatters (ghost dots at its range in
        # random directions).
        Xd = dsp.tdm_compensate(xp, Xd, cfg, sign=self.args.tdm_sign)
        ra = dsp.range_azimuth(xp, Xd)
        rd_np = np.asarray(to_np(rd))
        ra_np = np.asarray(to_np(ra))
        # --ema: incoherent averaging over frames (power domain) - trades
        # display latency for visual SNR on weak movers (Sub12 profiles).
        a = self.args.ema
        if a > 0:
            if self._rd_ema is not None and self._rd_ema.shape == rd_np.shape:
                rd_np = np.sqrt(a * self._rd_ema ** 2 + (1 - a) * rd_np ** 2)
                ra_np = np.sqrt(a * self._ra_ema ** 2 + (1 - a) * ra_np ** 2)
            self._rd_ema, self._ra_ema = rd_np, ra_np
        # CFAR runs on the raw map (it needs the noise statistics the
        # denoiser removes); only the DISPLAY maps are denoised.
        dets = dsp.cfar_2d(rd_np, thresh_db=self.args.thresh_db)
        # Denoise toggle (set by the scope UI): per-range-bin robust noise
        # floor (MAD over Doppler / azimuth) + power-domain subtraction.
        if self.denoise:
            rd_np = dsp.denoise_mag(rd_np, axis=0)
            ra_np = dsp.denoise_mag(ra_np, axis=1)
            # A detection whose cell the denoiser zeroed would paint a dot on
            # visually empty space - drop it.
            dets = [t for t in dets if rd_np[t[0], t[1]] > 0]
        # 2-frame confirmation (+-1 bin): CFAR fires on single-frame noise
        # flickers; requiring a neighbor detection in the previous frame too
        # keeps dots on persistent targets only.
        cur = {(d, r) for d, r, _ in dets}
        dets = [t for t in dets
                if any((t[0] + i, t[1] + j) in self._prev_dets
                       for i in (-1, 0, 1) for j in (-1, 0, 1))]
        self._prev_dets = cur
        # Harmonic/multipath gate: RX compression on a strong close target
        # puts ghost returns at ~2x/3x its range (visible as waterfall
        # "harmonics"). Drop a detection if a much stronger one sits at ~1/k
        # of its range with a similar velocity.
        nd = rd_np.shape[0]
        c = nd // 2
        gated = []
        for d, r, snr in dets:
            ghost = False
            for d2, r2, snr2 in dets:
                if snr2 < snr + 6:
                    continue
                for k in (2, 3):
                    # a k-th harmonic scales BOTH range and Doppler by k
                    if abs(r - k * r2) <= max(2, 0.08 * r) and abs(
                        (d - c) - k * (d2 - c)
                    ) <= max(2, nd // 10):
                        ghost = True
            if not ghost:
                gated.append((d, r, snr))
        dets = gated
        pc = dsp.detections_pointcloud(xp, Xd, dets, cfg)
        # Overhead mount (--overhead H): sensor on the ceiling looking down.
        # Sensor frame -> room frame: boresight (sensor y) points at the
        # floor, so floor_x = x (azimuth), floor_y = z (elevation direction),
        # height above floor = H - forward distance. The tracker and all
        # spatial views then work in floor coordinates.
        if self.args.overhead is not None:
            h = self.args.overhead
            pc = [(x, z, h - y, rng, vel, az, el, snr)
                  for x, y, z, rng, vel, az, el, snr in pc]
        # Cluster + track (ported TI-demo pipeline, see radar_track.py).
        m = cfg.metrics()
        fps = m["fps"]
        if self._tracker is None or self._tracker_fps != cfg.raw:
            self._tracker = radar_track.Tracker(fps, floor_z=self._floor_z)
            self._vitals = radar_vitals.VitalsMonitor(fps, m["lambda_m"])
            self._tracker_fps = cfg.raw  # any profile change resets both
        tracks = [dict(id=t.id, x=t.x[0], y=t.x[1], vx=t.x[2], vy=t.x[3],
                       z=t.z, top=t.top, zext=t.zext, xyext=t.xyext,
                       posture=t.posture, fall=t.fall_frames > 0)
                  for t in self._tracker.update(pc)]
        # Micro-Doppler range bin: ALWAYS from the MTI range-Doppler map, which
        # highlights movers. (Skip the first few bins - residual DC / antenna
        # coupling near range 0 otherwise wins argmax and the spectrogram shows
        # that clutter cell's noise instead of a mover.) This must NOT use the
        # NN/mean display profile below: that averages over slow time, so movers
        # cancel and the peak pins to static -> NN "breaks" the micro-Doppler.
        mti_prof = np.asarray(to_np(rd_np)).max(axis=0)
        peak_bin = int(np.argmax(mti_prof[NEAR_BIN:]) + NEAR_BIN)
        # Range profile for DISPLAY (plot + waterfall). Declutter ON: the MTI
        # profile (rd.max) - complex-domain static removal, so stationary
        # reflectors (walls/furniture, e.g. the strong ~4 m returns) drop out
        # and movers stay, no noise-floor amplification. Declutter OFF: the full
        # static-inclusive profile (mean per-chirp magnitude).
        if self.declutter:
            disp_lin = mti_prof
        else:
            M = np.abs(np.stack([np.asarray(to_np(prof)) for _, prof in chirps]))
            disp_lin = M.reshape(-1, M.shape[-1]).mean(axis=0)
        rprof = 20 * np.log10(np.asarray(disp_lin, dtype=float) + 1e-6)
        peak_doppler = np.asarray(to_np(xp.abs(Xd[:, :, peak_bin]).sum(axis=0)))
        if self.denoise:
            peak_doppler = dsp.denoise_mag(peak_doppler, axis=0)
        # Vitals: slow-time complex sample at the subject's range bin, from
        # the PRE-MTI cube (breathing is quasi-static; MTI would eat it).
        # Candidate bin = nearest confirmed track's range, else the post-MTI
        # peak (a lone breathing person still dominates the MTI residual).
        rr = m["range_res_m"]
        if tracks:
            t0 = min(tracks, key=lambda t: t["id"])  # oldest track
            if self.args.overhead is not None:
                rng = np.sqrt(t0["x"]**2 + t0["y"]**2
                              + (self.args.overhead - t0["z"])**2)
            else:
                rng = np.sqrt(t0["x"]**2 + t0["y"]**2)
            cand = int(np.clip(rng / rr, 1, rd_np.shape[1] - 1))
        else:
            cand = peak_bin
        prof0 = np.asarray(to_np(X[0].mean(axis=0)))  # burst-mean, vant 0
        vitals = self._vitals.feed(prof0, cand)

        p = Products()
        p.cfg = cfg
        p.rd = rd_np
        p.ra = ra_np
        p.rprof = rprof
        p.pc = pc
        p.range_axis = cfg.range_axis(rd_np.shape[1])
        p.vel_axis = cfg.velocity_axis()
        p.peak_doppler = peak_doppler
        p.tracks = tracks
        p.vitals = vitals
        self.stats["frames"] += 1
        dt = time.time() - self._t0
        self.stats["cps"] = self.stats["chirps"] / dt if dt else 0.0
        p.stats = dict(self.stats)
        with self._lock:
            self._latest = p


class Scope(QtWidgets.QWidget):
    """pyqtgraph dashboard; pulls the latest Products on a timer."""

    def __init__(self, reader, max_range, overhead=None, zone=None):
        super().__init__()
        self.reader = reader
        self.max_range = max_range
        self.overhead = overhead
        self.zone = zone            # (x0, x1, y0, y1, z0, z1) or None
        self.win3d = None
        self.setWindowTitle("IWRL6432 radar scope"
                            + (" [overhead]" if overhead is not None else ""))
        self.resize(1800, 900)
        cmap = pg.colormap.get("inferno")
        self._lut = cmap.getLookupTable(0.0, 1.0, 256)
        self._ppi_map = None       # (r_idx, a_idx, extent) cache, per cfg shape
        self._wf = None            # waterfall ring
        self._spec = None          # spectrogram ring

        glw = pg.GraphicsLayoutWidget()
        lay = QtWidgets.QVBoxLayout(self)

        # Intensity controls. Auto = per-frame min/max (the old behavior).
        # Manual: each image keeps a captured reference (lo, hi) and the two
        # sliders pick the displayed floor/ceiling as percentages of that
        # span, so thresholding tracks across all four maps. "Capture" takes
        # each image's CURRENT data min/max as the new reference.
        bar = QtWidgets.QHBoxLayout()
        self.chk_auto = QtWidgets.QCheckBox("Auto levels")
        self.chk_auto.setChecked(False)
        btn_cap = QtWidgets.QPushButton("Capture levels")
        btn_cap.clicked.connect(self._capture_levels)
        self.sl_floor = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sl_ceil = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sl_floor.setRange(0, 100); self.sl_floor.setValue(0)
        self.sl_ceil.setRange(0, 100); self.sl_ceil.setValue(100)
        self.btn_dn = QtWidgets.QPushButton("Denoise")
        self.btn_dn.setCheckable(True)
        self.btn_dn.toggled.connect(
            lambda on: setattr(self.reader, "denoise", bool(on)))
        self.btn_dc = QtWidgets.QPushButton("Declutter")
        self.btn_dc.setCheckable(True)
        self.btn_dc.setChecked(self.reader.declutter)
        self.btn_dc.toggled.connect(
            lambda on: setattr(self.reader, "declutter", bool(on)))
        btn_3d = QtWidgets.QPushButton("3D view")
        btn_3d.clicked.connect(self._open_3d)
        bar.addWidget(self.btn_dn)
        bar.addWidget(self.btn_dc)
        bar.addWidget(btn_3d)
        bar.addWidget(self.chk_auto)
        bar.addWidget(btn_cap)
        bar.addWidget(QtWidgets.QLabel("floor"))
        bar.addWidget(self.sl_floor, stretch=1)
        bar.addWidget(QtWidgets.QLabel("ceiling"))
        bar.addWidget(self.sl_ceil, stretch=1)
        lay.addLayout(bar)
        self._ref = {}      # name -> captured (lo, hi) reference
        self._last = {}     # name -> latest data array (for Capture)

        lay.addWidget(glw)

        self.p_ppi = glw.addPlot(row=0, col=0, title="range-azimuth PPI (m)")
        self.img_ppi = pg.ImageItem(); self.img_ppi.setLookupTable(self._lut)
        self.p_ppi.addItem(self.img_ppi); self.p_ppi.setAspectLocked(True)
        self.sc_ppi = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(0, 255, 0, 200), size=9)
        self.p_ppi.addItem(self.sc_ppi)

        self.p_rd = glw.addPlot(row=0, col=1, title="range-Doppler (m vs m/s)")
        self.img_rd = pg.ImageItem(); self.img_rd.setLookupTable(self._lut)
        self.p_rd.addItem(self.img_rd)
        self.p_rd.setLabel("bottom", "velocity", "m/s"); self.p_rd.setLabel("left", "range", "m")

        pc_title = ("point cloud (floor plan, m)" if overhead is not None
                    else "point cloud (birds-eye, m)")
        self.p_pc = glw.addPlot(row=0, col=2, title=pc_title)
        self.sc_pc = pg.ScatterPlotItem(pen=None, size=12)
        self.p_pc.addItem(self.sc_pc); self.p_pc.setAspectLocked(True)
        self.p_pc.setXRange(-max_range, max_range); self.p_pc.setYRange(0, max_range)
        # Kalman tracks: red rings + ID labels + fading history trail.
        self.sc_trk = pg.ScatterPlotItem(
            pen=pg.mkPen(255, 60, 60, width=2), brush=None, size=18, symbol="o")
        self.sc_trail = pg.ScatterPlotItem(pen=None, size=6)
        self.p_pc.addItem(self.sc_trail)
        self.p_pc.addItem(self.sc_trk)
        self.sc_trk_ppi = pg.ScatterPlotItem(
            pen=pg.mkPen(255, 60, 60, width=2), brush=None, size=16, symbol="o")
        self.p_ppi.addItem(self.sc_trk_ppi)
        self._trk_labels = {}          # track id -> pg.TextItem on p_pc
        self._trail = []               # list of frames: [(x, y), ...]

        # Side elevation: horizontal = pc row y (forward distance, or floor y
        # when --overhead), vertical = pc row z (height). Elevation comes from
        # the 2-element interferometer, so expect coarse but real angles.
        el_title = ("side view (floor-y vs height, m)" if overhead is not None
                    else "side elevation (forward vs height, m)")
        self.p_el = glw.addPlot(row=0, col=3, title=el_title)
        self.sc_el = pg.ScatterPlotItem(pen=None, size=10)
        self.p_el.addItem(self.sc_el); self.p_el.setAspectLocked(True)
        if overhead is not None:
            self.p_el.setXRange(-max_range, max_range)
            self.p_el.setYRange(0, overhead)
        else:
            self.p_el.setXRange(0, max_range)
            self.p_el.setYRange(-max_range / 2, max_range / 2)

        # Zone (e.g. the bed): rectangle on the birds-eye view, colored by
        # occupancy (any confirmed track inside).
        self.zone_rect = None
        if zone is not None:
            x0, x1, y0, y1 = zone[:4]
            self.zone_rect = pg.PlotCurveItem(
                [x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0],
                pen=pg.mkPen(120, 220, 120, width=2))
            self.p_pc.addItem(self.zone_rect)

        # Vital signs: breathing (blue) + heart (red) displacement waveforms
        # from radar_vitals; rates + status in the title.
        self.p_vit = glw.addPlot(row=1, col=3, title="vitals (warming up)")
        self.p_vit.setLabel("left", "displacement", "mm")
        self.crv_breath = self.p_vit.plot(pen=pg.mkPen((80, 160, 255), width=2))
        self.crv_heart = self.p_vit.plot(pen=pg.mkPen((255, 90, 90), width=2))

        self.p_prof = glw.addPlot(row=1, col=0, title="range profile (dB)")
        self.curve = self.p_prof.plot(pen="y")
        self.p_prof.setLabel("bottom", "range", "m")

        self.p_wf = glw.addPlot(row=1, col=1, title="range-time (waterfall)")
        self.img_wf = pg.ImageItem(); self.img_wf.setLookupTable(self._lut)
        self.p_wf.addItem(self.img_wf); self.p_wf.setLabel("left", "range", "m")

        self.p_spec = glw.addPlot(row=1, col=2, title="micro-Doppler (peak range)")
        self.img_spec = pg.ImageItem(); self.img_spec.setLookupTable(self._lut)
        self.p_spec.addItem(self.img_spec); self.p_spec.setLabel("left", "velocity", "m/s")

        self.health = glw.addLabel("starting...", row=2, col=0, colspan=4, justify="left")

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

    def _capture_levels(self):
        for name, data in self._last.items():
            self._ref[name] = (float(np.min(data)), float(np.max(data)))

    def _set_img(self, img, name, data):
        """setImage with the shared leveling policy (see the control bar)."""
        self._last[name] = data
        if self.chk_auto.isChecked():
            img.setImage(data, autoLevels=True)
            return
        if name not in self._ref:
            self._ref[name] = (float(np.min(data)), float(np.max(data)))
        lo, hi = self._ref[name]
        span = hi - lo if hi > lo else 1.0
        f = lo + span * self.sl_floor.value() / 100.0
        c = lo + span * self.sl_ceil.value() / 100.0
        img.setImage(data, autoLevels=False, levels=(f, max(c, f + 1e-9)))

    def _ppi_indices(self, nr, na, range_axis):
        """Cartesian->polar sample-index maps for the PPI wedge, cached."""
        if self._ppi_map and self._ppi_map[0] == (nr, na):
            return self._ppi_map[1]
        rmax = range_axis[-1]
        W = 240
        xs = np.linspace(-rmax, rmax, W)
        ys = np.linspace(0, rmax, W)
        gx, gy = np.meshgrid(xs, ys)
        rng = np.sqrt(gx ** 2 + gy ** 2)
        az = np.degrees(np.arctan2(gx, gy))
        r_idx = np.clip((rng / rmax * (nr - 1)).astype(int), 0, nr - 1)
        a_idx = np.clip(np.interp(az, dsp.AZ_ANGLE_AXIS, np.arange(na)).astype(int), 0, na - 1)
        mask = rng <= rmax
        maps = (r_idx, a_idx, mask, (-rmax, rmax, 0, rmax))
        self._ppi_map = ((nr, na), maps)
        return maps

    def update(self):
        p = self.reader.latest()
        if p is None:
            return
        nr = p.rd.shape[1]

        # PPI wedge
        r_idx, a_idx, mask, (x0, x1, y0, y1) = self._ppi_indices(nr, p.ra.shape[1], p.range_axis)
        wedge = p.ra[r_idx, a_idx] * mask
        self._set_img(self.img_ppi, "ppi", wedge.T)
        self.img_ppi.setRect(QtCore.QRectF(x0, y0, x1 - x0, y1 - y0))
        # PPI dots stay in the SENSOR frame (recomputed from range/az, which
        # survive the --overhead floor remap of the x/y/z columns).
        if p.pc:
            self.sc_ppi.setData(
                [d[3] * np.sin(np.radians(d[5])) for d in p.pc],
                [d[3] * np.cos(np.radians(d[5])) for d in p.pc])
        else:
            self.sc_ppi.setData([], [])

        # range-Doppler (velocity on x, range on y)
        self._set_img(self.img_rd, "rd", p.rd.T)
        v0, v1 = p.vel_axis[0], p.vel_axis[-1]
        self.img_rd.setRect(QtCore.QRectF(v0, 0, v1 - v0, p.range_axis[-1]))

        # point cloud, colored by velocity
        if p.pc:
            vmax = max(p.cfg.metrics()["v_max_ms"], 1e-3)
            spots = [{"pos": (d[0], d[1]),
                      "brush": pg.mkBrush(*self._vel_color(d[4], vmax)),
                      "size": 8 + min(d[7], 30) / 3} for d in p.pc]
            self.sc_pc.setData(spots)
            self.sc_el.setData([{"pos": (d[1], d[2]),
                                 "brush": pg.mkBrush(*self._vel_color(d[4], vmax)),
                                 "size": 8 + min(d[7], 30) / 3} for d in p.pc])
        else:
            self.sc_pc.setData([])
            self.sc_el.setData([])
        if self.win3d is not None and self.win3d.isVisible():
            self.win3d.update_data(p)

        # Kalman tracks: rings on both spatial views, ID + posture labels +
        # trail on birds-eye.
        tp = [(t["x"], t["y"]) for t in p.tracks]
        self.sc_trk.setData([x for x, _ in tp], [y for _, y in tp])
        self.sc_trk_ppi.setData([x for x, _ in tp], [y for _, y in tp])
        live = set()
        for t in p.tracks:
            tid = t["id"]
            live.add(tid)
            if tid not in self._trk_labels:
                lbl = pg.TextItem("", color=(255, 120, 120))
                self.p_pc.addItem(lbl)
                self._trk_labels[tid] = lbl
            txt = str(tid)
            if t["posture"]:
                txt += f" {t['posture']}"
            if t["fall"]:
                txt += "  FALL!"
            self._trk_labels[tid].setText(txt)
            self._trk_labels[tid].setColor(
                (255, 40, 40) if t["fall"] else (255, 120, 120))
            self._trk_labels[tid].setPos(t["x"], t["y"])
        for tid in [t for t in self._trk_labels if t not in live]:
            self.p_pc.removeItem(self._trk_labels.pop(tid))
        # zone occupancy coloring
        if self.zone_rect is not None:
            x0, x1, y0, y1 = self.zone[:4]
            occ = any(x0 <= t["x"] <= x1 and y0 <= t["y"] <= y1
                      for t in p.tracks)
            self.zone_rect.setPen(pg.mkPen(
                (255, 120, 120) if occ else (120, 220, 120), width=2))
        # vitals panel
        if p.vitals is not None:
            v = p.vitals
            n = len(v["breath_wave"])
            if n:
                tax = np.arange(n) / p.cfg.metrics()["fps"]
                self.crv_breath.setData(tax, v["breath_wave"])
                hw = v["heart_wave"][-n:]
                self.crv_heart.setData(np.arange(len(hw)) / p.cfg.metrics()["fps"], hw)
            self.p_vit.setTitle(
                f"vitals: {v['status']}  breath {v['breath_bpm']:.0f}/min  "
                f"heart {v['heart_bpm']:.0f}/min  bin {v['bin']}  "
                f"dev {v['deviation_mm']:.2f}mm")
        self._trail.append(tp)
        self._trail = self._trail[-30:]
        pts, brushes = [], []
        n = len(self._trail)
        for i, frame in enumerate(self._trail):
            a = int(160 * (i + 1) / n)
            for x, y in frame:
                pts.append((x, y))
                brushes.append(pg.mkBrush(255, 255, 255, a))
        self.sc_trail.setData(
            [x for x, _ in pts], [y for _, y in pts], brush=brushes)

        # range profile
        self.curve.setData(p.range_axis, p.rprof)

        # waterfall (range-time)
        if self._wf is None or self._wf.shape[1] != nr:
            self._wf = np.zeros((HIST, nr))
        self._wf = np.roll(self._wf, -1, axis=0)
        self._wf[-1] = p.rprof
        self._set_img(self.img_wf, "wf", self._wf)
        self.img_wf.setRect(QtCore.QRectF(0, 0, HIST, p.range_axis[-1]))

        # micro-Doppler spectrogram at the peak range bin
        nd = len(p.peak_doppler)
        if self._spec is None or self._spec.shape[1] != nd:
            self._spec = np.zeros((HIST, nd))
        self._spec = np.roll(self._spec, -1, axis=0)
        self._spec[-1] = 20 * np.log10(p.peak_doppler + 1e-6)
        self._set_img(self.img_spec, "spec", self._spec)
        self.img_spec.setRect(QtCore.QRectF(0, p.vel_axis[0], HIST, p.vel_axis[-1] - p.vel_axis[0]))

        s, m = p.stats, p.cfg.metrics()
        self.health.setText(
            f"{p.cfg}  |  backend={s['backend']}  cps={s['cps']:.0f}  frames={s['frames']}  "
            f"dropped={s['dropped']}  gaps={s['gaps']}  bad={s['bad']}  |  res={m['range_res_m']*100:.1f}cm  "
            f"v_max=+-{m['v_max_ms']:.2f}m/s  fps={m['fps']:.0f}  |  targets={len(p.pc)}")

    def _open_3d(self):
        if self.win3d is None:
            try:
                self.win3d = View3D(self.max_range, self.overhead,
                                    zone=self.zone,
                                    floor_z=self.reader._floor_z)
            except Exception as e:  # PyOpenGL missing / no GL context
                QtWidgets.QMessageBox.warning(
                    self, "3D view",
                    f"could not create the OpenGL view: {e}\n"
                    "(run `uv sync` to pull pyopengl)")
                return
        self.win3d.show()
        self.win3d.raise_()

    @staticmethod
    def _vel_color(v, vmax):
        t = np.clip((v / vmax + 1) / 2, 0, 1)  # 0=approach(blue) 1=recede(red)
        return int(255 * t), int(80), int(255 * (1 - t)), 220


# Kelly-style max-contrast track colors (RGBA 0..1), reused modulo id.
TRACK_COLORS = [
    (1.00, 0.70, 0.00, 1.0), (0.50, 0.24, 0.46, 1.0),
    (1.00, 0.41, 0.00, 1.0), (0.65, 0.74, 0.84, 1.0),
    (0.76, 0.00, 0.13, 1.0), (0.81, 0.63, 0.38, 1.0),
    (0.50, 0.44, 0.40, 1.0), (0.00, 0.49, 0.20, 1.0),
]

# 12 edges of a unit box as GLLinePlotItem 'lines' segment pairs.
_BOX_EDGES = np.array([(a, b) for a, b in [
    (0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4),
    (0, 4), (1, 5), (2, 6), (3, 7)]]).ravel()


def _box_lines(x0, x1, y0, y1, z0, z1):
    v = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1)
                  for x in (x0, x1)])
    return v[_BOX_EDGES]


PERSIST_FRAMES = 12  # fading point-cloud history in the 3D view


class View3D(QtWidgets.QWidget):
    """Pop-out orbitable 3D view: fading persistent point cloud, per-track
    wireframe person boxes (Kelly colors) with posture/height/FALL labels,
    and the optional occupancy zone box.

    Axes = the pc rows' x/y/z: sensor frame normally (y = boresight, z = up),
    floor frame with --overhead (z = height above floor). Grid on z=0."""

    def __init__(self, max_range, overhead, zone=None, floor_z=0.0):
        super().__init__()
        from pyqtgraph import opengl as gl  # deferred: needs pyopengl
        self.gl = gl
        self.floor_z = floor_z
        self.setWindowTitle("IWRL6432 3D point cloud")
        self.resize(900, 750)
        lay = QtWidgets.QVBoxLayout(self)
        self.view = gl.GLViewWidget()
        lay.addWidget(self.view)
        self.view.setCameraPosition(distance=max(2 * max_range, 4.0),
                                    elevation=25, azimuth=-60)
        grid = gl.GLGridItem()
        grid.setSize(2 * max_range, 2 * max_range)
        grid.setSpacing(1, 1)
        grid.translate(0, 0, floor_z)
        self.view.addItem(grid)
        ax = gl.GLAxisItem()
        ax.setSize(max_range, max_range, overhead if overhead else max_range / 2)
        self.view.addItem(ax)
        self.sc = gl.GLScatterPlotItem(pxMode=True, size=8)
        self.sc.setGLOptions("translucent")
        self.view.addItem(self.sc)
        self.zone_item = None
        if zone is not None:
            x0, x1, y0, y1 = zone[:4]
            z0, z1 = (zone[4], zone[5]) if len(zone) == 6 else (floor_z, floor_z + 1.0)
            self.zone_item = gl.GLLinePlotItem(
                pos=_box_lines(x0, x1, y0, y1, z0, z1), mode="lines",
                color=(0.4, 0.9, 0.4, 0.9), width=2, antialias=True)
            self.view.addItem(self.zone_item)
            self.zone = zone
        self._hist = []            # persistent cloud: [(pos, col), ...]
        self._boxes = {}           # track id -> (GLLinePlotItem, GLTextItem)

    def update_data(self, p):
        # persistent cloud with age-fading alpha
        if p.pc:
            pos = np.array([[d[0], d[1], d[2]] for d in p.pc])
            vmax = max(p.cfg.metrics()["v_max_ms"], 1e-3)
            col = np.array([[c / 255 for c in Scope._vel_color(d[4], vmax)]
                            for d in p.pc])
        else:
            pos, col = np.zeros((0, 3)), np.zeros((0, 4))
        self._hist.append((pos, col))
        self._hist = self._hist[-PERSIST_FRAMES:]
        n = len(self._hist)
        allp, allc = [], []
        for i, (hp, hc) in enumerate(self._hist):
            if len(hp) == 0:
                continue
            c = hc.copy()
            c[:, 3] *= (i + 1) / n
            allp.append(hp)
            allc.append(c)
        if allp:
            self.sc.setData(pos=np.vstack(allp), color=np.vstack(allc))
        else:
            self.sc.setData(pos=np.zeros((0, 3)), color=np.zeros((0, 4)))

        # person boxes: cluster-extent wireframe from floor-ish to top
        gl = self.gl
        live = set()
        for t in p.tracks:
            tid = t["id"]
            live.add(tid)
            color = (1.0, 0.2, 0.2, 1.0) if t["fall"] else \
                TRACK_COLORS[tid % len(TRACK_COLORS)]
            r = max(t["xyext"], 0.2)
            ztop = t["top"]
            zbot = ztop - max(t["zext"], 0.4)
            if t["posture"]:  # floor is known: box stands on it
                zbot = min(zbot, self.floor_z) if t["posture"] == "lying" \
                    else self.floor_z
            lines = _box_lines(t["x"] - r, t["x"] + r,
                               t["y"] - r, t["y"] + r, zbot, ztop)
            if tid not in self._boxes:
                box = gl.GLLinePlotItem(mode="lines", width=2, antialias=True)
                txt = gl.GLTextItem(color=(255, 255, 255, 255))
                self.view.addItem(box)
                self.view.addItem(txt)
                self._boxes[tid] = (box, txt)
            box, txt = self._boxes[tid]
            box.setData(pos=lines, color=color)
            label = str(tid)
            if t["posture"]:
                h = t["top"] - self.floor_z
                label += f" {t['posture']} {h:.2f}m"
            if t["fall"]:
                label += "  FALL!"
            txt.setData(pos=(t["x"], t["y"], ztop + 0.15), text=label)
        for tid in [k for k in self._boxes if k not in live]:
            box, txt = self._boxes.pop(tid)
            self.view.removeItem(box)
            self.view.removeItem(txt)
        # zone occupancy color
        if self.zone_item is not None:
            x0, x1, y0, y1 = self.zone[:4]
            occ = any(x0 <= t["x"] <= x1 and y0 <= t["y"] <= y1
                      for t in p.tracks)
            self.zone_item.setData(color=(0.95, 0.35, 0.35, 0.9) if occ
                                   else (0.4, 0.9, 0.4, 0.9))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cpu", action="store_true", help="force NumPy (no CuPy)")
    ap.add_argument("--replay", help="process a capture.bin instead of live FTDI")
    ap.add_argument("--freq", type=float, default=20e6, help="SPI clock Hz")
    ap.add_argument("--iters", type=int, default=60, help="FISTA iters (Sub12)")
    ap.add_argument("--thresh-db", type=float, default=11.0, help="CFAR threshold")
    ap.add_argument("--ema", type=float, default=0.0,
                    help="frame-averaging factor 0..0.95 (visual SNR boost "
                         "for weak movers, at the cost of display latency)")
    ap.add_argument("--max-range", type=float, default=8.0, help="plot range limit m")
    ap.add_argument("--tdm-sign", type=int, default=1, choices=(-1, 0, 1),
                    help="TDM motion-phase correction: 1/-1 = sign, 0 = off")
    ap.add_argument("--overhead", type=float, default=None, metavar="H",
                    help="ceiling-mount mode: sensor at height H (m) looking "
                         "down; point cloud/tracks become floor x/y + height")
    ap.add_argument("--sensor-height", type=float, default=1.0,
                    help="wall/tripod mount height (m) above the floor; used "
                         "for posture + fall classification (ignored with "
                         "--overhead, where the floor is z=0)")
    ap.add_argument("--zone", default=None, metavar="X0:X1,Y0:Y1[,Z0:Z1]",
                    help="occupancy zone (e.g. the bed) in point-cloud "
                         "coords; drawn on birds-eye + 3D, colored by "
                         "track occupancy")
    args = ap.parse_args()
    if args.zone is not None:
        parts = [float(v) for rng in args.zone.split(",")
                 for v in rng.split(":")]
        if len(parts) not in (4, 6):
            ap.error("--zone needs X0:X1,Y0:Y1 or X0:X1,Y0:Y1,Z0:Z1")
        args.zone = parts

    reader = Reader(args)
    reader.start()

    app = pg.mkQApp("IWRL6432 radar scope")
    scope = Scope(reader, args.max_range, overhead=args.overhead,
                  zone=args.zone)
    scope.show()
    try:
        app.exec()
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
