"""Clustering + multi-object tracking for the scope's CFAR detections.

Ported from an earlier clustering script that consumed the TI demo's
pregenned point clouds, adapted for this pipeline's sparser clouds:
- fixed PHYSICAL clustering scales (meters / m/s) instead of a per-frame
  StandardScaler, which is unstable when a frame has only a few points;
- hand-rolled O(n^2) DBSCAN (n is tens at most; drops the sklearn dep);
- Kalman dt taken from the actual frame rate;
- tracks are only reported once seen in MIN_VISIBLE frames, so a surviving
  flicker still doesn't paint a track.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment


def dbscan(feats, eps, min_samples):
    """Labels for `feats` [n, d]: cluster id per point, -1 = noise."""
    n = len(feats)
    labels = np.full(n, -1)
    if n == 0:
        return labels
    d = np.linalg.norm(feats[:, None, :] - feats[None, :, :], axis=2)
    neigh = d <= eps
    core = neigh.sum(axis=1) >= min_samples
    cid = 0
    for i in range(n):
        if labels[i] != -1 or not core[i]:
            continue
        stack = [i]
        labels[i] = cid
        while stack:
            j = stack.pop()
            if not core[j]:
                continue
            for k in np.nonzero(neigh[j])[0]:
                if labels[k] == -1:
                    labels[k] = cid
                    stack.append(k)
        cid += 1
    return labels


class KalmanTracker:
    """One object: constant-velocity Kalman filter over [x, y, vx, vy]."""

    count = 0

    def __init__(self, xy, dt, z=0.0, stats=None):
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        # z is measured (elevation interferometer) but not filtered: the
        # kinematic state stays 2D; z / extent stats are EMA-smoothed for
        # display + posture classification.
        self.z = z
        st = stats or (z, z, 0.3, 1)
        self.top = st[1]            # cluster max height
        self.zext = st[1] - st[0]   # vertical extent
        self.xyext = st[2]          # horizontal radius
        self.npts = st[3]
        # TI fall rule (Applications_Visualizer fall_detection.py): fall =
        # current top height < proportion * top height ~2.5 s ago.
        self.top_hist = []          # (filled by Tracker; len = 2.5 s)
        self.fall_frames = 0        # >0 -> display "FALL"
        self.fall_pending = 0       # consecutive frames meeting the fall rule
        self.posture = ""
        self.P = np.diag([0.5, 0.5, 2.0, 2.0])
        self.F = np.array([[1, 0, dt, 0], [0, 1, 0, dt],
                           [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        # Process noise sized for a maneuvering person (~1 m/s^2 accel):
        # too-small Q + the deliberately loose cross-range R makes the
        # filter's ANGLE response so stiff a lateral/oblique walker renders
        # as a radial approach.
        self.Q = np.diag([0.05, 0.05, 2.0, 2.0]) * dt
        # Measurement covariance is per-measurement (see Tracker._meas_cov):
        # radar noise is anisotropic - tight in range, loose cross-range.
        self.id = KalmanTracker.count
        KalmanTracker.count += 1
        self.age = 0
        self.total_visible = 1
        self.invisible = 0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.invisible += 1
        return self.x

    def innovation(self, xy, R):
        """(Mahalanobis distance^2, innovation, S) of measurement `xy` with
        per-measurement covariance R."""
        y = xy - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        return float(y @ np.linalg.solve(S, y)), y, S

    def update(self, xy, R, z=None, stats=None):
        if z is not None:
            self.z = 0.7 * self.z + 0.3 * z
        if stats is not None:
            zmin, zmax, xyr, npts = stats
            self.top = 0.7 * self.top + 0.3 * zmax
            self.zext = 0.7 * self.zext + 0.3 * max(zmax - zmin, 0.1)
            self.xyext = 0.7 * self.xyext + 0.3 * max(xyr, 0.15)
            self.npts = npts
        y = xy - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.total_visible += 1
        self.invisible = 0


class Tracker:
    """DBSCAN over (x, y, w*vel) -> cluster centroids -> Hungarian-matched
    Kalman tracks with coasting, pruning and merge of duplicates."""

    MIN_VISIBLE = 5  # matched frames before a track is reported
    GATE2 = 9.21     # chi-square 99% for 2 dof: Mahalanobis association gate

    # Posture thresholds on top-of-cluster height above the floor (m).
    LYING_MAX = 0.6
    SITTING_MAX = 1.15
    FALL_PROPORTION = 0.55   # tighter than TI's 0.6 default
    FALL_WINDOW_S = 2.5
    FALL_SHOW_S = 5.0
    FALL_MIN_FROM = 1.0      # reference must have been upright (m above floor)
    FALL_MIN_DROP = 0.6      # and the drop must be this big in absolute terms
    FALL_CONFIRM_S = 1.5     # condition must hold this long before firing
                             # (a real fall stays down; a bend/crouch doesn't)

    def __init__(self, fps, eps=0.4, min_samples=2, doppler_weight=0.3,
                 max_invisible=10, merge_thresh=0.4, singleton_snr_db=18.0,
                 sigma_range=0.10, sigma_az_deg=5.0, floor_z=None):
        self.singleton_snr_db = singleton_snr_db
        # floor_z: z of the floor in point-cloud coordinates (0 in --overhead
        # mode where z is already height above floor; -sensor_height for a
        # wall/tripod mount). None disables posture/fall classification.
        self.floor_z = floor_z
        self.fall_hist_len = max(int(self.FALL_WINDOW_S * fps), 4)
        self.fall_show = int(self.FALL_SHOW_S * fps)
        self.fall_confirm = max(int(self.FALL_CONFIRM_S * fps), 2)
        self.fall_med = max(int(0.5 * fps), 2)   # median window (~0.5 s)
        self.dt = 1.0 / max(fps, 1.0)
        self.eps = eps
        self.min_samples = min_samples
        self.w = doppler_weight
        self.max_invisible = max_invisible
        self.merge_thresh = merge_thresh
        self.sigma_range = sigma_range
        self.sigma_az = np.radians(sigma_az_deg)
        self.trackers = []

    def _meas_cov(self, xy):
        """Anisotropic measurement covariance at position `xy`: tight along
        the radial (range accuracy ~a bin), loose cross-range (range *
        azimuth error - the 4-element array jitters several degrees, more
        when the target moves). Euclidean gating with isotropic noise is
        what made lateral movers drop their track."""
        rng = max(np.linalg.norm(xy), 0.3)
        u = xy / rng                        # radial unit vector
        t = np.array([-u[1], u[0]])         # cross-range unit vector
        sr2 = self.sigma_range ** 2
        st2 = max(0.15, rng * self.sigma_az) ** 2
        return sr2 * np.outer(u, u) + st2 * np.outer(t, t)

    def _centroids(self, pc):
        """pc rows: (x, y, z, rng, vel, az, el, snr) -> (centroids, weak
        singletons); each det is (xy, z, (zmin, zmax, xy_radius, npts))."""
        if not pc:
            return [], []
        feats = np.array([[p[0], p[1], self.w * p[4]] for p in pc])
        zs = np.array([p[2] for p in pc])
        labels = dbscan(feats, self.eps, self.min_samples)
        cents = []
        for cid in set(labels) - {-1}:
            m = labels == cid
            xy = feats[m, :2]
            xyr = float(np.linalg.norm(xy - xy.mean(axis=0), axis=1).max())
            stats = (float(zs[m].min()), float(zs[m].max()), xyr, int(m.sum()))
            cents.append((xy.mean(axis=0), float(zs[m].mean()), stats))
        # DBSCAN noise points only become NEW targets when clearly strong:
        # weak isolated detections (multipath/harmonic residue) otherwise
        # seed ghost tracks meters away. Weak singletons are returned
        # separately - update() still lets them feed an EXISTING track,
        # because a laterally moving person (near zero Doppler, in the MTI
        # clutter notch) often yields exactly one weak detection per frame
        # and starving the track kills it mid-motion.
        snrs = [p[7] for p in pc]
        weak = []
        for i in np.nonzero(labels == -1)[0]:
            z = float(zs[i])
            det = (feats[i, :2], z, (z, z, 0.2, 1))
            if snrs[i] >= self.singleton_snr_db:
                cents.append(det)
            else:
                weak.append(det)
        return cents, weak

    def _merge(self):
        keep = []
        for t in sorted(self.trackers, key=lambda t: -t.age):
            if all(np.linalg.norm(t.x[:2] - k.x[:2]) >= self.merge_thresh
                   for k in keep):
                keep.append(t)
        self.trackers = keep

    def update(self, pc):
        """One frame of point-cloud rows -> list of confirmed tracks."""
        dets, weak = self._centroids(pc)
        for t in self.trackers:
            t.predict()
        # Weak singletons may sustain an existing track (they never seed
        # one): admit those inside some track's association gate.
        weak_ids = set()
        for d in weak:
            if not self.trackers:
                break
            R = self._meas_cov(d[0])
            if min(t.innovation(d[0], R)[0]
                   for t in self.trackers) < self.GATE2:
                weak_ids.add(id(d))
                dets.append(d)
        if dets and self.trackers:
            covs = [self._meas_cov(d) for d, _, _ in dets]
            cost = np.array([[t.innovation(d, R)[0]
                              for (d, _, _), R in zip(dets, covs)]
                             for t in self.trackers])
            rows, cols = linear_sum_assignment(cost)
            matched = set()
            for r, c in zip(rows, cols):
                if cost[r, c] < self.GATE2:
                    self.trackers[r].update(dets[c][0], covs[c],
                                            z=dets[c][1], stats=dets[c][2])
                    matched.add(c)
            dets = [d for i, d in enumerate(dets) if i not in matched]
        for det in dets:
            if id(det) in weak_ids:
                continue  # weak singletons sustain tracks, never seed them
            d, z, st = det
            self.trackers.append(KalmanTracker(d, self.dt, z=z, stats=st))
        self.trackers = [t for t in self.trackers
                         if t.invisible <= self.max_invisible]
        self._merge()
        self._posture_fall()
        # Report tracks that are both established (MIN_VISIBLE matches) and
        # consistently seen (>60% of their lifetime) - a ghost that gets a
        # few lucky matches while coasting never qualifies.
        return [t for t in self.trackers
                if t.total_visible >= self.MIN_VISIBLE
                and t.total_visible > 0.6 * (t.age + 1)]

    def _posture_fall(self):
        """Per-track posture label + fall detection.

        Based on TI's rule (top height < FALL_PROPORTION * height 2.5 s ago)
        but hardened for our jumpier interferometer heights: both ends are
        MEDIANS over ~0.5 s (a single-frame spike can't trigger or seed a
        reference), the reference must show the person was actually upright
        (FALL_MIN_FROM), the drop must also be large in absolute terms
        (FALL_MIN_DROP), and the condition must hold FALL_CONFIRM_S running
        before it fires."""
        if self.floor_z is None:
            for t in self.trackers:
                t.posture = ""
            return
        k = self.fall_med
        for t in self.trackers:
            top = t.top - self.floor_z          # height above the floor
            if top < self.LYING_MAX:
                t.posture = "lying"
            elif top < self.SITTING_MAX:
                t.posture = "sitting"
            else:
                t.posture = "standing"
            t.top_hist.append(top)
            if len(t.top_hist) > self.fall_hist_len:
                t.top_hist.pop(0)
            t.fall_frames = max(t.fall_frames - 1, 0)
            falling = False
            if len(t.top_hist) == self.fall_hist_len:
                ref = float(np.median(t.top_hist[:k]))    # ~2.5 s ago
                now = float(np.median(t.top_hist[-k:]))   # last ~0.5 s
                falling = (ref >= self.FALL_MIN_FROM
                           and now < self.FALL_PROPORTION * ref
                           and ref - now >= self.FALL_MIN_DROP)
            t.fall_pending = t.fall_pending + 1 if falling else 0
            if t.fall_pending >= self.fall_confirm:
                t.fall_frames = self.fall_show
