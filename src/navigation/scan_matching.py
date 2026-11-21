"""2D scan matching (ICP) of RPLidar scans: where the robot is, from what
the LiDAR sees, against a reference scan.

Used by the calibrations (navigation/calibration), by the odometry accuracy
test (tests/testOdometryAccuracy.py, live on the Pi) and by its offline tools
(Pi or Mac). Pure NumPy; uses SciPy's KD-tree when available, brute-force
nearest neighbours otherwise.

Frames
    Robot frame: origin at the midpoint between the two drive wheels,
    x forward, y left, θ CCW-positive (same as navigation/sensor_thread.Odometry).
    RPLidar: 0° = straight ahead, angle increases CLOCKWISE, distance in mm.
    The scans are robot-oriented: where the LiDAR is read, its mounting
    direction is already removed (navigation.lidar_reader.rotate_scan).

Every scan taken at a stop is aligned against the reference scan taken at
the start. The transform that aligns them IS the robot's pose in the start
frame. Points are converted to the robot frame (LiDAR mounting offset
applied) BEFORE matching, so the result is the pose of the wheel-axle
midpoint, not of the LiDAR.
"""
import math

import numpy as np

try:
    from scipy.spatial import cKDTree as _KDTree
except ImportError:           # SciPy is optional
    _KDTree = None

# ── LiDAR mounting: one source, the robot profile (through navigation/slam.py)
from navigation.slam import LIDAR_DX, LIDAR_DY, SCAN_MIN_M

# Extra rotation applied to the scans: none, they are already robot-oriented.
# Kept as a parameter to try another mounting direction on a recorded run
# (plotOdometryRun.py --lidar-yaw).
LIDAR_YAW_DEG = 0.0

SCAN_MAX_M    = 6.0      # ignore returns farther than this (noisy, sparse)
VOXEL_M       = 0.02     # downsample clouds to one point per 2 cm cell
INLIER_M      = 0.05     # a matched point closer than this counts as an inlier


# ── scans → point clouds ────────────────────────────────────────────────────
def scan_to_points(scan, dx=LIDAR_DX, dy=LIDAR_DY, yaw_deg=LIDAR_YAW_DEG,
                   min_m=SCAN_MIN_M, max_m=SCAN_MAX_M):
    """One RPLidar scan [(quality, angle_deg_cw, dist_mm), ...] → (N,2) points
    in the ROBOT frame, metres."""
    arr = np.asarray(scan, dtype=float).reshape(-1, 3)
    d = arr[:, 2] / 1000.0
    keep = (d > min_m) & (d < max_m)
    d, a = d[keep], np.radians(arr[keep, 1])
    lx, ly = d * np.cos(a), -d * np.sin(a)          # CW angle → CCW frame
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.column_stack([c * lx - s * ly + dx, s * lx + c * ly + dy])


def voxel_downsample(p, size=VOXEL_M):
    """One point per grid cell: the CENTROID of the cell's points. (Keeping
    the first point instead biased merged clouds by up to ~0.5 cm.)"""
    if len(p) == 0:
        return p
    keys = np.floor(p / size).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    out = np.zeros((len(counts), 2))
    np.add.at(out, inv, p)
    return out / counts[:, None]


def scans_to_cloud(scans, **kw):
    """Several scans taken at the same (stationary) pose → one downsampled cloud."""
    pts = [scan_to_points(s, **kw) for s in scans]
    pts = [p for p in pts if len(p)]
    return voxel_downsample(np.vstack(pts)) if pts else np.zeros((0, 2))


# ── ICP ────────────────────────────────────────────────────────────────────
def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def transform(p, pose):
    x, y, th = pose
    return p @ _rot(th).T + np.array([x, y])


class _Nearest:
    def __init__(self, ref):
        self.ref = ref
        self.tree = _KDTree(ref) if _KDTree is not None else None

    def query(self, q):
        if self.tree is not None:
            return self.tree.query(q)
        dist = np.empty(len(q))
        idx = np.empty(len(q), dtype=np.int64)
        for i in range(0, len(q), 512):          # chunked to bound memory
            d2 = ((q[i:i + 512, None, :] - self.ref[None, :, :]) ** 2).sum(-1)
            j = d2.argmin(1)
            idx[i:i + 512] = j
            dist[i:i + 512] = np.sqrt(d2[np.arange(len(j)), j])
        return dist, idx


def _rigid_2d(a, b):
    """Least-squares rotation+translation mapping points a onto b (Kabsch)."""
    ca, cb = a.mean(0), b.mean(0)
    h = (a - ca).T @ (b - cb)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[1] *= -1
        r = vt.T @ u.T
    return r, cb - r @ ca


def icp(src, nearest, init, max_iter=60, tol=1e-6):
    """Align src (robot-frame cloud) to the reference. Returns the robot pose
    (x, y, θ) in the reference frame plus fit quality."""
    x, y, th = init
    max_corr = 0.5
    for it in range(max_iter):
        w = transform(src, (x, y, th))
        d, j = nearest.query(w)
        keep = d < max_corr
        if keep.sum() < 20:
            break
        keep &= d <= np.quantile(d[keep], 0.9)   # trim the worst 10 %
        r, t = _rigid_2d(w[keep], nearest.ref[j[keep]])
        x, y = r @ np.array([x, y]) + t
        dth = math.atan2(r[1, 0], r[0, 0])
        th += dth
        max_corr = max(0.08, min(max_corr, 4.0 * float(np.median(d[keep]))))
        if abs(dth) < tol and math.hypot(*t) < tol:
            break
    d, _ = nearest.query(transform(src, (x, y, th)))
    inl = d < INLIER_M
    rms = float(np.sqrt(np.mean(d[inl] ** 2))) if inl.any() else float('inf')
    return {'x': float(x), 'y': float(y), 'theta': float(th),
            'rms_m': rms, 'inlier_frac': float(inl.mean()) if len(d) else 0.0,
            'n_points': int(len(src)), 'iters': it + 1}


def match_pose(src, ref, init, yaw_starts_deg=(0, -6, 6, -12, 12)):
    """ICP from several initial headings around `init`; keep the best fit.
    The odometry guess is usually within a few degrees, multi-start guards
    against the occasional local minimum."""
    nearest = _Nearest(ref)
    best = None
    for dyaw in yaw_starts_deg:
        r = icp(src, nearest, (init[0], init[1], init[2] + math.radians(dyaw)))
        if best is None or (r['inlier_frac'], -r['rms_m']) > (best['inlier_frac'], -best['rms_m']):
            best = r
    best['theta'] = wrap(best['theta'])
    return best


def match_stop(scans, ref, init, min_inlier_frac=0.5, **kw):
    """Ground truth at one stop: match EACH scan on its own against the
    reference, then take the median pose. Merging the scans first biased the
    result by up to ~0.5 cm; separate matches also give an uncertainty (the
    spread between scans of the same stationary pose). Returns None if no
    scan matched well."""
    res = [match_pose(scans_to_cloud([s], **kw), ref, init) for s in scans]
    good = [r for r in res if r['inlier_frac'] >= min_inlier_frac]
    if not good:
        return None
    xs = np.array([r['x'] for r in good])
    ys = np.array([r['y'] for r in good])
    th0 = good[0]['theta']
    ths = np.array([th0 + wrap(r['theta'] - th0) for r in good])   # unwrap around the first
    return {'x': float(np.median(xs)), 'y': float(np.median(ys)),
            'theta': wrap(float(np.median(ths))),
            'spread_m': float(max(np.ptp(xs), np.ptp(ys))),
            'spread_deg': float(math.degrees(np.ptp(ths))),
            'inlier_frac': float(np.mean([r['inlier_frac'] for r in good])),
            'rms_m': float(np.mean([r['rms_m'] for r in good])),
            'n_scans': len(good)}


def wrap(a):
    """Wrap angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))

