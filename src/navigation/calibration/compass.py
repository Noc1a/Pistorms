"""Compass calibration, using the magnetometer_calibration library (a git
submodule at the repo root, unmodified).

Both procedures record a CSV that the library reads with load_data(), then
calibrate with its MagnetometerCalibrator and save with save_calibration():

  3d   MANUAL. Motors off: turn the robot by hand in every direction
       (~60 s, coverage shown live) → the library's ellipsoid fit
       (calibrate()) → mag_calibration.json (used by Madgwick MARG).
  2d   AUTOMATIC. The robot pivots in 20° steps and averages the compass
       at each stop with the motors off — how the compass methods use
       it (continuous: one spin, motors running) → ellipse fit on
       x, y → mag_calibration_2d.json (used by the compass-heading
       methods), plus a deviation card.
       The library has no 2D fit, and a flat spin leaves its ellipsoid
       fit degenerate, so only the ellipse fit itself lives here; the
       result is loaded into a MagnetometerCalibrator and applied /
       saved / plotted by the library like any calibration.
       The reference heading of the spin is the gyro's Z: the IMU's axes
       must be known (the IMU calibration) and its Z vertical.

Both files are written next to the robot profile.

CSV layout: <time s | gyro heading rad>, x, y, z — load_data() skips the
first column of a 4-column CSV.

Images: the library's plot_data() — 3D trajectory (the sphere) and XY/XZ/YZ
projections — before and after calibration, plus the deviation curve in 2d,
saved as PNG next to the CSV (show=True also opens the windows).

Keep the robot ≥ 1.5 m away from laptops, speakers and metal furniture: the
calibration would otherwise learn THEIR field as if it were north.
"""
import json
import math
import os
import sys
import threading
import time
from datetime import datetime
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

from ..robot_profile import DEFAULT_PATH, RobotProfile
from .base import Calibration, CalibrationContext, files_dir, run_file
from .imu_axes import calibrate_gyro
from .moves import WheelTravel, stop_motors, turn_left

if TYPE_CHECKING:
    from pistorms.devices import AbsoluteIMU
    from ..drive_base import DriveBase

# The library is a third-party folder at the repo root, not an installable package
LIBRARY_DIR  = os.path.join(os.path.dirname(DEFAULT_PATH), 'magnetometer_calibration')
FIELD_UT     = 48.0      # local Earth field (µT) — only scales the 3D output
GYRO_CAL_SEC = 4.0       # stationary gyro bias, before the spin
SPIN_SPEED   = 15        # motor %, pivot (continuous)
SPIN_TURNS   = 2.0
SPIN_TIMEOUT = 60.0      # s
SAMPLE_DT    = 0.02      # s
# Stepped 2D calibration (default): pivot STEP_DEG, stop, then average the
# compass over STILL_SECS with the motors OFF — the condition in which the
# compass methods use it (MAG_ONLY_WHEN_STILL). The motor currents' own field
# differs between "pivoting" and "stopped", so a calibration learned while
# spinning was off by ≈5° at rest.
STEP_DEG     = 20.0
SETTLE_SECS  = 0.5
STILL_SECS   = 1.0


def mag_calibration_path(mode: str) -> str:
    """The calibration file of this mode ('3d' or '2d'), next to the profile."""
    name = 'mag_calibration.json' if mode == '3d' else 'mag_calibration_2d.json'
    return os.path.join(files_dir(), name)


def magnetometer_calibrator() -> type:
    """The library's MagnetometerCalibrator class."""
    if not os.path.isfile(os.path.join(LIBRARY_DIR, 'calibrate.py')):
        raise RuntimeError(f"the magnetometer_calibration library is not in {LIBRARY_DIR} — "
                           f"git submodule update --init magnetometer_calibration")
    if LIBRARY_DIR not in sys.path:
        sys.path.insert(0, LIBRARY_DIR)
    from calibrate import MagnetometerCalibrator
    return MagnetometerCalibrator


# ── deviation card (not in the library) ────────────────────────────────────
def deviation_rad(h: Any, cal_json: Optional[dict]) -> Any:
    """Deviation (rad) at absolute compass angle h for a calibration JSON
    written by this module; 0 if it has no deviation card. Also used by
    testOdometryAccuracy.CompassReader, so the correction lives in one place."""
    dev = cal_json.get('deviation') if cal_json else None
    if not dev:
        return np.zeros_like(h) if isinstance(h, np.ndarray) else 0.0
    c1, s1, c2, s2 = dev['coef_deg']
    return dev['sign'] * np.radians(c1 * np.cos(h) + s1 * np.sin(h)
                                    + c2 * np.cos(2 * h) + s2 * np.sin(2 * h))


def heading_residual(calibrator: Any, cal_json: Optional[dict], xyz: np.ndarray,
                     ref_heading: Any) -> tuple[np.ndarray, int, np.ndarray]:
    """Compass heading − reference heading (gyro) over a spin, constant and
    linear trend (gyro scale error) removed.
    Returns (residual deg, compass sign, absolute compass angle rad)."""
    m = calibrator.apply_calibration(xyz)
    h_abs = np.arctan2(m[:, 1], m[:, 0])
    h_abs = h_abs - deviation_rad(h_abs, cal_json)
    h = np.unwrap(h_abs)
    h -= h[0]
    th = np.asarray(ref_heading)
    sign = 1 if np.dot(h, th) > 0 else -1
    r = sign * h - th
    A = np.column_stack([np.ones_like(th), th])
    r -= A @ np.linalg.lstsq(A, r, rcond=None)[0]
    return np.degrees(r), sign, h_abs


def fit_deviation(res_deg: np.ndarray, sign: int, h_abs: np.ndarray) -> dict:
    """Compass deviation card: residual as 1×/2×-per-turn harmonics of the
    compass angle, subtracted from every later heading."""
    A = np.column_stack([np.ones_like(h_abs), np.cos(h_abs), np.sin(h_abs),
                         np.cos(2 * h_abs), np.sin(2 * h_abs)])
    coef = np.linalg.lstsq(A, res_deg, rcond=None)[0]
    return {'sign': int(sign), 'coef_deg': [float(v) for v in coef[1:]]}


# ── 2D ellipse fit (not in the library) ────────────────────────────────────
def fit_ellipse_2d(calibrator: Any, xyz: np.ndarray) -> float:
    """Least-squares conic on (x, y); stores hard iron (centre) and the 2×2
    soft-iron matrix (ellipse → circle) in the calibrator, z left untouched.
    Returns the circle radius."""
    x, y = xyz[:, 0], xyz[:, 1]
    D = np.column_stack([x * x, x * y, y * y, x, y])
    a, b, c, d, e = np.linalg.lstsq(D, np.ones(len(x)), rcond=None)[0]
    M = np.array([[a, b / 2], [b / 2, c]])
    centre = np.linalg.solve(2 * M, [-d, -e])
    f0 = centre @ M @ centre + d * centre[0] + e * centre[1]
    w, V = np.linalg.eigh(M / (1.0 - f0))        # centred: pᵀ·M·p = 1
    if np.any(w <= 0):
        raise ValueError("not an ellipse — less than a full turn, or too noisy")
    radius = 1.0 / math.sqrt(math.sqrt(w[0] * w[1]))
    A_1 = np.eye(3)
    A_1[:2, :2] = radius * (V @ np.diag(np.sqrt(w)) @ V.T)
    calibrator.b = np.array([[centre[0]], [centre[1]], [0.0]])
    calibrator.A_1 = A_1
    calibrator.F = radius
    return radius


# ── recording on the robot (the only part that touches hardware) ──────────
def _write_csv(path: str, first_col_name: str, first_col: list, xyz: list) -> None:
    with open(path, 'w') as f:
        f.write(f"{first_col_name},x,y,z\n")
        for v, (x, y, z) in zip(first_col, xyz):
            f.write(f"{v:.6f},{x:.4f},{y:.4f},{z:.4f}\n")
    print(f"Saved raw data: {path}")


def _gyro_ready(imu: 'AbsoluteIMU', z_sign: int) -> np.ndarray:
    """Set the gyro filter as the robot's Odometry does and measure the
    bias (rad/s, Z already flipped)."""
    from ..sensor_thread import Odometry      # here: needs picamera2
    imu.set_gyro_filter(Odometry.GYRO_FILTER)
    return calibrate_gyro(imu, GYRO_CAL_SEC, z_sign)


def record_2d(drive: 'DriveBase', imu: 'AbsoluteIMU', path: str) -> None:
    """One continuous spin of SPIN_TURNS, motors running."""
    z_sign = drive.profile.gyro_z_sign
    bias = _gyro_ready(imu, z_sign)
    wheels = WheelTravel(drive)
    xyz, ref = [], []
    th, watch = 0.0, {}
    print(f"Spinning {SPIN_TURNS:.0f} turns at speed {SPIN_SPEED} ...")
    drive.turn_left(SPIN_SPEED)                      # ramped (DriveBase.ACCELERATION)
    t_prev = t0 = time.monotonic()
    try:
        while True:
            try:
                _, m, g = imu.acc_mag_gyr()
            except OSError:
                m = g = None
            turns = wheels.turn_rad() / (2 * math.pi)
            wheels.check(watch)
            now = time.monotonic()
            dt, t_prev = now - t_prev, now
            if g is not None and m is not None:
                th += (math.radians(g[2] * z_sign) - bias[2]) * dt
                xyz.append(m)
                ref.append(th)
            if turns >= SPIN_TURNS:
                break
            if now - t0 > SPIN_TIMEOUT:
                print(f"⚠ spin TIMEOUT after {SPIN_TIMEOUT:.0f}s ({turns:.2f} turns)")
                break
            time.sleep(max(0.0, SAMPLE_DT - (time.monotonic() - now)))
    finally:
        stop_motors(drive)
    print(f"  {len(xyz)} samples, gyro says {math.degrees(th):.0f}°")
    _write_csv(path, 'gyro_heading_rad', ref, xyz)


def record_2d_stepped(drive: 'DriveBase', imu: 'AbsoluteIMU', path: str) -> None:
    """Pivot STEP_DEG at a time; at each stop, motors off, average the compass.
    A background thread integrates the gyro the whole time (also during the
    pivots) to give the reference heading of each stop."""
    z_sign = drive.profile.gyro_z_sign
    bias = _gyro_ready(imu, z_sign)
    lock, stop = threading.Lock(), threading.Event()
    state: dict = {'th': 0.0, 'buf': []}

    def imu_loop() -> None:
        t_prev = time.monotonic()
        while not stop.is_set():
            now = time.monotonic()
            try:
                _, m, g = imu.acc_mag_gyr()
            except OSError:
                m = g = None
            dt, t_prev = now - t_prev, now
            with lock:
                if g is not None:
                    state['th'] += (math.radians(g[2] * z_sign) - bias[2]) * dt
                if m is not None:
                    state['buf'].append((state['th'], m))
            time.sleep(max(0.0, SAMPLE_DT - (time.monotonic() - now)))

    def still_point() -> Optional[tuple[float, np.ndarray]]:
        time.sleep(SETTLE_SECS)
        with lock:
            state['buf'].clear()
        time.sleep(STILL_SECS)
        with lock:
            pts = list(state['buf'])
        if not pts:
            return None
        return float(np.mean([p[0] for p in pts])), np.mean([p[1] for p in pts], axis=0)

    n_steps = int(round(360.0 * SPIN_TURNS / STEP_DEG))
    print(f"Stepped calibration: {n_steps} pivots of {STEP_DEG:.0f}°, compass averaged "
          f"{STILL_SECS:.1f} s at each stop with the motors off ...")
    th_list, xyz = [], []
    worker = threading.Thread(target=imu_loop, daemon=True)
    worker.start()
    try:
        for k in range(n_steps + 1):
            if k > 0:
                turn_left(drive, math.radians(STEP_DEG))
            p = still_point()
            if p is not None:
                th_list.append(p[0]); xyz.append(p[1])
                print(f"  stop {k:2d}/{n_steps}: gyro {math.degrees(p[0]):7.1f}°", flush=True)
    finally:
        stop_motors(drive)
        stop.set()
        worker.join(timeout=1.0)
    _write_csv(path, 'gyro_heading_rad', th_list, xyz)


# 14 directions (6 axes + 8 diagonals) to judge how much of the sphere a manual
# rotation has covered
_DIRS = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
                 + [[sx, sy, sz] for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)], float)
_DIRS /= np.linalg.norm(_DIRS, axis=1)[:, None]


def record_3d(imu: 'AbsoluteIMU', path: str, secs: float) -> None:
    """The robot turned by hand in every direction; the motors stay off."""
    print(f"Motors OFF. For {secs:.0f} s, turn the robot slowly by hand in EVERY direction:\n"
          f"  flat spins, on each side, upside down, nose up / nose down …")
    for k in (3, 2, 1):
        print(f"  start in {k} …"); time.sleep(1)
    xyz, ts = [], []
    hits = np.zeros(len(_DIRS), int)
    t0 = time.monotonic(); next_print = t0
    while time.monotonic() - t0 < secs:
        now = time.monotonic()
        try:
            m = imu.mag_all()
        except OSError:
            m = None
        if m is not None:
            xyz.append(m); ts.append(now - t0)
            if len(xyz) > 20:                         # direction around the running centre
                v = np.asarray(m) - np.mean(xyz, axis=0)
                n = np.linalg.norm(v)
                if n > 0:
                    hits[int(np.argmax(_DIRS @ (v / n)))] += 1
        if now >= next_print:
            print(f"  {now - t0:4.0f} s  {len(xyz):5d} samples  "
                  f"directions covered: {int((hits >= 10).sum())}/{len(_DIRS)}", flush=True)
            next_print += 2.0
        time.sleep(max(0.0, SAMPLE_DT - (time.monotonic() - now)))
    if (hits >= 10).sum() < 12:
        print("⚠ less than 12/14 directions covered — the fit may be poor; consider recording again")
    _write_csv(path, 'time_s', ts, xyz)


class CompassCalibration(Calibration):
    """Hard and soft iron of the magnetometer: mag_calibration.json (3d) or
    mag_calibration_2d.json (2d), next to the profile. The profile itself
    is not changed."""

    name = 'compass'
    title = 'Compass'

    def __init__(self, mode: str = '2d', continuous: bool = False, secs: float = 60.0,
                 show: bool = False) -> None:
        """mode: '2d' (the robot spins) or '3d' (turned by hand, `secs` long).
        continuous: 2d in one spin with the motors running, not in steps.
        show: also open the plot windows."""
        if mode not in ('2d', '3d'):
            raise ValueError(f"mode must be '2d' or '3d', not {mode!r}")
        self.mode: str = mode
        self.continuous: bool = continuous
        self.secs: float = secs
        self.show: bool = show
        # the spin's reference heading is the gyro: its sign must be known
        self.requires: tuple[str, ...] = ('imu',) if mode == '2d' else ()
        self.moves_robot: bool = mode == '2d'

    def unavailable(self, profile: RobotProfile) -> Optional[str]:
        if profile.imu_port is None:
            return "no IMU port in the profile"
        if self.mode == '2d' and profile.imu_up_axis is not None and profile.imu_up_axis[1] != 'z':
            return (f"the IMU's {profile.imu_up_axis} points up: the 2d calibration reads "
                    f"gyro Z and the compass on x, y — use --mode 3d")
        return None

    def is_done(self, profile: RobotProfile) -> bool:
        return os.path.exists(mag_calibration_path(self.mode))

    def record(self, context: CalibrationContext) -> dict:
        csv = run_file(f"compass{self.mode}", 'csv')
        context.ui.message(f"Compass {self.mode}",
                           ["Keep laptops, speakers and metal", "≥ 1.5 m away from the robot."], "Go")
        if self.mode == '3d':
            record_3d(context.imu(), csv, self.secs)
        elif self.continuous:
            record_2d(context.drive, context.imu(), csv)
        else:
            record_2d_stepped(context.drive, context.imu(), csv)
        return {'csv': csv}

    def load(self, path: str) -> dict:
        return {'csv': path}

    def solve(self, data: dict) -> dict:
        """Fit the calibration to a recorded CSV. Draws the figures (saved
        by save())."""
        import matplotlib
        if not self.show:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        MagnetometerCalibrator = magnetometer_calibrator()

        csv = data['csv']
        cal = MagnetometerCalibrator(FIELD_UT)
        xyz = cal.load_data(csv)                          # library loader (skips column 1)
        first_col = np.loadtxt(csv, delimiter=',', skiprows=1, usecols=0)
        lines = [f"{len(xyz)} samples"]
        cal.plot_data(xyz, 'Raw')

        extra: dict = {'mode': self.mode}
        if self.mode == '3d':
            cal.calibrate(xyz)
        else:
            if abs(first_col[-1]) < 2 * math.pi * 0.9:
                raise RuntimeError("less than ~1 turn recorded — can't fit an ellipse")
            radius = fit_ellipse_2d(cal, xyz)
            circ = cal.apply_calibration(xyz)
            rr = np.hypot(circ[:, 0], circ[:, 1])
            lines.append(f"Ellipse → circle of radius {radius:.1f}; "
                         f"corrected radius varies ±{100 * rr.std() / rr.mean():.1f} %")
            res, sign, h_abs = heading_residual(cal, extra, xyz, first_col)
            extra['deviation'] = fit_deviation(res, sign, h_abs)
            res_dev, _, _ = heading_residual(cal, extra, xyz, first_col)
            rms = lambda r: f"RMS {np.sqrt(np.mean(r ** 2)):.1f}°, max {np.abs(r).max():.1f}°"
            lines.append("Compass − gyro heading over the spin (constant + gyro-scale trend removed):")
            path_3d = mag_calibration_path('3d')
            if os.path.exists(path_3d):
                old = MagnetometerCalibrator(); old.load_calibration(path_3d)
                res_old, _, _ = heading_residual(old, {}, xyz, first_col)
                lines.append(f"  3D calibration ({os.path.basename(path_3d)}): {rms(res_old)}")
            lines += [f"  2D ellipse:                    {rms(res)}",
                      f"  2D ellipse + deviation card:   {rms(res_dev)}",
                      "  (fitted on this spin — check it on a run with LiDAR truth)"]
            fig = plt.figure(figsize=(8, 4))
            ax = fig.add_subplot(111)
            order = np.argsort(np.degrees(h_abs) % 360)
            ax.plot((np.degrees(h_abs) % 360)[order], res[order], '.', ms=3, label='2D ellipse only')
            ax.plot((np.degrees(h_abs) % 360)[order], res_dev[order], '.', ms=3, label='+ deviation card')
            ax.set_xlabel('compass angle (deg)'); ax.set_ylabel('compass − gyro (deg)')
            ax.set_title('Compass deviation over the spin'); ax.grid(alpha=0.3); ax.legend()

        cal.plot_data(cal.apply_calibration(xyz), 'Calibrated')
        return {'csv': csv, 'calibrator': cal, 'extra': extra, 'lines': lines}

    def result_lines(self, result: dict) -> list[str]:
        return result['lines'] + [f"File: {os.path.basename(mag_calibration_path(self.mode))}"]

    def save(self, result: dict) -> str:
        """Write the calibration file and the figures; returns the file's path."""
        import matplotlib.pyplot as plt
        out, csv = mag_calibration_path(self.mode), result['csv']
        result['calibrator'].save_calibration(out)        # library format …
        with open(out) as f:                              # … plus mode / deviation / provenance
            saved = json.load(f)
        saved.update(result['extra'], source=os.path.relpath(csv, files_dir()),
                     date=datetime.now().isoformat(timespec='seconds'))
        with open(out, 'w') as f:
            json.dump(saved, f, indent=4)
        base = os.path.splitext(csv)[0]
        for k, num in enumerate(plt.get_fignums(), 1):
            figure_path = f"{base}_fig{k}.png"
            plt.figure(num).savefig(figure_path, dpi=100)
            print(f"Saved figure: {figure_path}")
        if self.show:
            plt.show()
        plt.close('all')
        return out

    def apply(self, context: CalibrationContext, result: dict) -> RobotProfile:
        self.save(result)
        return context.profile

    def replay(self, path: str) -> list[str]:
        """Calibrate again from a recorded CSV, and write the calibration
        file again: unlike the others, its result is not in the profile."""
        result = self.solve(self.load(path))
        self.save(result)
        return self.result_lines(result)
