"""How the LiDAR is mounted: where it is on the robot (lidar_dx, lidar_dy)
and which way it points (lidar_yaw_deg).

Everything is measured on the LiDAR's raw scans:

  1. Position  one turn in place, 12 stops. The robot centre (midpoint of
               the drive wheels) stays put while the LiDAR, mounted at
               offset d = (dx, dy) from it, travels on a circle. At each
               stop i, scan matching (in the raw LiDAR frame, no offset)
               gives the LiDAR pose (x_i, y_i, θ_i) relative to its start, and

                   (x_i, y_i) = C + R(θ_i) · d      C = pivot point (unknown, ≈ −d)

               which is linear in (Cx, Cy, dx, dy) → least squares over all
               stops. The offset comes out in the LiDAR's own axes.
  2. Direction 25 cm forward and back. The LiDAR must see the robot move
               along its own heading; the offset known, the LiDAR's own
               swing when the robot turns a little on the way is removed.
  3. The offset is turned into robot axes with the direction.

Needs walls or furniture in sight, 40 cm free around the robot and 50 cm
ahead. The raw scans are saved to calibration_runs/lidarcal_<date>.json.
"""
import dataclasses
import json
import math
import time
from typing import TYPE_CHECKING, Optional

import numpy as np

from ..robot_profile import RobotProfile
from .base import Calibration, CalibrationContext, run_file
from .lidar_scans import LidarScanner
from .moves import WheelTravel, drive_straight, turn_left

if TYPE_CHECKING:
    from ..drive_base import DriveBase

# Spin in place that shows where the LiDAR is
STEP_DEG = 30.0
N_STEPS  = 12            # 12 × 30° = one full turn
SPIN_SETTLE_SECS = 0.4
MIN_INLIER_FRAC = 0.5    # a stop matched worse than this is not used
# Straight drives, forward then back, that show which way the LiDAR points
STRAIGHT_M        = 0.25
MAX_DRIVE_TURN_DEG = 8.0    # a "straight" drive that turned more is not used
YAW_GUESS_STEP_DEG = 30


# ── the arithmetic, testable without hardware ──────────────────────────────
def solve_offset(stops: list[dict]) -> tuple[float, float, dict]:
    """Where the LiDAR is, in the axes of the scans, from a spin in place.
    stops: [{'scans': [...], 'init_theta': rad}, ...], stop 0 = reference.
    Returns (dx, dy, details)."""
    from .. import scan_matching as sm      # here: it reads the robot profile, which the wizard may still have to write
    raw = dict(dx=0.0, dy=0.0, yaw_deg=0.0)
    ref = sm.scans_to_cloud(stops[0]['scans'], **raw)
    poses = [{'x': 0.0, 'y': 0.0, 'theta': 0.0, 'inlier_frac': 1.0, 'rms_m': 0.0}]
    for st in stops[1:]:
        # each scan matched separately, median pose (same as the odometry test)
        p = sm.match_stop(st['scans'], ref, (0.0, 0.0, st['init_theta']), **raw)
        poses.append(p if p is not None else
                     {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'inlier_frac': 0.0, 'rms_m': float('inf')})
    good = [p for p in poses if p['inlier_frac'] >= MIN_INLIER_FRAC]
    a, b = [], []
    for p in good:
        c, s = math.cos(p['theta']), math.sin(p['theta'])
        a.append([1, 0, c, -s]); b.append(p['x'])
        a.append([0, 1, s,  c]); b.append(p['y'])
    sol, *_ = np.linalg.lstsq(np.array(a), np.array(b), rcond=None)
    cx, cy, dx, dy = (float(v) for v in sol)
    # residual: where each stop puts the robot centre if d is right
    centres = []
    for p in poses:
        c, s = math.cos(p['theta']), math.sin(p['theta'])
        centres.append((p['x'] - (c * dx - s * dy), p['y'] - (s * dx + c * dy)))
    return dx, dy, {'pivot': (cx, cy), 'poses': poses, 'centres': centres, 'n_used': len(good)}


def offset_report_lines(dx: float, dy: float, det: dict) -> list[str]:
    """The spin stop by stop: what the offset was solved from."""
    lines = ["", "stop   LiDAR pose (x, y, θ)            fit            robot centre with solved offset"]
    for i, (p, (ccx, ccy)) in enumerate(zip(det['poses'], det['centres'])):
        lines.append(f"{i:>3}   {100*p['x']:+6.1f} {100*p['y']:+6.1f} cm {math.degrees(p['theta']):+8.2f}°   "
                     f"{100*p['inlier_frac']:3.0f}% {100*p['rms_m']:.1f}cm   ({100*ccx:+5.1f}, {100*ccy:+5.1f}) cm")
    good = [c for p, c in zip(det['poses'], det['centres']) if p['inlier_frac'] >= MIN_INLIER_FRAC]
    spread = np.std(np.array(good), axis=0)
    lines += ["", f"Used {det['n_used']}/{len(det['poses'])} stops.",
              f"LiDAR offset in the scans' axes:  dx = {100*dx:+.1f} cm   dy = {100*dy:+.1f} cm",
              f"Robot centre stays within ±{100*spread.max():.1f} cm (1σ) with this offset"]
    return lines


def move_direction_error_deg(x: float, y: float, theta: float, backward: bool) -> float:
    """How far a drive strays from the robot's own heading, in degrees, from
    where it ended (x ahead, y left, in the frame it started in) and how
    much it turned (theta, rad). A robot that turns while it drives follows
    an arc: along it, its mean heading is theta / 2, not 0."""
    if backward:
        x, y = -x, -y
    error = math.atan2(y, x) - theta / 2
    return math.degrees(math.atan2(math.sin(error), math.cos(error)))


def match_move(scans_before: list, scans_after: list, distance_m: float,
               dx: float, dy: float) -> dict:
    """Where the robot centre went during a drive of distance_m (the wheels'
    distance), from scans taken still before and after, offset (dx, dy)
    applied, in the frame of the scans. Which way the LiDAR points need not
    be known: the match is started from every YAW_GUESS_STEP_DEG."""
    from .. import scan_matching as sm      # here: it reads the robot profile, which the wizard may still have to write
    reference = sm.scans_to_cloud(scans_before, dx=dx, dy=dy)
    moved = sm.scans_to_cloud(scans_after, dx=dx, dy=dy)
    best = None
    for guess_deg in range(0, 360, YAW_GUESS_STEP_DEG):
        guess = math.radians(guess_deg)
        start = (abs(distance_m) * math.cos(guess), abs(distance_m) * math.sin(guess), 0.0)
        fit = sm.match_pose(moved, reference, start, yaw_starts_deg=(0.0,))
        if best is None or (fit['inlier_frac'], -fit['rms_m']) > (best['inlier_frac'], -best['rms_m']):
            best = fit
    # each scan on its own from the best start: median pose and its spread
    fit = sm.match_stop(scans_after, reference, (best['x'], best['y'], best['theta']), dx=dx, dy=dy)
    if fit is None:
        raise RuntimeError("the scans before and after a drive don't match — "
                           "too few walls in sight, or something moved")
    moved_m = math.hypot(fit['x'], fit['y'])
    if abs(moved_m - abs(distance_m)) > 0.3 * abs(distance_m):
        raise RuntimeError(f"the LiDAR saw a move of {moved_m * 100:.0f} cm, the wheels drove "
                           f"{abs(distance_m) * 100:.0f} cm — wheel size wrong, or a bad scan match")
    if abs(math.degrees(fit['theta'])) > MAX_DRIVE_TURN_DEG:
        raise RuntimeError(f"the robot turned {math.degrees(fit['theta']):+.1f}° on a straight drive "
                           f"— a wheel slipping or rubbing?")
    return fit


def mean_angle_deg(angles_deg: list[float]) -> float:
    """Mean of angles, right across ±180°."""
    sines = sum(math.sin(math.radians(angle)) for angle in angles_deg)
    cosines = sum(math.cos(math.radians(angle)) for angle in angles_deg)
    return math.degrees(math.atan2(sines, cosines))


def lidar_yaw_from_drives(drives: list[dict], dx: float, dy: float) -> tuple[float, list[float]]:
    """Which way the LiDAR points (lidar_yaw_deg: its 0° turned counter-
    clockwise from robot forward), from raw scans around straight drives.
    drives: [{'before': scans, 'after': scans, 'wheels_m': signed distance}].
    dx, dy: the offset in the LiDAR's own axes (from the spin, solved on the
    same raw scans), so the LiDAR's own swing when the robot turns is removed.
    Returns (the mean, the value of each drive).

    In the LiDAR's axes, the robot moves along (its mean heading) − yaw."""
    yaw_of_each = []
    for drive_scans in drives:
        fit = match_move(drive_scans['before'], drive_scans['after'], drive_scans['wheels_m'], dx, dy)
        error = move_direction_error_deg(fit['x'], fit['y'], fit['theta'], drive_scans['wheels_m'] < 0)
        yaw_of_each.append(-error)
    return mean_angle_deg(yaw_of_each), yaw_of_each


# ── recording on the robot ─────────────────────────────────────────────────
def record_spin(drive: 'DriveBase', lidar: LidarScanner) -> list[dict]:
    """Pivot one full turn in N_STEPS stops, scanning at each one. Returns
    the stops for solve_offset(). Brakes on the way out, also after an error."""
    stops = []
    try:
        wheels = WheelTravel(drive)
        print("Reference scan...")
        stops.append({'scans': lidar.collect_scans(), 'init_theta': 0.0})
        for i in range(1, N_STEPS + 1):
            print(f"[{i}/{N_STEPS}] turn +{STEP_DEG:.0f}°")
            turn_left(drive, math.radians(STEP_DEG))
            time.sleep(SPIN_SETTLE_SECS)
            th = wheels.turn_rad()
            stops.append({'scans': lidar.collect_scans(), 'init_theta': th})
            print(f"  wheels say {math.degrees(th):+.1f}° total")
    finally:
        try: drive.brake(acceleration=None)   # cleanup: already stopped unless a turn failed
        except Exception: pass
    return stops


def record_straight_drives(drive: 'DriveBase', lidar: LidarScanner) -> list[dict]:
    """STRAIGHT_M forward, then back to the start, scanning still before
    and after each: [{'before', 'after', 'wheels_m'}, …]."""
    drives = []
    scans = lidar.collect_scans()
    for distance_m in (STRAIGHT_M, -STRAIGHT_M):
        print(f"   {'forward' if distance_m > 0 else 'back'} {abs(distance_m) * 100:.0f} cm")
        wheels_m = drive_straight(drive, distance_m)
        scans_after = lidar.collect_scans()
        drives.append({'before': scans, 'after': scans_after, 'wheels_m': wheels_m})
        scans = scans_after
    return drives


class LidarMountCalibration(Calibration):
    """Where the LiDAR is on the robot and which way it points: lidar_dx,
    lidar_dy and lidar_yaw_deg of the profile."""

    name = 'lidar'
    title = 'LiDAR'

    def unavailable(self, profile: RobotProfile) -> Optional[str]:
        return None if profile.lidar_port is not None else "no LiDAR port in the profile"

    def is_done(self, profile: RobotProfile) -> bool:
        # a LiDAR never measured sits at (0, 0), pointing ahead
        return (profile.lidar_dx, profile.lidar_dy, profile.lidar_yaw_deg) != (0.0, 0.0, 0.0)

    def record(self, context: CalibrationContext) -> dict:
        ui, drive = context.ui, context.drive
        ui.show("LiDAR 1/2: position", ["Starting the LiDAR…"])
        lidar = context.open_lidar(yaw_deg=0.0)     # raw scans
        try:
            ui.show("LiDAR 1/2: position", [f"One turn in {N_STEPS} stops…"])
            stops = record_spin(drive, lidar)
            ui.show("LiDAR 2/2: direction", [f"{STRAIGHT_M * 100:.0f} cm forward, then back…"])
            drives = record_straight_drives(drive, lidar)
        finally:
            lidar.close()
        data = {'step_deg': STEP_DEG, 'stops': stops, 'drives': drives}
        path = run_file('lidarcal', 'json')
        with open(path, 'w') as f:
            json.dump(data, f)
        print(f"   raw scans saved: {path}")
        return data

    def solve(self, data: dict) -> dict:
        """With the drives: direction, and the offset in robot axes. A spin
        alone (no 'drives' in the recording): the offset in the axes of its
        scans, direction None."""
        stops = data['stops']
        lidar_dx, lidar_dy, details = solve_offset(stops)
        result = {'dx': lidar_dx, 'dy': lidar_dy, 'yaw_deg': None, 'yaw_of_each': [],
                  'n_used': details['n_used'], 'n_stops': len(stops),
                  'report': offset_report_lines(lidar_dx, lidar_dy, details)}
        if data.get('drives'):
            yaw_deg, yaw_of_each = lidar_yaw_from_drives(data['drives'], lidar_dx, lidar_dy)
            yaw = math.radians(yaw_deg)
            result.update(yaw_deg=yaw_deg, yaw_of_each=yaw_of_each,
                          dx=lidar_dx * math.cos(yaw) - lidar_dy * math.sin(yaw),
                          dy=lidar_dx * math.sin(yaw) + lidar_dy * math.cos(yaw))
        return result

    def detail_lines(self, result: dict) -> list[str]:
        return result['report']

    def result_lines(self, result: dict) -> list[str]:
        lines = []
        if result['yaw_deg'] is None:
            lines.append("Direction: not measured (a spin alone); offset in the scans' axes")
        else:
            each = ", ".join(f"{'forward' if number == 0 else 'back'} {yaw:+.1f}°"
                             for number, yaw in enumerate(result['yaw_of_each']))
            lines.append(f"Direction: {result['yaw_deg']:+.1f}°  ({each})")
        return lines + [f"Ahead of the axle: {result['dx'] * 1000:+.0f} mm",
                        f"Left of the centre: {result['dy'] * 1000:+.0f} mm",
                        f"{result['n_used']}/{result['n_stops']} spin stops used"]

    def apply(self, context: CalibrationContext, result: dict) -> RobotProfile:
        return dataclasses.replace(context.profile, lidar_yaw_deg=round(result['yaw_deg'], 1),
                                   lidar_dx=round(result['dx'], 3), lidar_dy=round(result['dy'], 3))
