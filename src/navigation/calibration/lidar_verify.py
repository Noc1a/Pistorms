"""Check the LiDAR calibration of the robot profile: its direction
(lidar_yaw_deg) and its place on the robot (lidar_dx, lidar_dy).

Two checks, with the scans converted exactly as the robot code does
(robot-oriented, then moved by the offset):

  1. Straight  25 cm forward, then back, ramped. The LiDAR must see the
               robot move along its own heading (half of what it turned on
               the way: it follows an arc); a constant angle between the
               two is the direction error. The distance seen against the
               wheels' also tells whether the wheel size is right.
  2. Spin      one turn in place, 12 stops (as the calibration). The
               offset is solved again from this spin and must match the
               profile's. The robot centre found at each stop must stay in
               a tight cluster: a wrong offset spreads it on a circle. (The
               robot sliding once, e.g. on its first turn, only shifts the
               cluster: reported, not an error.)

Saves the raw scans and a picture (walls of every stop overlaid: sharp
walls = good calibration) to calibration_runs/verify_<date>.json / .png.

Needs the robot on the floor with walls or furniture in sight, 40 cm free
around it and 40 cm free ahead.
"""
import json
import math
import os
from datetime import datetime
from typing import Optional

import numpy as np

from ..robot_profile import WHEELS, RobotProfile
from .base import Calibration, CalibrationContext, run_file
from .lidar_mount import (N_STEPS, STRAIGHT_M, match_move, mean_angle_deg, move_direction_error_deg,
                          record_spin, record_straight_drives, solve_offset)
from .moves import EncoderFault

# Verdicts
MAX_YAW_ERROR_DEG = 1.5      # mean of the forward and back drives
MAX_OFFSET_ERROR_M = 0.005   # offset solved again from the spin, against the profile's
MAX_CENTRE_SPREAD_M = 0.01   # robot centre around its mean over the spin (pivot wobble)
MAX_DISTANCE_ERROR = 0.03    # LiDAR distance against the wheels': wheel size ±3 %


# ── analysis: no hardware, used live and on recorded runs ──────────────────
def check_straight(drives: list[dict], dx: float, dy: float) -> dict:
    """How the LiDAR saw the straight drives, scans robot-oriented with the
    profile's direction, offset (dx, dy) applied. drives: as
    lidar_mount.record_straight_drives returns them. Returns each
    drive's pose and direction error, their mean (the direction error of
    the profile: + = the profile's lidar_yaw_deg is that much too high),
    and the distance seen by the LiDAR against the wheels'."""
    legs = []
    for drive_scans in drives:
        fit = match_move(drive_scans['before'], drive_scans['after'], drive_scans['wheels_m'], dx, dy)
        legs.append({'x': fit['x'], 'y': fit['y'], 'theta_deg': math.degrees(fit['theta']),
                     'wheels_m': drive_scans['wheels_m'], 'lidar_m': math.hypot(fit['x'], fit['y']),
                     'yaw_error_deg': move_direction_error_deg(fit['x'], fit['y'], fit['theta'],
                                                               drive_scans['wheels_m'] < 0)})
    lidar_m = sum(leg['lidar_m'] for leg in legs)
    wheels_m = sum(abs(leg['wheels_m']) for leg in legs)
    return {'legs': legs, 'yaw_error_deg': mean_angle_deg([leg['yaw_error_deg'] for leg in legs]),
            'lidar_m': lidar_m, 'wheels_m': wheels_m, 'distance_error': lidar_m / wheels_m - 1}


def check_spin(stops: list, dx: float, dy: float) -> dict:
    """The robot centre at each stop of a spin in place, found by scan
    matching with this offset (stop 0 = start = origin). stops: as
    lidar_mount.record_spin returns them (robot-oriented scans)."""
    from .. import scan_matching as sm      # here: it reads the robot profile, which the wizard may still have to write
    reference = sm.scans_to_cloud(stops[0]['scans'], dx=dx, dy=dy)
    centres = [{'x': 0.0, 'y': 0.0, 'theta_deg': 0.0}]
    for stop in stops[1:]:
        match = sm.match_stop(stop['scans'], reference, (0.0, 0.0, stop['init_theta']), dx=dx, dy=dy)
        if match is not None:
            centres.append({'x': match['x'], 'y': match['y'],
                            'theta_deg': math.degrees(match['theta'])})
    mean_x = float(np.mean([centre['x'] for centre in centres]))
    mean_y = float(np.mean([centre['y'] for centre in centres]))
    spread = [math.hypot(centre['x'] - mean_x, centre['y'] - mean_y) for centre in centres]
    return {'centres': centres, 'n_matched': len(centres), 'n_stops': len(stops),
            'max_spread_m': max(spread), 'shift_m': math.hypot(mean_x, mean_y)}


def analyse(run: dict) -> dict:
    dx, dy = run['profile']['lidar_dx'], run['profile']['lidar_dy']
    drives = run['straight']
    if isinstance(drives, dict):             # runs of the first version: one forward drive
        drives = [drives]
    result = {'straight': check_straight(drives, dx, dy), 'spin': None, 'solved': None}
    if 'spin' not in run:                    # the spin failed: run['spin_error'] says why
        return result
    result['spin'] = check_spin(run['spin'], dx, dy)
    # The offset solved again from the same spin: should agree with the profile
    solved_dx, solved_dy, _ = solve_offset(run['spin'])
    result['solved'] = {'dx': solved_dx, 'dy': solved_dy,
                        'error_m': math.hypot(solved_dx - dx, solved_dy - dy)}
    return result


def verdict_lines(run: dict, result: dict) -> tuple[bool, list[str]]:
    """(all good, short lines for the terminal)."""
    profile = run['profile']
    straight, spin, solved = result['straight'], result['spin'], result['solved']
    yaw_ok = abs(straight['yaw_error_deg']) <= MAX_YAW_ERROR_DEG
    offset_ok = solved is not None and solved['error_m'] <= MAX_OFFSET_ERROR_M
    cluster_ok = spin is not None and spin['max_spread_m'] <= MAX_CENTRE_SPREAD_M
    centre_ok = offset_ok and cluster_ok
    wheel_ok = abs(straight['distance_error']) <= MAX_DISTANCE_ERROR

    def mark(ok: bool) -> str:
        return "OK " if ok else "BAD"

    legs = "  ".join(f"{'fwd' if leg['wheels_m'] > 0 else 'back'} {leg['yaw_error_deg']:+.1f}°"
                     f" (turned {leg['theta_deg']:+.1f}°)" for leg in straight['legs'])
    lines = [f"{mark(yaw_ok)} direction: {straight['yaw_error_deg']:+.1f}° off   [{legs}]"]
    if spin is None:
        lines += ["BAD centre: not checked, the spin failed:",
                  f"    {run.get('spin_error', '?')[:60]}",
                  "→ wheels blocked? robot touching something?"]
    else:
        lines += [f"{mark(offset_ok)} offset: profile {profile['lidar_dx'] * 1000:+.0f} / "
                  f"{profile['lidar_dy'] * 1000:+.0f} mm, spin says {solved['dx'] * 1000:+.0f} / "
                  f"{solved['dy'] * 1000:+.0f} mm ({solved['error_m'] * 1000:.0f} mm apart)",
                  f"{mark(cluster_ok)} centre: within {spin['max_spread_m'] * 100:.1f} cm over "
                  f"{spin['n_matched']}/{spin['n_stops']} stops"
                  f" (robot slid {spin['shift_m'] * 100:.1f} cm on the spin)"]
    lines.append(f"{mark(wheel_ok)} wheels: LiDAR {straight['lidar_m'] * 100:.1f} / "
                 f"wheels {straight['wheels_m'] * 100:.1f} cm")
    if not yaw_ok:
        lines.append(f"→ lidar_yaw_deg ≈ {profile['lidar_yaw_deg'] - straight['yaw_error_deg']:.1f}")
    if spin is not None and not centre_ok:
        lines.append("→ redo it: python3 -m navigation.calibration lidar")
    if not wheel_ok:
        diameter = profile.get('wheel_diameter') or WHEELS[profile['wheel']]
        lines.append(f"→ wheel_diameter ≈ {diameter * (1 + straight['distance_error']):.4f}")
    return yaw_ok and centre_ok, lines


def save_picture(run: dict, result: dict, path: str) -> None:
    """Walls of every spin stop, each moved by its matched pose: sharp walls
    and a tight cluster of centres mean a good calibration."""
    from .. import scan_matching as sm      # here: it reads the robot profile, which the wizard may still have to write
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    dx, dy = run['profile']['lidar_dx'], run['profile']['lidar_dy']
    figure, (walls_axes, centres_axes) = plt.subplots(1, 2, figsize=(12, 6))
    for stop, centre in zip(run['spin'], result['spin']['centres']):
        cloud = sm.scans_to_cloud(stop['scans'], dx=dx, dy=dy)
        pose = (centre['x'], centre['y'], math.radians(centre['theta_deg']))
        points = sm.transform(cloud, pose)
        walls_axes.plot(points[:, 0], points[:, 1], '.', ms=1)
    walls_axes.plot(0, 0, 'k+', ms=12)
    walls_axes.set_aspect('equal')
    walls_axes.set_title('Walls of the 12 stops overlaid (sharp = good)')
    centre_x = [centre['x'] * 100 for centre in result['spin']['centres']]
    centre_y = [centre['y'] * 100 for centre in result['spin']['centres']]
    centres_axes.plot(centre_x, centre_y, 'o-')
    limit = max(2.0, max(abs(value) for value in centre_x + centre_y) * 1.3)
    mean_x, mean_y = float(np.mean(centre_x)), float(np.mean(centre_y))
    centres_axes.add_patch(plt.Circle((mean_x, mean_y), MAX_CENTRE_SPREAD_M * 100, fill=False, ls='--'))
    centres_axes.set_xlim(-limit, limit)
    centres_axes.set_ylim(-limit, limit)
    centres_axes.set_aspect('equal')
    centres_axes.set_xlabel('cm ahead')
    centres_axes.set_ylabel('cm left')
    centres_axes.set_title(f"Robot centre at each stop (dashed: ±{MAX_CENTRE_SPREAD_M * 100:.0f} cm)")
    centres_axes.grid(alpha=0.3)
    figure.tight_layout()
    figure.savefig(path, dpi=90)
    print(f"Saved picture: {path}")


class LidarMountCheck(Calibration):
    """Drives and spins once more and tells whether the profile's LiDAR
    direction and offset hold. Changes nothing in the profile."""

    name = 'verify'
    title = 'LiDAR check'
    requires = ('lidar',)

    def unavailable(self, profile: RobotProfile) -> Optional[str]:
        return None if profile.lidar_port is not None else "no LiDAR port in the profile"

    def is_done(self, profile: RobotProfile) -> bool:
        return False        # a check has no result to keep: nothing requires it

    def record(self, context: CalibrationContext) -> dict:
        """Drive and scan, filling the run as it goes: what was measured
        before an error is kept and saved (a failed spin leaves
        run['spin_error'])."""
        ui, drive, profile = context.ui, context.drive, context.profile
        ui.message("Verify calibration", ["Robot on the floor, walls in sight,",
                                          "40 cm free around, 40 cm ahead."], "Go")
        run: dict = {'date': datetime.now().isoformat(timespec='seconds'), 'profile': profile.to_json()}
        ui.show("Check 1/2: straight", ["Starting the LiDAR…"])
        lidar = context.open_lidar(profile.lidar_yaw_deg)      # robot-oriented scans
        try:
            ui.show("Check 1/2: straight", [f"{STRAIGHT_M * 100:.0f} cm forward, then back…"])
            run['straight'] = record_straight_drives(drive, lidar)

            ui.show("Check 2/2: spin", [f"One turn in {N_STEPS} stops…"])
            try:
                run['spin'] = record_spin(drive, lidar)
            except EncoderFault as fault:
                # the wheels did not turn: blocked, or a motor command lost
                print(f"\n⚠ spin stopped: {fault}")
                run['spin_error'] = str(fault)
        finally:
            lidar.close()
            # also what was measured before an error or Ctrl-C
            if 'straight' in run:
                run['path'] = run_file('verify', 'json')
                with open(run['path'], 'w') as f:
                    json.dump(run, f)
                print(f"Saved: {run['path']}")
        return run

    def load(self, path: str) -> dict:
        run = super().load(path)
        run['path'] = path
        return run

    def solve(self, data: dict) -> dict:
        result = analyse(data)
        result['good'], result['verdict'] = verdict_lines(data, result)
        if result['spin'] is not None:
            save_picture(data, result, os.path.splitext(data['path'])[0] + '.png')
        return result

    def detail_lines(self, result: dict) -> list[str]:
        lines = []
        if result['spin'] is not None:
            lines.append("\nRobot centre at each spin stop (should stay at 0, 0):")
            for number, centre in enumerate(result['spin']['centres']):
                lines.append(f"  {number:2d}  θ {centre['theta_deg']:+7.1f}°   "
                             f"x {centre['x'] * 100:+5.1f}  y {centre['y'] * 100:+5.1f} cm")
        lines.append("\nStraight drives, as the LiDAR saw them:")
        for leg in result['straight']['legs']:
            lines.append(f"  wheels {leg['wheels_m'] * 100:+5.1f} cm   LiDAR x {leg['x'] * 100:+5.1f}  "
                         f"y {leg['y'] * 100:+5.1f} cm  turned {leg['theta_deg']:+.1f}°   "
                         f"→ direction {leg['yaw_error_deg']:+.1f}° off")
        return lines

    def result_lines(self, result: dict) -> list[str]:
        return result['verdict'] + ["All good" if result['good'] else "Check failed"]
