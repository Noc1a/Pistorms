"""The slowest motor speed that starts the wheels: min_start_speed of the
profile. It depends on the robot (weight, wheels, motors) and the floor,
so it is measured on the floor, the robot loaded as it drives.

For each move — turn left, turn right, forward, back (the robot ends where
it started) — from standstill, one step at a time (4, 6, 8 … %): the wheels
ramp to the speed, hold, then brake (ramped). The move starts at the first
speed that turns both wheels at least MIN_MOVE_DEG_PER_S. min_start_speed is
the slowest speed that starts every move, plus START_MARGIN_PERCENT: the
threshold rises as the battery runs down.

The controllers raise any wheel speed below it to it (Controller._drive):
too high, and every small correction turns the robot fast and past its
goal (2026-11-21, a fixed 18 % made the Follow mode turn past the person;
the first model measured 8 %).

Needs the robot on the floor with 40 cm free around it. The measurements
are saved to calibration_runs/motors_<date>.json.
"""
import dataclasses
import json
import time
from typing import TYPE_CHECKING, Callable, Optional

from ..robot_profile import TICKS_PER_REV, RobotProfile
from .base import Calibration, CalibrationContext, run_file
from .moves import WheelTravel

if TYPE_CHECKING:
    from ..drive_base import DriveBase

ACCELERATION         = 150     # %/s, every speed change (never a jump)
STEP_SPEEDS          = range(4, 32, 2)
HOLD_SECS            = 0.8     # each step
MEASURE_AFTER_SECS   = 0.3     # the ramp done, measure over the rest of the step
SETTLE_SECS          = 0.5     # after braking
MIN_MOVE_DEG_PER_S   = 10.0    # a wheel slower than this is standing (encoder noise: ~1°)
START_MARGIN_PERCENT = 2
MOVES                = ('turn left', 'turn right', 'forward', 'back')


# ── the arithmetic, testable without hardware ──────────────────────────────
def start_speed(steps: list[list[float]]) -> Optional[int]:
    """The first speed whose two wheel rates reach MIN_MOVE_DEG_PER_S, or
    None. steps: [[speed %, left °/s, right °/s], …] in increasing speed."""
    for speed, left_rate, right_rate in steps:
        if min(left_rate, right_rate) >= MIN_MOVE_DEG_PER_S:
            return int(speed)
    return None


# ── recording on the robot ─────────────────────────────────────────────────
def wheel_rates(drive: 'DriveBase', seconds: float) -> tuple[float, float]:
    """How fast each wheel turns (°/s, absolute) over the next `seconds`."""
    wheels = WheelTravel(drive)
    time.sleep(seconds)
    left_ticks, right_ticks = wheels.ticks()
    degrees_per_tick = 360.0 / TICKS_PER_REV
    return (abs(left_ticks) * degrees_per_tick / seconds,
            abs(right_ticks) * degrees_per_tick / seconds)


def record_start(drive: 'DriveBase', run: Callable[[float], None]) -> list[list[float]]:
    """Steps of one move from standstill, until both wheels move:
    [[speed %, left °/s, right °/s], …]."""
    steps = []
    for speed in STEP_SPEEDS:
        try:
            run(speed)
            time.sleep(MEASURE_AFTER_SECS)
            left_rate, right_rate = wheel_rates(drive, HOLD_SECS - MEASURE_AFTER_SECS)
        finally:
            drive.brake(acceleration=ACCELERATION)
        time.sleep(SETTLE_SECS)
        steps.append([speed, left_rate, right_rate])
        print(f"   {speed:3d} %   left {left_rate:6.1f} °/s   right {right_rate:6.1f} °/s")
        if min(left_rate, right_rate) >= MIN_MOVE_DEG_PER_S:
            break
    return steps


class MotorStartCalibration(Calibration):
    """The slowest motor speed that starts the wheels on the floor:
    min_start_speed of the profile."""

    name = 'motors'
    title = 'Motors'

    def is_done(self, profile: RobotProfile) -> bool:
        return profile.min_start_speed_measured is not None

    def record(self, context: CalibrationContext) -> dict:
        drive = context.drive
        moves: dict[str, Callable[[float], None]] = {
            'turn left':  lambda speed: drive.turn_left(speed, acceleration=ACCELERATION, blocking=False),
            'turn right': lambda speed: drive.turn_right(speed, acceleration=ACCELERATION, blocking=False),
            'forward':    lambda speed: drive.forward(speed, acceleration=ACCELERATION, blocking=False),
            'back':       lambda speed: drive.backward(speed, acceleration=ACCELERATION, blocking=False),
        }
        data: dict = {'battery_v': context.psm.battVoltage(), 'moves': {}}
        for number, name in enumerate(MOVES, 1):
            context.ui.show(f"Motors {number}/{len(MOVES)}: {name}", ["Slower to faster, from standstill…"])
            data['moves'][name] = record_start(drive, moves[name])
        path = run_file('motors', 'json')
        with open(path, 'w') as f:
            json.dump(data, f)
        print(f"   saved: {path}")
        return data

    def solve(self, data: dict) -> dict:
        starts = {name: start_speed(steps) for name, steps in data['moves'].items()}
        never_started = [name for name, speed in starts.items() if speed is None]
        if never_started:
            raise RuntimeError(f"the wheels did not start ({', '.join(never_started)}) below "
                               f"{STEP_SPEEDS[-1]} % — robot blocked, or a motor not plugged in?")
        slowest_for_all = max(starts.values())
        return {'starts': starts, 'battery_v': data.get('battery_v'),
                'min_start_speed': slowest_for_all + START_MARGIN_PERCENT}

    def result_lines(self, result: dict) -> list[str]:
        lines = [f"{name}: starts at {speed} %" for name, speed in result['starts'].items()]
        if result['battery_v'] is not None:
            lines.append(f"Battery {result['battery_v']:.2f} V")
        lines.append(f"min_start_speed = {result['min_start_speed']} % (+{START_MARGIN_PERCENT} % margin)")
        return lines

    def apply(self, context: CalibrationContext, result: dict) -> RobotProfile:
        return dataclasses.replace(context.profile, min_start_speed_measured=result['min_start_speed'])
