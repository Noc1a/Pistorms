"""How the IMU is mounted: which of its axes points up (and the gyro sign),
and which points ahead.

    pivot left   → which IMU axis points up: the rotation vector of a
                   counter-clockwise turn points up
    short dashes → which IMU axis points ahead

Not measured: WHERE the IMU is on the robot. Nothing uses it (a gyro and a
compass read the same anywhere on a rigid robot), and the accelerations of
this robot are too small to find it reliably.
"""
import dataclasses
import math
import time
from typing import TYPE_CHECKING, Any, Callable, Optional

import numpy as np

from ..robot_profile import TICKS_PER_REV, RobotProfile
from .base import Calibration, CalibrationContext
from .moves import SETTLE_SECS, pos_safe

if TYPE_CHECKING:
    from pistorms.devices import AbsoluteIMU
    from ..drive_base import DriveBase

STILL_SECS        = 2.0     # IMU at rest: gyro bias and gravity
SPIN_SPEED        = 20      # motor %, pivot for the IMU's up axis
SPIN_SETTLE_SECS  = 0.4
SPIN_SECS         = 1.5
MIN_SPIN_DPS      = 5.0     # a slower rotation on the gyro = it saw nothing
MIN_AXIS_SHARE    = 0.9     # share of the rotation on one axis; less = IMU tilted
DASH_SPEED        = 40      # motor %, dashes for the IMU's forward axis
DASH_HOLD_SECS    = 0.2
DASH_LAG_SECS     = 0.1     # the wheels follow the commanded ramp this much later
DASH_DIRECTIONS   = (1, -1, 1, -1)     # ahead, back, ahead, back: ends where it started
MIN_DASH_ACCEL    = 0.3     # m/s², summed over the dashes: less = nothing felt

GYRO_CAL_MAX_STD_DPS = 0.6  # σ ≈ 0.2 °/s when still; more = robot moved → redo
GYRO_CAL_ATTEMPTS    = 3


# ── the arithmetic, testable without hardware ──────────────────────────────
def axis_name(index: int, sign: float) -> str:
    """'+x', '-z'… for an axis index (0 = x) and a sign."""
    return ('+' if sign > 0 else '-') + 'xyz'[index]


def up_axis_from_spin(gyro_dps: np.ndarray) -> tuple[str, float]:
    """The IMU axis that points up, from the gyro's mean reading (°/s, bias
    removed) while the robot pivots LEFT: the rotation vector of a
    counter-clockwise turn points up. Returns the axis ('+z', '-y'…) and
    the share of the rotation it carries (1.0 = the IMU sits level)."""
    rate = float(np.linalg.norm(gyro_dps))
    if rate < MIN_SPIN_DPS:
        raise RuntimeError(f"the gyro saw no rotation ({rate:.1f} °/s) — wrong IMU port?")
    index = int(np.argmax(np.abs(gyro_dps)))
    return axis_name(index, gyro_dps[index]), abs(float(gyro_dps[index])) / rate


def forward_axis_from_dashes(accel_change: np.ndarray, accel_at_rest: np.ndarray,
                             up_axis: str) -> Optional[str]:
    """The IMU axis that points ahead, or None when the dashes don't show it.
    accel_change: accelerometer reading while speeding up ahead minus while
    slowing down (m/s², summed over the dashes). accel_at_rest: reading
    with the robot still. up_axis: from up_axis_from_spin().

    An accelerometer reads gravity as +g or as −g on the axis pointing up,
    depending on the chip: the reading at rest tells which, and a push
    ahead then shows with the same sign on the axis pointing ahead."""
    up_index = 'xyz'.index(up_axis[1])
    up_sign = 1 if up_axis[0] == '+' else -1
    chip_sign = 1 if accel_at_rest[up_index] * up_sign > 0 else -1
    horizontal = [index for index in range(3) if index != up_index]
    strongest, weakest = sorted(horizontal, key=lambda index: -abs(accel_change[index]))
    if abs(accel_change[strongest]) < MIN_DASH_ACCEL:
        return None
    if abs(accel_change[strongest]) < 2 * abs(accel_change[weakest]):
        return None       # the IMU sits diagonally, or the robot swerved
    return axis_name(strongest, accel_change[strongest] * chip_sign)


# ── reading the IMU ────────────────────────────────────────────────────────
def mean_reading(read: Callable[[], Any], seconds: float) -> np.ndarray:
    """Mean of an IMU reading (a 3-vector) over this time."""
    samples = []
    end_time = time.monotonic() + seconds
    while time.monotonic() < end_time:
        try:
            samples.append(read())
        except OSError:
            pass
        time.sleep(0.005)
    if not samples:
        raise RuntimeError("the IMU does not answer — wrong port?")
    return np.mean(np.asarray(samples, dtype=float), axis=0)


def calibrate_gyro(imu: 'AbsoluteIMU', seconds: float, z_sign: int) -> np.ndarray:
    """Gyro bias (rad/s, 3 axes): the gyro averaged over `seconds` with the
    robot still. Redone (up to GYRO_CAL_ATTEMPTS) if the Z reading is too
    noisy — the robot was moving. z_sign: the profile's gyro_z_sign; the Z
    bias is returned flipped by it, as the samples are where they are read."""
    for attempt in range(1, GYRO_CAL_ATTEMPTS + 1):
        print(f"Calibrating gyro bias ({seconds:.1f}s stationary — hold still)...")
        samples = []
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            try:
                samples.append(imu.gyro_all())
            except OSError:
                pass
            time.sleep(0.005)
        arr = np.asarray(samples, dtype=float)     # deg/s
        std_z = float(arr[:, 2].std())
        if std_z <= GYRO_CAL_MAX_STD_DPS:
            break
        print(f"  ⚠ gyro Z too noisy (σ {std_z:.2f} °/s > {GYRO_CAL_MAX_STD_DPS}) — "
              f"robot not still? [{attempt}/{GYRO_CAL_ATTEMPTS}]")
    bias = np.radians(arr.mean(axis=0))            # rad/s
    # Match the sample-time Z flip so bias subtraction lands on the same axis.
    bias[2] *= z_sign
    print(f"  gyro bias (deg/s): "
          f"x={math.degrees(bias[0]):+.3f}  y={math.degrees(bias[1]):+.3f}  z={math.degrees(bias[2]):+.3f}"
          f"   (σ z {std_z:.2f} °/s, {len(arr)} samples)")
    return bias


def accel_change_of_dash(drive: 'DriveBase', imu: 'AbsoluteIMU', direction: int) -> np.ndarray:
    """One short dash ahead (direction 1) or back (−1): accelerometer
    reading while speeding up minus while slowing down, as for a dash ahead."""
    ramp_secs = DASH_SPEED / drive.ACCELERATION + DASH_LAG_SECS
    move = drive.forward if direction > 0 else drive.backward
    try:
        move(DASH_SPEED, blocking=False)
        speeding_up = mean_reading(imu.accel_all, ramp_secs)
        time.sleep(DASH_HOLD_SECS)
        drive.brake(blocking=False)
        slowing_down = mean_reading(imu.accel_all, ramp_secs)
    finally:
        drive.brake()
    time.sleep(SETTLE_SECS)
    return (speeding_up - slowing_down) * direction


class ImuAxesCalibration(Calibration):
    """Which IMU axis points up and which points ahead: imu_up_axis,
    imu_forward_axis and gyro_z_sign of the profile."""

    name = 'imu'
    title = 'IMU'

    def unavailable(self, profile: RobotProfile) -> Optional[str]:
        return None if profile.imu_port is not None else "no IMU port in the profile"

    def is_done(self, profile: RobotProfile) -> bool:
        return profile.imu_up_axis is not None

    def record(self, context: CalibrationContext) -> dict:
        ui, drive = context.ui, context.drive
        imu = context.imu()
        ui.show("IMU 1/3: at rest", ["Don't touch the robot…"])
        gyro_bias = mean_reading(imu.gyro_all, STILL_SECS)
        accel_at_rest = mean_reading(imu.accel_all, STILL_SECS)

        ui.show("IMU 2/3: up axis", ["The robot pivots left…"])
        try:
            drive.turn_left(SPIN_SPEED)
            time.sleep(SPIN_SETTLE_SECS)
            # two fresh reads (no fallback): the wheels roll further during
            # the spin than pos_safe accepts between two reads
            ticks_before = drive.wheel_ticks(pos_safe(drive.left, None), pos_safe(drive.right, None))
            time_before = time.monotonic()
            gyro_spin = mean_reading(imu.gyro_all, SPIN_SECS) - gyro_bias
            ticks_after = drive.wheel_ticks(pos_safe(drive.left, None), pos_safe(drive.right, None))
            spin_secs = time.monotonic() - time_before
        finally:
            drive.brake()
        time.sleep(SETTLE_SECS)
        profile = context.profile
        left_ticks = ticks_after[0] - ticks_before[0]
        right_ticks = ticks_after[1] - ticks_before[1]
        wheel_turn = (right_ticks - left_ticks) * profile.wheel_circumference / TICKS_PER_REV / profile.wheel_base
        wheel_rate_dps = math.degrees(wheel_turn) / spin_secs
        if wheel_rate_dps < MIN_SPIN_DPS:
            raise RuntimeError(f"the wheels did not turn the robot left ({wheel_rate_dps:+.0f} °/s) — "
                               f"redo the motor test")
        print(f"   gyro {np.round(gyro_spin, 1)} °/s, wheels {wheel_rate_dps:.1f} °/s")

        ui.show("IMU 3/3: forward axis", ["Short dashes ahead and back…"])
        accel_change = np.zeros(3)
        for direction in DASH_DIRECTIONS:
            accel_change += accel_change_of_dash(drive, imu, direction)
        print(f"   accelerometer, speeding up − slowing down: {np.round(accel_change, 2)} m/s²")
        return {'gyro_spin_dps': gyro_spin.tolist(), 'wheel_rate_dps': wheel_rate_dps,
                'accel_at_rest': accel_at_rest.tolist(), 'accel_change': accel_change.tolist()}

    def solve(self, data: dict) -> dict:
        gyro_spin = np.asarray(data['gyro_spin_dps'], dtype=float)
        up_axis, share = up_axis_from_spin(gyro_spin)
        forward_axis = forward_axis_from_dashes(np.asarray(data['accel_change'], dtype=float),
                                                np.asarray(data['accel_at_rest'], dtype=float), up_axis)
        result = {'up_axis': up_axis, 'forward_axis': forward_axis, 'share': share,
                  'gyro_rate_dps': float(np.linalg.norm(gyro_spin)),
                  'wheel_rate_dps': data['wheel_rate_dps'], 'gyro_z_sign': None}
        if up_axis[1] == 'z':
            result['gyro_z_sign'] = 1 if up_axis[0] == '+' else -1
        return result

    def result_lines(self, result: dict) -> list[str]:
        lines = [f"Up: {result['up_axis']}   ahead: {result['forward_axis'] or 'unclear'}",
                 f"Gyro {result['gyro_rate_dps']:.0f} °/s, wheels {result['wheel_rate_dps']:.0f} °/s"]
        if result['share'] < MIN_AXIS_SHARE:
            lines.append(f"IMU tilted: {result['share'] * 100:.0f} % on {result['up_axis']}")
        if result['gyro_z_sign'] is not None:
            lines.append(f"gyro_z_sign = {result['gyro_z_sign']:+d}")
        else:
            # the robot's Odometry reads the up axis; the test scripts
            # (testOdometryAccuracy…) and the compass calibration read gyro Z
            lines.append("Z is not vertical: fine for driving,")
            lines.append("but the test scripts read gyro Z")
        return lines

    def apply(self, context: CalibrationContext, result: dict) -> RobotProfile:
        updates: dict[str, Any] = {'imu_up_axis': result['up_axis'],
                                   'imu_forward_axis': result['forward_axis']}
        if result['gyro_z_sign'] is not None:
            updates['gyro_z_sign'] = result['gyro_z_sign']
        return dataclasses.replace(context.profile, **updates)
