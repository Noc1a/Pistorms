"""Moves that stop on the wheel encoders, with reads that survive I²C errors.

Used by the calibrations and by tests/testOdometryAccuracy.py. Ports and
directions come from the robot profile, through DriveBase: positive wheel
speeds and ticks mean forward.
"""
import math
import time
from typing import TYPE_CHECKING, Optional

from ..robot_profile import TICKS_PER_REV, RobotProfile

# For the type hints only: importing pistorms needs smbus, and saved runs are
# also solved again on the Mac
if TYPE_CHECKING:
    from pistorms import PiStormsMotor
    from ..drive_base import DriveBase

# ── I²C-safe encoder read ───────────────────────────────────────────────────
# mindsensors_i2c raises OSError when a bus error persists through its own
# retries. A read can ALSO return a valid-looking integer with a corrupted
# byte (bit-flip in byte 3 → ~2^24 ticks of phantom motion, i.e. ~13 km of
# wheel travel). So we validate both: error → retry; delta-vs-fallback beyond
# a physically plausible maximum → retry. If all retries fail we return the
# fallback (skip-this-tick semantics: no motion registered for this sample).
MAX_TICK_DELTA_PER_READ = 500   # ≈27× worst-case one-tick delta at 100 Hz poll

# ── runaway protection ─────────────────────────────────────────────────────
# pos_safe's fallback hides bad reads — but once a read is rejected the
# fallback goes stale, every later read looks like a jump too, and the motion
# loop drives blind (this happened: the robot spun while the script saw
# "frozen" encoders). So every rejected read is logged (READ_EVENTS) and the
# motion loops stop the motors on either symptom below.
ENCODER_STALL_SECS   = 1.0   # motors commanded but encoders unchanged this long
MAX_CONSEC_BAD_READS = 20    # per motor, consecutive pos_safe calls that failed

# Safety timeout — a bug in the terminate condition must not send the robot
# on a runaway.
MAX_SECS_TURN = 8.0

# Two-phase pivot turn:
#   Fast phase at SPEED_TURN until Δθ ≥ target − TURN_APPROACH_DEG
#   Slow phase at SPEED_TURN_SLOW until Δθ ≥ target − TURN_COAST_DEG
#   Then brake. Slow phase makes total coast small enough that a small
#   TURN_COAST_DEG works across surfaces / battery levels.
SPEED_TURN         = 20     # % for pivot turns
SPEED_TURN_SLOW    = 10
TURN_APPROACH_DEG  = 20.0   # start slow phase this far before target
TURN_COAST_DEG     = 3.0    # brake this far before target (slow-phase coast)
POLL_HZ            = 100    # motion-primitive poll rate

# Straight drive, ramped at start and stop (DriveBase.ACCELERATION)
STRAIGHT_SPEED     = 30     # motor %
MAX_STRAIGHT_SECS  = 6.0    # runaway guard
SETTLE_SECS        = 0.5    # wait after braking before measuring


class EncoderFault(RuntimeError):
    """Encoders can't be trusted while the motors are running — stop now."""


READ_EVENTS: list[dict] = []        # every rejected encoder read: {t, motor, reason}
_bad_streak: dict[str, int] = {}    # motor label → consecutive failed pos_safe calls
_t_start: float = time.monotonic()


def _motor_label(motor: 'PiStormsMotor') -> str:
    return f"M{getattr(motor, 'motornum', '?')}"


def _log_bad_read(label: str, reason: str) -> None:
    READ_EVENTS.append({'t': round(time.monotonic() - _t_start, 3),
                        'motor': label, 'reason': reason})
    n = len(READ_EVENTS)
    if n <= 30 or n % 100 == 0:   # don't flood the terminal
        print(f"  ⚠ encoder read rejected #{n} {label}: {reason}", flush=True)


def pos_safe(motor: 'PiStormsMotor', fallback: Optional[int], retries: int = 4) -> int:
    """Read `motor.pos()` with I²C corruption defence.

    If `fallback is None`, this is a bootstrap read — accept any non-None
    integer (no delta check possible since there's no prior value).
    Otherwise reject reads whose delta vs fallback exceeds
    MAX_TICK_DELTA_PER_READ, since that's physically impossible in one
    sample interval and always indicates I²C corruption.
    Every rejected attempt is logged; consecutive failures are counted per
    motor so the motion loops can stop (see check_encoders).
    """
    label = _motor_label(motor)
    for _ in range(retries):
        try:
            v = motor.pos()
            if v is None:
                reason = "read returned None"
            elif fallback is None or abs(v - fallback) <= MAX_TICK_DELTA_PER_READ:
                _bad_streak[label] = 0
                return v
            else:
                reason = f"jump {v - fallback:+d} ticks (raw {v}, previous {fallback})"
        except (OSError, TypeError) as e:
            reason = f"{type(e).__name__}: {e}"
        _log_bad_read(label, reason)
        time.sleep(0.002)
    _bad_streak[label] = _bad_streak.get(label, 0) + 1
    return fallback if fallback is not None else 0


def check_encoders(watch: dict, l: int, r: int) -> None:
    """Raise EncoderFault if the encoders stopped changing while the motors
    are commanded, or if reads keep failing. `watch` is a per-move dict."""
    now = time.monotonic()
    if (l, r) != watch.get('last'):
        watch['last'], watch['t'] = (l, r), now
    elif now - watch['t'] > ENCODER_STALL_SECS:
        last = READ_EVENTS[-1]['reason'] if READ_EVENTS else 'no rejected reads logged'
        raise EncoderFault(f"encoders unchanged for {ENCODER_STALL_SECS:.1f}s while motors "
                           f"are commanded (L={l}, R={r}); last rejected read: {last}")
    for label, n in _bad_streak.items():
        if n >= MAX_CONSEC_BAD_READS:
            raise EncoderFault(f"{label}: {n} consecutive failed reads; "
                               f"last: {READ_EVENTS[-1]['reason'] if READ_EVENTS else '?'}")


def stop_motors(drive: 'DriveBase') -> bool:
    """Brake, retrying and REPORTING failures (never silently). Falls back
    to float if brake keeps failing."""
    for attempt in range(1, 6):
        try:
            drive.brake(acceleration=None)
            return True
        except Exception as e:
            print(f"  ⚠ brake attempt {attempt}/5 failed: {e!r}", flush=True)
            time.sleep(0.05)
    try:
        drive.float()
        print("  ⚠ brake failed — motors set to float", flush=True)
    except Exception as e:
        print(f"  ⚠⚠ COULD NOT STOP THE MOTORS: {e!r} — lift the robot / cut power", flush=True)
    return False


class WheelTravel:
    """How far the wheels have rolled since this object was made, read with
    pos_safe. Each read is checked against the PREVIOUS one, not the start:
    pos_safe rejects reads too far from its fallback (a full turn of the
    robot is ~650 ticks)."""

    def __init__(self, drive: 'DriveBase') -> None:
        self.drive: 'DriveBase' = drive
        self.start: tuple[int, int] = (pos_safe(drive.left, None), pos_safe(drive.right, None))
        self.raw: list[int] = list(self.start)     # last raw reads (left, right)

    def ticks(self) -> tuple[int, int]:
        """Read both encoders: forward-signed ticks of (left, right) since the start."""
        self.raw[0] = pos_safe(self.drive.left, self.raw[0])
        self.raw[1] = pos_safe(self.drive.right, self.raw[1])
        return self.drive.wheel_ticks(self.raw[0] - self.start[0], self.raw[1] - self.start[1])

    def distance_m(self) -> float:
        """Read both encoders: distance driven (m, + = forward)."""
        left_ticks, right_ticks = self.ticks()
        return (left_ticks + right_ticks) / 2 * self.drive.profile.wheel_circumference / TICKS_PER_REV

    def turn_rad(self) -> float:
        """Read both encoders: how much the robot turned (rad, + = left)."""
        profile = self.drive.profile
        left_ticks, right_ticks = self.ticks()
        return (right_ticks - left_ticks) * profile.wheel_circumference / TICKS_PER_REV / profile.wheel_base

    def check(self, watch: dict) -> None:
        """check_encoders on the last reads."""
        check_encoders(watch, self.raw[0], self.raw[1])


def turn_rad_to_wheel_ticks(profile: RobotProfile, rad: float) -> int:
    """Pivot turn: each wheel traces an arc of wheel_base·|rad|/2 metres."""
    per_wheel_m = profile.wheel_base * abs(rad) / 2.0
    return int(round(per_wheel_m / profile.wheel_circumference * TICKS_PER_REV))


def turn(drive: 'DriveBase', angle_rad: float,
         speed_fast: float = SPEED_TURN, speed_slow: float = SPEED_TURN_SLOW) -> None:
    """Pivot in place by angle_rad: > 0 left (CCW), < 0 right (CW).
    Two-phase, wheel-tick-terminated.

    Wheel-diff drives physical rotation directly, and on this robot it
    matches truth better than the gyro (which appears to have a slight
    scale-factor error). We terminate on per-wheel forward-signed
    differential (dr − dl)/2 hitting the geometry target, minus a small
    coast margin.

    Phase 1 (fast): pivot at speed_fast until diff ≥ target − TURN_APPROACH.
    Phase 2 (slow): pivot at speed_slow until diff ≥ target − TURN_COAST.
    Then brake. Enforces MAX_SECS_TURN as runaway guard.

    Starts and brakes without a ramp (acceleration=None): the stopping
    margins (TURN_COAST_DEG…) were tuned that way."""
    s = 1 if angle_rad >= 0 else -1        # +1: left wheel back, right forward
    profile = drive.profile
    wheels = WheelTravel(drive)
    target     = turn_rad_to_wheel_ticks(profile, angle_rad)
    coast_t    = turn_rad_to_wheel_ticks(profile, math.radians(TURN_COAST_DEG))
    approach_t = turn_rad_to_wheel_ticks(profile, math.radians(TURN_APPROACH_DEG))
    slow_at   = target - approach_t
    brake_at  = target - coast_t
    print(f"  → turn {math.degrees(angle_rad):+.1f}° "
          f"(fast {speed_fast} → slow {speed_slow} at "
          f"{slow_at} ticks → brake at {brake_at} ticks, target {target})")
    pivot = drive.turn_left if s > 0 else drive.turn_right
    pivot(speed_fast, acceleration=None)
    slowed = False
    t0 = time.monotonic()
    watch: dict = {}
    try:
        poll_dt = 1.0 / POLL_HZ
        while True:
            dl, dr = wheels.ticks()
            wheels.check(watch)
            diff = s * (dr - dl) / 2.0  # differential per wheel, in the turn direction
            if diff >= brake_at:
                break
            if not slowed and diff >= slow_at:
                pivot(speed_slow, acceleration=None)
                slowed = True
            if time.monotonic() - t0 > MAX_SECS_TURN:
                print(f"  ⚠ turn TIMEOUT after {MAX_SECS_TURN:.1f}s "
                      f"(diff={diff:.0f}, brake_at={brake_at}) — braking")
                break
            time.sleep(poll_dt)
    finally:
        stop_motors(drive)


def turn_left(drive: 'DriveBase', angle_rad: float) -> None:
    """Pivot left (CCW) by |angle_rad|."""
    turn(drive, abs(angle_rad))


def drive_straight(drive: 'DriveBase', distance_m: float, speed: float = STRAIGHT_SPEED) -> float:
    """Drive distance_m straight (negative: backward), ramped at the start
    and at the stop: a sudden start or brake twists the robot. Stops on the
    wheel encoders. Returns the distance the wheels covered (m, signed)."""
    wheels = WheelTravel(drive)
    move = drive.forward if distance_m > 0 else drive.backward
    move(speed, blocking=False)
    start_time = time.monotonic()
    watch: dict = {}
    try:
        while abs(wheels.distance_m()) < abs(distance_m):
            wheels.check(watch)
            if time.monotonic() - start_time > MAX_STRAIGHT_SECS:
                print(f"   ⚠ straight drive TIMEOUT after {MAX_STRAIGHT_SECS:.0f} s")
                break
            time.sleep(0.01)
    finally:
        drive.brake()
    time.sleep(SETTLE_SECS)
    return wheels.distance_m()
