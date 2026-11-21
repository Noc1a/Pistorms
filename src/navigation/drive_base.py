"""The two drive wheels of a model, in robot terms: left / right instead of
M1 / M2, and a positive speed or tick count always means forward.

    from pistorms import PiStorms
    from navigation.drive_base import DriveBase

    drive = DriveBase(PiStorms())     # robot_profile.json
    drive.forward(30)                 # both wheels, until the next command
    drive.turn_left(20)               # pivot on the spot, counter-clockwise
    drive.drive(30, 20)               # two wheel speeds: an arc to the right
    drive.brake()

Every speed change and the brake are ramped by default (ACCELERATION), so the
robot does not jolt; pass acceleration=None for an immediate change — tuned
test moves, and safety stops.

Which ports and which way round come from the RobotProfile
(navigation/robot_profile.py). The ramps, cancellation and sync start stay
in PiStormsMotor (motor.py): this class only maps left/right and the signs.
"""
from typing import Callable, Optional

from pistorms import PiStorms, PiStormsMotor

from .robot_profile import RobotProfile, load_profile


class DriveBase:
    ACCELERATION = 150   # %/s, default ramp of every speed change and of the brake

    def __init__(self, psm: PiStorms, profile: Optional[RobotProfile] = None) -> None:
        if profile is None:
            profile = load_profile()
        self.profile: RobotProfile = profile
        self.left: PiStormsMotor = profile.left_motor_port(psm)
        self.right: PiStormsMotor = profile.right_motor_port(psm)
        # Sign of the motor speed that drives each wheel forward. The encoders
        # count in the commanded direction, so it is also the sign of the
        # ticks when the wheel rolls forward.
        self.left_sign: int = profile.left_fwd_sign
        self.right_sign: int = profile.right_fwd_sign

    def _motor_speeds(self, left_speed: float, right_speed: float) -> tuple[float, float]:
        """(speed of M1, speed of M2) for these wheel speeds (+ = forward)."""
        left_motor_speed = left_speed * self.left_sign
        right_motor_speed = right_speed * self.right_sign
        if self.left.index == 0:          # left wheel on M1
            return left_motor_speed, right_motor_speed
        return right_motor_speed, left_motor_speed

    def drive(self, left_speed: float, right_speed: float,
              acceleration: Optional[float] = ACCELERATION,
              blocking: bool = True, callback: Optional[Callable] = None) -> None:
        """Run both wheels at these speeds (−100…100, + = forward) until the
        next command, started together and ramped together (acceleration in
        %/s; None = at once)."""
        speed_m1, speed_m2 = self._motor_speeds(left_speed, right_speed)
        self.left.setSpeedSyncPair(speed_m1, speed_m2, acceleration, blocking, callback)

    def forward(self, speed: float, acceleration: Optional[float] = ACCELERATION,
                blocking: bool = True) -> None:
        """Drive straight ahead at `speed` (0…100) until the next command."""
        self.drive(speed, speed, acceleration, blocking)

    def backward(self, speed: float, acceleration: Optional[float] = ACCELERATION,
                 blocking: bool = True) -> None:
        """Drive straight back at `speed` (0…100) until the next command."""
        self.drive(-speed, -speed, acceleration, blocking)

    def turn_left(self, speed: float, acceleration: Optional[float] = ACCELERATION,
                  blocking: bool = True) -> None:
        """Pivot on the spot, counter-clockwise, each wheel at `speed` (0…100),
        until the next command."""
        self.drive(-speed, speed, acceleration, blocking)

    def turn_right(self, speed: float, acceleration: Optional[float] = ACCELERATION,
                   blocking: bool = True) -> None:
        """Pivot on the spot, clockwise, each wheel at `speed` (0…100), until
        the next command."""
        self.drive(speed, -speed, acceleration, blocking)

    def brake(self, acceleration: Optional[float] = ACCELERATION, blocking: bool = True) -> None:
        """Slow down, then stop both wheels with the brake (acceleration=None:
        brake at once — safety stops only)."""
        self.left.brakeSync(acceleration, blocking)

    def float(self) -> None:
        """Stop both wheels and let them turn freely."""
        self.left.floatSync()

    def wheel_ticks(self, raw_left: int, raw_right: int) -> tuple[int, int]:
        """Forward-signed ticks from raw encoder reads of (left, right)."""
        return raw_left * self.left_sign, raw_right * self.right_sign

    def ticks(self) -> tuple[int, int]:
        """Encoder counts of (left, right), growing when the wheel rolls forward."""
        return self.wheel_ticks(self.left.pos(), self.right.pos())

    def resetPos(self) -> None:
        """Set both encoder counts to 0."""
        self.left.resetPos()
        self.right.resetPos()
