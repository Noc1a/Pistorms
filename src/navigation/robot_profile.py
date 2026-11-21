"""Robot profile: what changes from one LEGO model to the next — which wheels,
on which motor ports, which way round, how big the robot is, where the IMU
and the LiDAR are.

One JSON file per model. The default is robot_profile.json at the repo root;
the environment variable ROBOT_PROFILE points to another one:

    ROBOT_PROFILE=profiles/small_bot.json python3 -u -m navigation.calibration all

navigation/calibration writes the file: a wizard in the terminal
asks for what must be measured by hand, and the robot measures the rest.

    {
      "wheel": "motorbike",       key of WHEELS below
      "wheel_diameter": 0.0935,   optional: measured diameter, wins over the catalog
      "wheel_base": 0.17,         m, between the middles of the two tyres
      "drive_bank": "A",          bank of the two drive motors (A or B)
      "left_motor": "M1",         which motor of that bank is the left wheel
      "left_fwd_sign": -1,        sign of the motor speed that drives this wheel forward
      "right_fwd_sign": -1,
      "body_front": 0.188,        m, nose of the robot ahead of the wheel axle
      "body_rear": 0.172,         m, back of the robot behind the wheel axle
      "body_width": 0.19,         m
      "min_start_speed": 10,      optional: motor %, the slowest that starts a wheel
                                  standing still, on the floor (measured by the
                                  motors calibration: it depends on the weight)

      "imu_port": "BBS2",         sensor port of the AbsoluteIMU; left out: no IMU
      "gyro_z_sign": -1,          -1 if a counter-clockwise turn reads negative gyro Z
      "imu_up_axis": "-z",        IMU axis that points up    } measured by
      "imu_forward_axis": "+x",   IMU axis that points ahead } the IMU calibration

      "lidar_port": "/dev/ttyUSB0",   serial port of the RPLidar; left out: no LiDAR
      "lidar_dx": 0.078,          m, LiDAR rotation centre ahead of the wheel axle
      "lidar_dy": 0.001,          m, LiDAR rotation centre left of the robot centre
      "lidar_yaw_deg": 0.0        deg, the LiDAR's 0° turned counter-clockwise from robot forward
    }

Both drive motors must be on the same bank: the board starts and stops the
two motors of a bank at the same instant (sync commands), not motors of
different banks. The profile can only describe such a pair.
"""
import json
import math
import os
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

# For the type hints only: importing pistorms needs smbus, and the profile
# is also read on the Mac, where the recorded runs are analysed
if TYPE_CHECKING:
    from pistorms import PiStorms, PiStormsMotor
    from pistorms.sensor_port import SensorPort

# Wheel catalog: nominal LEGO diameters (printed on the tyre), in m.
# The rolling diameter differs a little (tyre squash, load): measure it with
# tests/testOdometryAccuracy.py and put it in the profile's "wheel_diameter".
WHEELS: dict[str, float] = {
    "ev3":       0.0432,
    "ev3_small": 0.037,
    "motorbike": 0.0942,
}

TICKS_PER_REV = 360.0      # encoder ticks per wheel turn (LEGO NXT/EV3 motors)
# Motor % that starts a wheel standing still, when the profile has no measured
# min_start_speed: the first model measured 8 % (2026-11-21), plus a margin
DEFAULT_MIN_START_SPEED = 10
SENSOR_PORTS = ('BAS1', 'BAS2', 'BBS1', 'BBS2')
AXES = ('+x', '-x', '+y', '-y', '+z', '-z')

# The repo root: PiStorms/src/navigation/robot_profile.py → three levels up
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
DEFAULT_PATH = os.path.join(_REPO_ROOT, 'robot_profile.json')


@dataclass(frozen=True)
class RobotProfile:
    wheel: str
    wheel_base: float
    drive_bank: str
    left_motor: str
    left_fwd_sign: int
    right_fwd_sign: int
    body_front: float
    body_rear: float
    body_width: float
    imu_port: Optional[str] = None
    gyro_z_sign: int = 1
    imu_up_axis: Optional[str] = None
    imu_forward_axis: Optional[str] = None
    lidar_port: Optional[str] = None
    lidar_dx: float = 0.0
    lidar_dy: float = 0.0
    lidar_yaw_deg: float = 0.0
    wheel_diameter_measured: Optional[float] = None
    min_start_speed_measured: Optional[int] = None

    def __post_init__(self) -> None:
        if self.wheel not in WHEELS:
            raise ValueError(f"unknown wheel {self.wheel!r}: one of {', '.join(WHEELS)}")
        if self.drive_bank not in ('A', 'B'):
            raise ValueError(f"drive_bank must be 'A' or 'B', not {self.drive_bank!r}")
        if self.left_motor not in ('M1', 'M2'):
            raise ValueError(f"left_motor must be 'M1' or 'M2', not {self.left_motor!r}")
        for name in ('left_fwd_sign', 'right_fwd_sign', 'gyro_z_sign'):
            if getattr(self, name) not in (1, -1):
                raise ValueError(f"{name} must be 1 or -1")
        for name in ('wheel_base', 'body_front', 'body_rear', 'body_width'):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a length in m, above 0")
        if self.imu_port is not None and self.imu_port not in SENSOR_PORTS:
            raise ValueError(f"imu_port must be one of {', '.join(SENSOR_PORTS)}, not {self.imu_port!r}")
        for name in ('imu_up_axis', 'imu_forward_axis'):
            if getattr(self, name) is not None and getattr(self, name) not in AXES:
                raise ValueError(f"{name} must be one of {', '.join(AXES)}")
        if self.lidar_port is not None:
            self._check_lidar_is_on_the_robot()
        if self.min_start_speed_measured is not None and not 1 <= self.min_start_speed_measured <= 50:
            raise ValueError("min_start_speed must be a motor % between 1 and 50")

    def _check_lidar_is_on_the_robot(self) -> None:
        """The collision guard takes the robot's outline from body_front /
        body_rear / body_width: a LiDAR outside it means one of them is wrong
        (or a typing mistake), and the guard would watch the wrong place."""
        if not -self.body_rear <= self.lidar_dx <= self.body_front:
            raise ValueError(f"lidar_dx {self.lidar_dx} m is outside the robot "
                             f"(body_rear {self.body_rear} m, body_front {self.body_front} m)")
        if abs(self.lidar_dy) > self.body_width / 2:
            raise ValueError(f"lidar_dy {self.lidar_dy} m is outside the robot "
                             f"(body_width {self.body_width} m)")

    @property
    def wheel_diameter(self) -> float:
        """m: the measured diameter if the profile has one, else the catalog's."""
        if self.wheel_diameter_measured is not None:
            return self.wheel_diameter_measured
        return WHEELS[self.wheel]

    @property
    def min_start_speed(self) -> int:
        """Motor %: the slowest that starts a wheel standing still — the
        measured one if the profile has it, else DEFAULT_MIN_START_SPEED."""
        if self.min_start_speed_measured is not None:
            return self.min_start_speed_measured
        return DEFAULT_MIN_START_SPEED

    @property
    def wheel_circumference(self) -> float:
        return math.pi * self.wheel_diameter

    @property
    def right_motor(self) -> str:
        """The other motor of the drive bank."""
        return 'M2' if self.left_motor == 'M1' else 'M1'

    def left_motor_port(self, psm: 'PiStorms') -> 'PiStormsMotor':
        return getattr(psm, 'B' + self.drive_bank + self.left_motor)    # e.g. psm.BAM1

    def right_motor_port(self, psm: 'PiStorms') -> 'PiStormsMotor':
        return getattr(psm, 'B' + self.drive_bank + self.right_motor)

    def imu_sensor_port(self, psm: 'PiStorms') -> 'SensorPort':
        if self.imu_port is None:
            raise ValueError("this robot profile has no IMU (no imu_port)")
        return getattr(psm, self.imu_port)                               # e.g. psm.BBS2

    def to_json(self) -> dict:
        """The profile as written in the file: sensors and measurements the
        robot doesn't have are left out."""
        data = {name: value for name, value in self.__dict__.items() if value is not None}
        if 'wheel_diameter_measured' in data:
            data['wheel_diameter'] = data.pop('wheel_diameter_measured')
        if 'min_start_speed_measured' in data:
            data['min_start_speed'] = data.pop('min_start_speed_measured')
        if self.imu_port is None:
            data.pop('gyro_z_sign')
        if self.lidar_port is None:
            for name in ('lidar_dx', 'lidar_dy', 'lidar_yaw_deg'):
                data.pop(name)
        return data


def profile_path(path: Optional[str] = None) -> str:
    """`path`, else $ROBOT_PROFILE, else robot_profile.json at the repo root."""
    if path is not None:
        return path
    return os.environ.get('ROBOT_PROFILE', DEFAULT_PATH)


def profile_from_json(data: dict) -> RobotProfile:
    data = dict(data)
    # "wheel_diameter" and "min_start_speed" in the file are optional measured values
    data['wheel_diameter_measured'] = data.pop('wheel_diameter', None)
    data['min_start_speed_measured'] = data.pop('min_start_speed', None)
    return RobotProfile(**data)


def load_profile(path: Optional[str] = None) -> RobotProfile:
    path = profile_path(path)
    with open(path) as f:
        data = json.load(f)
    try:
        return profile_from_json(data)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path}: {error} — fix the file, or run "
                         f"python3 -u -m navigation.calibration setup on the robot") from error


def save_profile(profile: RobotProfile, path: Optional[str] = None) -> str:
    """Write the profile; returns the file's path."""
    path = profile_path(path)
    with open(path, 'w') as f:
        json.dump(profile.to_json(), f, indent=4)
        f.write('\n')
    return path
