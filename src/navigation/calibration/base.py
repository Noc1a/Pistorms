"""What every calibration has in common, and what they share on the robot.

A Calibration measures one thing in two steps kept apart:

    record(context)  moves the robot and returns the raw measurements
    solve(data)      works the result out of them — no hardware, so a saved
                     run can be solved again (replay) and unit-tested

The Calibrator (calibrator.py) runs them, in the right order.
"""
import json
import os
import time
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

from ..robot_profile import RobotProfile, profile_path

if TYPE_CHECKING:
    from pistorms import PiStorms
    from pistorms.devices import AbsoluteIMU
    from ..drive_base import DriveBase
    from .lidar_scans import LidarScanner
    from .wizard import TerminalWizard


def files_dir() -> str:
    """Where the calibration files live: next to the robot profile."""
    return os.path.dirname(os.path.abspath(profile_path()))


def runs_dir() -> str:
    """Where the raw recordings of the calibrations are saved."""
    return os.path.join(files_dir(), 'calibration_runs')


def run_file(prefix: str, extension: str) -> str:
    """A new dated file in runs_dir(): <prefix>_<date>_<time>.<extension>."""
    os.makedirs(runs_dir(), exist_ok=True)
    return os.path.join(runs_dir(), f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.{extension}")


class CalibrationContext:
    """The robot, as the calibrations share it: one board, one drive base,
    one IMU, the profile as measured so far. Made by the Calibrator."""

    def __init__(self, psm: 'PiStorms', ui: 'TerminalWizard', profile: RobotProfile) -> None:
        from ..drive_base import DriveBase
        self.psm: 'PiStorms' = psm
        self.ui: 'TerminalWizard' = ui
        self.profile: RobotProfile = profile      # updated after each calibration
        self.drive: 'DriveBase' = DriveBase(psm, profile)
        self._imu: Optional['AbsoluteIMU'] = None
        self._lidars: list['LidarScanner'] = []

    def imu(self) -> 'AbsoluteIMU':
        """The IMU of the profile, opened once."""
        if self._imu is None:
            from pistorms.devices import AbsoluteIMU
            self._imu = AbsoluteIMU(self.profile.imu_sensor_port(self.psm))
        return self._imu

    def open_lidar(self, yaw_deg: float) -> 'LidarScanner':
        """Start the LiDAR of the profile; scans come with yaw_deg removed
        (0 = raw). Close it when done; close() below does it otherwise."""
        from .lidar_scans import LidarScanner
        if self.profile.lidar_port is None:
            raise RuntimeError("the robot profile has no LiDAR (no lidar_port)")
        lidar = LidarScanner(self.profile.lidar_port, yaw_deg)
        self._lidars.append(lidar)
        return lidar

    def close(self) -> None:
        """Stop the wheels at once and the LiDAR: cleanup, also on Ctrl-C."""
        try:
            self.drive.brake(acceleration=None)
        finally:
            for lidar in self._lidars:
                lidar.close()
            self._lidars.clear()


class Calibration(ABC):
    """One thing the robot measures about itself."""

    name: str = ''                      # what it is called on the command line
    title: str = ''                     # what it is called on the pages
    requires: tuple[str, ...] = ()      # names of the calibrations it builds on
    moves_robot: bool = True            # False: the motors stay off

    def unavailable(self, profile: RobotProfile) -> Optional[str]:
        """Why it can't run on this robot (no such sensor…), or None."""
        return None

    @abstractmethod
    def is_done(self, profile: RobotProfile) -> bool:
        """True when its result is already there (in the profile or in its
        own file): as a requirement of another one, it is then not redone."""

    @abstractmethod
    def record(self, context: CalibrationContext) -> dict:
        """Move the robot and return the raw measurements."""

    @abstractmethod
    def solve(self, data: dict) -> dict:
        """The result, from what record() returned. No hardware."""

    @abstractmethod
    def result_lines(self, result: dict) -> list[str]:
        """The result in a few short lines."""

    def detail_lines(self, result: dict) -> list[str]:
        """The tables behind the result, printed before it."""
        return []

    def apply(self, context: CalibrationContext, result: dict) -> RobotProfile:
        """Keep the result: returns the profile with it (the Calibrator saves
        it), or writes its own file and returns the profile unchanged."""
        return context.profile

    def run(self, context: CalibrationContext) -> RobotProfile:
        """Record, solve, keep. Returns the profile to save."""
        result = self.solve(self.record(context))
        for line in self.detail_lines(result):
            print(line)
        profile = self.apply(context, result)
        context.ui.message(f"{self.title} done", self.result_lines(result))
        return profile

    def load(self, path: str) -> dict:
        """A recording saved by record(), for replay()."""
        with open(path) as f:
            return json.load(f)

    def replay(self, path: str) -> list[str]:
        """Solve a saved recording again, on the Pi or on the Mac: nothing
        moves, the profile is not touched. Returns the lines to print."""
        result = self.solve(self.load(path))
        return self.detail_lines(result) + self.result_lines(result)
