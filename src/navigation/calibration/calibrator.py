"""The Calibrator: the one entry to every calibration of the robot.

    from navigation.calibration import Calibrator

    Calibrator().run(['compass'])       # runs the IMU calibration first if
                                        # the profile doesn't have it yet
    Calibrator().run(['all'])           # wizard, IMU, LiDAR, compass

It knows what each calibration builds on (Calibration.requires) and runs
what is missing first; it opens the board once, saves the profile after each
step, and stops the wheels and the LiDAR on the way out. A sensor without a
port is skipped; a step that fails keeps the old values, and what builds on
it is skipped too.
"""
import json
import os
from typing import TYPE_CHECKING, Optional

from ..robot_profile import RobotProfile, load_profile, profile_path, save_profile
from .base import Calibration, CalibrationContext
from .compass import CompassCalibration
from .imu_axes import ImuAxesCalibration
from .lidar_mount import LidarMountCalibration
from .lidar_verify import LidarMountCheck
from .wizard import TerminalWizard, ask_settings, profile_from_values, read_values

if TYPE_CHECKING:
    from pistorms import PiStorms

SETUP = 'setup'                                     # the wizard: not a Calibration
ALL: tuple[str, ...] = (SETUP, 'imu', 'lidar', 'compass')


class Calibrator:

    def __init__(self, calibrations: Optional[list[Calibration]] = None,
                 psm: Optional['PiStorms'] = None, ui: Optional[TerminalWizard] = None) -> None:
        """calibrations: the ones it can run (default: IMU axes, LiDAR
        mounting, compass 2d, LiDAR check). psm: the board, opened on first
        use if not given."""
        if calibrations is None:
            calibrations = [ImuAxesCalibration(), LidarMountCalibration(),
                            CompassCalibration(), LidarMountCheck()]
        self.calibrations: dict[str, Calibration] = {calibration.name: calibration
                                                     for calibration in calibrations}
        self.ui: TerminalWizard = ui or TerminalWizard()
        self._psm: Optional['PiStorms'] = psm

    def board(self) -> 'PiStorms':
        if self._psm is None:
            from pistorms import PiStorms
            self._psm = PiStorms(screen=False)
        return self._psm

    def names(self) -> list[str]:
        """What run() accepts."""
        return [SETUP] + list(self.calibrations) + ['all']

    def plan(self, names: list[str]) -> list[Calibration]:
        """The calibrations to go through for these names, each once, what
        a calibration requires before it."""
        steps: list[Calibration] = []

        def add(name: str) -> None:
            if name not in self.calibrations:
                raise ValueError(f"unknown calibration {name!r}: one of {', '.join(self.names())}")
            calibration = self.calibrations[name]
            if calibration in steps:
                return
            for required in calibration.requires:
                add(required)
            steps.append(calibration)

        for name in names:
            if name != SETUP:
                add(name)
        return steps

    def setup(self) -> RobotProfile:
        """The wizard: ask for, or confirm, what must be measured by hand,
        and save the profile."""
        values = read_values()
        self.ui.message("Robot setup", ["Confirm or change each value.",
                                        f"File: {os.path.basename(profile_path())}"], "Start")
        ask_settings(self.ui, self.board(), values)
        profile = profile_from_values(values)
        print(f"Saved: {save_profile(profile)}")
        return profile

    def _saved_profile(self) -> Optional[RobotProfile]:
        """The profile on file, or None (and why) when the wizard must write it."""
        try:
            return load_profile()
        except (OSError, ValueError) as error:
            print(f"No usable robot profile ({error}): the wizard first")
            return None

    def run(self, names: list[str], force: bool = False) -> list[str]:
        """Run these calibrations ('setup', 'imu', 'lidar', 'compass',
        'verify', 'all'), and first what they require if it is not done yet
        (force: even if it is). Returns one line per step: what happened."""
        expanded: list[str] = []
        for name in names:
            expanded += ALL if name == 'all' else (name,)
        steps = self.plan(expanded)

        report: list[str] = []
        profile = None if SETUP in expanded else self._saved_profile()
        if profile is None:
            profile = self.setup()
            report.append("setup: saved")
        if not steps:
            return report

        if any(step.moves_robot for step in steps):
            self.ui.message("Sensors", ["The robot will now move by itself.",
                                        "Put it on the floor: 50 cm free",
                                        "ahead, 40 cm all around."], "Go")
        context = CalibrationContext(self.board(), self.ui, profile)
        blocked: set[str] = set()       # not there for what builds on them
        try:
            for step in steps:
                missing = [name for name in step.requires if name in blocked]
                reason = f"needs {missing[0]}" if missing else step.unavailable(context.profile)
                if reason is not None:
                    print(f"\n{step.title}: {reason} — skipped")
                    report.append(f"{step.name}: skipped ({reason})")
                    blocked.add(step.name)
                    continue
                if step.name not in expanded and step.is_done(context.profile) and not force:
                    continue            # only required, and already there
                if step.name not in expanded:
                    print(f"\n{step.title}: not done yet, and needed — first")
                try:
                    context.profile = step.run(context)
                    save_profile(context.profile)
                    report.append(f"{step.name}: done, saved")
                except Exception as error:
                    # e.g. the LiDAR measured outside the robot's outline, a
                    # sensor that does not answer, scans that don't match
                    print(f"\n⚠ {step.title} failed: {error!r}")
                    self.ui.message(f"{step.title} failed", [str(error)])
                    report.append(f"{step.name}: FAILED, old values kept")
                    blocked.add(step.name)
        finally:
            context.close()             # also on Ctrl-C
        self.ui.show("Finished", report + [f"File: {profile_path()}"])
        print(json.dumps(context.profile.to_json(), indent=4))
        return report

    def replay(self, name: str, path: str) -> list[str]:
        """Solve a saved recording of this calibration again. No robot
        needed: works on the Mac."""
        return self.plan([name])[-1].replay(path)
