"""The questions asked in the terminal, and the values of the robot profile
that must be measured by hand: the wheels, the wheel base, the motor ports
and which way each wheel turns (each motor is pulsed and the wizard asks
what moved), the size of the robot, the ports of the IMU and of the LiDAR.
"""
import json
import os
import time
from typing import TYPE_CHECKING, Any, Optional

from ..robot_profile import SENSOR_PORTS, WHEELS, RobotProfile, profile_from_json, profile_path

if TYPE_CHECKING:
    from pistorms import PiStorms, PiStormsMotor

# Starting values when there is no profile file yet
DEFAULTS: dict[str, Any] = {
    'wheel': 'ev3', 'wheel_base': 0.12, 'drive_bank': 'A', 'left_motor': 'M1',
    'left_fwd_sign': 1, 'right_fwd_sign': 1,
    'body_front': 0.10, 'body_rear': 0.10, 'body_width': 0.15,
}
LIDAR_PORTS = ('/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/rplidar')

ACCELERATION      = 150     # %/s, ramp of the wizard's own moves
PULSE_SPEED       = 20      # motor %, the motor test
PULSE_SECS        = 0.6


class TerminalWizard:
    """Questions and messages in the terminal (run it in the robot's
    screen session: it waits for answers). Enter keeps the current value."""

    def _print_page(self, title: str, lines: list[str]) -> None:
        print(f"\n== {title} ==")
        for line in lines:
            print(f"   {line}")

    def show(self, title: str, lines: list[str]) -> None:
        """What the robot is doing; no answer expected."""
        self._print_page(title, lines)

    def message(self, title: str, lines: list[str], button: str = "OK") -> None:
        """A message to read; returns when Enter is pressed."""
        self._print_page(title, lines)
        input(f"   [Enter] {button} ")

    def choose(self, title: str, options: list[tuple[str, Any]], current: Any = None,
               lines: Optional[list[str]] = None) -> Any:
        """Numbered options; the current value is marked with a star and
        chosen by Enter. options: (label, value). Returns the value chosen."""
        self._print_page(title, lines or [])
        current_number = None
        for number, (label, value) in enumerate(options, 1):
            is_current = value == current
            if is_current:
                current_number = number
            print(f"   {'*' if is_current else ' '} {number}. {label}")
        prompt = f"   Choice [{current_number}]: " if current_number else "   Choice: "
        while True:
            answer = input(prompt).strip()
            if answer == '' and current_number is not None:
                number = current_number
            elif answer.isdigit() and 1 <= int(answer) <= len(options):
                number = int(answer)
            else:
                print(f"   type a number from 1 to {len(options)}")
                continue
            print(f"   → {options[number - 1][0]}")
            return options[number - 1][1]

    def ask_mm(self, title: str, value_m: float, lines: Optional[list[str]] = None) -> float:
        """A length typed in mm; Enter keeps the current one. Returns it in m."""
        self._print_page(title, lines or [])
        value_mm = int(round(value_m * 1000))
        while True:
            answer = input(f"   mm [{value_mm}]: ").strip()
            if answer == '':
                return value_mm / 1000
            try:
                typed_mm = float(answer.replace(',', '.'))
            except ValueError:
                typed_mm = 0
            if typed_mm > 0:
                return round(typed_mm) / 1000
            print("   type a length in mm, above 0")


def read_values() -> dict[str, Any]:
    """The profile file as a plain dict (it may be incomplete or wrong: the
    wizard is there to fix it), completed with DEFAULTS."""
    values = dict(DEFAULTS)
    path = profile_path()
    if os.path.exists(path):
        with open(path) as f:
            values.update(json.load(f))
    return values


def pulse_and_ask(ui: TerminalWizard, motor: 'PiStormsMotor', title: str, lines: list[str],
                  options: list[tuple[str, Any]]) -> Any:
    """Turn one motor briefly at +PULSE_SPEED and ask what moved, again
    as often as asked."""
    while True:
        ui.show(title, ["The motor turns now…"])
        try:
            motor.setSpeed(PULSE_SPEED, acceleration=ACCELERATION)
            time.sleep(PULSE_SECS)
        finally:
            motor.brake(acceleration=ACCELERATION)
        answer = ui.choose(title, options + [("Again", 'again')], lines=lines)
        if answer != 'again':
            return answer


def test_motors(ui: TerminalWizard, psm: 'PiStorms', values: dict[str, Any]) -> None:
    """Find which motor is the left wheel and which way each one drives
    forward, by pulsing each motor of the drive bank."""
    bank = values['drive_bank']
    ui.message("Motor test", ["Each wheel turns briefly, one at a time.",
                              "Watch which one, and which way.",
                              "Robot on the floor, or wheels in the air."], "Start")
    side, m1_sign = pulse_and_ask(
        ui, getattr(psm, f"B{bank}M1"), f"Motor B{bank}M1", ["Which wheel turned, which way?"],
        [("Left, ahead", ('left', 1)), ("Left, back", ('left', -1)),
         ("Right, ahead", ('right', 1)), ("Right, back", ('right', -1))])
    m2_sign = pulse_and_ask(
        ui, getattr(psm, f"B{bank}M2"), f"Motor B{bank}M2", ["The other wheel turned…"],
        [("Ahead", 1), ("Back", -1)])
    if side == 'left':
        values.update(left_motor='M1', left_fwd_sign=m1_sign, right_fwd_sign=m2_sign)
    else:
        values.update(left_motor='M2', left_fwd_sign=m2_sign, right_fwd_sign=m1_sign)


def ask_settings(ui: TerminalWizard, psm: 'PiStorms', values: dict[str, Any]) -> None:
    """Ask for, or confirm, every value of the profile that the robot can't
    measure. Changes `values`."""
    wheel = ui.choose("Wheels", [(f"{name} {diameter * 1000:.1f}", name)
                                 for name, diameter in WHEELS.items()],
                      values['wheel'], ["Tyre diameter in mm (* = now)"])
    if wheel != values['wheel']:
        values.pop('wheel_diameter', None)      # measured on the old wheels
    values['wheel'] = wheel
    values['wheel_base'] = ui.ask_mm("Wheel base", values['wheel_base'],
                                        ["Between the middles of the two tyres"])
    values['drive_bank'] = ui.choose("Drive motors", [("Bank A", 'A'), ("Bank B", 'B')],
                                     values['drive_bank'], ["Both on the same bank (M1 and M2)"])
    motors_now = (f"Left wheel on {values['left_motor']}, ahead = "
                  f"{values['left_fwd_sign']:+d} / {values['right_fwd_sign']:+d}")
    if ui.choose("Motor directions", [("Test", 'test'), ("Keep", 'keep')], 'keep',
                 [motors_now, "Test: each wheel turns briefly."]) == 'test':
        test_motors(ui, psm, values)

    values['body_front'] = ui.ask_mm("Robot: front", values['body_front'],
                                        ["From the wheel axle to the nose"])
    values['body_rear'] = ui.ask_mm("Robot: back", values['body_rear'],
                                       ["From the wheel axle to the very back"])
    values['body_width'] = ui.ask_mm("Robot: width", values['body_width'],
                                        ["At the widest, wheels included"])

    imu_port = ui.choose("IMU port", [("No IMU", None)] + [(port, port) for port in SENSOR_PORTS],
                         values.get('imu_port'))
    lidar_ports = list(LIDAR_PORTS)
    if values.get('lidar_port') and values['lidar_port'] not in lidar_ports:
        lidar_ports.append(values['lidar_port'])
    lidar_port = ui.choose("LiDAR port", [("No LiDAR", None)] + [(port, port) for port in lidar_ports],
                           values.get('lidar_port'))
    for name, port in (('imu_port', imu_port), ('lidar_port', lidar_port)):
        if port is None:
            values.pop(name, None)
        else:
            values[name] = port


def profile_from_values(values: dict[str, Any]) -> RobotProfile:
    """The profile for these values. A LiDAR offset that makes no sense
    (outside the robot: a typing mistake, or the robot was rebuilt) is set
    to 0 until the calibration measures it."""
    try:
        return profile_from_json(values)
    except ValueError as error:
        if 'lidar_d' not in str(error):
            raise
        print(f"   ⚠ {error} — set to 0 until it is measured")
        return profile_from_json(dict(values, lidar_dx=0.0, lidar_dy=0.0))
