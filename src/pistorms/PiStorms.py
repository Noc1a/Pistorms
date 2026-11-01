#!/usr/bin/env python3
#
# Copyright (c) 2015 mindsensors.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#mindsensors.com invests time and resources providing this open source code,
#please support mindsensors.com  by purchasing products from mindsensors.com!
#Learn more product option visit us @  http://www.mindsensors.com/

import functools
import subprocess
import time
from typing import Callable, Optional

from . import banks, registers
from .mindsensors_i2c import mindsensors_i2c
from .motor import PiStormsMotor
from .sensor_port import SensorPort
from .ui.mindsensorsUI import mindsensorsUI


class PiStorms:
    """The PiStorms board: 4 motor ports, 4 sensor ports, the screen, the
    LED, the GO button and the battery.

        from pistorms import PiStorms
        from pistorms.devices import AbsoluteIMU
        psm = PiStorms()
        psm.BAM1.setSpeedSync(50, acceleration=100)   # motors BAM1 … BBM2
        imu = AbsoluteIMU(psm.BBS2)                     # sensors on BAS1 … BBS2
        psm.screen.termPrintln(f"Battery {psm.battVoltage():.1f} V")
    """

    def __init__(self, name: str = "PiStorms", rotation: int = 3,
                 reset: bool = True, screen: bool = True) -> None:
        """
        name, rotation: title and rotation of the screen.
        reset: reset the motors and encoders, clear the screen and the GO
               button count. Pass False from background scripts (battery
               monitor, loggers), so that starting them doesn't stop the
               motors of a robot program already running.
        screen: open the screen. Pass False from programs that only use
                motors and sensors: self.screen is then None.
        """
        self.bankA: mindsensors_i2c = banks.bank_a()
        self.bankB: mindsensors_i2c = banks.bank_b()

        # Motor ports, e.g. psm.BAM1.setSpeedSync(50)
        self.BAM1: PiStormsMotor = PiStormsMotor(self.bankA, 1)
        self.BAM2: PiStormsMotor = PiStormsMotor(self.bankA, 2)
        self.BBM1: PiStormsMotor = PiStormsMotor(self.bankB, 1)
        self.BBM2: PiStormsMotor = PiStormsMotor(self.bankB, 2)

        # Sensor ports: a sensor is a class of pistorms.devices that takes its
        # port, e.g. EV3GyroSensor(psm.BAS1) or AbsoluteIMU(psm.BBS2)
        self.BAS1: SensorPort = SensorPort(self.bankA, 1)
        self.BAS2: SensorPort = SensorPort(self.bankA, 2)
        self.BBS1: SensorPort = SensorPort(self.bankB, 1)
        self.BBS2: SensorPort = SensorPort(self.bankB, 2)

        if reset and self._isConnected():
            self._resetMotors()

        self.screen: Optional[mindsensorsUI]
        if screen:
            self.screen = mindsensorsUI(name, rotation, clear=reset)
        else:
            self.screen = None

        if reset:
            self.resetKeyPressCount()

    def _isConnected(self) -> bool:
        try:
            self.bankA.readByte(registers.PS_BattV)
            return True
        except OSError as e:
            # Keep going (the screen may still work), but every motor or
            # sensor call will raise OSError until the board answers.
            print(f"Could not connect to PiStorms: {e}")
            return False

    def _resetMotors(self) -> None:
        """Stop the motors, zero the encoders and clear the stale tacho
        setpoints (see PiStormsMotor._needs_reset): without this, the very
        first setSpeed after power-on can make a wheel twitch the wrong way."""
        self.bankA.writeByte(registers.PS_Command, registers.R)
        self.bankB.writeByte(registers.PS_Command, registers.R)
        try:
            time.sleep(0.05)
            for motor in (self.BAM1, self.BBM1):      # one motor per bank
                for i in (0, 1):
                    motor._reset_setpoint(i)
        except OSError as e:
            print(f"Motor setpoint reset failed at init: {e}")

    # ── Power ──────────────────────────────────────────────────────────────

    def battVoltage(self) -> Optional[float]:
        """Battery voltage in volts, or None if it couldn't be read. Never 0
        on a read error: callers treat a low voltage as a reason to shut down."""
        try:
            return self.bankB.readByte(registers.PS_BattV) * 0.04
        except OSError:
            return None

    def Shutdown(self) -> None:
        """Power off the Raspberry Pi. systemd stops everything cleanly
        (disks flushed, logs saved); at the very end, its hook
        /usr/lib/systemd/system-shutdown/pistorms-poweroff calls
        startBoardPowerOff() to cut the board power."""
        subprocess.run(["/bin/systemctl", "poweroff"])

    def startBoardPowerOff(self) -> None:
        """Start the board's power-cut countdown. Only for the systemd
        shutdown hook: called while the Pi runs, it cuts its power."""
        self.bankA.writeByte(registers.PS_Command, registers.H)

    # ── Board identification ───────────────────────────────────────────────

    def GetFirmwareVersion(self) -> str:
        return self.bankB.GetFirmwareVersion()

    def GetVendorName(self) -> str:
        return self.bankA.GetVendorName()

    def GetDeviceId(self) -> str:
        return self.bankA.GetDeviceId()

    # ── LED ────────────────────────────────────────────────────────────────

    def led(self, lednum: int, red: int, green: int, blue: int) -> None:
        """Set the LED color, each component 0–255. lednum 1 = bank A,
        2 = bank B (boards have one LED, reachable from either bank)."""
        if lednum == 1:
            bank = self.bankA
        else:
            bank = self.bankB
        bank.writeArray(registers.PS_R, [red, green, blue])
        time.sleep(0.001)

    # ── GO button ──────────────────────────────────────────────────────────
    # (the F1–F4 keys are areas of the touchscreen: see psm.screen)

    def isKeyPressed(self) -> int:
        """1 while the GO button is pressed, else 0."""
        try:
            return self.bankA.readByte(registers.PS_KeyPress) & 0x01
        except OSError:
            return 0

    def getKeyPressCount(self) -> int:
        """Number of GO button presses since the last reset."""
        try:
            return self.bankA.readByte(registers.PS_Key1Count)
        except OSError:
            return 0

    def resetKeyPressCount(self) -> None:
        try:
            self.bankA.writeByte(registers.PS_Key1Count, 0)
        except OSError:
            pass

    def waitForKeyPress(self) -> None:
        """Wait until the GO button is pressed."""
        self.untilKeyPress(functools.partial(time.sleep, 0.01))

    def untilKeyPress(self, func: Callable[..., object], *args, **kwargs) -> None:
        """Call func(*args, **kwargs) repeatedly until the GO button is pressed."""
        initial_count = self.getKeyPressCount()
        while self.getKeyPressCount() == initial_count:
            func(*args, **kwargs)

    def untilKeyPressOrTouch(self, func: Callable[..., object], *args, **kwargs) -> None:
        """Call func repeatedly until the GO button is pressed or the screen touched."""
        initial_count = self.getKeyPressCount()
        while self.getKeyPressCount() == initial_count and not self.screen.isTouched():
            func(*args, **kwargs)

    def untilTouch(self, func: Callable[..., object], *args, **kwargs) -> None:
        """Call func repeatedly until the screen is touched."""
        while not self.screen.isTouched():
            func(*args, **kwargs)
