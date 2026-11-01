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

#mindsensors.com invests time and resources providing this open source code,
#please support mindsensors.com  by purchasing products from mindsensors.com!
#Learn more product option visit us @  http://www.mindsensors.com/

import time

from . import registers
from .mindsensors_i2c import mindsensors_i2c


## SensorPort: one sensor port of the PiStorms board (psm.BAS1, BAS2, BBS1, BBS2).
#  It only knows what the board does on a port: the port type, the EV3 mode
#  and the port registers. The sensors plugged into it are classes of
#  pistorms.devices that receive the port:
#      EV3GyroSensor(psm.BAS1)   — LEGO sensor read by the board (registers below)
#      AbsoluteIMU(psm.BBS2)     — I2C device: the port is only put in I2C mode
class SensorPort():
    PS_SENSOR_TYPE_NONE = 0
    PS_SENSOR_TYPE_SWITCH = 1
    PS_SENSOR_TYPE_ANALOG = 2
    PS_SENSOR_TYPE_LIGHT_ACTIVE = 3
    PS_SENSOR_TYPE_LIGHT_INACTIVE = 4
    PS_SENSOR_TYPE_SOUND_DB = 5
    PS_SENSOR_TYPE_SOUND_DBA = 6
    PS_SENSOR_TYPE_LOWSPEED_9V = 7
    PS_SENSOR_TYPE_LOWSPEED = 8
    PS_SENSOR_TYPE_CUSTOM = 9
    PS_SENSOR_TYPE_COLORFULL = 13
    PS_SENSOR_TYPE_COLORRED = 14
    PS_SENSOR_TYPE_COLORGREEN = 15
    PS_SENSOR_TYPE_COLORBLUE = 16
    PS_SENSOR_TYPE_COLORNONE = 17
    PS_SENSOR_TYPE_EV3_SWITCH = 18
    PS_SENSOR_TYPE_EV3 = 19

    # Register areas of a port, as sensor-1 registers (Advanced Development Guide):
    ANALOG_DATA = 0x70   # NXT analog value (2 bytes) / LEGO color registers / EV3 "ready"
    EV3_ID      = 0x71   # EV3 sensor name, up to 16 characters
    EV3_DATA    = 0x83   # EV3 sensor data, up to 32 bytes

    # Every register of sensor 2 is the register of sensor 1 moved by this
    # offset (mode 0x6F → 0xA3, ready 0x70 → 0xA4, data 0x83 → 0xB7, ...).
    SENSOR2_OFFSET = 0x34

    def __init__(self, bank: mindsensors_i2c, num: int = 0) -> None:
        self.bank: mindsensors_i2c = bank
        self.sensornum: int = num
        self.type: int = self.PS_SENSOR_TYPE_NONE

    def _reg(self, sensor1_register: int) -> int:
        """Register of this port, given the sensor-1 register:
            _reg(EV3_DATA) → 0x83 on port 1, 0xB7 on port 2"""
        if self.sensornum == 2:
            return sensor1_register + self.SENSOR2_OFFSET
        return sensor1_register

    def setType(self, type: int) -> None:
        """Set the port type (PS_SENSOR_TYPE_...). Does nothing if it is
        already set; otherwise the board needs 1 s, except for I2C."""
        if(type != self.type):
            self.type = type
            self.bank.writeByte(self._reg(registers.PS_S1_Mode), type)
            if(self.type != self.PS_SENSOR_TYPE_CUSTOM):
                time.sleep(1)
    def getType(self) -> int:
        return self.type
    def setModeEV3(self, mode: int) -> None:
        """Mode of the EV3 sensor plugged into this port (see its class)."""
        self.bank.writeByte(self._reg(registers.PS_S1EV_Mode), mode)

    # Register access for the devices plugged into this port: each takes the
    # sensor-1 register (ANALOG_DATA, EV3_DATA, ... + offset) and reads the
    # register of this port, e.g. port.readByte(port.EV3_DATA + 1).
    def readByte(self, sensor1_register: int) -> int:
        return self.bank.readByte(self._reg(sensor1_register))
    def readByteSigned(self, sensor1_register: int) -> int:
        return self.bank.readByteSigned(self._reg(sensor1_register))
    def readInteger(self, sensor1_register: int) -> int:
        return self.bank.readInteger(self._reg(sensor1_register))
    def readIntegerSigned(self, sensor1_register: int) -> int:
        return self.bank.readIntegerSigned(self._reg(sensor1_register))
    def readArray(self, sensor1_register: int, length: int) -> list[int]:
        return self.bank.readArray(self._reg(sensor1_register), length)
    def writeByte(self, sensor1_register: int, value: int) -> None:
        self.bank.writeByte(self._reg(sensor1_register), value)

    def activateCustomSensorI2C(self) -> None:
        # Put the port in I2C mode, for a mindsensors I2C device.
        # Always write the mode register — the PiStorms board silently resets
        # the port to NONE after ~1s of inactivity, but setType caches the
        # type and would otherwise skip the re-write.
        self.type = self.PS_SENSOR_TYPE_CUSTOM
        self.bank.writeByte(self._reg(registers.PS_S1_Mode), self.PS_SENSOR_TYPE_CUSTOM)
