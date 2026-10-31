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

# History:
# Date      Author      Comments
#  July 2015  Henry     Initial Authoring

from .mindsensors_i2c import mindsensors_i2c
import time, numpy
import configparser
import ctypes

class PSSensor():
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

    PS_SENSOR_COLOR_NONE = 0
    PS_SENSOR_COLOR_BLACK = 1
    PS_SENSOR_COLOR_BLUE = 2
    PS_SENSOR_COLOR_GREEN = 3
    PS_SENSOR_COLOR_YELLOW = 4
    PS_SENSOR_COLOR_RED = 5
    PS_SENSOR_COLOR_WHITE = 6
    PS_SENSOR_COLOR_BROWN = 7

    SE_None = 0
    SE_Front = 1
    SE_Left = 2
    SE_Right = 3

    def __init__(self, bank, num=0):
        self.bank = bank
        self.sensornum = num
        self.type = self.PS_SENSOR_TYPE_NONE
        self.EV3Cache = [ 0, [0]*16, 0, 0, [0]*32 ] # ready, ID[16], mode, length, data[32]
    def setType(self, type):
        if(type != self.type):
            self.type = type
            if(self.sensornum == 1):
                self.bank.writeByte(PiStormsCom.PS_S1_Mode,type)
            if(self.sensornum == 2):
                self.bank.writeByte(PiStormsCom.PS_S2_Mode,type)
            if(self.type != self.PS_SENSOR_TYPE_CUSTOM):
                time.sleep(1)
    def getType(self):
        return self.type
    def EV3Retrieve(self):
        if(self.sensornum == 1):
            self.EV3Cache[0] = self.bank.readByte(PiStormsCom.PS_S1EV_Ready)
            self.EV3Cache[1] = self.bank.readArray(PiStormsCom.PS_S1EV_SensorID, 16)
            self.EV3Cache[2] = self.bank.readByte(PiStormsCom.PS_S1EV_Mode)
            self.EV3Cache[3] = self.bank.readByte(PiStormsCom.PS_S1EV_Length)
            self.EV3Cache[4] = self.bank.readArray(PiStormsCom.PS_S1EV_Data, 32)
        if(self.sensornum == 2):
            self.EV3Cache[0] = self.bank.readByte(PiStormsCom.PS_S2EV_Ready)
            self.EV3Cache[1] = self.bank.readArray(PiStormsCom.PS_S2EV_SensorID, 16)
            self.EV3Cache[2] = self.bank.readByte(PiStormsCom.PS_S2EV_Mode)
            self.EV3Cache[3] = self.bank.readByte(PiStormsCom.PS_S2EV_Length)
            self.EV3Cache[4] = self.bank.readArray(PiStormsCom.PS_S2EV_Data, 32)
    def isTouchedEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3_SWITCH)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data) == 1
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data) == 1
    def numTouchesEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3_SWITCH)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data+1)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data+1)
    def resetTouchesEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3_SWITCH)
        if(self.sensornum == 1):
            self.bank.writeByte(PiStormsCom.PS_S1EV_Data+1, 0)
        if(self.sensornum == 2):
            self.bank.writeByte(PiStormsCom.PS_S2EV_Data+1, 0)
    def setModeEV3(self, mode):
        if(self.sensornum == 1):
            self.bank.writeByte(PiStormsCom.PS_S1EV_Mode, mode)
        if(self.sensornum == 2):
            self.bank.writeByte(PiStormsCom.PS_S2EV_Mode, mode)
    def distanceIREV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readInteger(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readInteger(PiStormsCom.PS_S2EV_Data)
    def rawIREV3(self, mode):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(mode)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data)
    def headingIREV3(self, channel):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(1)
        if(self.sensornum == 1):
            return self.bank.readByteSigned(PiStormsCom.PS_S1EV_Data + (channel-1)*2)
        if(self.sensornum == 2):
            return self.bank.readByteSigned(PiStormsCom.PS_S2EV_Data + (channel-1)*2)
    def distanceRemoteIREV3(self, channel):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(1)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data + ((channel-1)*2)+1)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data + ((channel-1)*2)+1)
    def remoteLeft(self, channel):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(2)
        if(self.sensornum == 1):
            remote = self.bank.readByte(PiStormsCom.PS_S1EV_Data + (channel-1))
        if(self.sensornum == 2):
            remote = self.bank.readByte(PiStormsCom.PS_S2EV_Data + (channel-1))
        if(remote in [0, 3, 4]):
            return 0
        if(remote in [1, 5, 6]):
            return 1
        if(remote in [2, 7, 8]):
            return -1
    def remoteRight(self, channel):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(2)
        if(self.sensornum == 1):
            remote = self.bank.readByte(PiStormsCom.PS_S1EV_Data + (channel-1))
        if(self.sensornum == 2):
            remote = self.bank.readByte(PiStormsCom.PS_S2EV_Data + (channel-1))
        if(remote in [0, 1, 2]):
            return 0
        if(remote in [3, 5, 7]):
            return 1
        if(remote in [4, 6, 8]):
            return -1
    def distanceUSEV3cm(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readInteger(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readInteger(PiStormsCom.PS_S2EV_Data)
    def distanceUSEV3in(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(1)
        if(self.sensornum == 1):
            return self.bank.readInteger(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readInteger(PiStormsCom.PS_S2EV_Data)
    def presenceUSEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(2)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data) == 1
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data) == 1
    def rawGyro(self, mode):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(mode)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data)
    def gyroAngleEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readIntegerSigned(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readIntegerSigned(PiStormsCom.PS_S2EV_Data)
    def gyroRateEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(1)
        if(self.sensornum == 1):
            return self.bank.readIntegerSigned(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readIntegerSigned(PiStormsCom.PS_S2EV_Data)
    def reflectedLightSensorEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data)
    def ambientLightSensorEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(1)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data)
    def colorSensorEV3(self):
        self.setType(self.PS_SENSOR_TYPE_EV3)
        self.setModeEV3(2)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data)
    def readNXT(self):
        if(self.sensornum == 1):
            return self.bank.readInt(PiStormsCom.PS_S1EV_Ready)
        if(self.sensornum == 2):
            return self.bank.readInt(PiStormsCom.PS_S2EV_Ready)
    def isTouchedNXT(self):
        self.setType(self.PS_SENSOR_TYPE_SWITCH)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data) == 1
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data) == 1
    def numTouchesNXT(self):
        self.setType(self.PS_SENSOR_TYPE_SWITCH)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Data + 1)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Data + 1)
    def resetTouchesNXT(self):
        if(self.sensornum == 1):
            self.bank.writeByte(PiStormsCom.PS_S1EV_Data+1, 0)
        if(self.sensornum == 2):
            self.bank.writeByte(PiStormsCom.PS_S2EV_Data+1, 0)
    def lightSensorNXT(self, active=True):
        if(active):
            self.setType(self.PS_SENSOR_TYPE_LIGHT_ACTIVE)
        else:
            self.setType(self.PS_SENSOR_TYPE_LIGHT_INACTIVE)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readInteger(PiStormsCom.PS_S1EV_Ready)
        if(self.sensornum == 2):
            return self.bank.readInteger(PiStormsCom.PS_S2EV_Ready)
    def SumoEyes(self, long=True):
        if(long):
            self.setType(self.PS_SENSOR_TYPE_LIGHT_INACTIVE)
        else:
            self.setType(self.PS_SENSOR_TYPE_LIGHT_ACTIVE)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            comet = self.bank.readInteger(PiStormsCom.PS_S1EV_Ready)
        if(self.sensornum == 2):
            comet = self.bank.readInteger(PiStormsCom.PS_S2EV_Ready)
        def isNear(reference, delta, comet):
            return (comet > (reference - delta)) and (comet < (reference + delta))
        if(isNear(465, 30, comet)):
            return self.SE_Front
        elif(isNear(800, 30, comet)):
            return self.SE_Left
        elif(isNear(555, 30, comet)):
            return self.SE_Right
        else:
            return self.SE_None
    def colorSensorRawNXT(self, smode=13):
        self.setType(smode)
        self.setModeEV3(0)
        self.EV3Retrieve()
        return self.EV3Cache[0:1] + self.EV3Cache[1]
    def colorSensorNXT(self, smode=13):
        self.setType(smode)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Ready)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Ready)
    def colorSensorNoneNXT(self):
        return self.colorSensorRawNXT(self.PS_SENSOR_TYPE_COLORNONE)[0]
    def colorSensorRedNXT(self):
        return self.colorSensorRawNXT(self.PS_SENSOR_TYPE_COLORRED)[0]
    def colorSensorGreenNXT(self):
        return self.colorSensorRawNXT(self.PS_SENSOR_TYPE_COLORGREEN)[0]
    def colorSensorBlueNXT(self):
        return self.colorSensorRawNXT(self.PS_SENSOR_TYPE_COLORBLUE)[0]
    def analogSensor(self): # untested
        self.setType(self.PS_SENSOR_TYPE_ANALOG)
        self.setModeEV3(0)
        if(self.sensornum == 1):
            return self.bank.readByte(PiStormsCom.PS_S1EV_Ready)
        if(self.sensornum == 2):
            return self.bank.readByte(PiStormsCom.PS_S2EV_Ready)
    def activateCustomSensorI2C(self):
        # Always write the mode register — the PiStorms board silently resets
        # the port to NONE after ~1s of inactivity, but PSSensor.setType caches
        # the type and would otherwise skip the re-write.
        self.type = self.PS_SENSOR_TYPE_CUSTOM
        if(self.sensornum == 1):
            self.bank.writeByte(PiStormsCom.PS_S1_Mode, self.PS_SENSOR_TYPE_CUSTOM)
        if(self.sensornum == 2):
            self.bank.writeByte(PiStormsCom.PS_S2_Mode, self.PS_SENSOR_TYPE_CUSTOM)


def _clamp_speed(speed):
    # smbus keeps only the low byte: an out-of-range speed like -160 would
    # wrap to +96 and spin the wheel the wrong way.
    return max(-100, min(100, int(speed)))


class PSMotor():
    STOP_ATTEMPTS = 3   # outer retries for brake/float (each writeByte retries too)

    def __init__(self, bank, num=0):
        self.bank = bank
        self.motornum = num
        # Last commanded speed of each motor, stored on the BANK — not on this
        # object — because BAM1 and BAM2 are separate PSMotor objects that
        # both drive the bank's M1/M2 (sync commands write both). Per-object
        # copies drifted apart and broke _needs_reset.
        if not hasattr(bank, 'motor_speeds'):
            bank.motor_speeds = [0, 0]

    @property
    def _speed_M1(self):
        return self.bank.motor_speeds[0]

    @_speed_M1.setter
    def _speed_M1(self, v):
        self.bank.motor_speeds[0] = v

    @property
    def _speed_M2(self):
        return self.bank.motor_speeds[1]

    @_speed_M2.setter
    def _speed_M2(self, v):
        self.bank.motor_speeds[1] = v

    def _stop_command(self, cmd):
        """Write a brake/float command. Stopping the motors must not be lost to
        a bus glitch, so retry harder than ordinary writes and log failures."""
        for attempt in range(self.STOP_ATTEMPTS):
            try:
                self.bank.writeByte(PiStormsCom.PS_Command, cmd)
                return
            except OSError as e:
                print(f"Motor stop command 0x{cmd:02X} failed ({e}) "
                      f"[{attempt + 1}/{self.STOP_ATTEMPTS}]", flush=True)
                if attempt == self.STOP_ATTEMPTS - 1:
                    raise
                time.sleep(0.01)

    def pos(self):
        if self.motornum == 1:
            return ctypes.c_int32(self.bank.readLongSigned(PiStormsCom.PS_Position_M1)).value
        elif self.motornum == 2:
            return ctypes.c_int32(self.bank.readLongSigned(PiStormsCom.PS_Position_M2)).value
    def resetPos(self):
        if(self.motornum == 1):
            self.bank.writeByte(PiStormsCom.PS_Command, 114)
        elif(self.motornum == 2):
            self.bank.writeByte(PiStormsCom.PS_Command, 115)

    # ── firmware tacho-setpoint reset ────────────────────────────────────────
    # The PiStorms firmware keeps a latched tacho setpoint from the previous
    # move. When a fresh speed cmd is issued in a new direction (from stop OR
    # reversing sign), the position PID briefly pulls the wheel toward that
    # stale target before the speed loop wins — the "~1 s wrong-way twitch".
    # Writing SetPoint with TACHO+RELATIVE+GO (a runDegs(0) burst) arms the PID
    # with target = current position, effectively clearing the stale state.
    # SPEED-only writes to Speed register don't clear it — the TACHO bit must
    # be exercised. Once cleared, the actual setSpeed / sync-fire overrides.
    @staticmethod
    def _needs_reset(prev, new):
        if new == 0:
            return False
        return prev == 0 or prev * new < 0

    def _reset_setpoint_m1(self):
        ctrl = (PiStormsCom.PS_CONTROL_SPEED | PiStormsCom.PS_CONTROL_TACHO
                | PiStormsCom.PS_CONTROL_RELATIVE | PiStormsCom.PS_CONTROL_GO)
        self.bank.writeArray(PiStormsCom.PS_SetPoint_M1, [0, 0, 0, 0, 100, 0, 0, ctrl])

    def _reset_setpoint_m2(self):
        ctrl = (PiStormsCom.PS_CONTROL_SPEED | PiStormsCom.PS_CONTROL_TACHO
                | PiStormsCom.PS_CONTROL_RELATIVE | PiStormsCom.PS_CONTROL_GO)
        self.bank.writeArray(PiStormsCom.PS_SetPoint_M2, [0, 0, 0, 0, 100, 0, 0, ctrl])

    def setSpeedSync(self, speed):
        speed = _clamp_speed(speed)
        if self._needs_reset(self._speed_M1, speed): self._reset_setpoint_m1()
        if self._needs_reset(self._speed_M2, speed): self._reset_setpoint_m2()
        ctrl = PiStormsCom.PS_CONTROL_SPEED
        array = [int(speed), 0, 0, ctrl]
        self.bank.writeArray(PiStormsCom.PS_Speed_M1, array)
        self.bank.writeArray(PiStormsCom.PS_Speed_M2, array)
        self._speed_M1 = speed
        self._speed_M2 = speed
        self.bank.writeByte(PiStormsCom.PS_Command, PiStormsCom.S)

    def setSpeedSyncPair(self, speed_m1, speed_m2):
        """Write both motor speeds and fire them together with 'S'.
        Bypasses PS_CONTROL_GO so neither motor starts until the 'S' command,
        eliminating the I2C-transaction delay between the two writes that
        otherwise makes one wheel start visibly later than the other."""
        speed_m1, speed_m2 = _clamp_speed(speed_m1), _clamp_speed(speed_m2)
        if self._needs_reset(self._speed_M1, speed_m1): self._reset_setpoint_m1()
        if self._needs_reset(self._speed_M2, speed_m2): self._reset_setpoint_m2()
        ctrl = PiStormsCom.PS_CONTROL_SPEED
        self.bank.writeArray(PiStormsCom.PS_Speed_M1, [int(speed_m1), 0, 0, ctrl])
        self.bank.writeArray(PiStormsCom.PS_Speed_M2, [int(speed_m2), 0, 0, ctrl])
        self._speed_M1 = speed_m1
        self._speed_M2 = speed_m2
        self.bank.writeByte(PiStormsCom.PS_Command, PiStormsCom.S)
        sig = (int(speed_m1), int(speed_m2))
        if sig != getattr(self, '_last_sync_sig', None):
            print(f"[SYNC] M1={int(speed_m1):+4d} M2={int(speed_m2):+4d}", flush=True)
            self._last_sync_sig = sig

    def setSpeedSyncOpposite(self, speed):
        speed = _clamp_speed(speed)
        if self._needs_reset(self._speed_M1, speed): self._reset_setpoint_m1()
        if self._needs_reset(self._speed_M2, -speed): self._reset_setpoint_m2()
        ctrl = PiStormsCom.PS_CONTROL_SPEED
        array_M1 = [int(speed), 0, 0, ctrl]
        array_M2 = [int(-speed), 0, 0, ctrl]
        self.bank.writeArray(PiStormsCom.PS_Speed_M1, array_M1)
        self.bank.writeArray(PiStormsCom.PS_Speed_M2, array_M2)
        self._speed_M1 = speed
        self._speed_M2 = -speed
        self.bank.writeByte(PiStormsCom.PS_Command, PiStormsCom.S)

    def floatSync(self):
        self._stop_command(PiStormsCom.c)
        self._speed_M1 = 0
        self._speed_M2 = 0
    def brakeSync(self):
        # issue command 'C', break while stopping
        self._stop_command(PiStormsCom.C)
        self._speed_M1 = 0
        self._speed_M2 = 0
    def setSpeed(self, speed):
        speed = _clamp_speed(speed)
        if(speed == 0):
            self.float()
            return
        if self.motornum == 1:
            if self._needs_reset(self._speed_M1, speed): self._reset_setpoint_m1()
        elif self.motornum == 2:
            if self._needs_reset(self._speed_M2, speed): self._reset_setpoint_m2()
        ctrl = PiStormsCom.PS_CONTROL_SPEED | PiStormsCom.PS_CONTROL_GO
        array = [int(speed), 0, 0, ctrl]
        if (self.motornum == 1):
            self.bank.writeArray(PiStormsCom.PS_Speed_M1, array)
            self._speed_M1 = speed
        if (self.motornum == 2):
            self.bank.writeArray(PiStormsCom.PS_Speed_M2, array)
            self._speed_M2 = speed

    def getSpeed(self):
        if (self.motornum == 1):
            return self._speed_M1
        if (self.motornum == 2):
            return self._speed_M2
    def brake(self):
        if(self.motornum == 1):
            self._stop_command(PiStormsCom.A)
            self._speed_M1 = 0
        if(self.motornum == 2):
            self._stop_command(PiStormsCom.B)
            self._speed_M2 = 0
    def float(self):
        if(self.motornum == 1):
            self._stop_command(PiStormsCom.a)
            self._speed_M1 = 0
        if(self.motornum == 2):
            self._stop_command(PiStormsCom.b)
            self._speed_M2 = 0
    def hold(self):
        ctrl = PiStormsCom.PS_CONTROL_BRK | \
               PiStormsCom.PS_CONTROL_ON | \
               PiStormsCom.PS_CONTROL_TACHO | \
               PiStormsCom.PS_CONTROL_GO
        if(self.motornum == 1):
            self.bank.writeArray(PiStormsCom.PS_SetPoint_M1, [0,0,0,0])
            self.bank.writeByte(PiStormsCom.PS_CMDA_M1, ctrl)
            self._speed_M1 = 0
        else:
            self.bank.writeArray(PiStormsCom.PS_SetPoint_M2, [0,0,0,0])
            self.bank.writeByte(PiStormsCom.PS_CMDA_M2, ctrl)
            self._speed_M2 = 0
    def runSecs(self, secs, speed, brakeOnCompletion=False):
        if self.motornum == 1:
            if self._needs_reset(self._speed_M1, speed): self._reset_setpoint_m1()
        elif self.motornum == 2:
            if self._needs_reset(self._speed_M2, speed): self._reset_setpoint_m2()
        ctrl = PiStormsCom.PS_CONTROL_SPEED | \
               PiStormsCom.PS_CONTROL_TIME | \
               PiStormsCom.PS_CONTROL_GO
        if(brakeOnCompletion):
            ctrl |= PiStormsCom.PS_CONTROL_BRK
        array = [speed, secs, 0, ctrl]
        if(self.motornum == 1):
            self.bank.writeArray(PiStormsCom.PS_Speed_M1, array)
            self._speed_M1 = speed
        if(self.motornum == 2):
            self.bank.writeArray(PiStormsCom.PS_Speed_M2, array)
            self._speed_M2 = speed
    def status(self):
        if(self.motornum == 1):
            return self.bank.readByte(PiStormsCom.PS_Status_M1)
        if(self.motornum == 2):
            return self.bank.readByte(PiStormsCom.PS_Status_M2)
    def statusBit(self, bitno=0):
        return ((self.status() >> bitno) & 1) == 1
    def isBusy(self):
        return any(map(self.statusBit, [0,1,3,6]))
    def waitUntilNotBusy(self, timeout=-1):
        while(self.isBusy()):
            time.sleep(0.01)
            timeout -= 1
            if(timeout == 0):
                return 1
            if(timeout < -5):
                timeout = -1
        return 0
    def isStalled(self):
        return self.statusBit(7)
    def isOverloaded(self):
        return self.statusBit(5)
    def runDegs(self, degs, speed=100, brakeOnCompletion=False, holdOnCompletion=False):
        # holdOnCompletion is not supported on LEGO motors, ignored
        # Non-blocking: returns once the setpoint is written; use
        # waitUntilNotBusy() to wait for the move to finish.
        # smbus only accepts ints — a float here used to raise inside the
        # (formerly silent) writeArray, so the command was dropped.
        degs  = int(round(degs))
        speed = _clamp_speed(speed)
        ctrl = PiStormsCom.PS_CONTROL_SPEED | \
               PiStormsCom.PS_CONTROL_TACHO | \
               PiStormsCom.PS_CONTROL_RELATIVE | \
               PiStormsCom.PS_CONTROL_GO
        if(brakeOnCompletion):
            ctrl |= PiStormsCom.PS_CONTROL_BRK
        b4 = (degs//0x1000000)
        b3 = ((degs%0x1000000)//0x10000)
        b2 = (((degs%0x1000000)%0x10000)//0x100)
        b1 = (((degs%0x1000000)%0x10000)%0x100)
        array = [b1, b2, b3, b4, speed, 0, 0, ctrl]

        if(self.motornum == 1):
            self.bank.writeArray(PiStormsCom.PS_SetPoint_M1, array)
            self._speed_M1 = speed
        if(self.motornum == 2):
            self.bank.writeArray(PiStormsCom.PS_SetPoint_M2, array)
            self._speed_M2 = speed
    ## Reads the values of the PID control registers
    def ReadPerformanceParameters(self):
        try:
            b0 = self.bank.readInteger(PiStormsCom.PS_P_Kp)
            b1 = self.bank.readInteger(PiStormsCom.PS_P_Ki)
            b2 = self.bank.readInteger(PiStormsCom.PS_P_Kd)
            b3 = self.bank.readInteger(PiStormsCom.PS_S_Kp)
            b4 = self.bank.readInteger(PiStormsCom.PS_S_Ki)
            b5 = self.bank.readInteger(PiStormsCom.PS_S_Kd)
            b6 = self.bank.readByte(PiStormsCom.PS_PassCount)
            b7 = self.bank.readByte(PiStormsCom.PS_PassTolerance)
            return [b0, b1, b2, b3, b4, b5, b6, b7]
        except:
            print ("Error: Could not read PID values")
            return []
    def SetPerformanceParameters(self, Kp_tacho, Ki_tacho, Kd_tacho, Kp_speed, Ki_speed, Kd_speed, passcount, tolerance): # untested
        Kp_t1 = int(Kp_tacho%0x100)
        Kp_t2 = int(Kp_tacho/0x100)
        Ki_t1 = int(Ki_tacho%0x100)
        Ki_t2 = int(Ki_tacho/0x100)
        Kd_t1 = int(Kd_tacho%0x100)
        Kd_t2 = int(Kd_tacho/0x100)
        Kp_s1 = int(Kp_speed%0x100)
        Kp_s2 = int(Kp_speed/0x100)
        Ki_s1 = int(Ki_speed%0x100)
        Ki_s2 = int(Ki_speed/0x100)
        Kd_s1 = int(Kd_speed%0x100)
        Kd_s2 = int(Kd_speed/0x100)
        passcount = passcount
        tolerance = tolerance
        array = [Kp_t1, Kp_t2, Ki_t1, Ki_t2, \
                 Kd_t1, Kd_t2, Kp_s1, Kp_s2, \
                 Ki_s1, Ki_s2, Kd_s1, Kd_s2, \
                 passcount, tolerance]
        self.bank.writeArray(PiStormsCom.PS_P_Kp, array)


## PiStormsCom: this class provides communication functions for PiStorms.
#  Do not use this class directly in user programs, instead use functions provided by the LegoDevices or MsDevices classes.
class PiStormsCom(object):
    PS_SENSOR_TYPE_NONE = 0
    PS_SENSOR_TYPE_SWITCH = 1
    PS_SENSOR_TYPE_ANALOG = 2
    PS_SENSOR_TYPE_LIGHT_REFLECTED = 3
    PS_SENSOR_TYPE_LIGHT_AMBIENT = 4
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

    PS_SENSOR_MODE_EV3_COLOR_REFLECTED = 0
    PS_SENSOR_MODE_EV3_COLOR_AMBIENT = 1
    PS_SENSOR_MODE_EV3_COLOR_COLOR = 2
    PS_SENSOR_MODE_EV3_GYRO_ANGLE =  0
    PS_SENSOR_MODE_EV3_GYRO_RATE = 1
    PS_SENSOR_MODE_EV3_ULTRASONIC_DIST_CM = 0
    PS_SENSOR_MODE_EV3_ULTRASONIC_DIST_IN = 1
    PS_SENSOR_MODE_EV3_ULTRASONIC_DETECT = 2
    PS_SENSOR_MODE_EV3_IR_PROXIMITY = 0
    PS_SENSOR_MODE_EV3_IR_CHANNEL = 1
    PS_SENSOR_MODE_EV3_IR_REMOTE = 2
    PS_SENSOR_MODE_NXT_LIGHT_REFLECTED = 0
    PS_SENSOR_MODE_NXT_LIGHT_AMBIENT = 0
    PS_SENSOR_MODE_NXT_COLOR_COLOR = 0

    PS_A_ADDRESS = 0x34
    PS_B_ADDRESS = 0x36

    # Registers
    PS_Command = 0x41

    PS_SetPoint_M1 =0x42
    PS_Speed_M1 = 0x46
    PS_Time_M1 = 0x47
    PS_CMDB_M1 = 0x48
    PS_CMDA_M1 = 0x49

    PS_SetPoint_M2 =0x4A
    PS_Speed_M2 = 0x4E
    PS_Time_M2 = 0x4F
    PS_CMDB_M2 = 0x50
    PS_CMDA_M2 = 0x51

    PS_Position_M1 = 0x52
    PS_Position_M2 = 0x56

    PS_Status_M1 = 0x5A
    PS_Status_M2 = 0x5B

    PS_Tasks_M1 = 0x5C
    PS_Tasks_M2 = 0x5D

    PS_P_Kp = 0x5E
    PS_P_Ki = 0x60
    PS_P_Kd = 0x62
    PS_S_Kp = 0x64
    PS_S_Ki = 0x66
    PS_S_Kd = 0x68
    PS_PassCount = 0x6A
    PS_PassTolerance = 0x6B
    PS_ChkSum = 0x6C
    PS_BattV = 0x6E

    # Sensor 1
    PS_S1_Mode = 0x6F
    # EV3
    PS_S1EV_Ready = 0x70
    PS_S1EV_SensorID = 0x71
    PS_S1EV_Mode = 0x81
    PS_S1EV_Length = 0x82
    PS_S1EV_Data = 0x83
    # LEGO Analog
    PS_S1AN_Read = 0x70
    # LEGO Color
    PS_S1C_Color = 0x70
    PS_S1C_R_cal = 0x71
    PS_S1C_G_cal = 0x72
    PS_S1C_B_cal = 0x73
    PS_S1C_N_cal = 0x74
    PS_S1C_R_raw = 0x75
    PS_S1C_G_raw = 0x76
    PS_S1C_B_raw = 0x77
    PS_S1C_N_raw = 0x78

    # Sensor 2
    PS_S2_Mode = 0xA3
    # EV3
    PS_S2EV_Ready = 0xA4
    PS_S2EV_SensorID = 0xA5
    PS_S2EV_Mode = 0xB5
    PS_S2EV_Length = 0xB6
    PS_S2EV_Data = 0xB7
    # LEGO Analog
    PS_S2AN_Read = 0xA4
    # LEGO Color
    PS_S2C_Color = 0xA4
    PS_S2C_R_cal = 0xA5
    PS_S2C_G_cal = 0xA6
    PS_S2C_B_cal = 0xA7
    PS_S2C_N_cal = 0xA8
    PS_S2C_R_raw = 0xA9
    PS_S2C_G_raw = 0xAA
    PS_S2C_B_raw = 0xAB
    PS_S2C_N_raw = 0xAC

    ## Touchscreen x- and y-axis registers
    PS_TSX = 0xE3
    PS_TSY = 0xE5

    # Touchscreen Calibration
    PS_TS_CALIBRATION_DATA_READY = 0x70
    PS_TS_CALIBRATION_DATA = 0x71

    # LED
    PS_R = 0xD7
    PS_G = 0xD8
    PS_B = 0xD9

    # Buttons
    PS_KeyPress = 0xDA
    PS_Key1Count = 0xDB
    PS_Key2Count = 0xDC

    PS_CONTROL_SPEED = 0x01
    PS_CONTROL_RAMP = 0x02
    PS_CONTROL_RELATIVE = 0x04
    PS_CONTROL_TACHO = 0x08
    PS_CONTROL_BRK = 0x10
    PS_CONTROL_ON = 0x20
    PS_CONTROL_TIME = 0x40
    PS_CONTROL_GO = 0x80

    # Supported I2C commands
    R = 0x52
    S = 0x53
    a = 0x61
    b = 0x62
    c = 0x63
    A = 0x41
    B = 0x42
    C = 0x43
    H = 0x48
    E = 0x45
    t = 0x74
    T = 0x54
    w = 0x77
    l = 0x6C

    bankA = mindsensors_i2c(PS_A_ADDRESS >> 1)
    bankB = mindsensors_i2c(PS_B_ADDRESS >> 1)

    BAM1 = PSMotor(bankA, 1)
    BAM2 = PSMotor(bankA, 2)
    BBM1 = PSMotor(bankB, 1)
    BBM2 = PSMotor(bankB, 2)

    BAS1 = PSSensor(bankA, 1)
    BAS2 = PSSensor(bankA, 2)
    BBS1 = PSSensor(bankB, 1)
    BBS2 = PSSensor(bankB, 2)

    ## @param reset Reset both banks and clear the motors' tacho setpoints.
    #  Pass False from monitoring scripts (battery monitor, loggers) that may
    #  start while a robot program is running: the reset zeroes the encoders
    #  and stops the motors under that program's feet.
    def __init__(self, reset=True):
        try:
            self.bankA.readByte(self.PS_BattV)
        except:
            print ("Could not connect to PiStorms.")
        else:
            if not reset:
                return
            self.bankA.writeByte(self.PS_Command,self.R)
            self.bankB.writeByte(self.PS_Command,self.R)
            # Clear the firmware's stale tacho setpoint on all four motors at
            # boot — without this, the very first setSpeed after power-on can
            # produce a wrong-way twitch identical to the stop-then-reverse
            # case. See PSMotor._reset_setpoint_m1 comment for the mechanism.
            try:
                time.sleep(0.05)
                self.BAM1._reset_setpoint_m1()
                self.BAM1._reset_setpoint_m2()
                self.BBM1._reset_setpoint_m1()
                self.BBM1._reset_setpoint_m2()
            except Exception as e:
                print(f"Motor setpoint reset failed at init: {e}")

        config = configparser.RawConfigParser()
        config.read("/usr/local/mindsensors/conf/msdev.cfg")
        
        
    def Shutdown(self):
        self.bankA.writeByte(self.PS_Command,self.H)

    def command(self, cmd, bank):
        if(bank == 1):
            self.bankA.writeByte(self.PS_Command,cmd)
        elif(bank == 2):
            self.bankB.writeByte(self.PS_Command,cmd)

    ## Returns the battery voltage in volts, or None if it couldn't be read.
    #  Never returns 0 on a read error — callers treat low voltage as a reason
    #  to shut down.
    def battVoltage(self):
        try:
            return self.bankB.readByte(self.PS_BattV)*0.04
        except OSError:
            return None

    def GetFirmwareVersion(self):
        try:
            return self.bankB.readString(0x00, 8)
        except:
            return "ReadErr"

    def GetVendorName(self):
        try:
            return self.bankA.readString(0x08, 8)
        except:
            return "ReadErr"

    def GetDeviceId(self):
        try:
            return self.bankA.readString(0x10, 8)
        except:
            return "ReadErr"

    def GetDeviceFeatures(self):
        try:
            return self.bankA.readString(0x18, 8)
        except:
            return "ReadErr"

    def led(self, lednum, red, green, blue):
        array = [red, green, blue]
        if(lednum == 1):
            self.bankA.writeArray(self.PS_R, array)
        if(lednum == 2):
            self.bankB.writeArray(self.PS_R, array)
        time.sleep(0.001)

    def isKeyPressed(self):
        try:
            x = self.bankA.readByte(self.PS_KeyPress)
            return int(0x01&x)
        except:
            return 0

    def getKeyPressValue(self):
        try:
            x, y = self.getTouchscreenCoordinates()
            if x > 300 and y > 0:
                return [8, 16, 24, 40][int((y-1)/(240/4))]
            else:
                return 0
        except:
            return 0

    def getKeyPressCount(self):
        try:
            return self.bankA.readByte(self.PS_Key1Count)
        except:
            return 0

    def resetKeyPressCount(self):
        try:
            self.bankA.writeByte(self.PS_Key1Count, 0)
        except:
            pass

    def ping(self):
        self.bankA.readByte(0x00)

    def getTouchscreenCoordinates(self):
        sampleSize = 3
        tolerance = 3
        x = [0]*sampleSize
        y = [0]*sampleSize
        try:
            for i in range(sampleSize):
                x[i] = self.bankA.readInteger(self.PS_TSX)
                y[i] = self.bankA.readInteger(self.PS_TSY)
        except:
            print ("Failed to read touchscreen")
            return (0, 0)
        if (numpy.std(x) < tolerance and numpy.std(y) < tolerance):
            return (int(numpy.mean(x)), int(numpy.mean(y)))
        else:
            return (0, 0)

if __name__ == '__main__':
    psc = PiStormsCom()
    print ("Version = {0}".format(str(psc.GetFirmwareVersion())))
    print ("Vendor = {0}".format(str(psc.GetVendorName())))
    print ("Device = {0}".format(str(psc.GetDeviceId())))
    try:
        while(True):
            print ("{0}".format(psc.battVoltage()))
            print ("{0}".format(psc.BAS1.SumoEyes(True)))
            print ("{0}".format(psc.BAS2.colorSensorNXT()))
            print ("{0}".format(psc.BBS1.lightSensorNXT(True)))
            psc.BAM1.runSecs(1,100,True)
            psc.BBM1.runSecs(1,100,True)
            psc.BAM2.runSecs(1,100,True)
            psc.BBM2.runSecs(1,100,True)
            psc.BAM1.waitUntilNotBusy()

            psc.BAM1.runSecs(1,-100,True)
            psc.BBM1.runSecs(1,-100,True)
            psc.BAM2.runSecs(1,-100,True)
            psc.BBM2.runSecs(1,-100,True)
            psc.BAM1.waitUntilNotBusy()

            psc.BAM1.hold()
            psc.BAM2.hold()
            psc.BBM1.hold()
            psc.BBM2.hold()

            time.sleep(5)

            psc.BAM1.float()
            psc.BAM2.float()
            psc.BBM1.float()
            psc.BBM2.float()

            time.sleep(1)

    except KeyboardInterrupt:
        psc.BAM1.float()
        psc.BAM2.float()
        psc.BBM1.float()
        psc.BBM2.float()
