import time
import os as _os, sys as _sys
_sys_dir = _os.path.normpath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'sys'))
if _sys_dir not in _sys.path: _sys.path.insert(0, _sys_dir)

from mindsensors_i2c import mindsensors_i2c


class LineLeader(mindsensors_i2c):

    LL_ADDRESS    = 0x02
    LL_COMMAND    = 0x41
    LL_STEERING   = 0x42
    LL_AVERAGE    = 0x43
    LL_RESULT     = 0x44
    LL_SETPOINT   = 0x45
    LL_Kp         = 0x46
    LL_KI         = 0x47
    LL_KD         = 0x48
    LL_KPfactor   = 0x61
    LL_KIfactor   = 0x62
    LL_KDfactor   = 0x63
    LL_CALIBRATED = 0x49
    LL_UNCALIBRATED = 0x74

    def __init__(self, port, address=LL_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, command):
        self.writeByte(self.LL_COMMAND, int(command))

    def White_Cal(self):   self.command(87)
    def Black_Cal(self):   self.command(66)
    def Wakeup(self):      self.command(80)
    def Sleep(self):       self.command(68)

    def ReadRaw_Calibrated(self):
        try:
            return self.readArray(self.LL_CALIBRATED, 8)
        except:
            print("Error: Could not read Lineleader")
            return ""

    def ReadRaw_Uncalibrated(self):
        try:
            s1 = self.readInteger(self.LL_UNCALIBRATED)
            s2 = self.readInteger(self.LL_UNCALIBRATED + 2)
            s3 = self.readInteger(self.LL_UNCALIBRATED + 4)
            s4 = self.readInteger(self.LL_UNCALIBRATED + 6)
            s5 = self.readInteger(self.LL_UNCALIBRATED + 8)
            s6 = self.readInteger(self.LL_UNCALIBRATED + 10)
            s7 = self.readInteger(self.LL_UNCALIBRATED + 12)
            s8 = self.readInteger(self.LL_UNCALIBRATED + 14)
            return [s1, s2, s3, s4, s5, s6, s7, s8]
        except:
            print("Error: Could not read Lineleader")
            return ""

    def steering(self):
        try: return self.readByteSigned(self.LL_STEERING)
        except: print("Error: Could not read Lineleader"); return ""

    def average(self):
        try: return self.readByte(self.LL_AVERAGE)
        except: print("Error: Could not read Lineleader"); return ""

    def result(self):
        try: return self.readByte(self.LL_RESULT)
        except: print("Error: Could not read Lineleader"); return ""

    def getSetPoint(self):
        try: return self.readByte(self.LL_SETPOINT)
        except: print("Error: Could not read Lineleader"); return ""

    def setSetPoint(self):
        try: return self.writeByte(self.LL_SETPOINT)
        except: print("Error: Could not write to Lineleader"); return ""

    def setKP(self):
        try: return self.writeByte(self.LL_KP)
        except: print("Error: Could not write to Lineleader"); return ""

    def setKI(self):
        try: return self.writeByte(self.LL_KI)
        except: print("Error: Could not write to Lineleader"); return ""

    def setKD(self):
        try: return self.writeByte(self.LL_KD)
        except: print("Error: Could not write to Lineleader"); return ""

    def setKPfactor(self):
        try: return self.writeByte(self.LL_KPfactor)
        except: print("Error: Could not write to Lineleader"); return ""

    def setKIfactor(self):
        try: return self.writeByte(self.LL_KIfactor)
        except: print("Error: Could not write to Lineleader"); return ""

    def setKDfactor(self):
        try: return self.writeByte(self.LL_KDfactor)
        except: print("Error: Could not write to Lineleader"); return ""

    def getKP(self):
        try: return self.readByte(self.LL_KP)
        except: print("Error: Could not write to Lineleader"); return ""

    def getKI(self):
        try: return self.readByte(self.LL_KI)
        except: print("Error: Could not write to Lineleader"); return ""

    def getKD(self):
        try: return self.readByte(self.LL_KD)
        except: print("Error: Could not write to Lineleader"); return ""

    def getKPfactor(self):
        try: return self.readByte(self.LL_KPfactor)
        except: print("Error: Could not write to Lineleader"); return ""

    def getKIfactor(self):
        try: return self.readByte(self.LL_KIfactor)
        except: print("Error: Could not write to Lineleader"); return ""

    def getKDfactor(self):
        try: return self.readByte(self.LL_KDfactor)
        except: print("Error: Could not write to Lineleader"); return ""


class LightSensorArray(mindsensors_i2c):

    LSA_ADDRESS     = 0x14
    LSA_COMMAND     = 0x41
    LSA_CALIBRATED  = 0x42
    LSA_UNCALIBRATED = 0x6A

    def __init__(self, port, address=LSA_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, cmd):   self.writeByte(self.LSA_COMMAND, int(cmd))
    def White_Cal(self):      self.command(87)
    def Black_Cal(self):      self.command(66)
    def Wakeup(self):         self.command(80)
    def Sleep(self):          self.command(68)

    def ReadRaw_Calibrated(self):
        try: return self.readArray(self.LSA_CALIBRATED, 8)
        except: print("Error: Could not read LSArray"); return ""

    def ReadRaw_Uncalibrated(self):
        try:
            return [self.readInteger(self.LSA_UNCALIBRATED + i * 2) for i in range(8)]
        except:
            print("Error: Could not read LSArray")
            return ""


class SumoEyes(mindsensors_i2c):

    SE_None   = [0, "None"]
    SE_Values = {465: [1, "Front"], 555: [3, "Right"], 800: [2, "Left"]}

    PS_S1EV_Ready = 0x70
    PS_S2EV_Ready = 0xA4

    LONG_RANGE  = True
    SHORT_RANGE = False

    def __init__(self, port):
        self.sensor = port.pssensor
        mindsensors_i2c.__init__(self, self.sensor.bank.address)
        self.setRange()
        self.sensor.setModeEV3(0)

    def detectObstactleZone(self, verbose=False):
        reading = self.readSensorValue()
        for reference in self.SE_Values.keys():
            if self.isNear(reference, reading):
                output = self.SE_Values[reference]
                return output[1] if verbose else output[0]
        return self.SE_None[1] if verbose else self.SE_None[0]

    def readSensorValue(self):
        return self.readInteger(self.PS_S1EV_Ready if self.sensor.sensornum == 1 else self.PS_S2EV_Ready)

    def setRange(self, range=LONG_RANGE):
        if range == self.LONG_RANGE:
            self.sensor.setType(self.sensor.PS_SENSOR_TYPE_LIGHT_INACTIVE)
        elif range == self.SHORT_RANGE:
            self.sensor.setType(self.sensor.PS_SENSOR_TYPE_LIGHT_ACTIVE)

    def isNear(self, reference, value, tolerance=40):
        return (value > (reference - tolerance)) and (value < (reference + tolerance))


class IRThermometer(mindsensors_i2c):

    IRT_ADDRESS         = 0x2A
    IRT_COMMAND         = 0x41
    IRT_AMBIENT_CELSIUS = 0x42
    IRT_TARGET_CELSIUS  = 0x44
    IRT_AMBIENT_FAHR    = 0x46
    IRT_TARGET_FAHR     = 0x48

    def __init__(self, port, address=IRT_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def readAmbientCelsius(self): return float(self.readInteger(self.IRT_AMBIENT_CELSIUS)) / 100
    def readTargetCelsius(self):  return float(self.readInteger(self.IRT_TARGET_CELSIUS)) / 100
    def readAmbientFahr(self):    return float(self.readInteger(self.IRT_AMBIENT_FAHR)) / 100
    def readTargetFahr(self):     return float(self.readInteger(self.IRT_TARGET_FAHR)) / 100


class DISTNx(mindsensors_i2c):

    DIST_ADDRESS = (0x02)
    COMMAND  = 0x41
    DISTANCE = 0x42
    VOLTAGE  = 0x44

    def __init__(self, port, address=DIST_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, command): self.writeByte(self.COMMAND, int(command))

    def get_distance(self):
        try: return self.readInteger(self.DISTANCE)
        except: print("Error: Could not read distance"); return ""

    def get_distance_inches(self):
        try: return self.get_distance() / 25
        except: print("Error: Could not read distance"); return ""

    def get_voltage(self):
        try: return self.readInteger(self.VOLTAGE)
        except: print("Error: Could not read voltage"); return ""


class AngleSensor(mindsensors_i2c):

    ANGLE_ADDRESS = (0x30)
    COMMAND = 0x41
    ANGLE   = 0x42
    RAW     = 0x46
    RPM     = 0x4A

    def __init__(self, port, address=ANGLE_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, command): self.writeByte(self.COMMAND, int(command))

    def get_angle(self):
        try: return self.readLongSigned(self.ANGLE)
        except: print("Error: Could not read angle"); return ""

    def get_raw(self):
        try: return self.readLongSigned(self.RAW)
        except: print("Error: Could not read raw angle"); return ""

    def get_rpm(self):
        try: return self.readIntegerSigned(self.RPM)
        except: print("Error: Could not read rpm"); return ""


class CurrentMeter(mindsensors_i2c):

    CURRENT_ADDRESS = (0x28)
    COMMAND = 0x41
    CAL     = 0x43
    REL     = 0x45
    REF     = 0x47

    def __init__(self, port, address=CURRENT_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, cmd): self.writeByte(self.COMMAND, int(cmd))

    def get_calibrated(self):
        try: return self.readIntegerSigned(self.CAL)
        except: print("Error: Could not read calibrated current"); return ""

    def get_relative(self):
        try: return self.readIntegerSigned(self.REL)
        except: print("Error: Could not read relative current"); return ""

    def get_reference(self):
        try: return self.readIntegerSigned(self.REF)
        except: print("Error: Could not read reference current"); return ""

    def set_reference(self):
        try: self.command(68)
        except: print("Error: Could not set reference current"); return ""


class VoltMeter(mindsensors_i2c):

    VOLT_ADDRESS = (0x26)
    COMMAND = 0x41
    CAL     = 0x43
    REL     = 0x45
    REF     = 0x47

    def __init__(self, port, address=VOLT_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, cmd): self.writeByte(self.COMMAND, int(cmd))

    def get_calibrated(self):
        try: return self.readIntegerSigned(self.CAL)
        except: print("Error: Could not read calibrated voltage"); return ""

    def get_relative(self):
        try: return self.readIntegerSigned(self.REL)
        except: print("Error: Could not read relative voltage"); return ""

    def get_reference(self):
        try: return self.readIntegerSigned(self.REF)
        except: print("Error: Could not read reference voltage"); return ""

    def set_reference(self):
        try: self.command(68)
        except: print("Error: Could not set reference voltage"); return ""


class PressureSensor(mindsensors_i2c):

    PPS58_ADDRESS = (0x18)
    COMMAND = 0x41
    UNIT    = 0x42
    ABS     = 0x43
    GAUGE   = 0x45
    REF     = 0x47
    RAW     = 0x53

    def __init__(self, port, address=PPS58_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, cmd): self.writeByte(self.COMMAND, int(cmd))

    def get_unit(self):
        try: return self.readByte(self.UNIT)
        except: print("Error: Could not read unit value"); return ""

    def get_absolute(self):
        try: return self.readIntegerSigned(self.ABS)
        except: print("Error: Could not read absolute pressure"); return ""

    def get_gauge(self):
        try: return self.readIntegerSigned(self.GAUGE)
        except: print("Error: Could not read gauge pressure"); return ""

    def get_reference(self):
        try: return self.readIntegerSigned(self.REF)
        except: print("Error: Could not read reference pressure"); return ""

    def get_raw(self):
        try: return self.readLongSigned(self.RAW)
        except: print("Error: Could not read raw pressure"); return ""

    def set_reference(self):
        try: self.command(68)
        except: print("Error: Could not set reference pressure"); return ""

    def set_unit_PSI(self):
        try: self.command(80)
        except: print("Error: Could not set PSI as unit"); return ""

    def set_unit_mbar(self):
        try: self.command(98)
        except: print("Error: Could not set millibar as unit"); return ""

    def set_unit_kpascal(self):
        try: self.command(107)
        except: print("Error: Could not set kilopascal as unit"); return ""


class EV3SensAdapt(mindsensors_i2c):
    """EV3 Sensor Adapter / Sensor Mux. Pass channel address (0xA0/0xA2/0xA4) for mux channels."""

    EV3SensAdapt_ADDRESS = (0x32)
    SETMODE = 0x52
    DATA1   = 0x54
    DATA2   = 0x55
    DATA3   = 0x56
    DATA4   = 0x57
    DATA5   = 0x58
    DATA6   = 0x59
    DATA7   = 0x5A
    DATA8   = 0x5B

    def __init__(self, port, address=EV3SensAdapt_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def setMode(self, mode): self.writeByte(self.SETMODE, mode)
    def getMode(self):       return self.readByte(self.SETMODE)

    def isTouchedEV3(self):
        try: self.setMode(15); return self.readByte(self.DATA1) == 1
        except: print("Error: Could not read EV3 Touch Sensor"); return ""

    def numTouchesEV3(self):
        try: self.setMode(15); return self.readByte(self.DATA2)
        except: print("Error: Could not read EV3 Touch Sensor"); return ""

    def resetTouchesEV3(self):
        try: self.setMode(15); return self.writeByte(self.DATA2, 0)
        except: print("Error: Could not reset EV3 Touch Sensor"); return ""

    def distanceIREV3(self):
        try: self.setMode(0); return self.readByte(self.DATA1)
        except: print("Error: Could not read EV3 IR Sensor"); return ""

    def headingIREV3(self, channel):
        try: self.setMode(1); return self.readByteSigned(self.DATA1 + (channel - 1) * 2)
        except: print("Error: Could not read EV3 IR Sensor"); return ""

    def distanceRemoteIREV3(self, channel):
        try: self.setMode(1); return self.readByte(self.DATA1 + (channel - 1) * 2 + 1)
        except: print("Error: Could not read EV3 IR Sensor"); return ""

    def remoteLeft(self, channel):
        try:
            self.setMode(2)
            r = self.readByte(self.DATA1 + channel - 1)
            if r in (0, 3, 4): return  0
            if r in (1, 5, 6): return  1
            if r in (2, 7, 8): return -1
        except: print("Error: Could not read EV3 IR Sensor"); return ""

    def remoteRight(self, channel):
        try:
            self.setMode(2)
            r = self.readByte(self.DATA1 + channel - 1)
            if r in (0, 1, 2): return  0
            if r in (3, 7, 5): return  1
            if r in (4, 6, 8): return -1
        except: print("Error: Could not read EV3 IR Sensor"); return ""

    def distanceUSEV3(self):
        try: self.setMode(0); return self.readInteger(self.DATA1)
        except: print("Error: Could not read EV3 Ultrasonic"); return ""

    def distanceUSEV3in(self):
        try: self.setMode(1); return self.readInteger(self.DATA1)
        except: print("Error: Could not read EV3 Ultrasonic"); return ""

    def presenceUSEV3(self):
        try: self.setMode(2); return self.readByte(self.DATA1) == 1
        except: print("Error: Could not read EV3 Ultrasonic"); return ""

    def gyroAngleEV3(self):
        try: self.setMode(0); return self.readIntegerSigned(self.DATA1)
        except: print("Error: Could not read EV3 Gyro"); return ""

    def gyroRateEV3(self):
        try: self.setMode(1); return self.readIntegerSigned(self.DATA1)
        except: print("Error: Could not read EV3 Gyro"); return ""

    def reflectedLightSensorEV3(self):
        try: self.setMode(0); return self.readInteger(self.DATA1)
        except: print("Error: Could not read EV3 Color Sensor"); return ""

    def ambientLightSensorEV3(self):
        try: self.setMode(1); return self.readInteger(self.DATA1)
        except: print("Error: Could not read EV3 Color Sensor"); return ""

    def colorSensorEV3(self):
        try: self.setMode(2); return self.readInteger(self.DATA1)
        except: print("Error: Could not read EV3 Color Sensor"); return ""
