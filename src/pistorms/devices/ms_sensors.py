from ..sensor_port import SensorPort
from .lego_sensors import remote_left_direction, remote_right_direction
from .ms_device import MindsensorsI2CDevice

# All the I2C sensors below raise OSError when the bus fails, like the rest
# of the library (they used to print a message and return "").


class _LineSensorArray(MindsensorsI2CDevice):
    """Common part of LineLeader and LightSensorArray: 8 light sensors in a
    row, calibrated on a white and a black surface."""

    CALIBRATED = None     # register of the 8 calibrated readings (1 byte each)
    UNCALIBRATED = None   # register of the 8 raw readings (2 bytes each)

    def White_Cal(self):   self.command(87)   # 'W'
    def Black_Cal(self):   self.command(66)   # 'B'
    def Wakeup(self):      self.command(80)   # 'P'
    def Sleep(self):       self.command(68)   # 'D'

    def ReadRaw_Calibrated(self):
        return self.readArray(self.CALIBRATED, 8)

    def ReadRaw_Uncalibrated(self):
        readings = []
        for sensor in range(8):
            readings.append(self.readInteger(self.UNCALIBRATED + sensor * 2))
        return readings


class LineLeader(_LineSensorArray):

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

    # Same default address as DISTNx (0x02): two of them on the same PiStorms
    # bank need different addresses.
    DEFAULT_ADDRESS = LL_ADDRESS
    CALIBRATED = LL_CALIBRATED
    UNCALIBRATED = LL_UNCALIBRATED

    def steering(self):     return self.readByteSigned(self.LL_STEERING)
    def average(self):      return self.readByte(self.LL_AVERAGE)
    def result(self):       return self.readByte(self.LL_RESULT)

    def getSetPoint(self):  return self.readByte(self.LL_SETPOINT)
    def getKP(self):        return self.readByte(self.LL_Kp)
    def getKI(self):        return self.readByte(self.LL_KI)
    def getKD(self):        return self.readByte(self.LL_KD)
    def getKPfactor(self):  return self.readByte(self.LL_KPfactor)
    def getKIfactor(self):  return self.readByte(self.LL_KIfactor)
    def getKDfactor(self):  return self.readByte(self.LL_KDfactor)

    def setSetPoint(self, value):  self.writeByte(self.LL_SETPOINT, value)
    def setKP(self, value):        self.writeByte(self.LL_Kp, value)
    def setKI(self, value):        self.writeByte(self.LL_KI, value)
    def setKD(self, value):        self.writeByte(self.LL_KD, value)
    def setKPfactor(self, value):  self.writeByte(self.LL_KPfactor, value)
    def setKIfactor(self, value):  self.writeByte(self.LL_KIfactor, value)
    def setKDfactor(self, value):  self.writeByte(self.LL_KDfactor, value)


class LightSensorArray(_LineSensorArray):

    LSA_ADDRESS      = 0x14
    LSA_COMMAND      = 0x41
    LSA_CALIBRATED   = 0x42
    LSA_UNCALIBRATED = 0x6A

    DEFAULT_ADDRESS = LSA_ADDRESS
    CALIBRATED = LSA_CALIBRATED
    UNCALIBRATED = LSA_UNCALIBRATED


class SumoEyes:
    """mindsensors SumoEyes: obstacle zone in front of the robot. Not an I2C
    device: it is an analog sensor read through the PiStorms port, and the
    zone is coded in the value."""

    LONG_RANGE  = True
    SHORT_RANGE = False

    ZONE_NONE, ZONE_FRONT, ZONE_LEFT, ZONE_RIGHT = 0, 1, 2, 3
    ZONE_NAMES = {0: "None", 1: "Front", 2: "Left", 3: "Right"}
    # Analog value of each zone, ± TOLERANCE
    ZONE_VALUES = {ZONE_FRONT: 465, ZONE_LEFT: 800, ZONE_RIGHT: 555}
    TOLERANCE = 30

    def __init__(self, port: SensorPort):
        self.port = port     # e.g. psm.BAS1
        self.setRange(self.LONG_RANGE)

    def setRange(self, range=LONG_RANGE):
        # Long range = light off (ambient type), short range = light on
        if range == self.LONG_RANGE:
            self.port.setType(SensorPort.PS_SENSOR_TYPE_LIGHT_INACTIVE)
        else:
            self.port.setType(SensorPort.PS_SENSOR_TYPE_LIGHT_ACTIVE)

    def readSensorValue(self):
        return self.port.readInteger(self.port.ANALOG_DATA)

    def detectObstactleZone(self, verbose=False):
        """ZONE_NONE, ZONE_FRONT, ZONE_LEFT or ZONE_RIGHT — or its name if verbose."""
        value = self.readSensorValue()
        zone = self.ZONE_NONE
        for candidate_zone, zone_value in self.ZONE_VALUES.items():
            if abs(value - zone_value) < self.TOLERANCE:
                zone = candidate_zone
        if verbose:
            return self.ZONE_NAMES[zone]
        return zone


class IRThermometer(MindsensorsI2CDevice):

    IRT_ADDRESS         = 0x2A
    IRT_COMMAND         = 0x41
    IRT_AMBIENT_CELSIUS = 0x42
    IRT_TARGET_CELSIUS  = 0x44
    IRT_AMBIENT_FAHR    = 0x46
    IRT_TARGET_FAHR     = 0x48

    DEFAULT_ADDRESS = IRT_ADDRESS

    # Temperatures are stored in hundredths of a degree
    def readAmbientCelsius(self): return self.readInteger(self.IRT_AMBIENT_CELSIUS) / 100
    def readTargetCelsius(self):  return self.readInteger(self.IRT_TARGET_CELSIUS) / 100
    def readAmbientFahr(self):    return self.readInteger(self.IRT_AMBIENT_FAHR) / 100
    def readTargetFahr(self):     return self.readInteger(self.IRT_TARGET_FAHR) / 100


class DISTNx(MindsensorsI2CDevice):

    DIST_ADDRESS = 0x02
    DISTANCE = 0x42
    VOLTAGE  = 0x44

    # Same default address as LineLeader (0x02)
    DEFAULT_ADDRESS = DIST_ADDRESS

    def get_distance(self):
        """Distance in millimeters."""
        return self.readInteger(self.DISTANCE)

    def get_distance_inches(self):
        return self.get_distance() / 25.4

    def get_voltage(self):
        return self.readInteger(self.VOLTAGE)


class AngleSensor(MindsensorsI2CDevice):

    ANGLE_ADDRESS = 0x30
    ANGLE   = 0x42
    RAW     = 0x46
    RPM     = 0x4A

    DEFAULT_ADDRESS = ANGLE_ADDRESS

    def get_angle(self): return self.readLongSigned(self.ANGLE)
    def get_raw(self):   return self.readLongSigned(self.RAW)
    def get_rpm(self):   return self.readIntegerSigned(self.RPM)


class _ReferenceMeter(MindsensorsI2CDevice):
    """Common part of CurrentMeter and VoltMeter: a calibrated reading, a
    reading relative to a stored reference, and the reference itself."""

    CAL = 0x43
    REL = 0x45
    REF = 0x47

    def get_calibrated(self): return self.readIntegerSigned(self.CAL)
    def get_relative(self):   return self.readIntegerSigned(self.REL)
    def get_reference(self):  return self.readIntegerSigned(self.REF)

    def set_reference(self):
        """Store the present reading as the reference."""
        self.command(68)   # 'D'


class CurrentMeter(_ReferenceMeter):
    CURRENT_ADDRESS = 0x28
    DEFAULT_ADDRESS = CURRENT_ADDRESS


class VoltMeter(_ReferenceMeter):
    VOLT_ADDRESS = 0x26
    DEFAULT_ADDRESS = VOLT_ADDRESS


class PressureSensor(MindsensorsI2CDevice):

    PPS58_ADDRESS = 0x18
    UNIT    = 0x42
    ABS     = 0x43
    GAUGE   = 0x45
    REF     = 0x47
    RAW     = 0x53

    DEFAULT_ADDRESS = PPS58_ADDRESS

    def get_unit(self):      return self.readByte(self.UNIT)
    def get_absolute(self):  return self.readIntegerSigned(self.ABS)
    def get_gauge(self):     return self.readIntegerSigned(self.GAUGE)
    def get_reference(self): return self.readIntegerSigned(self.REF)
    def get_raw(self):       return self.readLongSigned(self.RAW)

    def set_reference(self):    self.command(68)    # 'D'
    def set_unit_PSI(self):     self.command(80)    # 'P'
    def set_unit_mbar(self):    self.command(98)    # 'b'
    def set_unit_kpascal(self): self.command(107)   # 'k'


class EV3SensAdapt(MindsensorsI2CDevice):
    """EV3 Sensor Adapter / Sensor Mux. Pass channel address (0xA0/0xA2/0xA4) for mux channels."""

    EV3SensAdapt_ADDRESS = 0x32
    SETMODE = 0x52
    DATA1   = 0x54
    DATA2   = 0x55
    DATA3   = 0x56
    DATA4   = 0x57
    DATA5   = 0x58
    DATA6   = 0x59
    DATA7   = 0x5A
    DATA8   = 0x5B

    DEFAULT_ADDRESS = EV3SensAdapt_ADDRESS
    TOUCH_MODE = 15

    def setMode(self, mode): self.writeByte(self.SETMODE, mode)
    def getMode(self):       return self.readByte(self.SETMODE)

    # Touch sensor
    def isTouchedEV3(self):
        self.setMode(self.TOUCH_MODE)
        return self.readByte(self.DATA1) == 1

    def numTouchesEV3(self):
        self.setMode(self.TOUCH_MODE)
        return self.readByte(self.DATA2)

    def resetTouchesEV3(self):
        self.setMode(self.TOUCH_MODE)
        self.writeByte(self.DATA2, 0)

    # Infrared sensor
    def distanceIREV3(self):
        self.setMode(0)
        return self.readByte(self.DATA1)

    def headingIREV3(self, channel):
        self.setMode(1)
        channel_offset = (channel - 1) * 2       # 2 bytes per channel: heading, distance
        return self.readByteSigned(self.DATA1 + channel_offset)

    def distanceRemoteIREV3(self, channel):
        self.setMode(1)
        channel_offset = (channel - 1) * 2
        return self.readByte(self.DATA1 + channel_offset + 1)

    def _readRemote(self, channel):
        self.setMode(2)
        return self.readByte(self.DATA1 + channel - 1)

    def remoteLeft(self, channel):
        return remote_left_direction(self._readRemote(channel))

    def remoteRight(self, channel):
        return remote_right_direction(self._readRemote(channel))

    # Ultrasonic sensor
    def distanceUSEV3(self):
        self.setMode(0)
        return self.readInteger(self.DATA1)

    def distanceUSEV3in(self):
        self.setMode(1)
        return self.readInteger(self.DATA1)

    def presenceUSEV3(self):
        self.setMode(2)
        return self.readByte(self.DATA1) == 1

    # Gyro sensor
    def gyroAngleEV3(self):
        self.setMode(0)
        return self.readIntegerSigned(self.DATA1)

    def gyroRateEV3(self):
        self.setMode(1)
        return self.readIntegerSigned(self.DATA1)

    # Color sensor
    def reflectedLightSensorEV3(self):
        self.setMode(0)
        return self.readInteger(self.DATA1)

    def ambientLightSensorEV3(self):
        self.setMode(1)
        return self.readInteger(self.DATA1)

    def colorSensorEV3(self):
        self.setMode(2)
        return self.readInteger(self.DATA1)
