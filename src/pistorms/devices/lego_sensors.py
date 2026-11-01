"""LEGO NXT and EV3 sensors plugged into a PiStorms sensor port.

Each class takes the port it is plugged into:

    from pistorms import PiStorms
    from pistorms.devices import EV3GyroSensor
    psm = PiStorms()
    gyro = EV3GyroSensor(psm.BAS1)
    print(gyro.angle())

The PiStorms board talks to the sensor; these classes only select the port
type and the EV3 mode, and read the port registers (see the PiStorms
Advanced Development Guide). Bus errors raise OSError.
"""
from ..sensor_port import SensorPort

# Color numbers returned by the EV3 and NXT color sensors
COLOR_NONE   = 0
COLOR_BLACK  = 1
COLOR_BLUE   = 2
COLOR_GREEN  = 3
COLOR_YELLOW = 4
COLOR_RED    = 5
COLOR_WHITE  = 6
COLOR_BROWN  = 7


# EV3 IR remote button codes (one byte per channel):
#   0 none, 1 red up, 2 red down, 3 blue up, 4 blue down,
#   5 red up + blue up, 6 red up + blue down, 7 red down + blue up,
#   8 red down + blue down, 9 beacon, 10 red up + down, 11 blue up + down.
# Left = red buttons, right = blue buttons; 1 = forward (up), -1 = backward
# (down), 0 = not pressed — also for 9 to 11, where no single direction applies.
def remote_left_direction(remote_code):
    if remote_code in [1, 5, 6]:
        return 1
    if remote_code in [2, 7, 8]:
        return -1
    return 0


def remote_right_direction(remote_code):
    if remote_code in [3, 5, 7]:
        return 1
    if remote_code in [4, 6, 8]:
        return -1
    return 0


class LegoSensor:
    """Base of the LEGO sensors: sets the port type once, and the EV3 mode
    only when it changes (each mode change is an I2C write)."""

    SENSOR_TYPE = None    # SensorPort.PS_SENSOR_TYPE_..., set by each sensor

    def __init__(self, port: SensorPort):
        self.port = port          # e.g. psm.BAS1
        self._mode = None
        port.setType(self.SENSOR_TYPE)    # the board needs 1 s after a type change

    def _useMode(self, mode):
        if mode != self._mode:
            self.port.setModeEV3(mode)
            self._mode = mode


# ── Touch sensors ───────────────────────────────────────────────────────────

class _TouchSensor(LegoSensor):
    """The board counts the presses since power-on (or the last reset)."""

    def is_pressed(self):
        return self.port.readByte(self.port.EV3_DATA) == 1

    def bump_count(self):
        return self.port.readByte(self.port.EV3_DATA + 1)

    def reset_bump_count(self):
        self.port.writeByte(self.port.EV3_DATA + 1, 0)


class EV3TouchSensor(_TouchSensor):
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_EV3_SWITCH


class NXTTouchSensor(_TouchSensor):
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_SWITCH


# ── EV3 sensors ─────────────────────────────────────────────────────────────

class EV3ColorSensor(LegoSensor):
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_EV3
    MODE_REFLECTED = 0
    MODE_AMBIENT   = 1
    MODE_COLOR     = 2

    def reflected(self):
        """Reflected red light, 0–100 %."""
        self._useMode(self.MODE_REFLECTED)
        return self.port.readByte(self.port.EV3_DATA)

    def ambient(self):
        """Ambient light, 0–100 %."""
        self._useMode(self.MODE_AMBIENT)
        return self.port.readByte(self.port.EV3_DATA)

    def color(self):
        """Color number, COLOR_NONE … COLOR_BROWN."""
        self._useMode(self.MODE_COLOR)
        return self.port.readByte(self.port.EV3_DATA)


class EV3GyroSensor(LegoSensor):
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_EV3
    MODE_ANGLE = 0
    MODE_RATE  = 1

    def angle(self):
        """Angle in degrees since the sensor was powered or put in angle mode."""
        self._useMode(self.MODE_ANGLE)
        return self.port.readIntegerSigned(self.port.EV3_DATA)

    def rate(self):
        """Rotation speed in degrees per second."""
        self._useMode(self.MODE_RATE)
        return self.port.readIntegerSigned(self.port.EV3_DATA)


class EV3UltrasonicSensor(LegoSensor):
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_EV3
    MODE_DISTANCE_CM     = 0
    MODE_DISTANCE_INCHES = 1
    MODE_PRESENCE        = 2

    def distance_cm(self):
        # The sensor gives tenths of a centimeter (millimeters)
        self._useMode(self.MODE_DISTANCE_CM)
        return self.port.readInteger(self.port.EV3_DATA) / 10

    def distance_inches(self):
        # The sensor gives tenths of an inch
        self._useMode(self.MODE_DISTANCE_INCHES)
        return self.port.readInteger(self.port.EV3_DATA) / 10

    def presence(self):
        """True if another ultrasonic sensor is emitting nearby."""
        self._useMode(self.MODE_PRESENCE)
        return self.port.readByte(self.port.EV3_DATA) == 1


class EV3InfraredSensor(LegoSensor):
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_EV3
    MODE_PROXIMITY = 0
    MODE_BEACON    = 1
    MODE_REMOTE    = 2

    def proximity(self):
        """Distance to an obstacle, 0–100 (roughly 0–70 cm)."""
        self._useMode(self.MODE_PROXIMITY)
        return self.port.readInteger(self.port.EV3_DATA)

    # The beacon data has 2 bytes per channel (1 to 4): heading, distance
    def beacon_heading(self, channel):
        """Direction of the beacon on `channel`, -25 (left) … 25 (right)."""
        self._useMode(self.MODE_BEACON)
        channel_offset = (channel - 1) * 2
        return self.port.readByteSigned(self.port.EV3_DATA + channel_offset)

    def beacon_distance(self, channel):
        """Distance of the beacon on `channel`, 0–100."""
        self._useMode(self.MODE_BEACON)
        channel_offset = (channel - 1) * 2
        return self.port.readByte(self.port.EV3_DATA + channel_offset + 1)

    def _remote_code(self, channel):
        self._useMode(self.MODE_REMOTE)
        return self.port.readByte(self.port.EV3_DATA + channel - 1)

    def remote_left(self, channel):
        """Red buttons of the remote on `channel`: 1 up, -1 down, 0 none."""
        return remote_left_direction(self._remote_code(channel))

    def remote_right(self, channel):
        """Blue buttons of the remote on `channel`: 1 up, -1 down, 0 none."""
        return remote_right_direction(self._remote_code(channel))


# ── NXT sensors ─────────────────────────────────────────────────────────────

class AnalogSensor(LegoSensor):
    """Any NXT analog sensor: raw value 0–1023."""
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_ANALOG

    def value(self):
        return self.port.readInteger(self.port.ANALOG_DATA)


class NXTLightSensor(AnalogSensor):
    """NXT light sensor; reflected=True turns its red LED on."""

    def __init__(self, port, reflected=True):
        if reflected:
            self.SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_LIGHT_ACTIVE
        else:
            self.SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_LIGHT_INACTIVE
        LegoSensor.__init__(self, port)


class NXTColorSensor(LegoSensor):
    """NXT color sensor in full color mode. Registers from the guide
    (not tested on this robot)."""
    SENSOR_TYPE = SensorPort.PS_SENSOR_TYPE_COLORFULL

    def color(self):
        """Color number, COLOR_NONE … COLOR_BROWN."""
        return self.port.readByte(self.port.ANALOG_DATA)

    def calibrated(self):
        """(red, green, blue, no light) calibrated values."""
        return tuple(self.port.readArray(self.port.ANALOG_DATA + 1, 4))

    def raw(self):
        """(red, green, blue, no light) raw values."""
        return tuple(self.port.readArray(self.port.ANALOG_DATA + 5, 4))
