import time

from ..mindsensors_i2c import mindsensors_i2c
from ..sensor_port import SensorPort


class MindsensorsI2CDevice(mindsensors_i2c):
    """Base of every mindsensors I2C device plugged into a PiStorms sensor
    port (sensors, actuators, cameras, IMU).

    - `port` is the PiStorms sensor port it is plugged into (e.g. psm.BAS1).
    - The port must be in I2C ("custom") mode, but the PiStorms board puts an
      idle port back to NONE after about 1 s. Every read or write therefore
      re-activates the port first when it has been idle for a while.
    - Every mindsensors device has its command register at 0x41.
    - Bus errors are raised as OSError, like the rest of the library.

    A subclass only sets DEFAULT_ADDRESS (8-bit address, as printed in the
    mindsensors documentation) and its registers."""

    DEFAULT_ADDRESS = None
    COMMAND = 0x41
    PORT_IDLE_LIMIT = 0.5   # s; re-activate after this long without traffic (board resets at ~1 s)
    PORT_SETTLE_TIME = 0.05 # s; time a device needs to answer after re-activation

    def __init__(self, port: SensorPort, address=None):
        if address is None:
            address = self.DEFAULT_ADDRESS
        self.port = port
        port.activateCustomSensorI2C()
        self._last_access = time.monotonic()
        mindsensors_i2c.__init__(self, address >> 1)

    def _keepPortActive(self):
        now = time.monotonic()
        idle_time = now - self._last_access
        if idle_time > self.PORT_IDLE_LIMIT:
            self.port.activateCustomSensorI2C()
            time.sleep(self.PORT_SETTLE_TIME)
        self._last_access = time.monotonic()

    # The four I2C primitives: every other read (readInteger, readLongSigned,
    # readString, ...) goes through one of them.
    def readByte(self, reg):
        self._keepPortActive()
        return mindsensors_i2c.readByte(self, reg)

    def readArray(self, reg, length):
        self._keepPortActive()
        return mindsensors_i2c.readArray(self, reg, length)

    def writeByte(self, reg, value):
        self._keepPortActive()
        mindsensors_i2c.writeByte(self, reg, value)

    def writeArray(self, reg, arr):
        self._keepPortActive()
        mindsensors_i2c.writeArray(self, reg, arr)

    def command(self, command):
        self.writeByte(self.COMMAND, int(command))
