import math
import struct
import time
import os as _os, sys as _sys
_sys_dir = _os.path.normpath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'sys'))
if _sys_dir not in _sys.path: _sys.path.insert(0, _sys_dir)

from mindsensors_i2c import mindsensors_i2c


class AbsoluteIMU(mindsensors_i2c):

    ABSIMU_ADDRESS = (0x22)
    COMMAND  = 0x41
    TILT_X   = 0x42
    TILT_Y   = 0x43
    TILT_Z   = 0x44
    ACCEL_X  = 0x45
    ACCEL_Y  = 0x47
    ACCEL_Z  = 0x49
    CMPS     = 0x4B
    MAG_X    = 0x4D
    MAG_Y    = 0x4F
    MAG_Z    = 0x51
    GYRO_X   = 0x53
    GYRO_Y   = 0x55
    GYRO_Z   = 0x57
    GYRO_FILTER = 0x5A

    gyro_unit = 8.75

    def __init__(self, port, address=ABSIMU_ADDRESS):
        self._port = port
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, command):
        self.writeByte(self.COMMAND, int(command))

    def get_acc_mag_gyr(self):
        """Single 20-byte block read from ACCEL_X — returns (acc m/s², mag nT, gyr deg/s).
        Re-activates the sensor port before reading: PiStorms silently reverts the port to
        NONE after ~1s idle, and the IMU needs a few ms to respond after re-arming, so we
        retry once with a settle delay. Returns (None,None,None) if all attempts fail."""
        for attempt in range(3):
            self._port.activateCustomSensorI2C()
            if attempt > 0:
                time.sleep(0.05)
            try:
                raw = self.bus.read_i2c_block_data(self.address, self.ACCEL_X, 20)
                break
            except Exception as e:
                last_err = e
        else:
            print(f"get_acc_mag_gyr I2C error: {last_err}")
            return None, None, None
        acc_raw  = struct.unpack("<3h", bytes(raw[0:6]))    # ACCEL_X..ACCEL_Z
        mag_raw  = struct.unpack("<3h", bytes(raw[8:14]))   # MAG_X..MAG_Z   (skip CMPS 2 bytes)
        gyro_raw = struct.unpack("<3h", bytes(raw[14:20]))  # GYRO_X..GYRO_Z
        return self._convert_acc(acc_raw), self._convert_mag(mag_raw), self._convert_gyro(gyro_raw)

    def get_all_values(self):
        """Single 23-byte block read from TILT_X. Returns (tilt, compass, gyr deg/s, acc m/s², mag nT), or (None,...) on error."""
        try:
            raw = self.bus.read_i2c_block_data(self.address, self.TILT_X, 23)
        except Exception:
            return None, None, None, None, None
        result = struct.unpack("<3b10h", bytes(raw))
        return result[0:3], result[6], self._convert_gyro(result[10:13]), self._convert_acc(result[3:6]), self._convert_mag(result[7:10])

    def get_tiltx(self):
        try:
            t = self.readByteSigned(self.TILT_X)
            return math.degrees(math.asin(max(-1.0, min(1.0, t / 128.0))))
        except Exception:
            return None

    def get_tilty(self):
        try:
            t = self.readByteSigned(self.TILT_Y)
            return math.degrees(math.asin(max(-1.0, min(1.0, t / 128.0))))
        except Exception:
            return None

    def get_tiltz(self):
        try:
            t = self.readByteSigned(self.TILT_Z)
            return math.degrees(math.asin(max(-1.0, min(1.0, t / 128.0))))
        except Exception:
            return None

    def get_tiltall(self):
        """Returns (x, y, z) tilt in degrees via single block read, or None on error."""
        try:
            raw = self.bus.read_i2c_block_data(self.address, self.TILT_X, 3)
            vals = struct.unpack("<3b", bytes(raw))
            return tuple(math.degrees(math.asin(max(-1.0, min(1.0, v / 128.0)))) for v in vals)
        except Exception:
            return None

    def get_accelx(self):
        try: return self._convert_acc([self.readIntegerSigned(self.ACCEL_X), 0, 0])[0]
        except Exception: return None

    def get_accely(self):
        try: return self._convert_acc([0, self.readIntegerSigned(self.ACCEL_Y), 0])[1]
        except Exception: return None

    def get_accelz(self):
        try: return self._convert_acc([0, 0, self.readIntegerSigned(self.ACCEL_Z)])[2]
        except Exception: return None

    def get_accelall(self):
        """Returns (x, y, z) accelerometer in m/s² via single block read, or None on error."""
        try:
            raw = self.bus.read_i2c_block_data(self.address, self.ACCEL_X, 6)
            return tuple(self._convert_acc(struct.unpack("<3h", bytes(raw))))
        except Exception:
            return None

    def get_heading(self):
        try:
            for _ in range(5):
                head = self.readInteger(self.CMPS)
                if 0 <= head <= 360:
                    return head
            return None
        except Exception:
            return None

    def get_magx(self):
        try: return self._convert_mag([self.readIntegerSigned(self.MAG_X), 0, 0])[0]
        except Exception: return None

    def get_magy(self):
        try: return self._convert_mag([0, self.readIntegerSigned(self.MAG_Y), 0])[1]
        except Exception: return None

    def get_magz(self):
        try: return self._convert_mag([0, 0, self.readIntegerSigned(self.MAG_Z)])[2]
        except Exception: return None

    def get_magall(self):
        """Returns (x, y, z) magnetometer in nT via single block read, or None on error."""
        try:
            raw = self.bus.read_i2c_block_data(self.address, self.MAG_X, 6)
            return tuple(self._convert_mag(struct.unpack("<3h", bytes(raw))))
        except Exception:
            return None

    def get_gyrox(self):
        try: return self._convert_gyro([self.readIntegerSigned(self.GYRO_X), 0, 0])[0]
        except Exception: return None

    def get_gyroy(self):
        try: return self._convert_gyro([0, self.readIntegerSigned(self.GYRO_Y), 0])[1]
        except Exception: return None

    def get_gyroz(self):
        try: return self._convert_gyro([0, 0, self.readIntegerSigned(self.GYRO_Z)])[2]
        except Exception: return None

    def get_gyroall(self):
        """Returns (x, y, z) gyroscope in deg/s via single block read, or None on error."""
        try:
            raw = self.bus.read_i2c_block_data(self.address, self.GYRO_X, 6)
            return tuple(self._convert_gyro(struct.unpack("<3h", bytes(raw))))
        except Exception:
            return None

    def start_cmpscal(self):
        try: self.command(67)
        except Exception: return None

    def stop_cmpscal(self):
        try: self.command(99)
        except Exception: return None

    def get_gyro_filter(self):
        return self.readByte(self.GYRO_FILTER)

    def set_gyro_filter(self, value):
        self.writeByte(self.GYRO_FILTER, value)

    def accel_2G(self):
        """Set accel 2G / gyro 250°/s. Datasheet requires 50ms settle time after command."""
        try: self.command(49); self.gyro_unit = 8.75; time.sleep(0.05)
        except Exception: return None

    def accel_4G(self):
        """Set accel 4G / gyro 500°/s. Datasheet requires 50ms settle time after command."""
        try: self.command(50); self.gyro_unit = 17.5; time.sleep(0.05)
        except Exception: return None

    def accel_8G(self):
        """Set accel 8G / gyro 2000°/s. Datasheet requires 50ms settle time after command."""
        try: self.command(51); self.gyro_unit = 70; time.sleep(0.05)
        except Exception: return None

    def accel_16G(self):
        """Set accel 16G / gyro 2000°/s. Datasheet requires 50ms settle time after command."""
        try: self.command(52); self.gyro_unit = 70; time.sleep(0.05)
        except Exception: return None

    def get_tilt_angle(self, axis='x'):
        """Tilt angle in degrees. Alias for get_tiltx/y/z()."""
        return {'x': self.get_tiltx, 'y': self.get_tilty, 'z': self.get_tiltz}[axis]()

    def _convert_gyro(self, raw_gyro):
        """Converts raw gyroscope values to deg/s using the current sensitivity setting."""
        gyro = [0.0, 0.0, 0.0]
        for i in range(3):
            gyro[i] = (raw_gyro[i] / 1000.0) * self.gyro_unit
        return gyro

    def _convert_acc(self, raw_acc):
        """Converts raw accelerometer values to m/s²."""
        acc = [0.0, 0.0, 0.0]
        for i in range(3):
            acc[i] = raw_acc[i] / 1000.0 * 9.80665
        return acc

    def _convert_mag(self, raw_mag):
        """Converts raw magnetometer values to nT."""
        mag = [0.0, 0.0, 0.0]
        mag[0] =  raw_mag[0] / 0.03432
        mag[1] = -raw_mag[1] / 0.03432
        mag[2] = -raw_mag[2] / 0.03432
        return mag
