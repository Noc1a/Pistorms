import math
import struct
import time
from typing import Literal, NamedTuple, Optional

from ..sensor_port import SensorPort
from .ms_device import MindsensorsI2CDevice

Axis = Literal['x', 'y', 'z']
AXES: tuple[Axis, Axis, Axis] = ('x', 'y', 'z')


class Vector3(NamedTuple):
    """A reading of the 3 axes: imu.gyro_all().z, or x, y, z = imu.accel_all()."""
    x: float
    y: float
    z: float


class AbsoluteIMU(MindsensorsI2CDevice):
    """mindsensors AbsoluteIMU-ACG: accelerometer, compass, magnetometer, gyro.

    Each sensor reads all its axes (gyro_all(), 6 bytes) or one axis
    (gyro('z'), 2 bytes: faster, for the 50 Hz odometry).

    Bus errors raise OSError, like the rest of the library. Units, checked
    on the robot (mindsensorsDevices skill, references/absolute_imu.md):
        accel m/s², gyro °/s, tilt and compass in degrees,
        magnetometer in "pseudo-nT" (see MAG_SCALE).
    """

    DEFAULT_ADDRESS = 0x22

    # Registers. 16-bit values are little-endian signed, X then Y then Z.
    # Firmware tilt, one byte per axis (128 = level): NOT used by tilt(). It
    # is asin(a/g) on one byte, which overflows near +1 g (255 → 0: an axis
    # pointing down reads −90°) and clips near −1 g; measured 2026-11-06
    # (mindsensorsDevices skill, absolute_imu.md). Read only by
    # tests/testAbsoluteIMU.py to show the fault.
    TILT_REGISTERS = 0x42
    ACCEL       = 0x45   # milli-g
    HEADING     = 0x4B   # compass, degrees 0–359
    MAG         = 0x4D   # raw magnetic field
    GYRO        = 0x53   # raw rate, × the unit of the sensitivity level
    GYRO_FILTER = 0x5A   # 0–7

    # The 4 sensitivity levels, by accelerometer range (g): one command sets
    # both ranges. → (command, gyro range °/s, gyro milli-degrees/s per raw unit).
    # Levels 3 and 4 share the gyro range: only the accelerometer differs.
    SENSITIVITIES = {
        2:  ('1', 250, 8.75),
        4:  ('2', 500, 17.5),
        8:  ('3', 2000, 70.0),
        16: ('4', 2000, 70.0),
    }
    # The guide says 50 ms after a sensitivity command, but measured on the
    # robot (2026-11-01) the accelerometer reads up to +110 % for 0.3–0.4 s
    # after a change (e.g. 2092 instead of 1000 milli-g at rest): wait 0.5 s.
    SENSITIVITY_SETTLE_TIME = 0.5

    ACCEL_SCALE = 9.80665 / 1000   # milli-g → m/s²
    # The guide gives no magnetometer unit. This conversion (origin unknown)
    # is kept because the calibration files (mag_calibration*.json) are
    # expressed in it, but it reads the earth's field ≈1.8× too small on the
    # robot (≈26 instead of 48 µT): "pseudo-nT", fine for relative use only.
    MAG_SCALE = 1 / 0.03432
    MAG_SIGNS = Vector3(1, -1, -1)
    NO_SIGN_CHANGE = Vector3(1, 1, 1)

    def __init__(self, port: SensorPort, address: Optional[int] = None,
                 accel_range_g: int = 2) -> None:
        super().__init__(port, address)
        # The IMU keeps its sensitivity until power-off, whatever program set
        # it: set it here so that the gyro unit is always the right one
        self.set_sensitivity(accel_range_g)

    def set_sensitivity(self, accel_range_g: int) -> None:
        """Sensitivity level, by accelerometer range:
            2 → ±2 g,  gyro ±250 °/s (default)
            4 → ±4 g,  gyro ±500 °/s
            8 → ±8 g,  gyro ±2000 °/s
            16 → ±16 g, gyro ±2000 °/s"""
        command, gyro_range_dps, gyro_milli_degrees_per_unit = self.SENSITIVITIES[accel_range_g]
        self.command(ord(command))
        time.sleep(self.SENSITIVITY_SETTLE_TIME)
        self.accel_range_g: int = accel_range_g
        self.gyro_range_dps: int = gyro_range_dps
        self.gyro_unit: float = gyro_milli_degrees_per_unit / 1000   # °/s per raw unit

    # ── Readings ───────────────────────────────────────────────────────────

    def accel_all(self) -> Vector3:
        """Acceleration in m/s²; at rest, the axis pointing down reads +9.8."""
        return self._scaled_vector(self._read_vector(self.ACCEL), self.ACCEL_SCALE)

    def accel(self, axis: Axis) -> float:
        return self._read_axis(self.ACCEL, axis) * self.ACCEL_SCALE

    def gyro_all(self) -> Vector3:
        """Rotation rate in °/s."""
        return self._scaled_vector(self._read_vector(self.GYRO), self.gyro_unit)

    def gyro(self, axis: Axis) -> float:
        return self._read_axis(self.GYRO, axis) * self.gyro_unit

    def mag_all(self) -> Vector3:
        """Magnetic field in pseudo-nT (see MAG_SCALE)."""
        return self._scaled_vector(self._read_vector(self.MAG), self.MAG_SCALE, self.MAG_SIGNS)

    def mag(self, axis: Axis) -> float:
        sign = self.MAG_SIGNS[AXES.index(axis)]
        return self._read_axis(self.MAG, axis) * self.MAG_SCALE * sign

    def acc_mag_gyr(self) -> tuple[Vector3, Vector3, Vector3]:
        """accel_all(), mag_all() and gyro_all() in one 20-byte read
        (5.4 ms instead of 3 × 2.2 ms): 0x45–0x58 holds accel (6 bytes),
        compass heading (2), mag (6) and gyro (6)."""
        raw = bytes(self.readArray(self.ACCEL, 20))
        accel = self._scaled_vector(struct.unpack("<3h", raw[0:6]), self.ACCEL_SCALE)
        mag = self._scaled_vector(struct.unpack("<3h", raw[8:14]), self.MAG_SCALE, self.MAG_SIGNS)
        gyro = self._scaled_vector(struct.unpack("<3h", raw[14:20]), self.gyro_unit)
        return accel, mag, gyro

    def tilt_all(self) -> Vector3:
        """Tilt of each axis in degrees, from the accelerometer:
        asin(a_axis / |a|), 0 = axis level, +90 = axis pointing down.
        Like any asin, 100° reads 80°: for a full-turn angle use atan2 on
        accel_all(). Only meaningful at rest (shakes add acceleration)."""
        accel = self.accel_all()
        return Vector3(self._tilt_degrees(accel.x, accel), self._tilt_degrees(accel.y, accel),
                       self._tilt_degrees(accel.z, accel))

    def tilt(self, axis: Axis) -> float:
        """Tilt of one axis (reads the 3 accelerometer axes: |a| is needed)."""
        accel = self.accel_all()
        return self._tilt_degrees(accel[AXES.index(axis)], accel)

    def heading(self) -> int:
        """Compass heading in degrees: 0 = the word "Compass" printed on the
        sensor points north. Calibrate the compass on the robot first."""
        heading = self.readInteger(self.HEADING)
        if heading > 359:
            raise OSError(f"AbsoluteIMU: compass heading out of range ({heading})")
        return heading

    # ── Settings ───────────────────────────────────────────────────────────

    def gyro_filter(self) -> int:
        """Current gyro filter order (see set_gyro_filter)."""
        return self.readByte(self.GYRO_FILTER)

    def set_gyro_filter(self, order: int) -> None:
        """FIR filter on the gyro: 0 = none (fast, noisy) … 7 = strongest
        (less noise, readings ~10 ms slower). Factory value 4; the IMU keeps
        the value from one program to the next."""
        self.writeByte(self.GYRO_FILTER, order)

    def begin_compass_calibration(self) -> None:
        """Then turn the whole robot slowly around each axis, and call
        end_compass_calibration() (stored in the IMU, kept at power-off)."""
        self.command(ord('C'))

    def end_compass_calibration(self) -> None:
        self.command(ord('c'))

    # ── Raw reads and conversions ──────────────────────────────────────────

    def _read_vector(self, register: int) -> tuple[int, int, int]:
        """Three signed 16-bit values (X, Y, Z) from register on: 6 bytes."""
        return struct.unpack("<3h", bytes(self.readArray(register, 6)))

    def _read_axis(self, register: int, axis: Axis) -> int:
        """The signed 16-bit value of one axis: 2 bytes."""
        return self.readIntegerSigned(register + 2 * AXES.index(axis))

    def _scaled_vector(self, raw: tuple[int, int, int], scale: float,
                       signs: Vector3 = NO_SIGN_CHANGE) -> Vector3:
        x, y, z = raw
        return Vector3(x * scale * signs.x, y * scale * signs.y, z * scale * signs.z)

    @staticmethod
    def _tilt_degrees(axis_accel: float, accel: Vector3) -> float:
        norm = math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2)
        if norm == 0:          # free fall: no gravity to measure the tilt with
            return 0.0
        return math.degrees(math.asin(max(-1.0, min(1.0, axis_accel / norm))))
