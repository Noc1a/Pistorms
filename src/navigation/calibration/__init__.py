"""Calibration of the robot: its profile (robot_profile.json) and how its
sensors are mounted.

    python3 -u -m navigation.calibration all        # see __main__.py

    from navigation.calibration import Calibrator
    Calibrator().run(['lidar'])

    calibrator.py    Calibrator: runs the calibrations, what they require first
    base.py          Calibration (record / solve) and what they share on the robot
    wizard.py        the values measured by hand, asked in the terminal
    motor_start.py   MotorStartCalibration: the slowest speed that starts the wheels
    imu_axes.py      ImuAxesCalibration: which IMU axis points up, which ahead
    lidar_mount.py   LidarMountCalibration: where the LiDAR is, which way it points
    lidar_verify.py  LidarMountCheck: does the profile's LiDAR mounting hold
    compass.py       CompassCalibration: hard and soft iron of the magnetometer
    moves.py         moves that stop on the wheel encoders
    lidar_scans.py   scans taken while the robot stands still
"""
from .base import Calibration, CalibrationContext
from .calibrator import Calibrator
from .compass import CompassCalibration
from .imu_axes import ImuAxesCalibration
from .lidar_mount import LidarMountCalibration
from .lidar_verify import LidarMountCheck
from .motor_start import MotorStartCalibration
