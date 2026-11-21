"""Calibrate the robot: set up its profile (robot_profile.json) and measure
how its sensors are mounted.

    python3 -u -m navigation.calibration all            # a new or rebuilt model
    python3 -u -m navigation.calibration setup imu lidar
    python3 -u -m navigation.calibration compass [--mode 3d] [--secs 60] [--continuous]
    python3 -u -m navigation.calibration verify

  setup    wizard in the terminal: wheels, motor ports and directions, size
           of the robot, sensor ports — what must be measured by hand
  imu      which IMU axis points up and which ahead (pivot, short dashes)
  lidar    where the LiDAR is and which way it points (a turn in 12 stops,
           25 cm forward and back)
  compass  hard and soft iron of the magnetometer (2d: the robot spins;
           3d: turned by hand)
  verify   checks the LiDAR direction and offset of the profile
  all      setup, imu, lidar, compass

What a calibration builds on is run first if it is not done yet (the
compass needs the IMU axes, the check needs the LiDAR calibration, all need
the profile). A sensor without a port in the profile is skipped. The
profile is saved after each step.

A saved recording can be solved again, on the Pi or on the Mac:

    python3 -m navigation.calibration lidar   --from calibration_runs/lidarcal_<date>.json
    python3 -m navigation.calibration verify  --from calibration_runs/verify_<date>.json
    python3 -m navigation.calibration compass --from calibration_runs/compass2d_<date>.csv

Needs the robot on the floor with 50 cm free ahead and 40 cm all around,
walls or furniture in sight for the LiDAR, and for the compass no laptop,
speaker or metal furniture within 1.5 m. Run it in the robot's screen
session (it waits for your answers); Enter keeps the value shown in brackets.
"""
import argparse

from .calibrator import Calibrator
from .compass import CompassCalibration
from .imu_axes import ImuAxesCalibration
from .lidar_mount import LidarMountCalibration
from .lidar_verify import LidarMountCheck


def main() -> None:
    parser = argparse.ArgumentParser(prog='python3 -m navigation.calibration', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('what', nargs='+', choices=['setup', 'imu', 'lidar', 'compass', 'verify', 'all'])
    parser.add_argument('--from', dest='source', help="solve a saved recording again (no robot)")
    parser.add_argument('--force', action='store_true',
                        help="redo what the calibrations asked for build on, even if already done")
    parser.add_argument('--mode', choices=['2d', '3d'], default='2d', help="compass (default 2d)")
    parser.add_argument('--secs', type=float, default=60.0, help="compass 3d: recording time")
    parser.add_argument('--continuous', action='store_true',
                        help="compass 2d: one spin with the motors running (default: stepped, motors off)")
    parser.add_argument('--show', action='store_true', help="compass: also open the plot windows")
    args = parser.parse_args()

    calibrator = Calibrator([ImuAxesCalibration(), LidarMountCalibration(),
                             CompassCalibration(args.mode, args.continuous, args.secs, args.show),
                             LidarMountCheck()])
    if args.source:
        if len(args.what) != 1 or args.what[0] not in ('lidar', 'compass', 'verify'):
            parser.error("--from goes with one of: lidar, compass, verify")
        for line in calibrator.replay(args.what[0], args.source):
            print(line)
    else:
        calibrator.run(args.what, force=args.force)


if __name__ == '__main__':
    main()
