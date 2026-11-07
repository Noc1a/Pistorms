"""Keep the RPLidar motor stopped while no program uses it.

The RPLidar A1 motor runs while the DTR line of its USB adapter (CP2102) is
off, and the CP2102 driver turns DTR off whenever the port is closed: with no
program running, the motor spins. This service keeps the port open with DTR
on. Programs using the LiDAR open the port too and drive DTR themselves
(RPLidar.start_motor / stop_motor): a program that ends with stop_motor()
leaves the motor stopped.

A program that crashes without stop_motor() leaves the motor running:
    sudo systemctl restart lidar-pause
Started by udev when the LiDAR appears (system/99-rplidar.rules).
"""
import time

import serial

PORT = "/dev/rplidar"     # symlink created by system/99-rplidar.rules


def main() -> None:
    port = serial.Serial(PORT)
    port.dtr = True       # DTR on: motor stopped
    print(f"LiDAR motor stopped, holding {PORT}")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
