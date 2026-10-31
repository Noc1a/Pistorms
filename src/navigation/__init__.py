"""Navigation for the person-follower robot. Import the submodules directly:

    from navigation.robot_state import RobotState
    from navigation.sensor_thread import Odometry, Camera, Lidar, Button
    from navigation.controllers import ManualController, FollowObjectController, GoHomeController

sensor_thread is deliberately not imported here: it needs picamera2 and
rplidar, and importing e.g. navigation.controllers must not require them.
"""
from .robot_state import Pose, RobotState
