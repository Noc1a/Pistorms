import math
import queue
import threading
from typing import TYPE_CHECKING, Optional

import numpy as np

if TYPE_CHECKING:
    from .slam import LidarSlam


class Pose:
    def __init__(self, x: int, y: int, angle_rad: float, pose_timestamp: float = None):
        self.x = x
        self.y = y
        self.angle_rad = angle_rad
        self.pose_timestamp = pose_timestamp

    def get_angle_deg(self):
        return np.rad2deg(self.angle_rad)


class RobotState:
    MODE_MANUAL = 'manual'
    MODE_FOLLOW = 'follow'
    MODE_HOME   = 'home'

    def __init__(self, running):
        # The robot's pose: the odometry corrected by the SLAM (Odometry thread,
        # navigation.pose_correction). Without SLAM, the odometry itself.
        self.pose = Pose(0, 0, 0.0)
        # The odometry as integrated (wheels + gyro), drifting while rolling:
        # what the SLAM needs, not what the controllers should use
        self.odom_pose = Pose(0, 0, 0.0)
        # Correction from the odometry's frame to the map's: written by the
        # Slam thread at each scan, eased into self.pose by the Odometry thread
        self.odom_correction_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.targetPose = Pose(0, 0, 0.0)
        self.frontObstacleDistance = None
        # Collision guard (navigation.collision): free room in the robot's
        # corridor, written by the Lidar thread at each scan
        self.front_clearance_m: float = math.inf
        self.rear_clearance_m: float = math.inf
        self.clearance_time: float = -math.inf    # time.monotonic() of the scan
        self.speed_mps: float = 0.0               # forward speed (Odometry), signed
        self.blocked: Optional[str] = None        # 'front', 'rear' or None (for display)
        self.targetDistance = None
        self.targetAngle = 0.0
        self.running = running
        self._pose_lock = threading.Lock()
        # visualization data (written by sensor threads, read by web_viz)
        self.last_scan = []          # [[quality, angle, distance], ...]
        # Every LiDAR scan, for the Slam thread: (monotonic time at the end of
        # the scan, points). The Lidar thread drops the oldest when it is full
        # (time, points, serial bytes waiting)
        self.slam_scans: queue.Queue = queue.Queue(maxsize=2)
        self.slam_scans_dropped: int = 0      # scans the Slam thread was too slow for
        # Set once the LiDAR motor turns (first scan received): the Odometry
        # thread waits for it to measure the gyro bias, which the LiDAR's
        # vibration shifts by ~0.13 °/s
        self.lidar_spinning: threading.Event = threading.Event()
        # Pose found by the Slam thread (odometry frame), None until its first scan
        self.slam_pose: Optional[Pose] = None
        # The Slam thread's LidarSlam (map pictures: slam.map_image()), None without SLAM
        self.slam: Optional["LidarSlam"] = None
        self.slam_update_secs: float = 0.0    # time of the last SLAM update (Pi load)
        self.last_detections = []    # [{'x','y','w','h','label','score'}, ...]
        self.last_frame = None       # latest camera frame as JPEG bytes
        self.last_frame_request = float('-inf')   # monotonic time a viewer last asked for a frame
        # control mode (written by web, read by main control loop)
        self.mode = RobotState.MODE_MANUAL
        self.manual_cmd = {'fwd': 0.0, 'turn': 0.0, 'speed': 30}   # -1..1, -1..1, 0..100
        self.follow_target = 'person'
