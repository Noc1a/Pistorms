import threading
import numpy as np


class Pose:
    def __init__(self, x: int, y: int, angle_rad: float, pose_timestamp: float = None):
        self.x = x
        self.y = y
        self.angle_rad = angle_rad
        self.pose_timestamp = pose_timestamp

    def get_angle_deg(self):
        return np.rad2deg(self.angle_rad)


class RobotState:
    def __init__(self, running):
        self.pose = Pose(0, 0, 0.0)
        self.targetPose = Pose(0, 0, 0.0)
        self.frontObstacleDistance = None
        self.targetDistance = None
        self.targetAngle = 0.0
        self.running = running
        self._pose_lock = threading.Lock()
        # visualization data (written by sensor threads, read by web_viz)
        self.last_scan = []          # [[quality, angle, distance], ...]
        self.last_detections = []    # [{'x','y','w','h','label','score'}, ...]
        self.last_frame = None       # latest camera frame as JPEG bytes
