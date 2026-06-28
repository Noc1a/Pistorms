from abc import abstractmethod
from math import cos, sin
import math
import os
import sys
import io
import threading
import time
import numpy as np

# Path setup — uses __file__ so this module works from any working directory
_nav_dir  = os.path.dirname(os.path.abspath(__file__))
_pist_dir = os.path.dirname(_nav_dir)
_root_dir = os.path.dirname(_pist_dir)
for _d in [_nav_dir, os.path.join(_pist_dir, 'sys')]:
    if _d not in sys.path: sys.path.insert(0, _d)

from PIL import Image as _PILImage
from picamera2 import Picamera2
from picamera2.devices import IMX500
from PiStorms import PiStorms, PiStormsSensor, PiStormsMotor
from MsDevices import AbsoluteIMU
from rplidar import RPLidar
from robot_state import Pose, RobotState


class Sensor:
    def __init__(self, state: RobotState):
        self.state = state
    @abstractmethod
    def run(self):
        raise NotImplementedError


class IR(Sensor):
    def __init__(self, state: RobotState, irSensor: PiStormsSensor):
        super().__init__(state)
        self.irSensor = irSensor

    def run(self):
        while self.state.running:
            self.state.targetDistance = self.irSensor.distanceIREV3()
            time.sleep(0.02)


class Button(Sensor):
    def __init__(self, state: RobotState, psm: PiStorms):
        super().__init__(state)
        self.psm = psm

    def run(self):
        while self.state.running:
            if self.psm.isKeyPressed():
                self.state.running = False
            time.sleep(0.02)


class Odometry(Sensor):
    """Wheel encoders + gyro Z fused into pose (x, y, angle_rad).

    Heading: complementary filter — α gyro / (1-α) wheel-differential.
    Position: average wheel travel projected onto fused heading.

    Madgwick + magnetometer were dropped: motor magnetic interference made the
    mag reference unreliable. Gyro Z handles short-term rotation, encoders bound
    long-term drift in absence of motion; an absolute reference (e.g. LiDAR scan
    match) would still be needed for unbounded long-term accuracy."""

    PERIOD = 0.020         # 50 Hz
    ALPHA  = 0.98          # gyro trust in heading fusion
    TICKS_PER_REV = 360.0
    GYRO_CAL_SAMPLES = 300

    def __init__(self, state: RobotState, imu: AbsoluteIMU,
                 motorLeft: PiStormsMotor, motorRight: PiStormsMotor,
                 wheel_diameter: float, wheel_base: float):
        super().__init__(state)
        self.imu = imu
        self.motorLeft = motorLeft
        self.motorRight = motorRight
        self.wheel_circumference = math.pi * wheel_diameter
        self.wheel_base = wheel_base
        self._gyro_bias_z_rad = 0.0

    def _calibrate_gyro_bias(self):
        """Average gyro Z while robot is stationary. Returns bias in rad/s."""
        print("Calibrating gyro bias — hold still...")
        samples = []
        for _ in range(self.GYRO_CAL_SAMPLES):
            gyr_z = self.imu.get_gyroz()
            if gyr_z is not None:
                samples.append(gyr_z)
            time.sleep(0.005)
        bias = math.radians(float(np.mean(samples)))
        print(f"Gyro Z bias: {math.degrees(bias):+.3f} deg/s")
        return bias

    def run(self):
        self.imu.set_gyro_filter(7)
        self._gyro_bias_z_rad = self._calibrate_gyro_bias()

        ticks_left_prev  = self.motorLeft.pos()
        ticks_right_prev = self.motorRight.pos()

        t0 = time.monotonic()
        while self.state.running:
            gyr_z = self.imu.get_gyroz()
            ticks_left  = self.motorLeft.pos()
            ticks_right = self.motorRight.pos()

            now = time.monotonic()
            dt = now - t0
            t0 = now

            if gyr_z is None:
                time.sleep(self.PERIOD)
                continue

            # encoder deltas → wheel travel (m)
            dl = (ticks_left  - ticks_left_prev)  * self.wheel_circumference / self.TICKS_PER_REV
            dr = (ticks_right - ticks_right_prev) * self.wheel_circumference / self.TICKS_PER_REV
            ticks_left_prev, ticks_right_prev = ticks_left, ticks_right

            # heading: complementary filter on rotation increments
            d_theta_gyro = (math.radians(gyr_z) - self._gyro_bias_z_rad) * dt
            d_theta_odom = (dr - dl) / self.wheel_base
            d_theta = self.ALPHA * d_theta_gyro + (1 - self.ALPHA) * d_theta_odom

            distance = (dl + dr) / 2.0

            with self.state._pose_lock:
                self.state.pose.angle_rad += d_theta
                self.state.pose.x += distance * cos(self.state.pose.angle_rad)
                self.state.pose.y += distance * sin(self.state.pose.angle_rad)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, self.PERIOD - elapsed))


class Camera(Sensor):
    CAM_WIDTH  = 320
    CAM_HEIGHT = 320
    HFOV       = 78.0  # Approx horizontal field of view of the AI module

    def __init__(self, state: RobotState):
        super().__init__(state)
        MODEL_PATH = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"

        self.imx500 = IMX500(MODEL_PATH)
        self.picam2 = Picamera2(self.imx500.camera_num)
        intrinsics = self.imx500.network_intrinsics
        intrinsics.update_with_defaults()

        print(f"Inference rate recommandée: {intrinsics.inference_rate} fps")

        config = self.picam2.create_video_configuration(
            main={"format": "RGB888", "size": (Camera.CAM_WIDTH, Camera.CAM_HEIGHT)},
            controls={"FrameRate": intrinsics.inference_rate},
            buffer_count=12,
        )
        self.picam2.configure(config)

    def run(self):
        def _frame_cb(request):
            try:
                arr = request.make_array("main")
                buf = io.BytesIO()
                _PILImage.fromarray(arr).save(buf, format='JPEG', quality=65)
                self.state.last_frame = buf.getvalue()
            except Exception:
                pass

        self.picam2.pre_callback = _frame_cb
        self.picam2.start()
        labels = self.imx500.network_intrinsics.labels

        while self.state.running:
            metadata = self.picam2.capture_metadata()
            outputs = self.imx500.get_outputs(metadata, add_batch=True)

            if outputs is not None:
                # SSD outputs: [0]=boxes, [1]=scores, [2]=classes
                boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]

                detections = []
                for i in range(len(scores)):
                    if scores[i] > 0.5:
                        box = boxes[i]
                        x, y, w, h = self.imx500.convert_inference_coords(box, metadata, self.picam2)
                        label = labels[int(classes[i])] if labels else str(int(classes[i]))
                        detections.append({
                            'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h),
                            'label': label, 'score': float(scores[i]),
                        })
                        if label == "person":
                            self.state.targetPose.pose_timestamp = time.monotonic()
                            self.state.targetPose.x = int(x + (w // 2) - Camera.CAM_WIDTH // 2)
                            ratio = self.state.targetPose.x / (Camera.CAM_WIDTH / 2)
                            deviation_deg = ratio * (Camera.HFOV / 2)
                            self.state.targetPose.angle_rad = math.radians(deviation_deg)
                self.state.last_detections = detections

        self.picam2.stop()


class Lidar(Sensor):
    def __init__(self, state: RobotState, portName: str = '/dev/ttyUSB0'):
        super().__init__(state)
        self.portName = portName
        self.lidar = RPLidar(self.portName)

    def run(self):
        while self.state.running:
            for _, scan in enumerate(self.lidar.iter_scans()):
                if not self.state.running:
                    break

                min_target_distance = float('inf')
                min_front_distance  = float('inf')

                lidar_target_angle = self.state.targetPose.get_angle_deg() % 360

                for (quality, angle, distance) in scan:
                    if quality == 0 or distance <= 0:
                        continue

                    angle_diff = abs((angle - lidar_target_angle + 180) % 360 - 180)
                    if angle_diff <= 5 and distance < 2000:
                        min_target_distance = min(min_target_distance, distance)

                    is_front = angle <= 10 or angle >= 350
                    if is_front and distance < 300:
                        min_front_distance = min(min_front_distance, distance)

                self.state.last_scan = [[q, a, d] for q, a, d in scan]

                self.state.targetDistance        = min_target_distance if min_target_distance != float('inf') else None
                self.state.frontObstacleDistance = min_front_distance  if min_front_distance  != float('inf') else None

                if self.state.targetDistance is not None and self.state.frontObstacleDistance is not None:
                    print(f"LIDAR scan: Target dist={self.state.targetDistance:.1f} mm | "
                          f"Front obstacle={self.state.frontObstacleDistance:.1f} mm | "
                          f"Target angle={self.state.targetPose.get_angle_deg():.1f}°")

        self.lidar.stop()
        self.lidar.disconnect()
