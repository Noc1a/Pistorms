from abc import abstractmethod
from math import cos, sin
import math
import io
import threading
import time
import numpy as np

from PIL import Image as _PILImage
from picamera2 import Picamera2
from picamera2.devices import IMX500
from pistorms import PiStorms, PiStormsMotor
from pistorms.devices import AbsoluteIMU, EV3InfraredSensor
from rplidar import RPLidar
from .robot_state import Pose, RobotState


class Sensor:
    def __init__(self, state: RobotState):
        self.state = state
    @abstractmethod
    def run(self):
        raise NotImplementedError


class IR(Sensor):
    def __init__(self, state: RobotState, irSensor: EV3InfraredSensor):
        super().__init__(state)
        self.irSensor = irSensor

    def run(self):
        while self.state.running:
            self.state.targetDistance = self.irSensor.proximity()
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

    Pose frame: x forward at start, y to the left, angle CCW-positive
    (right-handed). All raw sensor signs are corrected here so controllers
    can use atan2 directly.

    Heading: complementary filter — α gyro / (1-α) wheel-differential.
    Position: average wheel travel projected onto the midpoint heading.

    Madgwick + magnetometer were dropped: motor magnetic interference made the
    mag reference unreliable. Blending gyro and wheel increments reduces gyro
    noise but does NOT bound heading drift; an absolute reference (e.g. LiDAR
    scan match) is needed for long-term accuracy."""

    PERIOD = 0.020         # 50 Hz
    ALPHA  = 0.98          # gyro trust in heading fusion
    TICKS_PER_REV = 360.0
    GYRO_CAL_SAMPLES = 300

    # Sign conventions (measured in tests/testOdometryAccuracy.py):
    # setSpeed(-N) drives forward, and encoders count in the commanded
    # direction, so forward motion DECREASES both tick counts.
    LEFT_TICK_SIGN  = -1
    RIGHT_TICK_SIGN = -1
    # The AbsoluteIMU's Z axis points down on this robot: a CCW turn reads
    # negative gyro Z.
    GYRO_Z_SIGN     = -1
    # A larger tick change in one 20 ms sample is physically impossible
    # (I²C corruption or an encoder reset by another process) — skip it.
    MAX_TICK_DELTA  = 500

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
                samples.append(gyr_z * self.GYRO_Z_SIGN)
            time.sleep(0.005)
        if not samples:
            print("Gyro bias calibration got no samples — assuming 0 deg/s")
            return 0.0
        bias = math.radians(float(np.mean(samples)))
        print(f"Gyro Z bias: {math.degrees(bias):+.3f} deg/s")
        return bias

    def _read_ticks(self):
        """Both encoders as forward-positive tick counts, or None on I²C error."""
        try:
            return (self.motorLeft.pos()  * self.LEFT_TICK_SIGN,
                    self.motorRight.pos() * self.RIGHT_TICK_SIGN)
        except (OSError, TypeError) as e:
            print(f"Odometry: encoder read failed ({e!r}) — skipping sample")
            return None

    def _integrate(self, d_left, d_right, gyr_z, dt):
        """Advance the pose by one sample. d_left/d_right are forward-positive
        tick deltas; gyr_z is the RAW gyro Z in deg/s (or None)."""
        # encoder deltas → forward-signed wheel travel (m)
        dl = d_left  * self.wheel_circumference / self.TICKS_PER_REV
        dr = d_right * self.wheel_circumference / self.TICKS_PER_REV

        # heading: complementary filter on rotation increments; without
        # a gyro sample, fall back to wheels only for this step
        d_theta_odom = (dr - dl) / self.wheel_base
        if gyr_z is None:
            d_theta = d_theta_odom
        else:
            d_theta_gyro = (math.radians(gyr_z * self.GYRO_Z_SIGN)
                            - self._gyro_bias_z_rad) * dt
            d_theta = self.ALPHA * d_theta_gyro + (1 - self.ALPHA) * d_theta_odom

        distance = (dl + dr) / 2.0

        with self.state._pose_lock:
            heading_mid = self.state.pose.angle_rad + d_theta / 2.0
            self.state.pose.angle_rad += d_theta
            self.state.pose.x += distance * cos(heading_mid)
            self.state.pose.y += distance * sin(heading_mid)

    def run(self):
        try:
            self.imu.set_gyro_filter(7)
        except OSError as e:
            print(f"Odometry: could not set gyro filter ({e!r})")
        self._gyro_bias_z_rad = self._calibrate_gyro_bias()

        ticks = None
        while ticks is None and self.state.running:
            ticks = self._read_ticks()
            if ticks is None:
                time.sleep(self.PERIOD)
        if ticks is None:
            return
        ticks_left_prev, ticks_right_prev = ticks

        t0 = time.monotonic()
        while self.state.running:
            try:
                gyr_z = self.imu.get_gyroz()
                ticks = self._read_ticks()

                now = time.monotonic()
                dt = now - t0
                t0 = now

                if ticks is None:
                    time.sleep(self.PERIOD)
                    continue
                ticks_left, ticks_right = ticks
                d_left  = ticks_left  - ticks_left_prev
                d_right = ticks_right - ticks_right_prev
                ticks_left_prev, ticks_right_prev = ticks_left, ticks_right
                if abs(d_left) > self.MAX_TICK_DELTA or abs(d_right) > self.MAX_TICK_DELTA:
                    print(f"Odometry: implausible encoder jump "
                          f"(dl={d_left}, dr={d_right} ticks) — skipping sample")
                    time.sleep(self.PERIOD)
                    continue

                self._integrate(d_left, d_right, gyr_z, dt)
            except Exception as e:
                # never let one bad sample kill the thread: a dead odometry
                # thread freezes the pose and GoHome would drive forever
                print(f"Odometry error: {e!r}")

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

    # JPEG-encode preview frames only while a web viewer has asked for one
    # recently — encoding every frame costs CPU the control loop needs.
    FRAME_VIEWER_TIMEOUT = 2.0   # s

    def run(self):
        def _frame_cb(request):
            try:
                if time.monotonic() - self.state.last_frame_request > self.FRAME_VIEWER_TIMEOUT:
                    return
                arr = request.make_array("main")
                buf = io.BytesIO()
                _PILImage.fromarray(arr).save(buf, format='JPEG', quality=65)
                self.state.last_frame = buf.getvalue()
            except Exception:
                pass

        self.picam2.pre_callback = _frame_cb
        self.picam2.start()
        labels = self.imx500.network_intrinsics.labels

        try:
            while self.state.running:
                try:
                    self._process_frame(labels)
                except Exception as e:
                    # one bad inference result must not kill the thread —
                    # Follow mode would then spin searching forever
                    print(f"Camera error: {e!r}")
                    time.sleep(0.1)
        finally:
            try: self.picam2.stop()
            except Exception: pass
            try: self.picam2.close()
            except Exception: pass

    def _process_frame(self, labels):
        metadata = self.picam2.capture_metadata()
        outputs = self.imx500.get_outputs(metadata, add_batch=True)
        if outputs is None:
            return

        # SSD outputs: [0]=boxes, [1]=scores, [2]=classes
        boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]

        detections = []
        best = None   # highest-scoring detection of the follow target
        for i in range(len(scores)):
            if scores[i] > 0.5:
                box = boxes[i]
                x, y, w, h = self.imx500.convert_inference_coords(box, metadata, self.picam2)
                label = labels[int(classes[i])] if labels else str(int(classes[i]))
                det = {
                    'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h),
                    'label': label, 'score': float(scores[i]),
                }
                detections.append(det)
                if label == self.state.follow_target and (best is None or det['score'] > best['score']):
                    best = det

        if best is not None:
            target_x = int(best['x'] + (best['w'] // 2) - Camera.CAM_WIDTH // 2)
            deviation_deg = target_x / (Camera.CAM_WIDTH / 2) * (Camera.HFOV / 2)
            # Publish a NEW Pose in one assignment so readers never see the new
            # timestamp paired with the old x/angle.
            self.state.targetPose = Pose(target_x, 0, math.radians(deviation_deg),
                                         pose_timestamp=time.monotonic())
        self.state.last_detections = detections


class Lidar(Sensor):
    def __init__(self, state: RobotState, portName: str = '/dev/ttyUSB0'):
        super().__init__(state)
        self.portName = portName
        self.lidar = RPLidar(self.portName)

    def _reconnect(self):
        """Reset the lidar cleanly — used after a stale/corrupt session leaves
        get_health() returning a 3-tuple that the library can't unpack."""
        try: self.lidar.stop()
        except Exception: pass
        try: self.lidar.disconnect()
        except Exception: pass
        time.sleep(1.0)
        self.lidar = RPLidar(self.portName)

    def run(self):
        # max_buf_meas: larger buffer tolerates brief consumer stalls without warnings
        backoff = 1.0
        while self.state.running:
            try:
                for scan in self.lidar.iter_scans(max_buf_meas=6000):
                    backoff = 1.0  # got a scan — reset backoff
                    if not self.state.running:
                        break

                    min_target_distance = float('inf')
                    min_front_distance  = float('inf')
                    lidar_target_angle  = self.state.targetPose.get_angle_deg() % 360

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
            except Exception as e:
                if not self.state.running: break
                print(f"Lidar error: {e!r} — reconnecting in {backoff:.1f}s")
                time.sleep(backoff)
                backoff = min(backoff * 2, 15.0)
                try: self._reconnect()
                except Exception as e2:
                    print(f"Lidar reconnect failed: {e2!r}")

        try: self.lidar.stop()
        except Exception: pass
        try: self.lidar.disconnect()
        except Exception: pass
