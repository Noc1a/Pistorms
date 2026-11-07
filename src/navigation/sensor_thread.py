from abc import abstractmethod
from math import cos, sin
import math
import io
import json
import multiprocessing
import queue
import threading
import time
from typing import Optional

import numpy as np

from PIL import Image as _PILImage
from picamera2 import Picamera2
from picamera2.devices import IMX500
from pistorms import PiStorms, PiStormsMotor
from pistorms.devices import AbsoluteIMU, EV3InfraredSensor
from . import collision, lidar_reader, pose_correction
from .robot_state import Pose, RobotState
from .slam import ODOMETRY_LAG_SECS, LidarSlam


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
    # IMU gyro filter (FIR order 0–7, set_gyro_filter). Measured 2026-11-07
    # with tests/testOdometryLag.py against the wheel encoders: the gyro lags
    # 85 ms with 7, 35 ms with 4, 25 ms with 0; noise at rest 0.08 / 0.18 /
    # 0.24 °/s (≈0.2° per minute of heading at most: negligible). 4, the
    # factory value, is the compromise. Every test that reads the gyro for
    # the odometry uses this value. The lag is compensated by the SLAM
    # (navigation/slam.py, ODOMETRY_LAG_SECS).
    GYRO_FILTER = 4
    TICKS_PER_REV = 360.0
    GYRO_CAL_SAMPLES = 300
    SPEED_SMOOTHING = 0.2      # state.speed_mps: share of each new sample (~0.1 s at 50 Hz)
    LIDAR_WAIT_SECS = 15.0     # longest wait for the LiDAR to turn before the bias calibration
    LIDAR_SETTLE_SECS = 1.0    # then let its vibration settle

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
        # SLAM correction applied to state.pose, easing towards
        # state.odom_correction_target (navigation.pose_correction)
        self._correction: pose_correction.Correction = pose_correction.IDENTITY
        # Set once the gyro bias is calibrated and the pose starts updating:
        # the robot must stand still until then
        self.ready: threading.Event = threading.Event()

    def _calibrate_gyro_bias(self):
        """Average gyro Z while robot is stationary. Returns bias in rad/s."""
        print("Calibrating gyro bias — hold still...")
        samples = []
        for _ in range(self.GYRO_CAL_SAMPLES):
            gyr_z = self._read_gyro_z()
            if gyr_z is not None:
                samples.append(gyr_z * self.GYRO_Z_SIGN)
            time.sleep(0.005)
        if not samples:
            print("Gyro bias calibration got no samples — assuming 0 deg/s")
            return 0.0
        bias = math.radians(float(np.mean(samples)))
        print(f"Gyro Z bias: {math.degrees(bias):+.3f} deg/s")
        return bias

    def _read_gyro_z(self) -> Optional[float]:
        """Raw gyro Z in deg/s, or None on I²C error: that sample then uses
        the wheels only."""
        try:
            return self.imu.gyro('z')
        except OSError as e:
            print(f"Odometry: gyro read failed ({e!r}) — wheels only for this sample")
            return None

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

        # The SLAM's correction eases towards its latest target (no jump)
        if dt > 0:
            self._correction = pose_correction.blended(
                self._correction, self.state.odom_correction_target,
                min(1.0, dt / pose_correction.SMOOTHING_SECS))
        with self.state._pose_lock:
            odom = self.state.odom_pose
            heading_mid = odom.angle_rad + d_theta / 2.0
            odom.angle_rad += d_theta
            odom.x += distance * cos(heading_mid)
            odom.y += distance * sin(heading_mid)
            # Published pose: the odometry in the map's frame
            pose = self.state.pose
            pose.x, pose.y, pose.angle_rad = pose_correction.apply(
                self._correction, (odom.x, odom.y, odom.angle_rad))
        # Forward speed for the collision guard's stopping distance, smoothed
        # over ~0.1 s (5 samples at 50 Hz)
        if dt > 0:
            self.state.speed_mps += (distance / dt - self.state.speed_mps) * self.SPEED_SMOOTHING

    def run(self):
        try:
            self.imu.set_gyro_filter(self.GYRO_FILTER)
        except OSError as e:
            print(f"Odometry: could not set gyro filter ({e!r})")
        # Measure the bias with the LiDAR turning, as it will be: its
        # vibration shifts the gyro by ~0.13 °/s. Without it, a calibration
        # done LiDAR off drifted the heading ~0.11 °/s (live run 2026-11-07)
        if self.state.lidar_spinning.wait(timeout=self.LIDAR_WAIT_SECS):
            time.sleep(self.LIDAR_SETTLE_SECS)
        else:
            print("Odometry: no LiDAR turning — gyro bias measured without it")
        self._gyro_bias_z_rad = self._calibrate_gyro_bias()

        ticks = None
        while ticks is None and self.state.running:
            ticks = self._read_ticks()
            if ticks is None:
                time.sleep(self.PERIOD)
        if ticks is None:
            return
        ticks_left_prev, ticks_right_prev = ticks
        self.ready.set()

        t0 = time.monotonic()
        while self.state.running:
            try:
                gyr_z = self._read_gyro_z()
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
    """The LiDAR, read in a process of its own (navigation.lidar_reader: the
    camera, the SLAM and the web server no longer slow the reading). This
    thread receives each scan, works out the target and front obstacle
    distances, keeps it for web_viz (state.last_scan) and hands it to the
    Slam thread."""

    def __init__(self, state: RobotState, portName: str = '/dev/ttyUSB0'):
        super().__init__(state)
        self.portName = portName

    def _hand_to_slam(self, scan_time: float, scan: list, serial_bytes_waiting: int) -> None:
        """Queue the scan for the Slam thread, never blocking: if the queue
        is full (Slam behind, or not running), the oldest scan is dropped."""
        item = (scan_time, scan, serial_bytes_waiting)
        try:
            self.state.slam_scans.put_nowait(item)
        except queue.Full:
            try:
                self.state.slam_scans.get_nowait()
                self.state.slam_scans_dropped += 1
            except queue.Empty:
                pass
            self.state.slam_scans.put_nowait(item)

    def run(self):
        # 'spawn': a fresh process, safe in a program that already runs threads
        context = multiprocessing.get_context('spawn')
        scans = context.Queue(maxsize=lidar_reader.SCANS_QUEUE_SIZE)
        stop = context.Event()
        reader = context.Process(target=lidar_reader.read_lidar, args=(self.portName, scans, stop),
                                 name='lidar_reader', daemon=True)
        reader.start()
        try:
            while self.state.running:
                try:
                    scan_time, scan, serial_bytes_waiting = scans.get(timeout=1.0)
                except queue.Empty:
                    continue
                self.state.lidar_spinning.set()

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
                self._hand_to_slam(scan_time, scan, serial_bytes_waiting)
                self.state.targetDistance        = min_target_distance if min_target_distance != float('inf') else None
                self.state.frontObstacleDistance = min_front_distance  if min_front_distance  != float('inf') else None
                # Collision guard: free room in the robot's corridor (Controller._drive)
                self.state.front_clearance_m, self.state.rear_clearance_m = collision.clearances(scan)
                self.state.clearance_time = scan_time
        finally:
            # The reader stops its scan and its motor, then ends
            stop.set()
            reader.join(timeout=3.0)
            if reader.is_alive():
                reader.terminate()


class Slam(Sensor):
    """SLAM (navigation.slam.LidarSlam) on every scan of the Lidar thread
    (state.slam_scans) and the Odometry thread's raw pose (state.odom_pose).
    Writes state.slam_pose, state.slam_update_secs and the correction that
    keeps state.pose on the map (state.odom_correction_target,
    navigation.pose_correction); the map: state.slam.map_image().

    Each scan is matched with the odometry of ODOMETRY_LAG_SECS after its
    end (the odometry lags the robot): the thread waits that long first.
    A scan the SLAM is too slow for is dropped by the Lidar thread; the
    odometry still covers the move between two used scans.

    record=True keeps every scan with its timing and odometry, for
    save_recording() (a run tests/testSlam.py replays): ~15 MB per minute."""

    def __init__(self, state: RobotState, record: bool = False):
        super().__init__(state)
        self.slam: LidarSlam = LidarSlam()
        state.slam = self.slam              # for the map pictures (web_viz, screen)
        self.record = record
        self.recording: list[dict] = []
        self._recording_lock = threading.Lock()
        self._last_poses: Optional[tuple[tuple, tuple]] = None    # (SLAM pose, odometry) of the last update

    def run(self):
        while self.state.running:
            try:
                scan_time, points, serial_bytes_waiting = self.state.slam_scans.get(timeout=1.0)
            except queue.Empty:
                continue
            received = time.monotonic()
            time.sleep(max(0.0, scan_time + ODOMETRY_LAG_SECS - time.monotonic()))
            # The raw odometry: what the SLAM needs (deskewing, predicted move)
            with self.state._pose_lock:
                odom_pose = self.state.odom_pose
                odom = (odom_pose.x, odom_pose.y, odom_pose.angle_rad)
                published = (self.state.pose.x, self.state.pose.y, self.state.pose.angle_rad)
            try:
                update_start = time.monotonic()
                pose = self.slam.update(points, odom)
                self.state.slam_update_secs = time.monotonic() - update_start
            except Exception as e:
                # never let one bad scan kill the thread
                print(f"Slam error: {e!r}")
                continue
            if pose is not None:
                x, y, theta = pose
                self.state.slam_pose = Pose(x, y, theta, scan_time)
                self._update_correction(pose, odom)
            if self.record:
                with self._recording_lock:
                    self.recording.append({
                        'monotonic': scan_time, 'received': received,
                        'serial_bytes_waiting': serial_bytes_waiting,
                        'dropped_so_far': self.state.slam_scans_dropped,
                        'update_secs': self.state.slam_update_secs,
                        'points': [list(point) for point in points],
                        'odom': list(odom), 'slam_pose': None if pose is None else list(pose),
                        'published_pose': list(published)})

    def _update_correction(self, slam_pose: tuple, odom: tuple) -> None:
        """New correction target for the Odometry thread, unless the SLAM
        jumped (moved differently from the odometry since its last update)."""
        if self._last_poses is not None:
            previous_slam, previous_odom = self._last_poses
            if pose_correction.slam_jumped(previous_slam, slam_pose, previous_odom, odom):
                print("Slam: pose jumped against the odometry — correction kept", flush=True)
                self._last_poses = (slam_pose, odom)
                return
        self._last_poses = (slam_pose, odom)
        self.state.odom_correction_target = pose_correction.correction_between(slam_pose, odom)

    def save_recording(self, path: str) -> None:
        """Write the recorded scans as a testSlam.py run (scenario 'live').
        The odometry of each scan was already read ODOMETRY_LAG_SECS after
        it: the replay must not shift it again (odometry_lag_secs = 0)."""
        with self._recording_lock:
            scans = list(self.recording)
        if not scans:
            return
        start = scans[0]['monotonic']
        for scan in scans:
            scan['t'] = scan['monotonic'] - start
            scan['phase'] = 'live'
        run = {'scenario': 'live', 'date': time.strftime('%Y-%m-%dT%H:%M:%S'),
               'odometry_lag_secs': 0.0, 'live_odometry_lag_secs': ODOMETRY_LAG_SECS,
               'scans': scans}
        with open(path, 'w') as f:
            json.dump(run, f)
        print(f"SLAM recording saved: {path} ({len(scans)} scans)")
