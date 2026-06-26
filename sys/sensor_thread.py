from abc import abstractmethod
from math import cos, sin
import math
import math
import os
import sys
import numpy as np
from picamera2 import Picamera2
from picamera2.devices import IMX500
import threading
sys.path.append(os.path.join("PiStorms/sys"))
from PiStorms import PiStorms, PiStormsSensor
from PiStorms import PiStormsMotor
from MsDevices import AbsoluteIMU
from ahrs.filters.madgwick import Madgwick
from ahrs.common.orientation import am2q
from ahrs.common.quaternion import Quaternion
import time
from ahrs.utils import WMM
sys.path.append(os.path.join("magnetometer_calibration"))
from calibrate import MagnetometerCalibrator
from rplidar import RPLidar


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


class Imu(Sensor):
    PERIOD = 0.020  # 10 ms
    
    def __init__(self, state: RobotState, imu: AbsoluteIMU, psm: PiStorms):
        super().__init__(state)
        self.imu = imu
        self.psm = psm

        # gainr for 100hz :0.041 if lower (low CPU) multiply gain by sample 100/freq ... gain =0.082,
        self.ahrs = Madgwick( dt=Imu.PERIOD, gain =0.041)

    # Calibration à ajouter dans __init__ ou dans start(), avant la boucle :
    def _calibrate_gyro_bias(self, n_samples=200):
        print("Calibration gyro... Ne pas bouger le robot.")
        samples = []
        for _ in range(n_samples):
            _, _, gyr = self.imu.get_acc_mag_gyr()
            samples.append(gyr)
            time.sleep(0.01)
        bias = np.mean(samples, axis=0)
        print(f"Biais gyro (deg/s): {bias}")
        return bias

    def calibrate_magnetometer(self,imu_sensor):
        wmm = WMM(latitude=46.53787790166582, longitude=6.646301132600495, height=0.64)
        F = wmm.magnetic_elements['F'] / 1000  # nT → µT (≈ 47 µT à Lausanne)
        cal = MagnetometerCalibrator(magnetic_field_strength=F)

        try:
            cal.load_calibration('mag_calibration.json')
        except FileNotFoundError:
            print("No calibration file found. Calibration required.")
            self.psm.screen.termPrintln("Soft + Hard Iron calibration for magnetometer. Turn the IMU in all directions and press ENTER when done.")
            data = []
            while not self.psm.isKeyPressed():
                _,raw_mag,_ = imu_sensor.get_acc_mag_gyr()
                data.append(raw_mag)
                time.sleep(0.1)
            data = np.array(data)
            #np.savetxt('mag_out.txt', data, fmt='%.1f', delimiter=',')
            print("Captured {} magnetometer samples for calibration.".format(len(data)))
            cal.calibrate(data)
            cal.save_calibration('mag_calibration.json')

        return cal

    def run(self):
        self.imu.set_gyro_filter(7)  # Filtre passe-bas pour gyro à 5Hz (valeur empirique, à ajuster selon les besoins)
        #g_bias = self._calibrate_gyro_bias(1000)
        cal = self.calibrate_magnetometer(self.imu)
        acc, raw_mag, _ =self.imu.get_acc_mag_gyr()
        cal_mag = cal.apply_calibration(np.array(raw_mag).reshape(1,3)).flatten() 
        q_vec = am2q(acc, cal_mag, "NED")  # Initial orientation quaternion
        q = Quaternion(q_vec)


        o_yaw_rad = q.to_angles()[2] # Initial yaw angle for reference
        print(f"Initial yaw (mag): {np.degrees(o_yaw_rad):.1f}°")
        
        t0 = time.monotonic()  # capture start time
        while self.state.running:
            acc, raw_mag, gyr = self.imu.get_acc_mag_gyr()
            gyr = np.deg2rad(gyr)
            cal_mag = cal.apply_calibration(np.array(raw_mag).reshape(1,3)).flatten() 
            now = time.monotonic()
            dt = now - t0          # actual elapsed time since last iteration
            t0 = now

            #
            q = self.ahrs.updateMARG(q, gyr, acc, cal_mag, dt=dt)
            #q = self.ahrs.updateIMU(q, gyr, acc, dt=dt)  # fallback if magnetometer data is unreliable

            print("angle from mag with arctan2: {:.1f}°".format(np.degrees(np.arctan2(cal_mag[1], cal_mag[0]))))
            print("angle from compass: ", self.imu.get_heading())

            try:
                current_yaw_rad = q.to_angles()[2]
            except Exception as e:
                print("Error converting quaternion to angles:", e)
                print("Quaternion:", q)
                self.state.running = False
                current_yaw_rad = 0.0
            print("angle from madwick: ", np.degrees(current_yaw_rad))

            yaw_res_rad = current_yaw_rad - o_yaw_rad

            with self.state._pose_lock:
                self.state.pose.angle_rad = yaw_res_rad
                print(f"IMU angle: {math.degrees(yaw_res_rad):.1f}° acc: {acc} | mag: {cal_mag} | gyr: {gyr} | time: {now:.2f} s")
            

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, Imu.PERIOD - elapsed))


class WheelEncoder(Sensor):
    TICKS_PER_REV = 360.0         # ticks par tour moteur PiStorms

    def __init__(self, state: RobotState, motorLeft: PiStormsMotor, motorRight: PiStormsMotor, wheel_diameter: float):
        super().__init__(state)
        self.motorLeft = motorLeft
        self.motorRight = motorRight
        self._total_ticks_left = 0
        self._total_ticks_right = 0
        self.wheel_circumference = math.pi * wheel_diameter
    def run(self):
        while self.state.running:
            ticks_left = self.motorLeft.pos()
            ticks_right = self.motorRight.pos()            

            dl = (ticks_left - self._total_ticks_left) * self.wheel_circumference / WheelEncoder.TICKS_PER_REV
            dr = (ticks_right - self._total_ticks_right) * self.wheel_circumference / WheelEncoder.TICKS_PER_REV

            self._total_ticks_right = ticks_right
            self._total_ticks_left = ticks_left
            
            # Calcul odométrie
            distance = (dl + dr) / 2

            with self.state._pose_lock:
                self.state.pose.x += distance * cos(self.state.pose.angle_rad)
                self.state.pose.y += distance * sin(self.state.pose.angle_rad)
            
            time.sleep(0.02)

class Camera(Sensor):
    CAM_WIDTH = 320
    CAM_HEIGHT = 320
    HFOV = 78.0  # Champ de vision horizontal approximatif du module AI
    
    def __init__(self, state: RobotState):
        super().__init__(state)
            # 1. Model setup
        MODEL_PATH = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"

       
        self.imx500 = IMX500(MODEL_PATH)
        self.picam2 = Picamera2(self.imx500.camera_num)
        intrinsics = self.imx500.network_intrinsics
        intrinsics.update_with_defaults()  # important !

        print(f"Inference rate recommandée: {intrinsics.inference_rate} fps")

        config = self.picam2.create_video_configuration(
            main={"format": "RGB888", "size": (Camera.CAM_WIDTH, Camera.CAM_HEIGHT)},
            controls={"FrameRate": intrinsics.inference_rate},
            buffer_count=12  # ← voir point 2
        )
        self.picam2.configure(config)
        print("Reading coordinates via capture_metadata()...")

    def run(self):
        self.picam2.start()
        labels = self.imx500.network_intrinsics.labels

        while self.state.running:
            metadata = self.picam2.capture_metadata()
            outputs = self.imx500.get_outputs(metadata, add_batch=True)

            if outputs is not None:
                # For SSD: [0]=boxes, [1]=scores, [2]=classes
                boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]

                for i in range(len(scores)):
                    if scores[i] > 0.5:  # Confidence threshold
                        box = boxes[i]
                        
                        # Map coordinates (according to default IMX500 config)
                        x, y, w, h = self.imx500.convert_inference_coords(box, metadata, self.picam2)
                        
                        label = labels[int(classes[i])] if labels else int(classes[i])
                        # Clean print for your robotic use
                        #print(f"OBJ: {label} | Conf: {scores[i]:.2f} | timestamp: {time.monotonic()}")
                        if label == "person":
                            #print(f"X: {int(x)} Y: {int(y)} W: {int(w)} H: {int(h)}")
                            #print(f"middle X: {int(x + (w//2) - Camera.CAM_WIDTH // 2 )} middle Y: {int(y + (h//2))- Camera.CAM_HEIGHT // 2 }")
                            self.state.targetPose.pose_timestamp = time.monotonic()
                            self.state.targetPose.x = int(x + (w//2) - Camera.CAM_WIDTH // 2)  # Centered at 0
                   
                            # 1. Calcule le ratio de position (-1.0 à gauche, 0 au centre, +1.0 à droite)
                            ratio = self.state.targetPose.x / (Camera.CAM_WIDTH / 2)

                            # 2. Convertis ce ratio en degrés réels selon le FOV de la caméra
                            # Si l'IMX500 a un HFOV de 78°, alors la déviation max est de 39°
                            deviation_deg = ratio * (Camera.HFOV / 2)

                 
                            self.state.targetPose.angle_rad = math.radians(deviation_deg)

                
                        # else:
                        #     print(f"OBJ: {label} | Conf: {scores[i]:.2f} | X: {int(x)} Y: {int(y)} W: {int(w)} H: {int(h)}")

        self.picam2.stop()

class Lidar(Sensor):
    def __init__(self, state: RobotState, portName : str = '/dev/ttyUSB0'):
        super().__init__(state)
        self.portName = portName
        self.lidar = RPLidar(self.portName)

    def run(self):
        
        while self.state.running:
            for _, scan in enumerate(self.lidar.iter_scans()):
                if not self.state.running:
                    break

                min_target_distance = float('inf')
                min_front_distance = float('inf')

                # Convert camera angle to lidar space once per rotation
                lidar_target_angle = self.state.targetPose.get_angle_deg() % 360

                for (quality, angle, distance) in scan:
                    #print(f"angle | distance {angle:6.1f}° | {distance:5.1f} mm")

                    if quality == 0 or distance <= 0:
                        continue

                    # --- Target distance (person detected by camera) ---
                    # Angular difference, accounting for wrap-around
                    angle_diff = abs((angle - lidar_target_angle + 180) % 360 - 180)
                    if angle_diff <= 5 and distance < 2000:
                        min_target_distance = min(min_target_distance, distance)

                    # --- Front obstacle detection (±10° arc around 0°) ---
                    is_front = angle <= 10 or angle >= 350
                    if is_front and distance < 300:
                        min_front_distance = min(min_front_distance, distance)

                # --- Write to state once per full rotation ---
                if min_target_distance != float('inf'):
                    self.state.targetDistance = min_target_distance
                else:
                    self.state.targetDistance = None

                if min_front_distance != float('inf'):
                    self.state.frontObstacleDistance = min_front_distance
                else:
                    self.state.frontObstacleDistance = None
                if self.state.targetDistance is not None and self.state.frontObstacleDistance is not None:
                    print(f"LIDAR scan: Target dist={self.state.targetDistance:.1f} mm | Front obstacle={self.state.frontObstacleDistance:.1f} mm | Target angle={self.state.targetPose.get_angle_deg():.1f}°")


        self.lidar.stop()
        self.lidar.disconnect()
        