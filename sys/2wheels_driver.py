import math
import time
import threading
import numpy as np
from PiStorms import PiStormsMotor

WHEEL_BASE        = 0.17
WHEEL_DIAMETER    = 0.0942
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
TICKS_PER_REV     = 360.0
ARRIVAL_TOLERANCE = 0.05        # 5cm
HEADING_TOLERANCE = math.radians(2)  # 2°

class Driver:
    def __init__(self, state: RobotState, motorLeft: PiStormsMotor, motorRight: PiStormsMotor):
        self.state      = state
        self.motorLeft  = motorLeft
        self.motorRight = motorRight

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def goto(self, x: float, y: float, 
             accel_time: float = 1.0, 
             max_speed: int = 50,
             blocking: bool = True):
        """
        Drive the robot to (x, y) in world coordinates.
        - accel_time: seconds to ramp from 0 to max_speed (and back)
        - max_speed:  top speed (1-100)
        - blocking:   if False, runs in a background thread
        """
        if blocking:
            self._goto(x, y, accel_time, max_speed)
        else:
            t = threading.Thread(target=self._goto, 
                                 args=(x, y, accel_time, max_speed),
                                 daemon=True)
            t.start()

    def stop(self):
        self.motorLeft.brake()
        self.motorRight.brake()

    # ------------------------------------------------------------------ #
    #  Internal implementation                                            #
    # ------------------------------------------------------------------ #

    def _goto(self, tx: float, ty: float, accel_time: float, max_speed: int):
        # ---- 1. Turn phase ------------------------------------------- #
        self._turn_to_target(tx, ty)

        # ---- 2. Drive phase ------------------------------------------ #
        self._drive_to_target(tx, ty, accel_time, max_speed)

    # ------------------------------------------------------------------ #
    #  Turn in place to face target                                       #
    # ------------------------------------------------------------------ #

    def _turn_to_target(self, tx: float, ty: float):
        TURN_SPEED = 30   # fixed moderate speed for turning

        while self.state.running:
            with self.state._pose_lock:
                cx, cy, current_angle = (self.state.pose.x, 
                                         self.state.pose.y, 
                                         self.state.pose.angle_rad)

            target_angle  = math.atan2(ty - cy, tx - cx)
            angle_error   = self._normalize_angle(target_angle - current_angle)

            if abs(angle_error) <= HEADING_TOLERANCE:
                break

            # compute motor degrees for this rotation
            arc_length  = WHEEL_BASE * abs(angle_error)
            motor_degs  = (arc_length / WHEEL_CIRCUMFERENCE) * 360.0

            if angle_error > 0:
                # turn right: left forward, right backward
                # runDegs is blocking by default so we get fresh pose after
                self.motorLeft.runDegs(motor_degs,  TURN_SPEED,  True, False)
                self.motorRight.runDegs(motor_degs, -TURN_SPEED, True, False)
            else:
                # turn left
                self.motorLeft.runDegs(motor_degs,  -TURN_SPEED, True, False)
                self.motorRight.runDegs(motor_degs,  TURN_SPEED, True, False)

            time.sleep(0.02)  # let IMU update

    # ------------------------------------------------------------------ #
    #  Drive straight with heading PID correction                        #
    # ------------------------------------------------------------------ #

    def _drive_to_target(self, tx: float, ty: float, 
                         accel_time: float, max_speed: int):
        # PID gains — tune these
        Kp, Ki, Kd = 40.0, 0.0, 5.0

        integral    = 0.0
        last_error  = 0.0
        t0          = time.monotonic()
        last_t      = t0

        while self.state.running:
            now = time.monotonic()
            dt  = now - last_t
            last_t = now
            elapsed = now - t0

            with self.state._pose_lock:
                cx, cy, current_angle = (self.state.pose.x,
                                         self.state.pose.y,
                                         self.state.pose.angle_rad)

            # distance and heading to target
            dx       = tx - cx
            dy       = ty - cy
            distance = math.hypot(dx, dy)

            if distance <= ARRIVAL_TOLERANCE:
                break

            target_angle  = math.atan2(dy, dx)
            heading_error = self._normalize_angle(target_angle - current_angle)

            # PID
            if dt > 0:
                integral   += heading_error * dt
                derivative  = (heading_error - last_error) / dt
            else:
                derivative  = 0.0
            last_error = heading_error
            correction = Kp * heading_error + Ki * integral + Kd * derivative
            correction = max(-max_speed, min(max_speed, correction))  # clamp

            # speed with acceleration / deceleration ramp
            base_speed = self._ramp_speed(elapsed, distance, 
                                          accel_time, max_speed)

            # negative = forward on your robot
            left_speed  = -(base_speed + correction)
            right_speed = -(base_speed - correction)

            # clamp to valid range
            left_speed  = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))

            self.motorLeft.setSpeed(int(left_speed),  blocking=False)
            self.motorRight.setSpeed(int(right_speed), blocking=False)

            time.sleep(0.02)

        self.stop()

    # ------------------------------------------------------------------ #
    #  Helpers                                                            #
    # ------------------------------------------------------------------ #

    def _ramp_speed(self, elapsed: float, distance: float, 
                    accel_time: float, max_speed: int) -> float:
        """
        Trapezoidal speed profile:
        - ramp up over accel_time seconds
        - ramp down when distance < decel_distance
        """
        # deceleration distance = distance covered during ramp at max_speed
        decel_distance = 0.5 * max_speed * accel_time * 0.01  # rough estimate

        accel_factor = min(1.0, elapsed / accel_time) if accel_time > 0 else 1.0
        decel_factor = min(1.0, distance / decel_distance) if decel_distance > 0 else 1.0

        return max_speed * min(accel_factor, decel_factor)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle