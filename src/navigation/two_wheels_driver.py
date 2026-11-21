import math
import time
import threading
import numpy as np

from .drive_base import DriveBase
from .robot_state import RobotState

ARRIVAL_TOLERANCE = 0.05        # 5cm
HEADING_TOLERANCE = math.radians(2)  # 2°
MIN_DRIVE_SPEED   = 18          # motor deadband — below this the wheels stall
TURN_MOVE_TIMEOUT = 500         # waitUntilNotBusy units (10 ms) → 5 s per move
MAX_TURN_ATTEMPTS = 10
DRIVE_TIMEOUT     = 30.0        # s

class Driver:
    def __init__(self, state: RobotState, drive: DriveBase):
        """drive: the robot's wheels (ports, directions and sizes from the
        robot profile)."""
        self.state = state
        self.drive: DriveBase = drive

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
        # at once: _drive_to_target has already slowed down (_ramp_speed)
        self.drive.brake(acceleration=None)

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

        for _ in range(MAX_TURN_ATTEMPTS):
            if not self.state.running:
                return
            with self.state._pose_lock:
                cx, cy, current_angle = (self.state.pose.x,
                                         self.state.pose.y,
                                         self.state.pose.angle_rad)

            target_angle  = math.atan2(ty - cy, tx - cx)
            angle_error   = self._normalize_angle(target_angle - current_angle)

            if abs(angle_error) <= HEADING_TOLERANCE:
                return

            # pivot turn: each wheel travels half the wheel base times the angle
            profile = self.drive.profile
            arc_length  = (profile.wheel_base / 2.0) * abs(angle_error)
            motor_degs  = int(round((arc_length / profile.wheel_circumference) * 360.0))
            if motor_degs == 0:
                return

            # pose angle is CCW-positive: to turn left, the left wheel rolls
            # backward and the right one forward. The motor speed of each
            # wheel carries the profile's forward sign; the degrees carry the
            # same sign as the speed so the move direction is unambiguous
            # whichever of the two the firmware reads it from.
            direction = 1 if angle_error > 0 else -1
            left_motor_speed  = -direction * TURN_SPEED * self.drive.left_sign
            right_motor_speed =  direction * TURN_SPEED * self.drive.right_sign
            self.drive.left.runDegs(math.copysign(motor_degs, left_motor_speed),
                                    left_motor_speed, True, False)
            self.drive.right.runDegs(math.copysign(motor_degs, right_motor_speed),
                                     right_motor_speed, True, False)

            # runDegs only writes the setpoint — wait for both moves to finish
            # so the next iteration sees the post-turn pose
            self.drive.left.waitUntilNotBusy(TURN_MOVE_TIMEOUT)
            self.drive.right.waitUntilNotBusy(TURN_MOVE_TIMEOUT)

            time.sleep(0.02)  # let IMU update
        print(f"Driver: heading not reached after {MAX_TURN_ATTEMPTS} turn attempts")

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
            if elapsed > DRIVE_TIMEOUT:
                print(f"Driver: target not reached after {DRIVE_TIMEOUT:.0f}s — stopping")
                break

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

   
            # wheel speeds, positive = forward; heading_error > 0
            # (CCW-positive) → right wheel faster → turn left
            left_speed  = base_speed - correction
            right_speed = base_speed + correction

            # clamp to valid range
            left_speed  = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))

            # no motor ramp: base_speed is already ramped (_ramp_speed), and
            # this loop sends a new speed every 20 ms
            self.drive.drive(int(left_speed), int(right_speed), acceleration=None, blocking=False)

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

        # never drop below the motor deadband, or the robot stalls short of
        # the target and never satisfies ARRIVAL_TOLERANCE
        return max(MIN_DRIVE_SPEED, max_speed * min(accel_factor, decel_factor))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle