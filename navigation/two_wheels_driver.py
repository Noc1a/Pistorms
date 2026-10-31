import math
import time
import threading
import numpy as np
import os, sys

_nav_dir = os.path.dirname(os.path.abspath(__file__))
_sys_dir = os.path.join(os.path.dirname(_nav_dir), 'sys')
for _d in [_nav_dir, _sys_dir]:
    if _d not in sys.path: sys.path.insert(0, _d)

from PiStorms import PiStormsMotor
from robot_state import RobotState

WHEEL_BASE        = 0.17
WHEEL_DIAMETER    = 0.0942
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
TICKS_PER_REV     = 360.0
ARRIVAL_TOLERANCE = 0.05        # 5cm
HEADING_TOLERANCE = math.radians(2)  # 2°
MIN_DRIVE_SPEED   = 18          # motor deadband — below this the wheels stall
TURN_MOVE_TIMEOUT = 500         # waitUntilNotBusy units (10 ms) → 5 s per move
MAX_TURN_ATTEMPTS = 10
DRIVE_TIMEOUT     = 30.0        # s

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
            arc_length  = (WHEEL_BASE / 2.0) * abs(angle_error)
            motor_degs  = int(round((arc_length / WHEEL_CIRCUMFERENCE) * 360.0))
            if motor_degs == 0:
                return

            # pose angle is CCW-positive and setSpeed(+N) drives a wheel backward:
            # s > 0 → left backward, right forward → turn left (CCW).
            # Degrees carry the same sign as the speed so the move direction is
            # unambiguous whichever of the two the firmware reads it from.
            s = TURN_SPEED if angle_error > 0 else -TURN_SPEED
            self.motorLeft.runDegs(math.copysign(motor_degs,  s),  s, True, False)
            self.motorRight.runDegs(math.copysign(motor_degs, -s), -s, True, False)

            # runDegs only writes the setpoint — wait for both moves to finish
            # so the next iteration sees the post-turn pose
            self.motorLeft.waitUntilNotBusy(TURN_MOVE_TIMEOUT)
            self.motorRight.waitUntilNotBusy(TURN_MOVE_TIMEOUT)

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

   
            # negative = forward on this robot's motor wiring; heading_error > 0
            # (CCW-positive) → right wheel faster → turn left
            left_speed  = -(base_speed - correction)
            right_speed = -(base_speed + correction)

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

        # never drop below the motor deadband, or the robot stalls short of
        # the target and never satisfies ARRIVAL_TOLERANCE
        return max(MIN_DRIVE_SPEED, max_speed * min(accel_factor, decel_factor))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle