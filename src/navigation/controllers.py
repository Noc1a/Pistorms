"""
Mode controllers — one per RobotState.mode. The main loop dispatches step() each
tick (~20 Hz) to the active controller. Switching mode triggers stop() on the
outgoing controller so motors brake before the new one takes over.

Motor wiring convention: setSpeed(-N) drives forward on this robot.
"""
from abc import ABC, abstractmethod
import math
import time

from pistorms import RampedMotor
from .robot_state import RobotState


# ════════════════════════════════════════════════════════════════════════════
class Controller(ABC):
    """Wraps raw motors in RampedMotor so no controller can apply a sudden
    voltage step (which browns out the Pi). The wrapper's rate is loose enough
    that controller-level kinematic ramping (Manual/Follow) passes through;
    GoHome — which commands target speeds directly — gets ramping for free."""

    def __init__(self, state: RobotState, motorLeft, motorRight):
        self.state = state
        self.motorLeft  = motorLeft  if isinstance(motorLeft,  RampedMotor) else RampedMotor(motorLeft)
        self.motorRight = motorRight if isinstance(motorRight, RampedMotor) else RampedMotor(motorRight)

    @abstractmethod
    def step(self):
        """Called repeatedly while this controller is active. Must be non-blocking."""

    def stop(self):
        self.motorLeft.brake()
        self.motorRight.brake()

    # ── shared helpers ──
    @staticmethod
    def _ramp(current, target, max_step):
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    @staticmethod
    def _scale_motors(left, right, max_motor):
        m = max(abs(left), abs(right))
        if m > max_motor:
            f = max_motor / m
            return left * f, right * f
        return left, right

    @staticmethod
    def _apply_min_speed(s, mn):
        if 0 < s < mn:  return mn
        if -mn < s < 0: return -mn
        return s

    @staticmethod
    def _normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


# ════════════════════════════════════════════════════════════════════════════
class ManualController(Controller):
    """Drives motors from state.manual_cmd = {'fwd': -1..1, 'turn': -1..1, 'speed': 0..100}.

    Release feels instant: cur_base/cur_diff snap to 0 when target is 0 so the
    robot stops turning the moment the key is released. Growing magnitude ramps
    with ACC (voltage-safe soft start); shrinking magnitude or direction reversal
    also snaps — the wrapper's ACC=8 still guards true zero-to-max wheel jumps,
    but the wrapper's loose DEC lets these snaps through without extra clamping.
    """
    MAX_STEP_ACC = 4      # smooth acceleration for taps
    TURN_WEIGHT  = 0.6    # fraction of speed used for turn component

    def __init__(self, state, motorLeft, motorRight):
        super().__init__(state, motorLeft, motorRight)
        self._cur_base = 0.0
        self._cur_diff = 0.0
        self._braking  = False

    def _step_component(self, target, cur):
        # Snap on release or magnitude shrink → responsive feel.
        # ACC ramp only when growing magnitude in same direction (voltage-safe).
        if target == 0.0:
            return 0.0
        if cur * target < 0.0:            # direction reversal: snap through
            return target
        if abs(target) < abs(cur):        # slowing down: snap
            return target
        delta = target - cur              # same sign, growing: ACC ramp
        if abs(delta) <= self.MAX_STEP_ACC:
            return target
        return cur + math.copysign(self.MAX_STEP_ACC, delta)

    def step(self):
        cmd  = self.state.manual_cmd
        spd  = max(0, min(100, cmd.get('speed', 0)))
        fwd  = max(-1.0, min(1.0, cmd.get('fwd',  0.0)))
        turn = max(-1.0, min(1.0, cmd.get('turn', 0.0)))

        target_base = fwd  * spd
        target_diff = turn * spd * self.TURN_WEIGHT

        self._cur_base = self._step_component(target_base, self._cur_base)
        self._cur_diff = self._step_component(target_diff, self._cur_diff)

        left  = self._cur_base + self._cur_diff
        right = self._cur_base - self._cur_diff
        # fwd+turn together can reach ±160; scale both wheels down so the
        # curve keeps its shape instead of saturating one wheel.
        left, right = self._scale_motors(left, right, 100)

        # Log only when something changes to avoid 20 Hz spam.
        sig = (fwd, turn, spd, self._cur_base, self._cur_diff, left, right)
        if sig != getattr(self, '_last_log_sig', None):
            print(f"[MAN] fwd={fwd:+.0f} turn={turn:+.0f} spd={spd:3d} | "
                  f"base={self._cur_base:+6.1f} diff={self._cur_diff:+6.1f} | "
                  f"L={left:+6.1f} R={right:+6.1f}", flush=True)
            self._last_log_sig = sig

        # Brake actively on full release: setSpeed(0) coasts, letting angular
        # momentum from a preceding turn keep spinning the robot. brake() holds
        # position — critical to kill spin before the next command.
        if left == 0 and right == 0:
            if not self._braking:
                self.motorLeft.brakeSync()
                self.motorRight.brake()
                self._braking = True
        else:
            self._braking = False
            # Ramp both wheels then fire the two speed registers atomically via
            # the bank-level 'S' command so neither wheel starts before the other.
            # setSpeedSyncPair handles the tacho-setpoint reset internally on
            # direction change, so no explicit reset call is needed here.
            l = self.motorLeft._advance(-int(left))
            r = self.motorRight._advance(-int(right))
            self.motorLeft._motor.psmotor.setSpeedSyncPair(l, r)

    def stop(self):
        self._cur_base = self._cur_diff = 0.0
        super().stop()


# ════════════════════════════════════════════════════════════════════════════
class FollowObjectController(Controller):
    """Follow the detection of class state.follow_target.
    If target lost > LOSS_TIMEOUT seconds, rotate slowly to search."""

    LOSS_TIMEOUT   = 1.5      # s
    SEARCH_TURN    = 12       # motor units while searching
    KP_TURN        = 0.2
    KP_DIST        = 2.5
    MIN_SPEED      = 3
    MAX_TURN       = 40
    MAX_BASE       = 40
    MAX_MOTOR      = 50
    MAX_STEP_ACC   = 2
    MAX_STEP_DEC   = 4
    MAX_STEP_TURN  = 6
    DEAD_ZONE_X    = 5
    DEAD_ZONE_DIST = 5

    def __init__(self, state, motorLeft, motorRight):
        super().__init__(state, motorLeft, motorRight)
        self._cur_base = 0.0
        self._cur_turn = 0.0
        self._moving   = False

    def step(self):
        tgt = self.state.targetPose
        ts  = tgt.pose_timestamp
        target_visible = ts is not None and (time.monotonic() - ts) < self.LOSS_TIMEOUT

        if not target_visible:
            # search: drift base→0, hold a slow turn
            self._cur_base = self._ramp(self._cur_base, 0.0, self.MAX_STEP_DEC)
            self._cur_turn = self._ramp(self._cur_turn, self.SEARCH_TURN, 2)
            left  = self._cur_base + self._cur_turn
            right = self._cur_base - self._cur_turn
            self.motorLeft.setSpeed(-int(left))
            self.motorRight.setSpeed(-int(right))
            self._moving = True
            return

        x_error = tgt.x
        if self.state.frontObstacleDistance is not None:
            distance_error = self.state.frontObstacleDistance * 0.1 - 50
        elif self.state.targetDistance is not None:
            distance_error = self.state.targetDistance * 0.1 - 80
        else:
            distance_error = 0.0

        target_base = max(-self.MAX_BASE, min(self.MAX_BASE, distance_error * self.KP_DIST))
        target_turn = max(-self.MAX_TURN, min(self.MAX_TURN, x_error * self.KP_TURN))

        if abs(x_error) > self.DEAD_ZONE_X or abs(distance_error) > self.DEAD_ZONE_DIST:
            self._cur_base = self._ramp(self._cur_base, target_base, self.MAX_STEP_ACC)
            self._cur_turn = self._ramp(self._cur_turn, target_turn, self.MAX_STEP_TURN)
            left  = self._cur_base + self._cur_turn
            right = self._cur_base - self._cur_turn
            left, right = self._scale_motors(left, right, self.MAX_MOTOR)
            left  = self._apply_min_speed(left,  self.MIN_SPEED)
            right = self._apply_min_speed(right, self.MIN_SPEED)
            self.motorLeft.setSpeed(-int(left))
            self.motorRight.setSpeed(-int(right))
            self._moving = True
        elif self._moving:
            self._cur_base = self._ramp(self._cur_base, 0.0, self.MAX_STEP_DEC)
            self._cur_turn = self._ramp(self._cur_turn, 0.0, self.MAX_STEP_DEC)
            left  = self._cur_base + self._cur_turn
            right = self._cur_base - self._cur_turn
            if abs(left) < 1 and abs(right) < 1:
                self.motorLeft.brake()
                self.motorRight.brake()
                self._cur_base = self._cur_turn = 0.0
                self._moving = False
            else:
                self.motorLeft.setSpeed(-int(left))
                self.motorRight.setSpeed(-int(right))

    def stop(self):
        self._cur_base = self._cur_turn = 0.0
        self._moving = False
        super().stop()


# ════════════════════════════════════════════════════════════════════════════
class GoHomeController(Controller):
    """Drive back to (0,0) using straight-line + reactive lidar avoidance.

    Uses Odometry's frame: angle CCW-positive, so a positive heading error
    means "turn left" (right wheel faster) in every phase.

    State machine:
      TURN        — rotate in place to face origin
      DRIVE       — drive toward origin with heading correction; abort if obstacle
      AVOID       — sidestep around obstacle (turn aside + drive forward briefly)
      FINAL_TURN  — at origin, rotate to original heading (0 rad)
                    then hand back to manual mode."""

    PHASE_TURN       = 'turn'
    PHASE_DRIVE      = 'drive'
    PHASE_AVOID      = 'avoid'
    PHASE_FINAL_TURN = 'final_turn'

    HEADING_TOLERANCE  = math.radians(3)
    ARRIVAL_TOLERANCE  = 0.10        # m
    OBSTACLE_THRESHOLD = 350         # mm
    DRIVE_SPEED        = 25
    TURN_SPEED         = 22
    AVOID_DURATION     = 1.5         # s of forward after sidestep turn
    HEADING_KP         = 25.0        # P-gain for heading correction in DRIVE

    def __init__(self, state, motorLeft, motorRight):
        super().__init__(state, motorLeft, motorRight)
        self._phase = self.PHASE_TURN
        self._avoid_until = 0.0
        self._avoid_dir   = 1        # +1 = turn left, -1 = turn right

    def reset(self):
        self._phase = self.PHASE_TURN

    def step(self):
        with self.state._pose_lock:
            x, y, heading = self.state.pose.x, self.state.pose.y, self.state.pose.angle_rad

        if self._phase == self.PHASE_TURN:
            self._step_turn(x, y, heading)
        elif self._phase == self.PHASE_DRIVE:
            self._step_drive(x, y, heading)
        elif self._phase == self.PHASE_AVOID:
            self._step_avoid()
        elif self._phase == self.PHASE_FINAL_TURN:
            self._step_final_turn(heading)

    # ── phases ──
    def _step_turn(self, x, y, heading):
        dx, dy = -x, -y
        if math.hypot(dx, dy) <= self.ARRIVAL_TOLERANCE:
            self._phase = self.PHASE_FINAL_TURN
            return
        err = self._normalize_angle(math.atan2(dy, dx) - heading)
        if abs(err) <= self.HEADING_TOLERANCE:
            self._phase = self.PHASE_DRIVE
            return
        s = self.TURN_SPEED if err > 0 else -self.TURN_SPEED
        self.motorLeft.setSpeed(-int(-s))   # s > 0: left backward, right forward → CCW (left)
        self.motorRight.setSpeed(-int(s))

    def _step_drive(self, x, y, heading):
        # obstacle ahead?
        front = self.state.frontObstacleDistance
        if front is not None and front < self.OBSTACLE_THRESHOLD:
            self._avoid_dir   = self._pick_avoidance_side()
            self._avoid_until = time.monotonic() + self.AVOID_DURATION
            self._phase = self.PHASE_AVOID
            return

        dx, dy = -x, -y
        if math.hypot(dx, dy) <= self.ARRIVAL_TOLERANCE:
            self._phase = self.PHASE_FINAL_TURN
            return

        err = self._normalize_angle(math.atan2(dy, dx) - heading)
        correction = max(-self.DRIVE_SPEED, min(self.DRIVE_SPEED, self.HEADING_KP * err))
        left  = self.DRIVE_SPEED - correction   # err > 0 → right wheel faster → turn left
        right = self.DRIVE_SPEED + correction
        self.motorLeft.setSpeed(-int(left))
        self.motorRight.setSpeed(-int(right))

    def _step_avoid(self):
        if time.monotonic() < self._avoid_until:
            # roll forward while veering away from obstacle
            turn = self._avoid_dir * self.TURN_SPEED * 0.7
            left  = self.DRIVE_SPEED - turn     # _avoid_dir +1 → right wheel faster → turn left
            right = self.DRIVE_SPEED + turn
            self.motorLeft.setSpeed(-int(left))
            self.motorRight.setSpeed(-int(right))
        else:
            self._phase = self.PHASE_TURN     # re-aim at home

    def _step_final_turn(self, heading):
        err = self._normalize_angle(0.0 - heading)
        if abs(err) <= self.HEADING_TOLERANCE:
            self.motorLeft.brake()
            self.motorRight.brake()
            self._phase = self.PHASE_TURN     # reset for next invocation
            self.state.mode = RobotState.MODE_MANUAL
            return
        s = self.TURN_SPEED if err > 0 else -self.TURN_SPEED
        self.motorLeft.setSpeed(-int(-s))
        self.motorRight.setSpeed(-int(s))

    # ── helpers ──
    def _pick_avoidance_side(self):
        """Scan lidar arcs 30–90° (right) vs 270–330° (left); turn toward the freer side."""
        left_min = right_min = float('inf')
        for q, a, d in self.state.last_scan:
            if q == 0 or d <= 0: continue
            if 30 <= a <= 90:
                right_min = min(right_min, d)
            elif 270 <= a <= 330:
                left_min = min(left_min, d)
        # if both ~equal/no data, default to left
        return 1 if left_min >= right_min else -1

    def stop(self):
        self._phase = self.PHASE_TURN
        super().stop()
