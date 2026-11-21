"""
Mode controllers — one per RobotState.mode. The main loop dispatches step() each
tick (~20 Hz) to the active controller. Switching mode triggers stop() on the
outgoing controller so motors brake before the new one takes over.

Wheel speeds are forward-positive percentages: the DriveBase turns them into
motor commands for this model (robot profile).
"""
from abc import ABC, abstractmethod
import math
import time
from typing import Optional

from . import collision
from .drive_base import DriveBase
from .path_planner import PathPlanner
from .robot_state import RobotState


# ════════════════════════════════════════════════════════════════════════════
class Controller(ABC):
    """Every controller drives the wheels through _drive(): whatever speeds
    it asks for, the wheels ramp to them (motor.py: both together, in %/s,
    through 0 one percent at a time on a reversal), so the robot doesn't
    jump and no current peak browns out the Pi. _drive() is also the
    collision guard (navigation.collision): never drive the wheels any other
    way while moving."""

    ACCELERATION = 200   # %/s, every speed change of the wheels (50 % → 0 in 0.25 s)
    MIN_START = 18       # %: below, a wheel doesn't move (static friction)

    def __init__(self, state: RobotState, drive: DriveBase) -> None:
        self.state = state
        self.drive: DriveBase = drive
        self._sent_speeds: Optional[tuple[int, int]] = None

    @abstractmethod
    def step(self):
        """Called repeatedly while this controller is active. Must be non-blocking."""

    def stop(self):
        """Ramp down, then brake: mode change, arrival. (Braking at once made
        the robot jump; the safety stops — collision guard, end of the
        program — still brake at once.)"""
        self.drive.brake(acceleration=self.ACCELERATION, blocking=False)
        self._sent_speeds = None

    def _drive(self, left: float, right: float, min_speed: Optional[float] = None) -> None:
        """Ramp both wheels to these speeds (−100…100, positive =
        forward). The ramp runs in the background: it is sent only when the
        speeds change. (0, 0) ramps down, then brakes (stops a spin).
        A wheel asked to turn gets at least min_speed (default MIN_START,
        what starts it; less is enough for a wheel already turning).

        Collision guard, checked at every call: driving towards an obstacle
        in the robot's corridor (closer than its stopping distance) is
        removed from the command; turning and moving away stay. When that
        stops a motion towards the obstacle, the wheels brake at once."""
        left, right, blocked = self._guarded(left, right)
        if min_speed is None:
            min_speed = self.MIN_START
        speeds = (round(self._apply_min_speed(left, min_speed)),
                  round(self._apply_min_speed(right, min_speed)))
        was_driving_into_it = blocked is not None and blocked != self.state.blocked
        self._set_blocked(blocked)
        if was_driving_into_it:
            self.drive.brake(acceleration=None)     # now: no room left for a ramp
            self._sent_speeds = (0, 0)
        if speeds == self._sent_speeds:
            return
        self._sent_speeds = speeds
        if speeds == (0, 0):
            self.drive.brake(acceleration=self.ACCELERATION, blocking=False)
        else:
            self.drive.drive(*speeds, acceleration=self.ACCELERATION, blocking=False)

    def _guarded(self, left: float, right: float) -> tuple[float, float, Optional[str]]:
        """The command without its part towards an obstacle: (left, right,
        'front' / 'rear' / None). The command splits into a forward part
        (both wheels alike) and a turn part (wheels opposite)."""
        forward = (left + right) / 2
        turn_left = (right - left) / 2
        blocked = self._blocked_direction(forward)
        if blocked is not None:
            forward = 0.0
        return forward - turn_left, forward + turn_left, blocked

    def _blocked_direction(self, forward: float) -> Optional[str]:
        """'front' if driving forward would hit something, 'rear' backward,
        else None. Clearances older than MAX_SCAN_AGE_SECS (LiDAR stopped)
        block both ways."""
        if forward == 0:
            return None
        clearances_age = time.monotonic() - self.state.clearance_time
        trusted = clearances_age <= collision.MAX_SCAN_AGE_SECS
        # The faster of the measured and the commanded speed: with the measured
        # one alone, a robot stopped near a wall would start again, creeping on
        speed = max(abs(self.state.speed_mps), abs(forward) * collision.SPEED_MPS_PER_PERCENT)
        room_needed = collision.stop_distance(speed)
        if forward > 0 and (not trusted or self.state.front_clearance_m < room_needed):
            return 'front'
        if forward < 0 and (not trusted or self.state.rear_clearance_m < room_needed):
            return 'rear'
        return None

    def _set_blocked(self, blocked: Optional[str]) -> None:
        if blocked == self.state.blocked:
            return
        if blocked is None:
            print("Collision guard: free again", flush=True)
        elif time.monotonic() - self.state.clearance_time > collision.MAX_SCAN_AGE_SECS:
            print(f"Collision guard: no recent LiDAR scan — {blocked} blocked", flush=True)
        else:
            clearance = self.state.front_clearance_m if blocked == 'front' else self.state.rear_clearance_m
            print(f"Collision guard: obstacle {clearance * 100:.0f} cm {blocked} — stopped", flush=True)
        self.state.blocked = blocked

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

    cur_base/cur_diff follow the keys: they snap to 0 on release and on a
    reversal, and ramp with MAX_STEP_ACC when growing. The wheels themselves
    always follow them through _drive() (ramped, both together), which on
    release ramps down then brakes, so that a turn stops.
    """
    MAX_STEP_ACC = 4      # smooth acceleration for taps
    TURN_WEIGHT  = 0.6    # fraction of speed used for turn component

    def __init__(self, state: RobotState, drive: DriveBase) -> None:
        super().__init__(state, drive)
        self._cur_base = 0.0
        self._cur_diff = 0.0

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

        # On full release the wheels ramp down to 0, then brake: the brake
        # holds position and kills the spin of a preceding turn
        self._drive(left, right)

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

    def __init__(self, state: RobotState, drive: DriveBase) -> None:
        super().__init__(state, drive)
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
            self._drive(left, right)
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
            self._drive(left, right)
            self._moving = True
        elif self._moving:
            self._cur_base = self._ramp(self._cur_base, 0.0, self.MAX_STEP_DEC)
            self._cur_turn = self._ramp(self._cur_turn, 0.0, self.MAX_STEP_DEC)
            left  = self._cur_base + self._cur_turn
            right = self._cur_base - self._cur_turn
            if abs(left) < 1 and abs(right) < 1:
                self._drive(0, 0)     # ramps down, then brakes at 0
                self._cur_base = self._cur_turn = 0.0
                self._moving = False
            else:
                self._drive(left, right)

    def stop(self):
        self._cur_base = self._cur_turn = 0.0
        self._moving = False
        super().stop()


# ════════════════════════════════════════════════════════════════════════════
class NavigateController(Controller):
    """Drive to state.nav_goal = (x, y, final heading or None) along the way
    the PathPlanner finds on the SLAM map (navigation.path_planner): around
    the walls, and around the obstacles of the latest LiDAR scan. The way is
    planned again every REPLAN_SECS, and at once when the collision guard
    stops the robot.

    Uses Odometry's frame (corrected by the SLAM): angle CCW-positive, so a
    positive heading error means "turn left" (right wheel faster) in every phase.

    Phases:
      TURN        — rotate in place to face the next waypoint (more than
                    TURN_IN_PLACE_DEG away)
      DRIVE       — drive towards it with heading correction; a waypoint
                    reached within ARRIVAL_TOLERANCE gives way to the next,
                    without stopping; slower over the last SLOW_DOWN_M
      FINAL_TURN  — at the goal, rotate to its final heading, if it has one
    then ramp down, brake and hand back to manual mode."""

    PHASE_TURN       = 'turn'
    PHASE_DRIVE      = 'drive'
    PHASE_FINAL_TURN = 'final_turn'

    HEADING_TOLERANCE  = math.radians(3)
    # Turning in place: TURN_SPEED (~120 °/s) went 20–50° past the heading
    # and swung back and forth (live_20261107_220647, final turn). Slower
    # over the last TURN_SLOW_DEG, and stopped TURN_COAST_DEG early: the
    # robot keeps turning while the brake ramps down and the gyro reports
    # ~35 ms late (testSlam.MovingRobot.turn, same values: half turns of
    # 176–179°)
    TURN_SLOW_SPEED    = 10          # motor %, enough for wheels already turning
    TURN_SLOW_DEG      = 20
    TURN_COAST_DEG     = 8
    TURN_IN_PLACE_DEG  = 30          # a waypoint farther off than this: turn in place first
    ARRIVAL_TOLERANCE  = 0.10        # m
    SLOW_DOWN_M        = 0.30        # slower over the last of the way (down to MIN_START)
    DRIVE_SPEED        = 25
    TURN_SPEED         = 22
    HEADING_KP         = 25.0        # P-gain for heading correction in DRIVE

    def __init__(self, state: RobotState, drive: DriveBase) -> None:
        super().__init__(state, drive)
        self.planner: PathPlanner = PathPlanner(state)
        self._phase: str = self.PHASE_DRIVE
        self._goal: Optional[tuple[float, float, Optional[float]]] = None   # the goal being driven to
        self._path: Optional[list[tuple[float, float]]] = None             # the waypoints followed
        self._waypoint: int = 0                                            # index of the next one

    def goal(self) -> Optional[tuple[float, float, Optional[float]]]:
        """Where to go: the web page's click (state.nav_goal)."""
        return self.state.nav_goal

    def step(self):
        goal = self.goal()
        if goal is None:
            self.state.nav_status = 'idle'
            self._drive(0, 0)
            return
        if goal != self._goal:
            self._start(goal)

        if self.state.nav_path is not self._path:
            self._follow(self.state.nav_path)
        if self._path is None:
            self._drive(0, 0)                # planning, or no way: wait (ramped)
            return
        if self.state.blocked is not None:
            self.planner.replan_now()        # the collision guard stopped us: another way

        with self.state._pose_lock:
            x, y, heading = self.state.pose.x, self.state.pose.y, self.state.pose.angle_rad

        if self._phase == self.PHASE_FINAL_TURN:
            self._step_final_turn(heading)
            return
        self._skip_reached_waypoints(x, y)
        target_x, target_y = self._path[self._waypoint]
        is_last = self._waypoint == len(self._path) - 1
        distance = math.hypot(target_x - x, target_y - y)
        if is_last and distance <= self.ARRIVAL_TOLERANCE:
            self._arrived()
            return

        err = self._normalize_angle(math.atan2(target_y - y, target_x - x) - heading)
        if abs(err) > math.radians(self.TURN_IN_PLACE_DEG):
            self._phase = self.PHASE_TURN
        if self._phase == self.PHASE_TURN:
            if self._turned_to(err):
                self._phase = self.PHASE_DRIVE
            else:
                self.state.nav_status = 'turning'
                self._turn_toward(err)
                return
        self.state.nav_status = 'driving'
        self._step_drive(err, self._distance_to_goal(x, y))

    # ── the way ──
    def _start(self, goal: tuple[float, float, Optional[float]]) -> None:
        """A new goal: plan the way to it."""
        self._goal = goal
        self._path = None
        self._phase = self.PHASE_DRIVE
        print(f"Navigation: to ({goal[0]:+.2f}, {goal[1]:+.2f}) m", flush=True)
        self.planner.set_goal((goal[0], goal[1]))

    def _follow(self, path: Optional[list[tuple[float, float]]]) -> None:
        """A new plan: follow it from its first waypoint (it starts where
        the robot was when it was planned)."""
        self._path = path
        self._waypoint = 0

    def _skip_reached_waypoints(self, x: float, y: float) -> None:
        """Move on to the next waypoint while the current one is reached
        (within ARRIVAL_TOLERANCE), or already behind: closer to the next
        one than it is."""
        while self._waypoint < len(self._path) - 1:
            here_x, here_y = self._path[self._waypoint]
            next_x, next_y = self._path[self._waypoint + 1]
            to_waypoint = math.hypot(here_x - x, here_y - y)
            robot_to_next = math.hypot(next_x - x, next_y - y)
            waypoint_to_next = math.hypot(next_x - here_x, next_y - here_y)
            if to_waypoint > self.ARRIVAL_TOLERANCE and robot_to_next >= waypoint_to_next:
                return
            self._waypoint += 1

    def _distance_to_goal(self, x: float, y: float) -> float:
        """Along the waypoints still ahead."""
        distance = 0.0
        from_x, from_y = x, y
        for waypoint_x, waypoint_y in self._path[self._waypoint:]:
            distance += math.hypot(waypoint_x - from_x, waypoint_y - from_y)
            from_x, from_y = waypoint_x, waypoint_y
        return distance

    def _arrived(self) -> None:
        final_heading = self._goal[2]
        if final_heading is not None and self._phase != self.PHASE_FINAL_TURN:
            self._phase = self.PHASE_FINAL_TURN
            return
        print("Navigation: arrived", flush=True)
        self.stop()                          # ramp down, then brake
        self.state.nav_status = 'arrived'
        self.state.mode = RobotState.MODE_MANUAL

    # ── phases ──
    def _step_drive(self, err: float, distance_to_goal: float) -> None:
        speed = self.DRIVE_SPEED
        if distance_to_goal < self.SLOW_DOWN_M:
            speed = max(self.MIN_START, self.DRIVE_SPEED * distance_to_goal / self.SLOW_DOWN_M)
        correction = max(-speed, min(speed, self.HEADING_KP * err))
        left  = speed - correction           # err > 0 → right wheel faster → turn left
        right = speed + correction
        self._drive(left, right)

    def _step_final_turn(self, heading: float) -> None:
        err = self._normalize_angle(self._goal[2] - heading)
        if self._turned_to(err):
            self._arrived()
            return
        self.state.nav_status = 'turning'
        self._turn_toward(err)

    # ── helpers ──
    def _turning_direction(self) -> int:
        """+1 turning in place to the left, −1 to the right, 0 otherwise
        (from the speeds last sent to the wheels)."""
        if self._sent_speeds is None:
            return 0
        left, right = self._sent_speeds
        if left == -right != 0:
            return 1 if right > 0 else -1
        return 0

    def _turned_to(self, err: float) -> bool:
        """True once a heading error err (rad, > 0 = to the left) is done
        with: within TURN_COAST_DEG while turning towards it (the robot
        coasts the rest), else within HEADING_TOLERANCE."""
        if self._turning_direction() * err > 0:
            return abs(err) <= math.radians(self.TURN_COAST_DEG)
        return abs(err) <= self.HEADING_TOLERANCE

    def _turn_toward(self, err: float) -> None:
        """Turn in place towards err: TURN_SPEED, then TURN_SLOW_SPEED over
        the last TURN_SLOW_DEG once the wheels turn that way (from standstill
        they need MIN_START)."""
        direction = 1 if err > 0 else -1
        speed = self.TURN_SPEED
        if abs(err) <= math.radians(self.TURN_SLOW_DEG) and self._turning_direction() == direction:
            speed = self.TURN_SLOW_SPEED
        s = direction * speed
        self._drive(-s, s, min_speed=self.TURN_SLOW_SPEED)   # s > 0: left backward, right forward → CCW (left)

    def stop(self):
        """Mode change, arrival: ramp down, then brake; forget the goal."""
        if self._goal is not None and self.state.nav_goal == self._goal:
            self.state.nav_goal = None       # not a goal set meanwhile by a new click
        if self._goal is not None and self.state.nav_status not in ('arrived', 'no path', 'no map'):
            self.state.nav_status = 'stopped'
        self.planner.cancel()
        self._goal = None
        self._path = None
        self._phase = self.PHASE_DRIVE
        super().stop()


# ════════════════════════════════════════════════════════════════════════════
class GoHomeController(NavigateController):
    """Go back to where the robot started, facing the way it started:
    the navigation to (0, 0), final heading 0 (the odometry's origin)."""

    HOME = (0.0, 0.0, 0.0)

    def goal(self) -> tuple[float, float, Optional[float]]:
        if self._goal is None:
            self.state.nav_goal = self.HOME  # shown on the web page's map
        return self.HOME
