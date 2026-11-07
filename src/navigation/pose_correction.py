"""Odometry corrected by the SLAM: the math, on its own.

The odometry (wheels + gyro, 50 Hz) drifts while the robot rolls: the gyro
reads ~0.1–0.27 °/s too little on the floor (vibration; measured 2026-11-07
with tests/testGyroMotorBias.py and testSlam.py --straight). The SLAM stays
right. A correction C = (cx, cy, cθ) moves the odometry's frame onto the
map's (the classic "map → odom" transform):

    published pose = C ∘ odometry pose:
        x, y = R(cθ) · (odometry x, y) + (cx, cy)
        θ    = odometry θ + cθ

At each scan, the Slam thread has both poses at the same instant and works
out C (correction_between); the Odometry thread applies it at 50 Hz,
easing towards each new C (blended) so that the published pose never jumps.
"""
import math

from .slam import OdomPose

Correction = tuple[float, float, float]     # cx m, cy m, cθ rad
IDENTITY: Correction = (0.0, 0.0, 0.0)

# Time constant of the easing: the applied correction covers 63 % of the way
# to a new target in this time, 95 % in three times this
SMOOTHING_SECS = 0.5
# Between two SLAM updates (~0.14 s), the SLAM's move must follow the
# odometry's: normal drift is ~0.03° there. More than this = the SLAM jumped
# (scan matched on the wrong wall): its correction is not used.
MAX_JUMP_M = 0.10
MAX_JUMP_DEG = 10.0


def _wrap(angle: float) -> float:
    """Angle in (−π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def correction_between(slam_pose: OdomPose, odom_pose: OdomPose) -> Correction:
    """The correction that moves odom_pose onto slam_pose (same instant)."""
    slam_x, slam_y, slam_theta = slam_pose
    odom_x, odom_y, odom_theta = odom_pose
    turn = slam_theta - odom_theta
    cos_turn, sin_turn = math.cos(turn), math.sin(turn)
    return (slam_x - (cos_turn * odom_x - sin_turn * odom_y),
            slam_y - (sin_turn * odom_x + cos_turn * odom_y),
            turn)


def apply(correction: Correction, odom_pose: OdomPose) -> OdomPose:
    """The odometry pose in the map's frame."""
    shift_x, shift_y, turn = correction
    odom_x, odom_y, odom_theta = odom_pose
    cos_turn, sin_turn = math.cos(turn), math.sin(turn)
    return (cos_turn * odom_x - sin_turn * odom_y + shift_x,
            sin_turn * odom_x + cos_turn * odom_y + shift_y,
            odom_theta + turn)


def blended(current: Correction, target: Correction, fraction: float) -> Correction:
    """`fraction` (0–1) of the way from current to target; the angle by
    the shorter way round."""
    current_x, current_y, current_turn = current
    target_x, target_y, target_turn = target
    return (current_x + (target_x - current_x) * fraction,
            current_y + (target_y - current_y) * fraction,
            current_turn + _wrap(target_turn - current_turn) * fraction)


def slam_jumped(previous_slam: OdomPose, slam: OdomPose,
                previous_odom: OdomPose, odom: OdomPose) -> bool:
    """True if the SLAM moved differently from the odometry between two
    updates by more than MAX_JUMP_M or MAX_JUMP_DEG."""
    slam_turn = slam[2] - previous_slam[2]
    odom_turn = odom[2] - previous_odom[2]
    if abs(math.degrees(_wrap(slam_turn - odom_turn))) > MAX_JUMP_DEG:
        return True
    # The moves, each in its own frame's robot-at-previous-pose coordinates
    slam_move = _in_frame_of(previous_slam, slam)
    odom_move = _in_frame_of(previous_odom, odom)
    return math.dist(slam_move, odom_move) > MAX_JUMP_M


def _in_frame_of(origin: OdomPose, pose: OdomPose) -> tuple[float, float]:
    """Where pose is, seen from a robot at origin (forward, left)."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    cos_theta, sin_theta = math.cos(origin[2]), math.sin(origin[2])
    return dx * cos_theta + dy * sin_theta, -dx * sin_theta + dy * cos_theta
