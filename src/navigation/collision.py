"""Collision guard: how much free room the robot has in front of its nose
and behind its back, in a corridor as wide as the robot.

The robot's shape, measured from the LiDAR's centre (2026-11-07): nose
11 cm ahead, back 25 cm behind, 19 cm wide. Controller._drive() uses
clearances() and stop_distance() to stop driving into an obstacle, in every
mode. No hardware here: unit-tested on its own.
"""
import math

NOSE_M            = 0.11     # nose ahead of the LiDAR centre
REAR_M            = 0.25     # back of the robot behind the LiDAR centre
HALF_WIDTH_M      = 0.095    # robot 19 cm wide
SIDE_MARGIN_M     = 0.01     # corridor 1 cm wider than the robot on each side
STOP_MARGIN_M     = 0.05     # free room kept in front of the nose / behind the back
# Time between an obstacle showing up and the robot standing still: age of a
# scan (one LiDAR turn, 140 ms), processing, braking (1–3 cm measured with
# tests/testHWRampUp.py). The robot covers speed × this before it stops.
REACTION_SECS     = 0.3
MAX_SCAN_AGE_SECS = 0.5      # older clearances are not trusted: driving is blocked
# Robot speed for a motor command: 30 % turns the robot in place at ~160 °/s
# (tests/testOdometryLag.py), i.e. wheels at 2.8 rad/s × 0.085 m = 0.24 m/s
SPEED_MPS_PER_PERCENT = 0.008


def clearances(scan: list) -> tuple[float, float]:
    """Free room (m) in front of the nose and behind the back, in the
    robot's corridor; math.inf when nothing is there.
    scan: RPLidar points (quality, angle_deg clockwise, distance_mm). Points
    inside the robot's own outline are the robot itself: ignored."""
    corridor_half_width = HALF_WIDTH_M + SIDE_MARGIN_M
    front = math.inf
    rear = math.inf
    for quality, angle_cw, distance_mm in scan:
        if quality == 0 or distance_mm <= 0:
            continue
        # LiDAR frame: x forward, y left (RPLidar angles are clockwise)
        angle = math.radians(-angle_cw)
        x = distance_mm / 1000 * math.cos(angle)
        y = distance_mm / 1000 * math.sin(angle)
        if abs(y) > corridor_half_width:
            continue
        if x > NOSE_M:
            front = min(front, x - NOSE_M)
        elif x < -REAR_M:
            rear = min(rear, -REAR_M - x)
    return front, rear


def stop_distance(speed_mps: float) -> float:
    """Free room needed to stop in time at this speed: STOP_MARGIN_M plus
    what the robot covers during REACTION_SECS."""
    return STOP_MARGIN_M + abs(speed_mps) * REACTION_SECS
