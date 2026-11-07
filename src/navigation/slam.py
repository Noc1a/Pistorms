"""SLAM with BreezySLAM (RMHC_SLAM) on this robot's RPLidar A1 and odometry.

Settings measured and tuned with tests/testSlam.py (scenarios static, spin,
loop; see the comments there). Used live by the Slam sensor thread
(sensor_thread.py), and by tests/testSlam.py to replay recorded runs.

Frames (same as navigation.Odometry): x forward, y left, θ counter-clockwise.
RPLidar angles are clockwise, distances in mm; odometry poses are
(x m, y m, θ rad).
"""
import math
import threading
from typing import Optional

import numpy as np

OdomPose = tuple[float, float, float]      # x m, y m, θ rad

# ── LiDAR mounting ─────────────────────────────────────────────────────────
# Calibrated 2026-10-30 with tests/calibrateLidarOffset.py (in-place spin,
# 13 stops, robot centre stable to ±0.2 cm; ruler measurement was 7.0 / −0.4).
LIDAR_DX      = 0.078    # m, LiDAR rotation centre ahead of the drive-wheel axle
LIDAR_DY      = 0.001    # m, LiDAR rotation centre to the LEFT of the robot centre
LIDAR_YAW_DEG = 0.0      # deg, CCW rotation of LiDAR 0° vs robot forward
SCAN_MIN_M    = 0.15     # ignore returns closer than this (robot body, cables)

# ── Scans given to BreezySLAM ──────────────────────────────────────────────
SCAN_RATE_HZ      = 6.4      # measured with iter_scans on this LiDAR
SCAN_BINS         = 360      # one-degree bins
BIN_RADIUS_DEG    = 1.5      # a bin takes a measured point at most this far
MIN_SCAN_POINTS   = 100      # a scan with fewer filled bins is not used
# An empty bin (no return, or a gap between points) is a "no obstacle" ray of
# this length: 0 so that it erases nothing on the map.
NO_DETECTION_MM   = 0

# ── BreezySLAM ─────────────────────────────────────────────────────────────
MAP_SIZE_PIXELS   = 1000
MAP_SIZE_METERS   = 10       # 1 cm per pixel, robot starts at the centre
HOLE_WIDTH_MM     = 200      # wall thickness drawn behind each return (BreezySLAM default 600)
# 0–255: how much each scan changes the map (BreezySLAM default 50). Low: a
# scan matched slightly wrong (fast turn, a person walking by) barely marks
# the map, and walls stay single; see SIGMA_XY_MM for the measurements
MAP_QUALITY       = 5
# Pose search around the odometry. BreezySLAM's defaults (±100 mm, ±20°) are
# made for a poor odometry; ours is good to ~1 cm, and the wide search only
# lost precision (2026-11-01, loop_20261101_184600: ±30 mm / ±5° gave 9 mm /
# 0.6° against 34 mm / 1.9°). Narrowed again 2026-11-07 with MAP_QUALITY,
# replaying the 4-minute drive live_20261107_203904 (map consistency: share
# of scan points on a wall of the final map) and the loop with its LiDAR
# truth: ±20 mm / ±3° and quality 5 give consistency 0.90 (0.47 with
# ±30 mm / ±5° / 50) and 7.6 mm / 0.53° on the loop (12.4 mm / 0.64°).
SIGMA_XY_MM       = 20
SIGMA_THETA_DEG   = 3
# The odometry heading lags the real robot by the IMU gyro's delay: a scan is
# matched with the odometry of this much later. The delay is inside the IMU
# (its gyro filter, Odometry.GYRO_FILTER, and its own sensor): measured with
# tests/testOdometryLag.py against the wheel encoders (2026-11-07), 35 ms
# with filter 4; the I2C read and the register refresh add < 5 ms. (With
# filter 7 it was 85 ms, which is where the first, empirical 0.10 s came from.)
ODOMETRY_LAG_SECS = 0.035

# ── Map pictures ───────────────────────────────────────────────────────────
TRAIL_SPACING_M   = 0.02     # the robot's trail keeps one pose every 2 cm
MAP_VIEW_RADIUS_M = 3.0      # the map picture shows this much around the robot's trail
UNKNOWN_GRAY      = 127      # BreezySLAM's value of a map pixel never seen
SCREEN_WIDTH      = 320      # PiStorms screen (landscape): the map fills it
SCREEN_HEIGHT     = 240


def scan_bins(points: list[list[float]]) -> list[int]:
    """The scan as the 360 one-degree bins BreezySLAM expects.

    Bin i holds the distance at counter-clockwise angle i − 180° (BreezySLAM
    spans its bins from −180° to +180°). Each bin takes the closest measured
    point within BIN_RADIUS_DEG, else 0 (nothing drawn). BreezySLAM's own
    scan_angles_degrees option is not used: it interpolates linearly across
    the gaps of this LiDAR (~150 points per turn, gaps up to 50°) and draws
    walls that don't exist."""
    distances_mm = [0] * SCAN_BINS
    closest_offsets_deg = [math.inf] * SCAN_BINS
    for quality, angle_cw, distance_mm in points:
        if quality == 0 or distance_mm < SCAN_MIN_M * 1000:
            continue
        bin_angle = ((360.0 - angle_cw) + 180.0) % 360.0
        nearest_bin = round(bin_angle)
        for bin_number in (nearest_bin - 1, nearest_bin, nearest_bin + 1):
            offset_deg = abs(bin_angle - bin_number)
            bin_index = bin_number % SCAN_BINS
            if offset_deg <= BIN_RADIUS_DEG and offset_deg < closest_offsets_deg[bin_index]:
                closest_offsets_deg[bin_index] = offset_deg
                distances_mm[bin_index] = int(distance_mm)
    return distances_mm


def filled_bins(distances_mm: list[int]) -> int:
    return sum(1 for distance_mm in distances_mm if distance_mm)


def deskewed_points(points: list[list[float]], previous_odom: OdomPose,
                    current_odom: OdomPose) -> list[list[float]]:
    """The scan as if every point had been measured at the end of the scan.

    A LiDAR turn lasts 0.16 s: when the robot turns at 50°/s, the first and
    last points of a scan are measured 8° apart, which bends the scan. Each
    point is measured at a fraction of the scan (points come in measurement
    order); the robot pose at that moment is interpolated between the
    odometry at the end of the previous scan and at the end of this one, and
    the point is moved into the robot frame of the end of this scan.

    BreezySLAM has its own correction, but it assumes a scan that turns
    counter-clockwise from −180°; the RPLidar turns clockwise from the front,
    so it bends the scan the wrong way (it's disabled with dt = 0, see LidarSlam)."""
    x0, y0, theta0 = previous_odom
    x1, y1, theta1 = current_odom
    cos_end, sin_end = math.cos(theta1), math.sin(theta1)
    lidar_dx_mm, lidar_dy_mm = LIDAR_DX * 1000, LIDAR_DY * 1000
    deskewed: list[list[float]] = []
    for point_number, (quality, angle_cw, distance_mm) in enumerate(points):
        fraction = (point_number + 1) / len(points)
        # Robot pose (mm, rad) when the point was measured
        robot_x = (x0 + (x1 - x0) * fraction) * 1000
        robot_y = (y0 + (y1 - y0) * fraction) * 1000
        robot_theta = theta0 + (theta1 - theta0) * fraction
        # Point in the robot frame at that moment, then in the world
        angle_ccw = math.radians(-angle_cw)
        point_x = lidar_dx_mm + distance_mm * math.cos(angle_ccw)
        point_y = lidar_dy_mm + distance_mm * math.sin(angle_ccw)
        world_x = robot_x + point_x * math.cos(robot_theta) - point_y * math.sin(robot_theta)
        world_y = robot_y + point_x * math.sin(robot_theta) + point_y * math.cos(robot_theta)
        # Back into the robot frame at the end of the scan, seen from the LiDAR
        offset_x = world_x - x1 * 1000
        offset_y = world_y - y1 * 1000
        end_x = offset_x * cos_end + offset_y * sin_end - lidar_dx_mm
        end_y = -offset_x * sin_end + offset_y * cos_end - lidar_dy_mm
        deskewed.append([quality, (-math.degrees(math.atan2(end_y, end_x))) % 360.0,
                         math.hypot(end_x, end_y)])
    return deskewed


def odometry_change(previous: OdomPose, current: OdomPose) -> tuple[float, float]:
    """Move between two odometry poses as BreezySLAM wants it:
    (distance driven forward in mm, rotation in degrees counter-clockwise)."""
    x0, y0, theta0 = previous
    x1, y1, theta1 = current
    forward_m = (x1 - x0) * math.cos(theta0) + (y1 - y0) * math.sin(theta0)
    return forward_m * 1000, math.degrees(theta1 - theta0)


class LidarSlam:
    """BreezySLAM fed with one scan at a time and the odometry.

        slam = LidarSlam()
        pose = slam.update(points, odometry_pose)   # each LiDAR scan
        pixels = slam.map_pixels()                  # the map, any time
        picture = slam.map_image()                  # the map with the robot and its trail

    Poses are in the odometry frame: the SLAM starts at the odometry pose of
    its first scan. Thread-safe: update() and map_pixels() can be called
    from different threads."""

    def __init__(self, random_seed: Optional[int] = None) -> None:
        # Imported here: the module stays importable where BreezySLAM isn't
        # installed (the Mac, for the unit tests of the functions above)
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser

        laser = Laser(scan_size=SCAN_BINS, scan_rate_hz=SCAN_RATE_HZ,
                      detection_angle_degrees=360, distance_no_detection_mm=NO_DETECTION_MM,
                      offset_mm=int(LIDAR_DX * 1000))
        self._slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, hole_width_mm=HOLE_WIDTH_MM,
                               sigma_xy_mm=SIGMA_XY_MM, sigma_theta_degrees=SIGMA_THETA_DEG,
                               map_quality=MAP_QUALITY, random_seed=random_seed)
        self._start_x_mm, self._start_y_mm, _ = self._slam.getpos()
        self._origin: Optional[OdomPose] = None          # odometry pose of the first scan
        self._previous_odom: Optional[OdomPose] = None
        self.trail: list[OdomPose] = []                  # SLAM poses, one every TRAIL_SPACING_M
        self._lock = threading.Lock()

    def update(self, points: list[list[float]], odom: Optional[OdomPose]) -> Optional[OdomPose]:
        """Feed one scan. odom: the odometry pose at the end of the scan
        (already ODOMETRY_LAG_SECS later), or None to run without odometry.
        Returns the robot pose (x m, y m, θ rad), or None if the scan had
        too few points (it is then not used)."""
        with self._lock:
            if odom is not None and self._previous_odom is not None:
                points = deskewed_points(points, self._previous_odom, odom)
                forward_mm, turn_deg = odometry_change(self._previous_odom, odom)
            else:
                forward_mm, turn_deg = 0.0, 0.0
            distances_mm = scan_bins(points)
            if filled_bins(distances_mm) < MIN_SCAN_POINTS:
                return None
            self._previous_odom = odom
            if self._origin is None:
                self._origin = odom if odom is not None else (0.0, 0.0, 0.0)
            # dt = 0 turns off BreezySLAM's own (wrong-way) scan correction,
            # which works from the speeds move / dt: the scan is deskewed above
            self._slam.update(distances_mm, pose_change=(forward_mm, turn_deg, 0.0))
            pose = self._pose()
            if not self.trail or math.dist(self.trail[-1][:2], pose[:2]) >= TRAIL_SPACING_M:
                self.trail.append(pose)
            return pose

    def map_pixels(self) -> bytearray:
        """The map, MAP_SIZE_PIXELS² bytes, row 0 at the bottom: 0 = wall,
        255 = free, 127 = unknown. The first scan's pose is the centre,
        facing +x (right)."""
        pixels = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        with self._lock:
            self._slam.getmap(pixels)
        return pixels

    def map_image(self, width: int = 500, height: int = 500):
        """The map as a PIL RGB image of width × height, seen from above, the
        first scan's forward direction up (its left on the left): the robot's
        trail and at least MAP_VIEW_RADIUS_M around it, widened to the
        picture's proportions (never stretched). The trail (red) and the
        robot (green triangle pointing where it faces) are drawn after the
        resizing, so that they stay sharp when small."""
        from PIL import Image, ImageDraw

        # BreezySLAM's map: pixels[y][x], y up. The picture puts +x up and +y
        # on the left: picture row = N − x, column = N − y
        pixels = np.frombuffer(self.map_pixels(), dtype=np.uint8).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        picture = Image.fromarray(np.ascontiguousarray(np.flip(pixels.T))).convert('RGB')
        with self._lock:
            trail = list(self.trail)

        # Crop around the trail (the seen area is no good: a few long rays
        # reach the edge of the map), widened to the picture's proportions
        if trail:
            trail_pixels = [self._picture_pixel(pose) for pose in trail]
            columns = [column for column, _ in trail_pixels]
            rows = [row for _, row in trail_pixels]
        else:
            columns = rows = [MAP_SIZE_PIXELS / 2]
        margin = MAP_VIEW_RADIUS_M * MAP_SIZE_PIXELS / MAP_SIZE_METERS
        crop_width = max(columns) - min(columns) + 2 * margin
        crop_height = max(rows) - min(rows) + 2 * margin
        if crop_width / crop_height < width / height:
            crop_width = crop_height * width / height
        else:
            crop_height = crop_width * height / width
        left = self._inside_map((min(columns) + max(columns)) / 2 - crop_width / 2, crop_width)
        top = self._inside_map((min(rows) + max(rows)) / 2 - crop_height / 2, crop_height)
        picture = self._crop_with_unknown(picture, round(left), round(top),
                                          round(crop_width), round(crop_height))
        scale = width / crop_width
        picture = picture.resize((width, height))

        def on_picture(pose: OdomPose) -> tuple[float, float]:
            column, row = self._picture_pixel(pose)
            return (column - left) * scale, (row - top) * scale

        draw = ImageDraw.Draw(picture)
        if len(trail) > 1:
            draw.line([on_picture(pose) for pose in trail], fill=(255, 0, 0),
                      width=max(2, round(min(width, height) / 150)))
        if trail:
            robot_length = max(10.0, 0.3 * MAP_SIZE_PIXELS / MAP_SIZE_METERS * scale)
            self._draw_robot(draw, on_picture(trail[-1]), trail[-1][2] - self._origin[2], robot_length)
        return picture

    @staticmethod
    def _crop_with_unknown(picture, left: int, top: int, width: int, height: int):
        """Crop; where the crop goes past the map's edge, gray (unknown)
        instead of the black PIL fills in."""
        from PIL import Image
        cropped = Image.new('RGB', (width, height), (UNKNOWN_GRAY,) * 3)
        visible = (max(0, left), max(0, top), min(picture.width, left + width), min(picture.height, top + height))
        if visible[0] < visible[2] and visible[1] < visible[3]:
            cropped.paste(picture.crop(visible), (visible[0] - left, visible[1] - top))
        return cropped

    @staticmethod
    def _inside_map(start: float, length: float) -> float:
        """Shift a crop so that it stays on the map (if it fits)."""
        return max(0.0, min(start, MAP_SIZE_PIXELS - length))

    def _picture_pixel(self, pose: OdomPose) -> tuple[float, float]:
        """Column, row of an odometry-frame pose on the whole map picture."""
        origin_x, origin_y, origin_theta = self._origin
        dx, dy = pose[0] - origin_x, pose[1] - origin_y
        # In the frame of the first scan (BreezySLAM's map frame), in pixels
        pixels_per_m = MAP_SIZE_PIXELS / MAP_SIZE_METERS
        forward = dx * math.cos(origin_theta) + dy * math.sin(origin_theta)
        left = -dx * math.sin(origin_theta) + dy * math.cos(origin_theta)
        map_x = (self._start_x_mm / 1000 + forward) * pixels_per_m
        map_y = (self._start_y_mm / 1000 + left) * pixels_per_m
        return MAP_SIZE_PIXELS - map_y, MAP_SIZE_PIXELS - map_x

    @staticmethod
    def _draw_robot(draw, position: tuple[float, float], heading: float, length: float) -> None:
        """A green triangle at position, pointing along heading (map frame:
        0 = up on the picture, counter-clockwise)."""
        column, row = position
        ahead = (-math.sin(heading), -math.cos(heading))
        side = (-ahead[1], ahead[0])
        tip = (column + ahead[0] * length * 0.7, row + ahead[1] * length * 0.7)
        back_left = (column - ahead[0] * length * 0.3 + side[0] * length * 0.4,
                     row - ahead[1] * length * 0.3 + side[1] * length * 0.4)
        back_right = (column - ahead[0] * length * 0.3 - side[0] * length * 0.4,
                      row - ahead[1] * length * 0.3 - side[1] * length * 0.4)
        draw.polygon([tip, back_left, back_right], fill=(0, 200, 0))

    def _pose(self) -> OdomPose:
        """BreezySLAM's pose (mm and degrees from its start) in the odometry frame."""
        x_mm, y_mm, theta_deg = self._slam.getpos()
        dx = (x_mm - self._start_x_mm) / 1000
        dy = (y_mm - self._start_y_mm) / 1000
        origin_x, origin_y, origin_theta = self._origin
        return (origin_x + dx * math.cos(origin_theta) - dy * math.sin(origin_theta),
                origin_y + dx * math.sin(origin_theta) + dy * math.cos(origin_theta),
                origin_theta + math.radians(theta_deg))


def show_map_on_screen(screen, image) -> None:
    """Show a map image on the whole PiStorms screen (psm.screen). Make it
    with LidarSlam.map_image(SCREEN_WIDTH, SCREEN_HEIGHT): it is not stretched."""
    # Half a turn on the screen only, so that the map matches the room as
    # seen when looking at the robot's screen (checked on the robot; the
    # LiDAR frame itself is right: front/left/back/right distances verified)
    turned = image.rotate(180)
    screen.fillImgArray(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, np.array(turned))
