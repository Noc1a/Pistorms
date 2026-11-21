"""The way to a goal on the SLAM map, around the walls and the obstacles.

    grid = PlanningGrid.from_map(pixels, slam, scan, robot_pose)
    waypoints = plan_path(grid, (robot_x, robot_y), (goal_x, goal_y))   # or None

The map (navigation.slam.LidarSlam.map_pixels(): 1 cm pixels, 0 = wall,
255 = free, 127 = never seen) is cut into CELL_PIXELS cells. Each cell gets a
cost from its distance to the nearest wall:

    closer than FORBIDDEN_M      forbidden: the robot's side would touch it
    up to COMFORT_M              dearer the closer it is: room to turn in place
    never seen (gray)            UNKNOWN_COST_FACTOR times dearer: crossed
                                 only when there is no known way

plan_path() finds the cheapest way (A*, 8 neighbours), then keeps only the
corners: a few waypoints the robot drives to in straight lines.

PathPlanner does it in a thread of its own (a plan takes a few tenths of a
second on the Pi), again every REPLAN_SECS while the robot drives: walls the
map learns on the way, and obstacles in the latest LiDAR scan, change the way.
No hardware here: unit-tested on its own (unitTests/test_path_planner.py).
"""
import heapq
import math
import threading
import time
from typing import TYPE_CHECKING, Optional, Protocol

import numpy as np
from scipy.ndimage import distance_transform_edt

from . import collision
from .robot_profile import load_profile
from .slam import LIDAR_DX, LIDAR_DY, MAP_SIZE_METERS, MAP_SIZE_PIXELS, SCAN_MIN_M

if TYPE_CHECKING:
    from .robot_state import RobotState

Point = tuple[float, float]       # x m, y m (odometry frame)
Cell = tuple[int, int]            # row, column of the planning grid

_PROFILE = load_profile()

CELL_PIXELS        = 5            # map pixels per cell side: 5 cm cells, 200 × 200
WALL_LEVEL         = 90           # a cell with a pixel darker than this is a wall
FREE_LEVEL         = 160          # a cell lighter than this on average is known free
SCAN_MAX_M         = 4.0          # scan points farther than this are not added as walls
# Distances from a wall, to the robot's centre (the middle of the wheel axle)
FORBIDDEN_M        = collision.HALF_WIDTH_M + collision.SIDE_MARGIN_M + 0.04
COMFORT_M          = math.hypot(max(_PROFILE.body_front, _PROFILE.body_rear), collision.HALF_WIDTH_M) + 0.05
NEAR_WALL_COST     = 4.0          # extra cost of a cell at FORBIDDEN_M, falling to 0 at COMFORT_M
UNKNOWN_COST_FACTOR = 3.0
MAX_SNAP_M         = 0.30         # a start or goal in a forbidden cell moves this far at most
# A straight line between two corners may cost this much more per cell than
# the cells it replaces: 0 left a staircase of waypoints 5 cm apart around
# the end of a wall (27 waypoints on the maze test, 17 with 0.5, still 22 cm
# from the walls; 2.0 came down to 17 cm)
CORNER_COST_SLACK  = 0.5
REPLAN_SECS        = 2.0


class MapFrame(Protocol):
    """Where the map sits in the odometry frame: LidarSlam is one."""

    def world_to_map(self, x: float, y: float) -> tuple[float, float]: ...

    def map_to_world(self, map_x: float, map_y: float) -> tuple[float, float]: ...


class PlanningGrid:
    """The map cut into cells, with what each costs to drive through.
    allowed[row][column]: False where the robot can't go. cost: ≥ 1 where
    allowed. Rows go along the map's y, columns along its x."""

    def __init__(self, frame: MapFrame, allowed: np.ndarray, cost: np.ndarray) -> None:
        self.frame: MapFrame = frame
        self.allowed: np.ndarray = allowed
        self.cost: np.ndarray = cost
        self.rows, self.columns = allowed.shape
        self.cell_m: float = CELL_PIXELS * MAP_SIZE_METERS / MAP_SIZE_PIXELS

    @classmethod
    def from_map(cls, pixels: np.ndarray, frame: MapFrame, scan: Optional[list] = None,
                 robot_pose: Optional[tuple[float, float, float]] = None) -> 'PlanningGrid':
        """pixels: the map, MAP_SIZE_PIXELS² (pixels[map_y][map_x]). scan:
        the latest LiDAR points (quality, clockwise angle °, mm), robot-
        oriented, taken at robot_pose (x, y, θ): added as walls, since the map
        learns a new obstacle only slowly."""
        cells_per_side = MAP_SIZE_PIXELS // CELL_PIXELS
        blocks = pixels[:cells_per_side * CELL_PIXELS, :cells_per_side * CELL_PIXELS]
        blocks = blocks.reshape(cells_per_side, CELL_PIXELS, cells_per_side, CELL_PIXELS)
        wall = blocks.min(axis=(1, 3)) < WALL_LEVEL
        known_free = blocks.mean(axis=(1, 3)) > FREE_LEVEL
        grid = cls(frame, np.ones_like(wall), np.ones(wall.shape))
        if scan and robot_pose is not None:
            for row, column in grid._scan_cells(scan, robot_pose):
                wall[row, column] = True

        cell_m = grid.cell_m
        wall_distance_m = distance_transform_edt(~wall) * cell_m
        allowed = wall_distance_m >= FORBIDDEN_M
        # Near a wall: from 1 + NEAR_WALL_COST at FORBIDDEN_M down to 1 at COMFORT_M
        closeness = np.clip((COMFORT_M - wall_distance_m) / (COMFORT_M - FORBIDDEN_M), 0.0, 1.0)
        cost = 1.0 + NEAR_WALL_COST * closeness
        cost[~known_free] *= UNKNOWN_COST_FACTOR
        grid.allowed = allowed
        grid.cost = cost
        return grid

    def world_to_cell(self, point: Point) -> Cell:
        map_x, map_y = self.frame.world_to_map(*point)
        return int(map_y // CELL_PIXELS), int(map_x // CELL_PIXELS)

    def cell_to_world(self, cell: Cell) -> Point:
        """The centre of the cell."""
        row, column = cell
        return self.frame.map_to_world((column + 0.5) * CELL_PIXELS, (row + 0.5) * CELL_PIXELS)

    def inside(self, cell: Cell) -> bool:
        row, column = cell
        return 0 <= row < self.rows and 0 <= column < self.columns

    def _scan_cells(self, scan: list, robot_pose: tuple[float, float, float]) -> list[Cell]:
        """The cells of the scan points, from the robot at robot_pose."""
        robot_x, robot_y, heading = robot_pose
        cos_heading, sin_heading = math.cos(heading), math.sin(heading)
        cells = []
        for quality, angle_cw, distance_mm in scan:
            distance_m = distance_mm / 1000
            if quality == 0 or not SCAN_MIN_M < distance_m < SCAN_MAX_M:
                continue
            # In the robot's frame (x ahead, y left): RPLidar angles are clockwise
            angle = math.radians(-angle_cw)
            ahead = LIDAR_DX + distance_m * math.cos(angle)
            left = LIDAR_DY + distance_m * math.sin(angle)
            point_x = robot_x + ahead * cos_heading - left * sin_heading
            point_y = robot_y + ahead * sin_heading + left * cos_heading
            cell = self.world_to_cell((point_x, point_y))
            if self.inside(cell):
                cells.append(cell)
        return cells


def plan_path(grid: PlanningGrid, start: Point, goal: Point) -> Optional[list[Point]]:
    """The waypoints from start to goal (start not included, goal last), or
    None when there is no way. A start or goal in a forbidden cell (the robot
    against a wall, a click on a piece of furniture) moves to the nearest
    allowed cell within MAX_SNAP_M."""
    start_cell = _nearest_allowed(grid, grid.world_to_cell(start))
    goal_cell = _nearest_allowed(grid, grid.world_to_cell(goal))
    if start_cell is None or goal_cell is None:
        return None
    cells = _cheapest_cells(grid, start_cell, goal_cell)
    if cells is None:
        return None
    corners = _corners(grid, cells)
    waypoints = [grid.cell_to_world(cell) for cell in corners[1:]]
    goal_moved = goal_cell != grid.world_to_cell(goal)
    if not waypoints:
        waypoints = [goal]
    if not goal_moved:
        waypoints[-1] = goal         # the exact point, not the centre of its cell
    return waypoints


def _nearest_allowed(grid: PlanningGrid, cell: Cell) -> Optional[Cell]:
    """The cell itself if allowed, else the nearest allowed one within MAX_SNAP_M."""
    if not grid.inside(cell):
        return None
    if grid.allowed[cell]:
        return cell
    reach = int(MAX_SNAP_M / grid.cell_m)
    row, column = cell
    best_cell = None
    best_distance = math.inf
    for other_row in range(row - reach, row + reach + 1):
        for other_column in range(column - reach, column + reach + 1):
            other = (other_row, other_column)
            if not grid.inside(other) or not grid.allowed[other]:
                continue
            distance = math.hypot(other_row - row, other_column - column)
            if distance <= reach and distance < best_distance:
                best_cell, best_distance = other, distance
    return best_cell


# The 8 neighbours of a cell and the length of the step to each (in cells)
_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
               (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def _cheapest_cells(grid: PlanningGrid, start: Cell, goal: Cell) -> Optional[list[Cell]]:
    """A*: the cheapest chain of allowed cells from start to goal. A step
    costs its length times the cost of the cell it enters."""
    # Plain Python lists: much faster than NumPy item by item
    allowed = grid.allowed.tolist()
    cost = grid.cost.tolist()
    rows, columns = grid.rows, grid.columns

    def remaining(cell: Cell) -> float:
        """Octile distance to the goal: never more than the real cost (≥ 1 per cell)."""
        row_gap = abs(cell[0] - goal[0])
        column_gap = abs(cell[1] - goal[1])
        return max(row_gap, column_gap) + (math.sqrt(2) - 1) * min(row_gap, column_gap)

    cost_so_far: dict[Cell, float] = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    to_visit: list[tuple[float, float, Cell]] = [(remaining(start), 0.0, start)]
    while to_visit:
        _, cell_cost, cell = heapq.heappop(to_visit)
        if cell == goal:
            return _chain(came_from, goal)
        if cell_cost > cost_so_far[cell]:
            continue                 # already reached more cheaply
        row, column = cell
        for row_step, column_step, step_length in _NEIGHBOURS:
            next_row, next_column = row + row_step, column + column_step
            if not (0 <= next_row < rows and 0 <= next_column < columns):
                continue
            if not allowed[next_row][next_column]:
                continue
            next_cell = (next_row, next_column)
            next_cost = cell_cost + step_length * cost[next_row][next_column]
            if next_cost < cost_so_far.get(next_cell, math.inf):
                cost_so_far[next_cell] = next_cost
                came_from[next_cell] = cell
                heapq.heappush(to_visit, (next_cost + remaining(next_cell), next_cost, next_cell))
    return None


def _chain(came_from: dict[Cell, Cell], goal: Cell) -> list[Cell]:
    """The cells from the start to the goal, following came_from back."""
    cells = [goal]
    while cells[-1] in came_from:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    return cells


def _corners(grid: PlanningGrid, cells: list[Cell]) -> list[Cell]:
    """The cells where the robot must change direction: a straight line
    replaces the cells between two corners when it stays in allowed cells
    no dearer than the cells it replaces (it doesn't cut closer to a wall,
    nor through the unknown, than the cheapest way does, give or take
    CORNER_COST_SLACK)."""
    corners = [cells[0]]
    corner_index = 0
    while corner_index < len(cells) - 1:
        farthest = corner_index + 1
        dearest_replaced = grid.cost[cells[farthest]]
        for candidate in range(corner_index + 2, len(cells)):
            dearest_replaced = max(dearest_replaced, grid.cost[cells[candidate]])
            max_cost = dearest_replaced + CORNER_COST_SLACK
            if not _straight_line_fits(grid, cells[corner_index], cells[candidate], max_cost):
                break
            farthest = candidate
        corners.append(cells[farthest])
        corner_index = farthest
    return corners


def _straight_line_fits(grid: PlanningGrid, start: Cell, end: Cell, max_cost: float) -> bool:
    """True if every cell on the straight line from start to end is allowed
    and costs at most max_cost."""
    row_gap = end[0] - start[0]
    column_gap = end[1] - start[1]
    steps = 2 * max(abs(row_gap), abs(column_gap))       # half a cell per step
    for step in range(1, steps):
        progress = step / steps
        cell = (round(start[0] + row_gap * progress), round(start[1] + column_gap * progress))
        if not grid.allowed[cell] or grid.cost[cell] > max_cost:
            return False
    return True


class PathPlanner:
    """Plans the way to a goal in a thread of its own, again every
    REPLAN_SECS and when asked (replan_now()), from the robot's pose, the
    SLAM map and the latest LiDAR scan of the RobotState. Publishes
    state.nav_path (the waypoints, or None) and state.nav_status."""

    def __init__(self, state: 'RobotState') -> None:
        self.state: 'RobotState' = state
        self._goal: Optional[Point] = None
        self._goal_number: int = 0          # changes with every goal: a late plan is dropped
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def set_goal(self, goal: Point) -> None:
        """Plan the way to this goal now, then again every REPLAN_SECS."""
        with self._lock:
            self._goal = goal
            self._goal_number += 1
            self.state.nav_path = None
            self.state.nav_status = 'planning'
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        self._wake.set()

    def replan_now(self) -> None:
        self._wake.set()

    def cancel(self) -> None:
        """No goal any more: nothing is planned, the path is removed."""
        with self._lock:
            self._goal = None
            self._goal_number += 1
            self.state.nav_path = None

    def _run(self) -> None:
        while self.state.running:
            self._wake.wait(REPLAN_SECS)
            self._wake.clear()
            with self._lock:
                goal, goal_number = self._goal, self._goal_number
            if goal is None:
                continue
            try:
                path, status = self._plan(goal)
            except Exception as error:
                # never let one bad map kill the thread
                print(f"Path planner error: {error!r}", flush=True)
                continue
            with self._lock:
                if goal_number != self._goal_number:
                    continue                 # the goal changed meanwhile
                self.state.nav_path = path
                if status is not None:
                    self.state.nav_status = status

    def _plan(self, goal: Point) -> tuple[Optional[list[Point]], Optional[str]]:
        """(waypoints, status): status is None when a way was found (the
        controller reports how it drives)."""
        slam = self.state.slam
        if slam is None or not slam.has_map:
            return None, 'no map'
        pixels = np.frombuffer(slam.map_pixels(), dtype=np.uint8).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        with self.state._pose_lock:
            robot_pose = (self.state.pose.x, self.state.pose.y, self.state.pose.angle_rad)
        start_time = time.monotonic()
        grid = PlanningGrid.from_map(pixels, slam, list(self.state.last_scan), robot_pose)
        path = plan_path(grid, robot_pose[:2], goal)
        self.state.nav_plan_secs = time.monotonic() - start_time
        if path is None:
            return None, 'no path'
        return path, None
