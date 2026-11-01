"""Opt-in Stage 7 integration: A* + simulated LiDAR + EKF + DWA.

This is a fixed-step *Python* simulation, not Blender real-time physics.
The existing Stage 5 ``SingleRobot`` and Stage 2 scheduler are unchanged.

Static obstacles: Stage 4 inflated OccupancyGrid (and optional exact checker).
Dynamic obstacles: ground-truth agents are used ONLY by the simulated LiDAR
and an independent collision oracle. DWA receives estimated LiDAR tracks.

The simple Part 2 LiDAR adapter assumes pedestrian-sized AABB obstacles.
This is a demonstration, not a certified safety controller.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Sequence

from aifr.navigation.astar import find_path
from aifr.navigation.collision_checker import CollisionChecker
from aifr.navigation.dwa import DWAConfig, DWAPlanner, MovingObstacle
from aifr.navigation.kinematics import Pose, advance
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.perception.ekf import PoseEKF
from aifr.perception.lidar_adapter import LiDARObstacleTracker
from aifr.perception.sensors import SensorSuite, lidar_scan

# A scenario callback is a sensor-world model, NOT a planner input.
AgentProvider = Callable[[float], Sequence[MovingObstacle]]


@dataclass(frozen=True)
class ControlSample:
    time_s: float
    true_pose: Pose
    estimated_pose: Pose
    linear: float
    angular: float
    tracked_obstacles: int
    actual_clearance: float


@dataclass(frozen=True)
class TravelResult:
    reached: bool
    reason: str
    ticks: int
    path_points: int
    min_dynamic_clearance: float
    lidar_detection_ticks: int


@dataclass
class DWAIntegratedRobot:
    """Single robot that uses estimated pose and LiDAR detections for DWA.

    The true pose is used only to advance the simulated physics and generate
    synthetic sensor measurements; the planner uses ``filter.pose``.
    """

    grid: OccupancyGrid
    pose: Pose
    config: DWAConfig = field(default_factory=DWAConfig)
    sensors: SensorSuite = field(default_factory=SensorSuite)
    checker: CollisionChecker | None = None
    trace: list[Pose] = field(default_factory=list, init=False)
    controls: list[ControlSample] = field(default_factory=list, init=False)
    time_s: float = field(default=0.0, init=False)
    linear: float = field(default=0.0, init=False)
    angular: float = field(default=0.0, init=False)
    filter: PoseEKF = field(init=False)
    planner: DWAPlanner = field(init=False)
    tracker: LiDARObstacleTracker = field(init=False)

    def __post_init__(self) -> None:
        self.filter = PoseEKF(self.pose)
        self.planner = DWAPlanner(self.grid, self.config)
        self.tracker = LiDARObstacleTracker()
        self.trace.append(self.pose)
        if self.checker and not self.checker.is_pose_free(self.pose.x, self.pose.y, self.pose.theta):
            raise ValueError("Robot starts with a colliding footprint")
        if not self.grid.is_free(*self.grid.world_to_grid(self.pose.x, self.pose.y)):
            raise ValueError("Robot starts in an occupied grid cell")

    @staticmethod
    def _boxes(agents: Sequence[MovingObstacle]) -> list[tuple[float, float, float, float]]:
        """Part 2 LiDAR simulates axis-aligned pedestrian bounding boxes."""
        return [(o.x-o.radius, o.y-o.radius, o.x+o.radius, o.y+o.radius) for o in agents]

    def _ground_truth_clearance(self, pose: Pose, agents: Sequence[MovingObstacle]) -> float:
        """Independent AABB-vs-circular-envelope clearance, not fed to DWA."""
        robot_radius = self.config.robot_radius + self.config.safety_margin
        return min((math.hypot(
            pose.x - min(max(pose.x, a.x-a.radius), a.x+a.radius),
            pose.y - min(max(pose.y, a.y-a.radius), a.y+a.radius),
        ) - robot_radius for a in agents), default=float('inf'))

    def _motion_safe(self, start: Pose, end: Pose, provider: AgentProvider) -> tuple[bool, float]:
        """Check interpolated motion against map and actual moving AABBs.

        This oracle catches collisions caused by sensor/estimator error; it is
        deliberately separate from the DWA planner's perceived obstacles.
        """
        steps = max(2, math.ceil(math.hypot(end.x-start.x, end.y-start.y)
                                 / (self.grid.resolution/3)))
        min_gap = float('inf')
        for i in range(1, steps+1):
            f = i / steps
            pose = advance(start, self.linear, self.angular, self.config.dt*f)
            if not self.grid.is_free(*self.grid.world_to_grid(pose.x, pose.y)):
                return False, min_gap
            gap = self._ground_truth_clearance(pose, provider(self.time_s+self.config.dt*f))
            min_gap = min(min_gap, gap)
            if gap <= 0:
                return False, min_gap
        if self.checker and not self.checker.is_motion_free(
            (start.x, start.y, start.theta), (end.x, end.y, end.theta)
        ):
            return False, min_gap
        return True, min_gap

    def _dynamic_route(self, destination: tuple[float, float],
                       perceived: Sequence[MovingObstacle]) -> list[tuple[float, float]]:
        """Plan a temporary A* detour using only perceived moving objects.

        The base static occupancy grid is never modified. This is a pragmatic
        local reroute for blocked corridors, not multi-robot traffic control.
        """
        if not perceived:
            return []
        rows = [list(row) for row in self.grid.cells]
        g = self.grid
        for obs in perceived:
            radius = self.config.robot_radius + self.config.safety_margin + obs.radius + .03
            min_c, min_r = g.world_to_grid(obs.x-radius, obs.y-radius)
            max_c, max_r = g.world_to_grid(obs.x+radius, obs.y+radius)
            for row in range(max(0, min_r), min(g.height, max_r+1)):
                for col in range(max(0, min_c), min(g.width, max_c+1)):
                    x, y = g.grid_to_world(col, row)
                    if math.hypot(x-obs.x, y-obs.y) <= radius:
                        rows[row][col] = 1
        # Preserve the robot's own cell; a sensor may overestimate an obstacle.
        # DWA still independently rejects unsafe physical trajectories.
        start_cell = g.world_to_grid(self.filter.pose.x, self.filter.pose.y)
        if g.is_free(*start_cell):
            rows[start_cell[1]][start_cell[0]] = 0
        overlay = OccupancyGrid(g.origin_x, g.origin_y, g.resolution,
                                g.width, g.height, tuple(tuple(row) for row in rows))
        return find_path(overlay, (self.filter.pose.x, self.filter.pose.y), destination)

    def travel(self, destination: tuple[float, float],
               agents_at: AgentProvider | None = None,
               max_ticks: int = 2500,
               lidar_rays: int = 72) -> TravelResult:
        """Navigate to destination with A* guidance and LiDAR-informed DWA.

        Returns a structured failure on no route, unsafe trajectory, collision
        oracle rejection or timeout. Does not pretend an instant stop is
        physically feasible; no further simulated motion occurs on failure.
        """
        if max_ticks <= 0 or lidar_rays <= 0:
            raise ValueError("max_ticks and lidar_rays must be positive")
        if not all(math.isfinite(v) for v in destination):
            raise ValueError("Destination must contain finite coordinates")
        provider: AgentProvider = agents_at or (lambda _: ())
        path = find_path(self.grid, (self.pose.x, self.pose.y), destination)
        if not path:
            return TravelResult(False, "No A* route to destination", 0, 0, float('inf'), 0)
        # Each leg begins with a fresh scan tracker; time itself stays monotonic.
        self.tracker.reset()
        min_gap = float('inf')
        detection_ticks = 0
        for tick in range(max_ticks):
            # LiDAR raycast uses TRUE pose; tracker interprets returns using
            # the EKF estimate, which is the only pose seen by DWA.
            ranges = lidar_scan(self.pose, self._boxes(provider(self.time_s)),
                                rays=lidar_rays, max_range=4.0, step=0.02)
            perceived = self.tracker.observe(self.filter.pose, ranges, self.time_s)
            if perceived:
                detection_ticks += 1
            # Replan a detour at a bounded rate when perceived objects lie near
            # the current A* path. This uses LiDAR estimates, not true agents.
            if perceived and tick % 10 == 0:
                detour = self._dynamic_route(destination, perceived)
                if detour:
                    path = detour
            result = self.planner.plan(self.filter.pose, self.linear, self.angular,
                                       path, perceived)
            if not result.safe:
                return TravelResult(False, "DWA: " + result.reason, tick,
                                    len(path), min_gap, detection_ticks)
            if (math.hypot(self.pose.x-path[-1][0], self.pose.y-path[-1][1])
                    < self.config.goal_tolerance):
                self.linear = self.angular = 0.0
                return TravelResult(True, "Goal reached", tick,
                                    len(path), min_gap, detection_ticks)
            # Simulated robot advances under the commanded velocity.
            old = self.pose
            self.linear, self.angular = result.linear, result.angular
            nxt = advance(old, self.linear, self.angular, self.config.dt)
            ok, gap = self._motion_safe(old, nxt, provider)
            min_gap = min(min_gap, gap)
            if not ok:
                return TravelResult(False, "Ground-truth collision guard rejected motion", tick,
                                    len(path), min_gap, detection_ticks)
            self.pose = nxt
            self.trace.append(nxt)
            reading = self.sensors.odometry(self.linear, self.angular)
            self.filter.predict(reading.linear, reading.angular, self.config.dt)
            if len(self.trace) % 10 == 0:
                landmark = self.sensors.landmark(self.pose)
                self.filter.correct_xy(landmark.x, landmark.y)
            self.controls.append(ControlSample(self.time_s, self.pose, self.filter.pose,
                                               self.linear, self.angular, len(perceived), gap))
            self.time_s += self.config.dt
        return TravelResult(False, "DWA navigation timed out", max_ticks,
                            len(path), min_gap, detection_ticks)

    def deliver(self, pickup: tuple[float, float], table: tuple[float, float],
                agents_at: AgentProvider | None = None,
                max_ticks_per_leg: int = 2500) -> tuple[TravelResult, TravelResult | None]:
        """Opt-in delivery helper. Stage 2 scheduling/state machine is external."""
        first = self.travel(pickup, agents_at, max_ticks_per_leg)
        if not first.reached:
            return first, None
        second = self.travel(table, agents_at, max_ticks_per_leg)
        return first, second
