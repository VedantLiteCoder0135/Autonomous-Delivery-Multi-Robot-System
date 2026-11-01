"""Stage 7: pure-Python Dynamic Window Approach (DWA) local planner.

The planner receives a Stage 4 *inflated* occupancy grid, an A* route, and
moving obstacle estimates in world coordinates. It predicts feasible velocity
commands and rejects trajectories that collide with static or moving obstacles.

All distances are metres, time is seconds, angles are radians. The grid must
already account for the robot's static footprint, as OccupancyGrid does.
No Blender dependency and no change to the Stage 2 scheduling policy.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .kinematics import Pose, advance, normalize
from .occupancy_grid import OccupancyGrid


@dataclass(frozen=True)
class DWAConfig:
    """Physical limits and scoring parameters for one differential-drive robot."""

    dt: float = 0.05  # 20 Hz control tick
    horizon: float = 1.5
    simulation_step: float = 0.1
    max_linear: float = 0.35
    max_angular: float = 1.5
    linear_acceleration: float = 0.8
    angular_acceleration: float = 3.0
    linear_resolution: float = 0.05
    angular_resolution: float = 0.15
    robot_radius: float = math.hypot(0.20, 0.18)  # conservative circular footprint
    safety_margin: float = 0.05
    lookahead: float = 0.6
    goal_tolerance: float = 0.10
    progress_weight: float = 3.0
    heading_weight: float = 1.0
    clearance_weight: float = 0.25
    speed_weight: float = 0.20

    def __post_init__(self) -> None:
        positive = (
            self.dt, self.horizon, self.simulation_step,
            self.max_linear, self.max_angular,
            self.linear_acceleration, self.angular_acceleration,
            self.linear_resolution, self.angular_resolution,
            self.robot_radius, self.lookahead, self.goal_tolerance,
        )
        weights = (self.progress_weight, self.heading_weight,
                   self.clearance_weight, self.speed_weight)
        if any(not math.isfinite(x) or x <= 0 for x in positive):
            raise ValueError("DWA physical and sampling parameters must be finite and positive")
        if not math.isfinite(self.safety_margin) or self.safety_margin < 0:
            raise ValueError("safety_margin must be finite and nonnegative")
        if any(not math.isfinite(x) or x < 0 for x in weights):
            raise ValueError("DWA scoring weights must be finite and nonnegative")


@dataclass(frozen=True)
class MovingObstacle:
    """Constant-velocity estimate of a person/obstacle, in world coordinates."""

    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (self.x, self.y, self.radius, self.vx, self.vy)):
            raise ValueError("MovingObstacle values must be finite")
        if self.radius < 0:
            raise ValueError("MovingObstacle radius must be nonnegative")


@dataclass(frozen=True)
class DWAResult:
    """One control command and its predicted trajectory."""

    linear: float
    angular: float
    trajectory: tuple[Pose, ...]
    safe: bool
    target: tuple[float, float]
    reason: str


class DWAPlanner:
    def __init__(self, grid: OccupancyGrid, config: DWAConfig | None = None) -> None:
        if grid.resolution <= 0 or grid.width < 1 or grid.height < 1:
            raise ValueError("Invalid occupancy grid")
        self.grid = grid
        self.config = config or DWAConfig()

    @staticmethod
    def _samples(low: float, high: float, resolution: float) -> list[float]:
        """Include the window endpoints and zero, even if off the sample lattice."""
        if high < low - 1e-10:
            return []
        n = math.floor((high - low) / resolution)
        values = [low + i * resolution for i in range(n + 1)]
        values.append(high)
        if low <= 0 <= high:
            values.append(0.0)
        return sorted({round(x, 12) for x in values})

    def _window(self, linear: float, angular: float) -> tuple[list[float], list[float]]:
        c = self.config
        v_low = max(0.0, linear - c.linear_acceleration * c.dt)
        v_high = min(c.max_linear, linear + c.linear_acceleration * c.dt)
        w_low = max(-c.max_angular, angular - c.angular_acceleration * c.dt)
        w_high = min(c.max_angular, angular + c.angular_acceleration * c.dt)
        return (self._samples(v_low, v_high, c.linear_resolution),
                self._samples(w_low, w_high, c.angular_resolution))

    def _lookahead_target(self, pose: Pose, path: Sequence[tuple[float, float]]) -> tuple[float, float]:
        nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - pose.x, path[i][1] - pose.y))
        for x, y in path[nearest:]:
            if math.hypot(x - pose.x, y - pose.y) >= self.config.lookahead:
                return x, y
        return path[-1]

    def _pose_clearance(self, pose: Pose, t: float,
                        obstacles: Sequence[MovingObstacle]) -> float:
        """Conservative gap to predicted moving obstacle surfaces; <= 0 collides."""
        c = self.config
        return min((math.hypot(pose.x - (o.x + o.vx * t), pose.y - (o.y + o.vy * t))
                    - c.robot_radius - o.radius - c.safety_margin for o in obstacles),
                   default=float("inf"))

    def _static_clearance(self, pose: Pose, cap: float = 0.7) -> float:
        """Approximate additional clearance from blocked cells in an inflated grid."""
        g = self.grid
        col, row = g.world_to_grid(pose.x, pose.y)
        radius_cells = math.ceil(cap / g.resolution) + 1
        closest = cap
        for r in range(row - radius_cells, row + radius_cells + 1):
            for q in range(col - radius_cells, col + radius_cells + 1):
                if not g.is_free(q, r):
                    cx = g.origin_x + (q + 0.5) * g.resolution
                    cy = g.origin_y + (r + 0.5) * g.resolution
                    distance = math.hypot(pose.x - cx, pose.y - cy)
                    closest = min(closest, max(0.0, distance - g.resolution / math.sqrt(2)))
        return closest

    def _predict(self, pose: Pose, linear: float, angular: float,
                 obstacles: Sequence[MovingObstacle]) -> tuple[tuple[Pose, ...], float] | None:
        c = self.config
        # Limit translation between samples to half a grid cell, preventing
        # a predicted trajectory from jumping over a blocked cell.
        step = min(c.simulation_step, self.grid.resolution / (2 * c.max_linear))
        steps = max(1, math.ceil(c.horizon / step))
        dt = c.horizon / steps
        states = [pose]
        if not self.grid.is_free(*self.grid.world_to_grid(pose.x, pose.y)):
            return None
        gap = self._pose_clearance(pose, 0.0, obstacles)
        if gap <= 0:
            return None
        for i in range(1, steps + 1):
            nxt = advance(states[-1], linear, angular, dt)
            if not self.grid.is_free(*self.grid.world_to_grid(nxt.x, nxt.y)):
                return None
            gap = min(gap, self._pose_clearance(nxt, i * dt, obstacles))
            if gap <= 0:
                return None
            states.append(nxt)
        return tuple(states), gap

    def plan(self, pose: Pose, current_linear: float, current_angular: float,
             path: Sequence[tuple[float, float]],
             moving_obstacles: Sequence[MovingObstacle] = ()) -> DWAResult:
        """Choose the highest-scoring collision-free reachable velocity pair.

        A ``safe=False`` result requests an emergency stop/replan; it does NOT
        guarantee that stopping is safe when an obstacle is already approaching.
        """
        if not path:
            raise ValueError("DWA requires a nonempty global path")
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta,
                                              current_linear, current_angular)):
            raise ValueError("Pose and current velocities must be finite")
        if current_linear < 0 or current_linear > self.config.max_linear + 1e-9:
            raise ValueError("Current linear velocity outside configured limits")
        if abs(current_angular) > self.config.max_angular + 1e-9:
            raise ValueError("Current angular velocity outside configured limits")
        if not all(math.isfinite(x) and math.isfinite(y) for x, y in path):
            raise ValueError("Path coordinates must be finite")
        target = self._lookahead_target(pose, path)
        final_goal = path[-1]
        if math.hypot(pose.x - final_goal[0], pose.y - final_goal[1]) < self.config.goal_tolerance:
            stopped = self._predict(pose, 0.0, 0.0, moving_obstacles)
            return DWAResult(0.0, 0.0, (pose,), stopped is not None, target,
                             "Goal reached" if stopped is not None else "Obstacle approaching at goal")

        velocities, rotations = self._window(current_linear, current_angular)
        best_score = -float("inf")
        best: DWAResult | None = None
        c = self.config
        start_distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
        for linear in velocities:
            for angular in rotations:
                predicted = self._predict(pose, linear, angular, moving_obstacles)
                if predicted is None:
                    continue
                trajectory, dynamic_gap = predicted
                end = trajectory[-1]
                remaining = math.hypot(target[0] - end.x, target[1] - end.y)
                progress = start_distance - remaining
                desired_heading = math.atan2(target[1] - end.y, target[0] - end.x)
                heading = 1 - abs(normalize(desired_heading - end.theta)) / math.pi
                static_gap = self._static_clearance(end)
                clearance = min(static_gap, dynamic_gap, 0.7)
                score = (c.progress_weight * progress
                         + c.heading_weight * heading
                         + c.clearance_weight * clearance
                         + c.speed_weight * linear / c.max_linear)
                if score > best_score:
                    best_score = score
                    best = DWAResult(linear, angular, trajectory, True, target, "OK")
        if best is None:
            return DWAResult(0.0, 0.0, (pose,), False, target,
                             "No feasible collision-free trajectory; stop and replan")
        return best
