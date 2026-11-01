"""Stage 7 DWA unit tests; no Blender dependency."""
from __future__ import annotations

import math
import pytest

from aifr.navigation.dwa import DWAConfig, DWAPlanner, MovingObstacle
from aifr.navigation.kinematics import Pose
from aifr.navigation.occupancy_grid import OccupancyGrid


def grid(blocked: tuple[tuple[int, int], ...] = ()) -> OccupancyGrid:
    cells = [[0] * 40 for _ in range(40)]
    for x, y in blocked:
        cells[y][x] = 1
    return OccupancyGrid(-2.0, -2.0, 0.1, 40, 40, tuple(tuple(row) for row in cells))


def test_straight_route_selects_forward_motion() -> None:
    planner = DWAPlanner(grid())
    result = planner.plan(Pose(0, 0, 0), 0, 0, [(0, 0), (1, 0), (1.5, 0)])
    assert result.safe
    assert 0 < result.linear <= planner.config.linear_acceleration * planner.config.dt + 1e-10
    assert abs(result.angular) <= planner.config.angular_acceleration * planner.config.dt + 1e-10
    assert result.trajectory[-1].x > 0


def test_dynamic_window_acceleration_limits() -> None:
    planner = DWAPlanner(grid())
    result = planner.plan(Pose(0, 0, 0), 0.2, 0.5, [(0, 0), (1.5, 0)])
    assert result.safe
    assert abs(result.linear - 0.2) <= planner.config.linear_acceleration * planner.config.dt + 1e-10
    assert abs(result.angular - 0.5) <= planner.config.angular_acceleration * planner.config.dt + 1e-10


def test_blocked_route_does_not_cross_static_grid_cell() -> None:
    planner = DWAPlanner(grid(blocked=((21, 20), (22, 20))))
    result = planner.plan(Pose(0, 0, 0), 0, 0, [(0, 0), (1.5, 0)])
    assert result.safe
    assert all(planner.grid.is_free(*planner.grid.world_to_grid(p.x, p.y)) for p in result.trajectory)


def test_stationary_obstacle_is_avoided_in_prediction() -> None:
    planner = DWAPlanner(grid())
    obstacle = MovingObstacle(0.5, 0.0, 0.15)
    result = planner.plan(Pose(0, 0, 0), 0, 0, [(0, 0), (1.5, 0)], [obstacle])
    assert result.safe
    assert all(math.hypot(p.x - obstacle.x, p.y - obstacle.y) >
               planner.config.robot_radius + obstacle.radius + planner.config.safety_margin
               for p in result.trajectory)


def test_moving_obstacle_prediction_rejects_future_collision() -> None:
    planner = DWAPlanner(grid())
    # Starts far away, moves toward robot and would overlap during horizon.
    obstacle = MovingObstacle(1.0, 0.0, 0.1, vx=-0.6)
    result = planner.plan(Pose(0, 0, 0), 0, 0, [(0, 0), (1.5, 0)], [obstacle])
    assert not result.safe
    assert (result.linear, result.angular) == (0, 0)


def test_goal_stops_robot() -> None:
    result = DWAPlanner(grid()).plan(Pose(1, 1, 0), 0, 0, [(1.03, 1.0)])
    assert result.safe
    assert result.linear == 0 and result.angular == 0
    assert result.reason == "Goal reached"


def test_invalid_config_and_inputs() -> None:
    with pytest.raises(ValueError):
        DWAConfig(horizon=0)
    with pytest.raises(ValueError):
        DWAConfig(safety_margin=-0.01)
    with pytest.raises(ValueError):
        MovingObstacle(0, 0, -1)
    planner = DWAPlanner(grid())
    with pytest.raises(ValueError):
        planner.plan(Pose(0, 0, 0), 0, 0, [])
    with pytest.raises(ValueError):
        planner.plan(Pose(0, 0, 0), -0.1, 0, [(1, 1)])


def test_detour_path_can_pass_obstacle_without_collision() -> None:
    """Given a safe global detour, local DWA can track it around a person."""
    from aifr.navigation.kinematics import advance

    planner = DWAPlanner(grid())
    obstacle = MovingObstacle(0.85, 0.0, 0.18)
    path = [(0, 0), (0.3, -0.65), (0.85, -0.7), (1.3, -0.65), (1.5, 0)]
    pose = Pose(0, 0, 0)
    v = w = 0.0
    for _ in range(250):
        result = planner.plan(pose, v, w, path, [obstacle])
        assert result.safe, result.reason
        pose = advance(pose, result.linear, result.angular, planner.config.dt)
        assert math.hypot(pose.x - obstacle.x, pose.y - obstacle.y) > (
            planner.config.robot_radius + obstacle.radius + planner.config.safety_margin
        )
        v, w = result.linear, result.angular
        if math.hypot(pose.x - 1.5, pose.y) < planner.config.goal_tolerance:
            break
    else:
        pytest.fail("DWA did not reach the goal along the safe detour")
