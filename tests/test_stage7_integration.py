"""Stage 7 Part 3 integration checks; no Blender dependency."""
from __future__ import annotations

import math
from aifr.navigation.dwa import DWAConfig, MovingObstacle
from aifr.navigation.kinematics import Pose
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.perception.sensors import SensorSuite
from aifr.simulation.delivery import SingleRobot
from aifr.simulation.dwa_delivery import DWAIntegratedRobot


def grid(blocked: tuple[tuple[int, int], ...] = ()) -> OccupancyGrid:
    cells = [[0] * 50 for _ in range(50)]
    for x, y in blocked:
        cells[y][x] = 1
    return OccupancyGrid(-2.5, -2.5, .1, 50, 50, tuple(tuple(row) for row in cells))


def robot(g: OccupancyGrid | None = None, noise: float = 0.0) -> DWAIntegratedRobot:
    return DWAIntegratedRobot(g or grid(), Pose(0, 0, 0),
                              sensors=SensorSuite(seed=7, velocity_sigma=noise,
                                                  yaw_sigma=noise, position_sigma=noise))


def test_integrated_astar_dwa_ekf_reaches_goal():
    r = robot()
    outcome = r.travel((1.5, 0.0), max_ticks=250)
    assert outcome.reached, outcome.reason
    assert outcome.path_points > 2
    assert len(r.trace) == len(r.controls) + 1
    assert math.dist((r.filter.pose.x, r.filter.pose.y), (r.pose.x, r.pose.y)) < .001
    assert math.hypot(r.pose.x-1.5, r.pose.y) < .2


def test_lidar_is_consumed_by_integrated_dwa():
    r = DWAIntegratedRobot(grid(), Pose(0, 0, 0), config=DWAConfig(max_linear=.2),
                           sensors=SensorSuite(velocity_sigma=0, yaw_sigma=0,
                                               position_sigma=0))
    baseline = DWAIntegratedRobot(grid(), Pose(0, 0, 0), config=DWAConfig(max_linear=.2),
                                  sensors=SensorSuite(velocity_sigma=0, yaw_sigma=0,
                                                      position_sigma=0))
    # A pedestrian moves slowly alongside the route. True position is only
    # supplied to the LiDAR simulator and independent collision checker.
    def person(t: float):
        return (MovingObstacle(.8 + .02*t, .61, .16),)
    result = r.travel((1.5, 0), agents_at=person, max_ticks=300)
    assert result.reached, result.reason
    assert result.lidar_detection_ticks > 0
    assert all(s.actual_clearance > 0 for s in r.controls)
    assert baseline.travel((1.5, 0), max_ticks=300).reached
    assert any((a.linear, a.angular) != (b.linear, b.angular)
               for a, b in zip(r.controls, baseline.controls))


def test_static_grid_obstacle_respected():
    r = robot(grid(blocked=((32, 25),)))
    result = r.travel((1.5, 0), max_ticks=300)
    assert result.reached, result.reason
    assert all(r.grid.is_free(*r.grid.world_to_grid(p.x, p.y)) for p in r.trace)


def test_default_noisy_ekf_still_reaches_unobstructed_goal():
    r = DWAIntegratedRobot(grid(), Pose(0, 0, 0))
    result = r.travel((1.5, 0), max_ticks=300)
    assert result.reached, result.reason
    assert math.hypot(r.filter.pose.x - r.pose.x, r.filter.pose.y - r.pose.y) < .1


def test_integrated_delivery_two_legs():
    r = robot()
    pickup, drop = r.deliver((.9, 0), (.9, .9), max_ticks_per_leg=300)
    assert pickup.reached, pickup.reason
    assert drop is not None and drop.reached, drop.reason
    assert len(r.trace) > 10


def test_safety_fails_closed_if_pedestrian_blocks_path():
    r = robot()
    result = r.travel((1.5, 0), agents_at=lambda t: (MovingObstacle(.8, .4, .16),),
                      max_ticks=100)
    assert not result.reached
    assert result.reason.startswith("DWA:")
    assert all(s.actual_clearance > 0 for s in r.controls)


def test_no_route_fails_without_moving():
    g = grid(blocked=tuple((x, 25) for x in range(50)))
    r = DWAIntegratedRobot(g, Pose(0, -.5, 0))
    result = r.travel((0, .5))
    assert not result.reached
    assert result.reason.startswith('No A*')
    assert len(r.trace) == 1


def test_legacy_delivery_remains_available():
    legacy = SingleRobot(grid(), Pose(0, 0, 0))
    assert legacy.travel((.6, 0))


def test_rejects_bad_arguments():
    r = robot()
    try:
        r.travel((1, 1), max_ticks=0)
    except ValueError:
        pass
    else:
        assert False, 'Expected ValueError'
