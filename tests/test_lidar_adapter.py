"""Stage 7 Part 2: scan conversion and motion-estimation tests."""
from __future__ import annotations

import math
import pytest

from aifr.navigation.kinematics import Pose
from aifr.perception.lidar_adapter import (
    LiDARAdapterConfig, LiDARObstacleTracker, scan_clusters,
)
from aifr.perception.sensors import lidar_scan


def scan(pose: Pose, boxes: list[tuple[float, float, float, float]],
         rays: int = 72) -> list[float]:
    return lidar_scan(pose, boxes, rays=rays, max_range=4.0, step=0.02)


def test_empty_scan_produces_no_obstacles() -> None:
    assert scan_clusters(Pose(0, 0, 0), [4.0] * 72) == ()
    assert LiDARObstacleTracker().observe(Pose(0, 0), [4.0] * 72, 0.0) == ()


def test_detects_box_from_actual_stage6_lidar() -> None:
    pose = Pose(0, 0, 0)
    detections = scan_clusters(pose, scan(pose, [(0.9, -0.15, 1.1, 0.15)]))
    assert len(detections) == 1
    assert 0.8 < detections[0].x < 1.2
    assert abs(detections[0].y) < 0.2
    assert detections[0].radius >= 0.16
    assert detections[0].hit_count >= 2


def test_world_coordinates_rotate_with_robot() -> None:
    pose = Pose(2.0, 3.0, math.pi / 2)
    detections = scan_clusters(pose, scan(pose, [(1.85, 3.9, 2.15, 4.1)]))
    assert len(detections) == 1
    assert 1.7 < detections[0].x < 2.3
    assert 3.7 < detections[0].y < 4.3


def test_two_separated_boxes_form_two_clusters() -> None:
    pose = Pose(0, 0)
    result = scan_clusters(pose, scan(pose, [(1, -0.6, 1.2, -0.4), (1, 0.4, 1.2, 0.6)]))
    assert len(result) == 2
    assert result[0].y != result[1].y


def test_seam_wrap_merges_cluster() -> None:
    ranges = [4.0] * 36
    ranges[0] = 1.0
    ranges[1] = 1.0
    ranges[-1] = 1.0
    result = scan_clusters(Pose(0, 0), ranges)
    assert len(result) == 1
    assert result[0].hit_count == 3


def test_tracker_observes_motion_without_ground_truth() -> None:
    pose = Pose(0, 0)
    tracker = LiDARObstacleTracker(LiDARAdapterConfig(velocity_smoothing=1.0))
    first = tracker.observe(pose, scan(pose, [(0.8, -0.14, 1.08, 0.14)]), 0.0)
    second = tracker.observe(pose, scan(pose, [(0.88, -0.14, 1.16, 0.14)]), 0.15)
    assert len(first) == len(second) == 1
    assert first[0].vx == 0
    assert second[0].vx > 0.05
    assert abs(second[0].vy) < 0.1


def test_tracks_expire_when_out_of_range() -> None:
    pose = Pose(0, 0)
    tracker = LiDARObstacleTracker()
    assert len(tracker.observe(pose, scan(pose, [(0.9, -0.15, 1.1, 0.15)]), 0)) == 1
    assert len(tracker.observe(pose, [4.0] * 72, 0.05)) == 1
    assert tracker.observe(pose, [4.0] * 72, 0.25) == ()


def test_reset_clears_tracks_and_timestamp() -> None:
    tracker = LiDARObstacleTracker()
    tracker.observe(Pose(0, 0), [4.0] * 36, 1)
    tracker.reset()
    assert tracker.observe(Pose(0, 0), [4.0] * 36, 0) == ()


def test_rejects_bad_inputs() -> None:
    with pytest.raises(ValueError):
        LiDARAdapterConfig(max_range=-1)
    with pytest.raises(ValueError):
        LiDARAdapterConfig(velocity_smoothing=0)
    with pytest.raises(ValueError):
        LiDARAdapterConfig(maximum_radius=0.1)
    with pytest.raises(ValueError):
        scan_clusters(Pose(0, 0), [])
    with pytest.raises(ValueError):
        scan_clusters(Pose(0, 0), [float("nan")])
    with pytest.raises(ValueError):
        scan_clusters(Pose(0, 0), [4.1])
    tracker = LiDARObstacleTracker()
    tracker.observe(Pose(0, 0), [4.0], 1)
    with pytest.raises(ValueError):
        tracker.observe(Pose(0, 0), [4.0], 1)


def test_lidar_obstacle_changes_dwa_velocity() -> None:
    """DWA must actually consume LiDAR output rather than only log it."""
    from aifr.navigation.dwa import DWAPlanner
    from aifr.navigation.occupancy_grid import OccupancyGrid

    grid = OccupancyGrid(-2, -2, 0.1, 40, 40,
                         tuple(tuple(0 for _ in range(40)) for _ in range(40)))
    planner = DWAPlanner(grid)
    pose = Pose(0, 0)
    obstacles = LiDARObstacleTracker().observe(
        pose, scan(pose, [(0.44, -0.16, 0.76, 0.16)]), 0.0
    )
    baseline = planner.plan(pose, 0.1, 0.0, [(0, 0), (1.5, 0)])
    detected = planner.plan(pose, 0.1, 0.0, [(0, 0), (1.5, 0)], obstacles)
    assert len(obstacles) == 1
    assert detected.safe
    assert detected.linear < baseline.linear
