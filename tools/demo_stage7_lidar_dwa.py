"""Stage 7 Part 2: simulated LiDAR -> tracker -> DWA -> robot motion.

Run from project root:
    python tools/demo_stage7_lidar_dwa.py

Synthetic environment ONLY. The planner never receives the pedestrian's true
position or velocity. Ground truth is used by the sensor simulator and the
independent collision check, not as planner input.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

from aifr.navigation.dwa import DWAPlanner
from aifr.navigation.kinematics import Pose, advance
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.perception.lidar_adapter import LiDARObstacleTracker
from aifr.perception.sensors import lidar_scan


def main() -> None:
    grid = OccupancyGrid(
        -2.0, -2.0, 0.1, 40, 40,
        tuple(tuple(0 for _ in range(40)) for _ in range(40)),
    )
    planner = DWAPlanner(grid)
    tracker = LiDARObstacleTracker()
    path = [(0.0, 0.0), (0.3, -1.0), (0.85, -1.0),
            (1.3, -0.85), (1.5, 0.0)]
    pose = Pose(0, 0, 0)
    v = w = 0.0
    rows: list[tuple[float, ...]] = []
    reached = False
    lidar_affected_ticks = 0

    for tick in range(300):
        time_s = tick * planner.config.dt
        # Only the sensor simulator knows the moving pedestrian's position.
        person_x = 0.85 - 0.04 * time_s
        person_y = 0.10
        person_radius = 0.16
        boxes = [(person_x - person_radius, person_y - person_radius,
                  person_x + person_radius, person_y + person_radius)]
        ranges = lidar_scan(pose, boxes, rays=72, max_range=4.0, step=0.02)
        perceived = tracker.observe(pose, ranges, time_s)
        result = planner.plan(pose, v, w, path, perceived)
        baseline = planner.plan(pose, v, w, path, ())
        if (result.linear, result.angular, result.safe) != (baseline.linear, baseline.angular, baseline.safe):
            lidar_affected_ticks += 1
        if not result.safe:
            raise RuntimeError(f"No feasible DWA trajectory at {time_s:.2f}s: {result.reason}")
        pose = advance(pose, result.linear, result.angular, planner.config.dt)
        v, w = result.linear, result.angular

        # Independent ground-truth check, not supplied to DWA.
        # Collision test uses the full simulated AABB, not the tracked circle.
        # Evaluate the pedestrian at the end of the control tick.
        next_person_x = 0.85 - 0.04 * (time_s + planner.config.dt)
        nearest_x = min(max(pose.x, next_person_x - person_radius), next_person_x + person_radius)
        nearest_y = min(max(pose.y, person_y - person_radius), person_y + person_radius)
        clearance = math.hypot(pose.x - nearest_x, pose.y - nearest_y) - (
            planner.config.robot_radius + planner.config.safety_margin
        )
        if clearance <= 0:
            raise RuntimeError(f"Collision with pedestrian at {time_s:.2f}s")
        rows.append((time_s, pose.x, pose.y, pose.theta, v, w,
                     len(perceived), person_x, person_y, clearance))
        if math.hypot(pose.x - 1.5, pose.y) < planner.config.goal_tolerance:
            reached = True
            break

    if not reached:
        raise RuntimeError("LiDAR-guided DWA did not reach goal within 300 ticks")
    if not lidar_affected_ticks:
        raise RuntimeError("LiDAR never changed a DWA decision; demo is not meaningful")
    output = Path("outputs/stage7_lidar_dwa_demo.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "robot_x", "robot_y", "robot_theta", "linear_m_s",
                         "angular_rad_s", "tracked_obstacles", "person_x", "person_y",
                         "ground_truth_clearance_m"])
        writer.writerows(rows)
    print(f"STAGE 7 PART 2 PASS: {len(rows)} ticks, no collisions")
    print(f"LiDAR observations changed {lidar_affected_ticks} DWA decisions")
    print(f"Trace saved to {output}")
    print("Synthetic environment only; actual Blender/Stage 5 integration is pending.")


if __name__ == "__main__":
    main()
