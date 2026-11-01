"""Standalone Stage 7 DWA smoke demo. Does not load or modify Blender scenes.

Run from the project root after `pip install -e '.[dev]'`:
    python tools/demo_stage7_dwa.py
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

from aifr.navigation.dwa import DWAPlanner, MovingObstacle
from aifr.navigation.kinematics import Pose, advance
from aifr.navigation.occupancy_grid import OccupancyGrid


def main() -> None:
    # An artificial open map, NOT a replacement for the approved cafeteria.
    grid = OccupancyGrid(
        -2.0, -2.0, 0.1, 40, 40,
        tuple(tuple(0 for _ in range(40)) for _ in range(40)),
    )
    planner = DWAPlanner(grid)
    path = [(0.0, 0.0), (0.3, -0.65), (0.85, -0.7),
            (1.3, -0.65), (1.5, 0.0)]
    pose = Pose(0.0, 0.0, 0.0)
    linear = angular = 0.0
    rows: list[tuple[float, float, float, float, float, float, float]] = []
    reached = False

    for tick in range(250):
        t = tick * planner.config.dt
        # A simulated pedestrian drifts left along the original direct route.
        person = MovingObstacle(0.85 - 0.04 * t, 0.10, 0.16, vx=-0.04)
        result = planner.plan(pose, linear, angular, path, [person])
        if not result.safe:
            raise RuntimeError(f"DWA requested stop/replan at t={t:.2f}s: {result.reason}")
        pose = advance(pose, result.linear, result.angular, planner.config.dt)
        linear, angular = result.linear, result.angular
        if math.hypot(pose.x - person.x, pose.y - person.y) <= (
            planner.config.robot_radius + person.radius + planner.config.safety_margin
        ):
            raise RuntimeError(f"Robot/pedestrian collision at t={t:.2f}s")
        rows.append((t, pose.x, pose.y, pose.theta, linear, angular, person.x))
        if math.hypot(pose.x - 1.5, pose.y) < planner.config.goal_tolerance:
            reached = True
            break

    if not reached:
        raise RuntimeError("DWA failed to reach the goal within 250 control ticks")
    output = Path("outputs/stage7_dwa_demo.csv")
    output.parent.mkdir(exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "robot_x", "robot_y", "robot_theta", "linear_m_s",
                         "angular_rad_s", "person_x"])
        writer.writerows(rows)
    print(f"STAGE 7 PART 1 PASS: {len(rows)} control ticks; no collisions")
    print(f"Trace saved to {output}")
    print("Note: synthetic-map demo only; Blender/LiDAR integration is still pending.")


if __name__ == "__main__":
    main()
