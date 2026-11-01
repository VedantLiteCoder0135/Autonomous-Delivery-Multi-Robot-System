"""Stage 7 Part 3: A* + EKF + LiDAR tracker + DWA, pure Python.

Run from repository root after installing Parts 1 and 2:
    python tools/demo_stage7_integrated.py

The simulated pedestrian is visible to DWA ONLY through synthetic LiDAR.
This is not a live Blender demonstration and not multi-robot coordination.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

from aifr.navigation.dwa import DWAConfig, MovingObstacle
from aifr.navigation.kinematics import Pose
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.perception.sensors import SensorSuite
from aifr.simulation.dwa_delivery import DWAIntegratedRobot


def main() -> None:
    grid = OccupancyGrid(
        -2.5, -2.5, .1, 50, 50,
        tuple(tuple(0 for _ in range(50)) for _ in range(50)),
    )
    robot = DWAIntegratedRobot(
        grid, Pose(0, 0, 0),
        config=DWAConfig(max_linear=.2),
        sensors=SensorSuite(seed=7, velocity_sigma=0, yaw_sigma=0, position_sigma=0),
    )

    def pedestrian(time_s: float):
        return (MovingObstacle(.8 + .02 * time_s, .61, .16),)

    pickup, delivery = robot.deliver((1.5, 0), (1.5, -.7),
                                     agents_at=pedestrian, max_ticks_per_leg=400)
    if not pickup.reached or delivery is None or not delivery.reached:
        raise RuntimeError(f"Integrated delivery failed: {pickup} / {delivery}")
    if pickup.lidar_detection_ticks == 0:
        raise RuntimeError("The LiDAR tracker never detected the pedestrian")
    if any(sample.actual_clearance <= 0 for sample in robot.controls):
        raise RuntimeError("Ground-truth dynamic collision guard failed")

    output = Path('outputs/stage7_integrated_trace.csv')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s', 'true_x', 'true_y', 'true_theta',
                         'ekf_x', 'ekf_y', 'ekf_theta', 'linear_m_s',
                         'angular_rad_s', 'tracked_obstacles',
                         'ground_truth_clearance_m'])
        for s in robot.controls:
            writer.writerow([s.time_s, s.true_pose.x, s.true_pose.y,
                             s.true_pose.theta, s.estimated_pose.x,
                             s.estimated_pose.y, s.estimated_pose.theta,
                             s.linear, s.angular, s.tracked_obstacles,
                             s.actual_clearance])
    print('STAGE 7 PART 3 PASS: A* + EKF + LiDAR + DWA integrated in Python')
    print(f'Pickup: {pickup.ticks} ticks; delivery: {delivery.ticks} ticks')
    print(f'LiDAR detections during pickup: {pickup.lidar_detection_ticks} ticks')
    print(f'Ground-truth minimum dynamic clearance: '
          f'{min(s.actual_clearance for s in robot.controls):.3f} m')
    print(f'Trace: {output}')
    print('Limitations: synthetic scene, ideal odometry/landmarks in this demo; '
          'no Blender live control, no multi-robot traffic coordination.')


if __name__ == '__main__':
    main()
