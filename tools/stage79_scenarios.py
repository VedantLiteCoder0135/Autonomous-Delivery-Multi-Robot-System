"""Reproducible, geometry-validated V14 pedestrian demonstration scenario.

This is a deliberately gentle moving pedestrian near the T11 approach.
It is NOT a general crowded-cafeteria benchmark or an occlusion model.
"""
from __future__ import annotations

import math
from aifr.navigation.dwa import MovingObstacle
from aifr.navigation.occupancy_grid import OccupancyGrid


def v14_pedestrian(grid: OccupancyGrid):
    """Return a small moving obstacle near the validated Sandwich→T11 route.

    Calibrated against the supplied V14 exported grid, not arbitrary maps.
    Require a free disk throughout the trajectory or reject the scenario.
    """
    x0, y0, radius = 8.28, 5.95, .16
    for k in range(60):
        t = k * 1.3
        x = x0 + .01 * math.sin(.3*t)
        y = y0 + .03 * math.sin(.23*t)
        for j in range(24):
            a = 2*math.pi*j/24
            xx = x + radius*math.cos(a)
            yy = y + radius*math.sin(a)
            if not grid.is_free(*grid.world_to_grid(xx, yy)):
                raise ValueError('V14 pedestrian scenario is not free on the supplied grid')
    def at(t: float):
        return (MovingObstacle(x0 + .01*math.sin(.3*t),
                               y0 + .03*math.sin(.23*t), radius,
                               .003*math.cos(.3*t), .0069*math.cos(.23*t)),)
    return at
