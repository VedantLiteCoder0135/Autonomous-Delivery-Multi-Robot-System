"""Runtime-created pedestrians for the existing V14 cafeteria simulation.

Movement is deterministic along a user-chosen line segment; this is a kinematic
pedestrian model, not crowd planning or physical simulation. Each path is
validated against the collision-inflated reference grid before admission.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from aifr.navigation.dwa import MovingObstacle


@dataclass(frozen=True)
class Pedestrian:
    pedestrian_id: int
    start: tuple[float, float]
    end: tuple[float, float]
    radius: float
    created_at: float
    period: float = 8.0

    def at(self, time_s: float) -> MovingObstacle:
        phase = 2 * math.pi * max(0., time_s - self.created_at) / self.period
        f = (1.0 - math.cos(phase)) / 2.0
        df = math.pi / self.period * math.sin(phase)
        dx, dy = self.end[0] - self.start[0], self.end[1] - self.start[1]
        return MovingObstacle(self.start[0] + dx * f, self.start[1] + dy * f,
                              self.radius, dx * df, dy * df)


class LivePedestrians:
    def __init__(self, grid, max_agents: int = 12):
        self.grid = grid
        self.max_agents = max_agents
        self.agents: dict[int, Pedestrian] = {}

    def __call__(self, time_s: float):
        return tuple(agent.at(time_s) for agent in self.agents.values())

    def add(self, x: float, y: float, end_x: float, end_y: float,
            time_s: float, *, radius: float = 0.16,
            robot_positions=()) -> int:
        vals = (x, y, end_x, end_y, time_s, radius)
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError('Coordinates and radius must be finite')
        if not 0.08 <= radius <= 0.25:
            raise ValueError('Pedestrian radius must be in [0.08, 0.25] m')
        if len(self.agents) >= self.max_agents:
            raise ValueError('Pedestrian limit reached')
        if math.dist((x, y), (end_x, end_y)) > 3.0:
            raise ValueError('Pedestrian patrol must be at most 3 metres')
        # Check complete swept disk, not just grid-centre positions.
        n = max(1, math.ceil(math.dist((x, y), (end_x, end_y)) / .04))
        for i in range(n + 1):
            t = i / n
            cx, cy = x + (end_x-x)*t, y+(end_y-y)*t
            for j in range(16):
                angle = 2 * math.pi * j / 16
                gx, gy = self.grid.world_to_grid(cx + radius*math.cos(angle),
                                                  cy + radius*math.sin(angle))
                if not self.grid.is_free(gx, gy):
                    raise ValueError('Pedestrian path overlaps occupied or non-traversable map')
        start_clearance = radius + math.hypot(.20, .18) + .08
        if any(math.dist((x, y), (rx, ry)) <= start_clearance
               for rx, ry in robot_positions):
            raise ValueError('Pedestrian spawn is too close to a robot')
        if any(math.dist((x, y), (p.x, p.y)) <= radius + p.radius + .08
               for p in self(time_s)):
            raise ValueError('Pedestrian spawn overlaps another pedestrian')
        pid = max(self.agents, default=0) + 1
        self.agents[pid] = Pedestrian(pid, (x,y), (end_x,end_y), radius, time_s)
        return pid

    def remove(self, pedestrian_id: int):
        if pedestrian_id not in self.agents:
            raise ValueError('Unknown pedestrian ID')
        del self.agents[pedestrian_id]
