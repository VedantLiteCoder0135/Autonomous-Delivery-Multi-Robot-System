"""Stage 7 safe route-following DWA with explicit stopping feasibility.

Unlike the earlier constant-speed 1.5s planner, this variant can rotate in
place at corners and never chooses a command without a collision-free
acceleration-limited emergency braking rollout. Uses an inflated Stage 4 grid.
It is a prototype safety check, not a physical certification.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .dwa import DWAConfig, MovingObstacle
from .kinematics import Pose, advance, normalize
from .occupancy_grid import OccupancyGrid


@dataclass(frozen=True)
class RouteCommand:
    linear: float
    angular: float
    safe: bool
    reached: bool = False
    reason: str = "OK"


def line_clear(grid: OccupancyGrid, a: tuple[float, float], b: tuple[float, float], extra_clearance: float = 0.0) -> bool:
    """Conservative dense line-of-sight check on an inflated occupancy grid with diagonal corner protection."""
    distance = math.dist(a, b)
    n = max(1, math.ceil(distance / (grid.resolution * .1)))
    prev_cell = None
    for i in range(n + 1):
        t = i / n
        x = a[0] + t * (b[0] - a[0]); y = a[1] + t * (b[1] - a[1])
        probes = ((0., 0.),) if extra_clearance <= 0 else (
            (0., 0.), (extra_clearance, 0.), (-extra_clearance, 0.),
            (0., extra_clearance), (0., -extra_clearance),
            (.71*extra_clearance, .71*extra_clearance),
            (-.71*extra_clearance, .71*extra_clearance),
            (.71*extra_clearance, -.71*extra_clearance),
            (-.71*extra_clearance, -.71*extra_clearance))
        for dx, dy in probes:
            c, r = grid.world_to_grid(x + dx, y + dy)
            if not grid.is_free(c, r):
                return False
            if prev_cell is not None and prev_cell != (c, r):
                pc, pr = prev_cell
                if pc != c and pr != r:
                    if not grid.is_free(pc, r) or not grid.is_free(c, pr):
                        return False
            prev_cell = (c, r)
    return True


class RouteDWA:
    def __init__(self, grid: OccupancyGrid, config: DWAConfig | None = None):
        self.grid = grid
        self.config = config or DWAConfig()
        self.path: list[tuple[float, float]] = []
        self.index = 0
        self.last_target: tuple[float, float] | None = None

    def set_path(self, path: Sequence[tuple[float, float]]) -> None:
        if not path or any(not self.grid.is_free(*self.grid.world_to_grid(*p)) for p in path):
            raise ValueError('Route is empty or contains occupied points')
        if any(not line_clear(self.grid, a, b) for a, b in zip(path, path[1:])):
            raise ValueError('Route contains blocked segments')
        self.path = list(path)
        self.index = 0

    def _target(self, pose: Pose) -> tuple[float, float]:
        here = (pose.x, pose.y)
        # Near a grid corner, proximity is NOT enough to mark a waypoint
        # passed. Only advance if the next waypoint is directly visible from
        # the current pose on the inflated occupancy grid. Otherwise the
        # controller can cut through a blocked adjacent cell and stall.
        while (self.index < len(self.path) - 1
               and (math.dist(here, self.path[self.index]) < .22
                    or math.dist(here, self.path[self.index + 1]) < math.dist(here, self.path[self.index]))
               and line_clear(self.grid, here, self.path[self.index + 1])):
            self.index += 1
        # Look ahead only if the direct segment is clear; don't cut corners.
        target = self.path[self.index]
        for j in range(self.index + 1, min(len(self.path), self.index + 10)):
            if math.dist(here, self.path[j]) > .50:
                break
            if line_clear(self.grid, here, self.path[j]):
                target = self.path[j]
            else:
                break
        self.last_target = target
        return target

    def _gap(self, pose: Pose, time: float, obstacles: Sequence[MovingObstacle]) -> float:
        c = self.config
        return min((math.hypot(pose.x - o.x - o.vx*time,
                               pose.y - o.y - o.vy*time)
                    - c.robot_radius - c.safety_margin - o.radius for o in obstacles),
                   default=float('inf'))

    def _free(self, pose: Pose, time: float, obstacles: Sequence[MovingObstacle], min_allowed: float = 0.0) -> bool:
        if not self.grid.is_free(*self.grid.world_to_grid(pose.x, pose.y)):
            return False
        return self._gap(pose, time, obstacles) >= min_allowed and self._physical_free(pose, time, obstacles)

    def _physical_free(self, pose: Pose, time: float, obstacles: Sequence[MovingObstacle], tolerance: float = 0.0) -> bool:
        """Physical collision check for emergency braking rollout (preventing actual contact)."""
        if not self.grid.is_free(*self.grid.world_to_grid(pose.x, pose.y)):
            return False
        c = self.config
        return min((math.hypot(pose.x - o.x - o.vx*time,
                               pose.y - o.y - o.vy*time)
                    - c.robot_radius - o.radius - 0.02 + tolerance for o in obstacles),
                   default=float('inf')) > 0

    def _admissible(self, pose: Pose, v: float, w: float,
                    obstacles: Sequence[MovingObstacle]) -> tuple[bool, float]:
        """Predict constant command, then separately validate a physical stop."""
        c = self.config
        dt = c.dt
        # 0.65s lookahead for obstacle avoidance. At a corner a full 1.5s
        # constant-speed rollout is over-conservative; braking is checked too.
        steps = max(1, math.ceil(.65 / dt))
        state = pose
        min_gap = float('inf')
        cur_gap = self._gap(pose, 0., obstacles)
        # Already inside a safety margin (a person stepped in): permit only motion that
        # does not close the gap further. Allowing a few cm per tick let robots creep
        # to near contact and wedge themselves against the person.
        min_allowed = min(0.0, cur_gap - 0.002)
        for i in range(1, steps + 1):
            state = advance(state, v, w, dt)
            if not self._free(state, i*dt, obstacles, min_allowed=min_allowed):
                return False, min_gap
            min_gap = min(min_gap, self._gap(state, i*dt, obstacles))
        # Verify that after the selected tick, max-rate braking is safe.
        if not self._brake_safe(pose, v, w, obstacles):
            return False, min_gap
        return True, min_gap

    def _brake_safe(self, pose: Pose, v: float, w: float,
                    obstacles: Sequence[MovingObstacle]) -> bool:
        c=self.config;dt=c.dt
        state = advance(pose, v, w, dt)
        if not self._physical_free(state,dt,obstacles):return False
        vv, ww = v, w
        for i in range(2, 20):
            vv = math.copysign(max(0., abs(vv) - c.linear_acceleration*dt), vv)
            ww = math.copysign(max(0., abs(ww) - c.angular_acceleration*dt), ww)
            state = advance(state, vv, ww, dt)
            if not self._physical_free(state, i*dt, obstacles):
                return False
            if vv == 0 and ww == 0:
                break
        return True

    def plan(self, pose: Pose, current_v: float, current_w: float,
             obstacles: Sequence[MovingObstacle] = ()) -> RouteCommand:
        if not self.path:
            return RouteCommand(0., 0., False, reason='No global route')
        if not self.grid.is_free(*self.grid.world_to_grid(pose.x, pose.y)):
            return RouteCommand(0., 0., False, reason='Current estimated pose is blocked on static grid')
        if not self._physical_free(pose, 0., obstacles, tolerance=0.04):
            return RouteCommand(0., 0., False, reason='Current estimated pose is in physical collision')
        c = self.config
        goal = self.path[-1]
        remaining = math.dist((pose.x, pose.y), goal)
        if remaining <= c.goal_tolerance and abs(current_v) < .05:
            return RouteCommand(0., 0., True, reached=True, reason='Goal reached')
        target = self._target(pose)
        dx, dy = target[0] - pose.x, target[1] - pose.y
        error = normalize(math.atan2(dy, dx) - pose.theta)
        reverse = c.allow_reverse and abs(error) > math.pi / 2
        if reverse:
            error = normalize(error + math.pi)
        # Slow or rotate before a sharp corner; allow room to brake.
        desired_w = max(-c.max_angular, min(c.max_angular, 3.0*error))
        if abs(error) > .36:
            desired_v = 0.
        elif abs(error) > .16:
            desired_v = .09
        else:
            desired_v = min(.25, max(0., remaining - .06) * .65)
        if remaining < .20:
            desired_v = min(desired_v, .10)
        if reverse:
            desired_v = -desired_v
        # Strict dynamic window, with endpoint samples to allow gradual braking.
        vlow = max(-c.max_linear if c.allow_reverse else 0., current_v - c.linear_acceleration*c.dt)
        vhigh = min(c.max_linear, current_v + c.linear_acceleration*c.dt)
        wlow = max(-c.max_angular, current_w - c.angular_acceleration*c.dt)
        whigh = min(c.max_angular, current_w + c.angular_acceleration*c.dt)
        def samples(low, high, step, preferred):
            values = [low, high, max(low, min(high, preferred))]
            n = max(1, math.ceil((high-low)/step))
            values += [low + (high-low)*i/n for i in range(n+1)]
            if low <= 0 <= high:
                values.append(0.)
            return sorted(set(round(x, 8) for x in values))
        vs = samples(vlow, vhigh, .04, desired_v)
        ws = samples(wlow, whigh, .15, desired_w)
        best = None
        for v in vs:
            for w in ws:
                admissible, gap = self._admissible(pose, v, w, obstacles)
                if not admissible:
                    continue
                # Route tracking dominates speed; dynamically detected people
                # also bias the score toward more clearance.
                nxt = advance(pose, v, w, c.dt)
                new_error = abs(normalize(math.atan2(target[1]-nxt.y, target[0]-nxt.x)-nxt.theta))
                penalty = (3.0*abs(v-desired_v) + .9*abs(w-desired_w)
                           + .25*new_error + .01*max(0., .5-min(gap, .5)))
                if best is None or penalty < best[0]:
                    best = (penalty, v, w)
        if best is None:
            # Constant-velocity prediction can reject a safe emergency stop.
            # Execute the acceleration-limited braking command if its complete
            # braking rollout is collision-free; never teleport velocity to 0.
            fallback=[]
            brake_v = max(vlow, min(vhigh, 0.))
            for w in ws:
                if self._brake_safe(pose, brake_v, w, obstacles):
                    fallback.append((abs(w-desired_w),abs(w),w))
            if fallback:
                return RouteCommand(brake_v,min(fallback)[2],True,reason='Controlled braking')
            return RouteCommand(0., 0., False, reason='No collision-free braking-admissible DWA command')
        return RouteCommand(best[1], best[2], True)
