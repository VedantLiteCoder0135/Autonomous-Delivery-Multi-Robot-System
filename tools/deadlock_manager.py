"""Conservative live-cafeteria traffic arbitration.

One robot owns the shared-aisle permit at a time in Blender live mode.
This avoids head-on robot conflicts but reduces concurrency. Pedestrians
receive collision checks and bounded reroutes around active robot footprints.
It is NOT a proof of deadlock freedom in arbitrary crowd configurations.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from aifr.navigation.astar import find_path
from aifr.navigation.occupancy_grid import OccupancyGrid

from tools.traffic_coordinator import find_refuge


@dataclass(frozen=True)
class TrafficPolicy:
    robot_radius: float = math.hypot(.20, .18)
    pedestrian_radius: float = .14
    margin: float = .18  # 18 cm safety clearance (15-20 cm envelope)
    pedestrian_replan_after_ticks: int = 8
    pedestrian_warning_after_ticks: int = 120
    robot_replan_after_stalls: int = 20


class DeadlockManager:
    """Single shared-aisle robot permit and pedestrian priority handling."""

    def __init__(self, policy: TrafficPolicy | None = None):
        self.policy = policy or TrafficPolicy()
        self.customer_waits: dict[int, int] = {}
        self.customer_reroutes = 0
        self.customer_yields = 0
        self.warnings: set[int] = set()

    def can_admit_robot(self, robots, traffic_mode: str = "exclusive") -> bool:
        """Shared-aisle admission. In coordinated mode, multiple robots are permitted."""
        if traffic_mode == "coordinated":
            return True
        return not any(r.phase in ('TO_PICKUP','TO_CUSTOMER','RETURNING',
                                   'LOADING','UNLOADING') for r in robots.values())

    @staticmethod
    def robot_priority(robot) -> tuple[int, int, float, int]:
        """Ordering: carrying an order > heading to pickup > returning > others.

        Within a class, robots that have already yielded rank higher (aging, so a
        robot cannot be pushed aside forever), then the one closer to its goal, then
        the lower id as a deterministic final tie-break.
        """
        order = {'TO_CUSTOMER': 4, 'TO_PICKUP': 3, 'RETURNING': 2, 'LOADING': 1, 'UNLOADING': 1}
        p = order.get(getattr(robot, 'phase', ''), 0)
        dist = 0.0
        goal = getattr(robot, 'mission_destination', None) or getattr(robot, 'destination', None)
        if goal and getattr(robot, 'pose', None):
            dist = -math.dist((robot.pose.x, robot.pose.y), goal)
        rid = -int(getattr(robot, 'robot_id', 0))
        return (p, int(getattr(robot, 'yields', 0)), dist, rid)

    @staticmethod
    def swept_gap(a0, a1, b0, b1, radius: float) -> float:
        """Minimum centre distance over a tick (linear relative motion) minus ``radius``."""
        rx, ry = a0.x - b0.x, a0.y - b0.y
        vx, vy = (a1.x - a0.x) - (b1.x - b0.x), (a1.y - a0.y) - (b1.y - b0.y)
        denom = vx * vx + vy * vy
        t = max(0., min(1., -(rx * vx + ry * vy) / denom)) if denom > 1e-12 else 0.
        return math.hypot(rx + vx * t, ry + vy * t) - radius

    def robot_pair_conflict(self, a0, a1, b0, b1, envelope: float, physical: float) -> bool:
        """True when a proposed tick pair is unsafe.

        The soft ``envelope`` (physical footprints + safety margin) forbids *closing*
        inside it, but motion that does not reduce an already-violated gap is
        allowed. Otherwise robots whose bases sit inside each other's envelope
        veto every move and freeze forever. The hard ``physical`` footprint
        distance (plus a 2 cm buffer) is never negotiable.
        """
        if self.swept_gap(a0, a1, b0, b1, physical + .02) <= 0:
            return True
        now = self.swept_gap(a0, a0, b0, b0, envelope)
        swept = self.swept_gap(a0, a1, b0, b1, envelope)
        return swept <= 0 and swept < now - 1e-6

    def dynamic_obstacle_step_safe(self, old, new, dynamic_obstacles, radius=None):
        """Swept-circle safety check against generic dynamic obstacles (e.g., moved chairs)."""
        p = self.policy
        r = p.pedestrian_radius if radius is None else radius
        for obs in dynamic_obstacles:
            ox = getattr(obs, 'x', obs[0] if isinstance(obs, (tuple, list)) else 0.0)
            oy = getattr(obs, 'y', obs[1] if isinstance(obs, (tuple, list)) else 0.0)
            obs_radius = getattr(obs, 'radius', 0.20)
            dx, dy = new[0] - old[0], new[1] - old[1]
            denom = dx * dx + dy * dy
            t = (max(0., min(1., ((ox - old[0]) * dx + (oy - old[1]) * dy) / denom))
                 if denom > 1e-12 else 0.)
            if math.hypot(old[0] + dx * t - ox, old[1] + dy * t - oy) < obs_radius + r + p.margin:
                return False
        return True

    def clearance(self, x, y, robots, customer_radius=None):
        radius = self.policy.pedestrian_radius if customer_radius is None else customer_radius
        return min((math.hypot(x-r.pose.x, y-r.pose.y) -
                    self.policy.robot_radius - radius - self.policy.margin
                    for r in robots.values()), default=float('inf'))

    def pedestrian_step_safe(self, old, new, robots, radius=None):
        """Conservative swept-point vs current robot discs, including stationary robots."""
        p = self.policy
        r = p.pedestrian_radius if radius is None else radius
        for robot in robots.values():
            dx,dy=new[0]-old[0],new[1]-old[1]
            denom=dx*dx+dy*dy
            if denom < 1e-12:
                t=0.
            else:
                t=max(0.,min(1.,((robot.pose.x-old[0])*dx+(robot.pose.y-old[1])*dy)/denom))
            if math.hypot(old[0]+dx*t-robot.pose.x,old[1]+dy*t-robot.pose.y) < p.robot_radius+r+p.margin:
                return False
        return True

    def customer_step_safe(self, old, new, customers, moving_id, radius=None):
        """Reject swept pedestrian-to-pedestrian overlaps, including seated customers.

        QUEUED people are outside the scene; a seated customer still occupies
        their seat. This is deliberately conservative and not ORCA crowd flow.
        """
        r=self.policy.pedestrian_radius if radius is None else radius
        dx,dy=new[0]-old[0],new[1]-old[1]
        denom=dx*dx+dy*dy
        for other in customers.values():
            if other.customer_id == moving_id or other.phase in ('QUEUED','LEFT'):
                continue
            t=(max(0.,min(1.,((other.x-old[0])*dx+(other.y-old[1])*dy)/denom))
               if denom>1e-12 else 0.)
            if math.hypot(old[0]+dx*t-other.x,old[1]+dy*t-other.y) < r+other.radius+.02:
                return False
        return True

    def customer_blocker(self, old, new, customers, moving_id, radius=None):
        """The customer a swept step would overlap (nearest first), or None."""
        r=self.policy.pedestrian_radius if radius is None else radius
        dx,dy=new[0]-old[0],new[1]-old[1]
        denom=dx*dx+dy*dy
        hit=None
        for other in customers.values():
            if other.customer_id == moving_id or other.phase in ('QUEUED','LEFT'):
                continue
            t=(max(0.,min(1.,((other.x-old[0])*dx+(other.y-old[1])*dy)/denom))
               if denom>1e-12 else 0.)
            gap=math.hypot(old[0]+dx*t-other.x,old[1]+dy*t-other.y)-(r+other.radius+.02)
            if gap<0 and (hit is None or gap<hit[0]):
                hit=(gap,other)
        return None if hit is None else hit[1]

    def reroute_around_robots(self, grid: OccupancyGrid, start, goal, robots,
                              customer_radius: float | None = None):
        """Search with robot discs temporarily blocked; never mutate base grid."""
        radius=(self.policy.pedestrian_radius if customer_radius is None else customer_radius)
        block_radius=self.policy.robot_radius+radius+self.policy.margin
        rows=[list(row) for row in grid.cells]
        for robot in robots.values():
            cx,cy=robot.pose.x,robot.pose.y
            a,b=grid.world_to_grid(cx-block_radius, cy-block_radius)
            c,d=grid.world_to_grid(cx+block_radius, cy+block_radius)
            for row in range(max(0,b),min(grid.height,d+1)):
                for col in range(max(0,a),min(grid.width,c+1)):
                    x,y=grid.grid_to_world(col,row)
                    if math.hypot(x-cx,y-cy)<block_radius+grid.resolution*.71:
                        rows[row][col]=1
        sc, sr = grid.world_to_grid(*start)
        if grid.is_free(sc, sr):
            rows[sr][sc] = 0
        overlay=OccupancyGrid(grid.origin_x,grid.origin_y,grid.resolution,
                              grid.width,grid.height,tuple(map(tuple,rows)))
        if not overlay.is_free(sc, sr):
            return []  # Current position inside a keepout: never teleport.
        return find_path(overlay,start,goal)

    def _agent_overlay(self, grid, start, robots, customers, moving_id, radius, dynamic_obstacles):
        """Grid copy with robots, other pedestrians and loose obstacles blocked out.

        The mover's own start is never blocked. Returns None when the start itself is
        inside a keep-out (never teleport through anyone).
        """
        rows=[list(row) for row in grid.cells]
        obstacles=[(r.pose.x,r.pose.y,self.policy.robot_radius+radius+self.policy.margin)
                   for r in robots.values()]
        for other in customers.values():
            if other.customer_id != moving_id and other.phase not in ('QUEUED','LEFT'):
                obstacles.append((other.x,other.y,other.radius+radius+.02))
        for obs in dynamic_obstacles:
            ox = getattr(obs, 'x', obs[0] if isinstance(obs, (tuple, list)) else 0.0)
            oy = getattr(obs, 'y', obs[1] if isinstance(obs, (tuple, list)) else 0.0)
            obs_radius = getattr(obs, 'radius', 0.20)
            obstacles.append((ox, oy, obs_radius + radius + self.policy.margin))
        for x,y,keepout in obstacles:
            c0,r0=grid.world_to_grid(x-keepout-grid.resolution,y-keepout-grid.resolution)
            c1,r1=grid.world_to_grid(x+keepout+grid.resolution,y+keepout+grid.resolution)
            for rr in range(max(0,r0),min(grid.height,r1+1)):
                for cc in range(max(0,c0),min(grid.width,c1+1)):
                    px,py=grid.grid_to_world(cc,rr)
                    if math.hypot(px-x,py-y)<keepout+grid.resolution*.71:
                        rows[rr][cc]=1
        sc, sr = grid.world_to_grid(*start)
        if grid.is_free(sc, sr):
            rows[sr][sc] = 0
        overlay=OccupancyGrid(grid.origin_x,grid.origin_y,grid.resolution,
                              grid.width,grid.height,tuple(map(tuple,rows)))
        return overlay if overlay.is_free(sc, sr) else None

    def reroute_around_agents(self, grid: OccupancyGrid, start, goal, robots,
                              customers, moving_id, customer_radius=None,
                              dynamic_obstacles=()):
        """A* overlay of robots, pedestrians, and dynamic obstacles; original map unchanged.

        Don't block the customer's own start. If a seated customer or obstacle occupies the
        target, report no route rather than teleporting through them.
        """
        radius=self.policy.pedestrian_radius if customer_radius is None else customer_radius
        overlay=self._agent_overlay(grid,start,robots,customers,moving_id,radius,dynamic_obstacles)
        return [] if overlay is None else find_path(overlay,start,goal)

    def yield_route(self, grid: OccupancyGrid, lanes, start, robots, customers, moving_id,
                    customer_radius=None, dynamic_obstacles=()):
        """Route to a nearby passing bay where an oncoming agent can get by.

        ``(path, bay)`` or ``([], None)``. A single-lane aisle cannot be shared, so the
        walker that must give way has to walk clear of it, not just shuffle a few cm.
        """
        radius=self.policy.pedestrian_radius if customer_radius is None else customer_radius
        overlay=self._agent_overlay(grid,start,robots,customers,moving_id,radius,dynamic_obstacles)
        if overlay is None:
            return [], None
        reach=self.policy.robot_radius+radius+self.policy.margin+.5
        avoid=[(r.pose.x,r.pose.y,reach) for r in robots.values()]
        for other in customers.values():
            if other.customer_id == moving_id or other.phase in ('QUEUED','LEFT'):
                continue
            avoid.append((other.x,other.y,other.radius+radius+.7))
            if other.phase in ('TO_CASH','TO_TABLE'):
                avoid += [(px,py,other.radius+radius+.35)
                          for px,py in other.route[other.route_index:other.route_index+10]]
        bay=find_refuge(overlay,lanes,start,avoid,max_path=8.0)
        if bay is None:
            return [], None
        path=find_path(overlay,start,bay)
        return (path,bay) if path else ([],None)

    def record_block(self, customer_id: int):
        ticks=self.customer_waits.get(customer_id,0)+1
        self.customer_waits[customer_id]=ticks
        self.customer_yields+=1
        return ticks

    def clear_block(self, customer_id: int):
        self.customer_waits.pop(customer_id,None)
        self.warnings.discard(customer_id)
