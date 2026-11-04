"""Stages 7–9 fixed-step Python engine, independent of Blender.

FCFS order assignment is delegated to the original Stage 2 scheduler, with
opt-in deferred release until physical return to base. ALNS optimizes each
fixed route; RouteDWA selects local velocities; LiDAR and EKF supply local
perception/pose. Centralized next-step reservations prevent inter-robot overlap.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Callable, Sequence

from aifr.common.clock import SimClock
from aifr.common.events import EventBus
from aifr.environment.environment_mapper import EnvironmentMapper
from aifr.environment.waypoint_manager import WaypointKind, WaypointManager
from aifr.fleet.registry import RobotRegistry
from aifr.navigation.alns import ALNSRouteOptimizer
from aifr.navigation.astar import find_path
from aifr.navigation.collision_checker import CollisionChecker
from aifr.navigation.dwa import DWAConfig, MovingObstacle
from aifr.navigation.dwa_route import RouteDWA
from aifr.navigation.kinematics import Pose, advance
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.orders.manager import OrderManager
from aifr.orders.model import OrderStatus
from aifr.perception.ekf import PoseEKF
from aifr.perception.lidar_adapter import LiDARObstacleTracker
from aifr.perception.sensors import SensorSuite, lidar_scan
from aifr.scheduling.scheduler import GlobalScheduler

from tools.deadlock_manager import DeadlockManager, TrafficPolicy
from tools.traffic_coordinator import (LaneMap, WaitForGraph, corridor_conflict,
                                       find_refuge, lane_claim)

AgentProvider = Callable[[float], Sequence[MovingObstacle]]

_LANE_CACHE: dict[int, tuple[OccupancyGrid, LaneMap]] = {}


def _lane_map(grid: OccupancyGrid) -> LaneMap:
    """Single-lane classification is static per grid; compute it once."""
    hit = _LANE_CACHE.get(id(grid))
    if hit is None or hit[0] is not grid:
        hit = (grid, LaneMap(grid))
        _LANE_CACHE[id(grid)] = hit
    return hit[1]


def load_reference(project: Path):
    """Load prior *user-exported* cafeteria data; never infer geometry from .blend."""
    from tools.stage7_cafeteria_core import _environment_from_json, _grid_from_json
    folder = project / 'data' / 'reference_cafeteria'
    env = _environment_from_json(json.loads((folder/'stage3_environment.json').read_text()))
    grid = _grid_from_json(json.loads((folder/'stage4_occupancy_grid.json').read_text()))
    report = json.loads((folder/'stage36_report.json').read_text())
    if not report.get('success') or not report['mission'].get('trace_footprint_clear'):
        raise ValueError('Stage 1–6 reference report was not successful')
    # Validate metadata against *actual exported wall rectangles*.
    wall_objects = [{'name': w.name,
                     'bbox': {'min_x': w.rect.min_x, 'max_x': w.rect.max_x,
                              'min_y': w.rect.min_y, 'max_y': w.rect.max_y}}
                    for w in env.walls]
    door_data = report.get('v14_door_metadata')
    if not door_data:
        raise ValueError('Four-door metadata is missing from Stage 1–6 report')
    doors = EnvironmentMapper({'objects': wall_objects,
                               'custom_props': {'AIFR_doors': json.dumps(door_data)}}).map()
    if len(doors.entrances) != 4:
        raise ValueError('Expected exactly four wall-verified entrances')
    env.entrances = doors.entrances
    return env, grid, report


@dataclass
class RobotRuntime:
    robot_id: int
    base: Pose
    pose: Pose
    phase: str = 'IDLE'
    order_id: int | None = None
    destination: tuple[float,float] | None = None
    linear: float = 0.
    angular: float = 0.
    stalls: int = 0
    recovery_attempts: int = 0
    blocked_reason: str = ""
    nav_ticks: int = 0
    dwell: int = 0
    detected_ticks: int = 0
    trace: list[dict] = field(default_factory=list)
    tracker: LiDARObstacleTracker = field(default_factory=LiDARObstacleTracker)
    sensor: SensorSuite = field(default_factory=lambda: SensorSuite(seed=23,velocity_sigma=0.,yaw_sigma=0.,position_sigma=0.))
    ekf: PoseEKF = field(init=False)
    planner: RouteDWA | None = None
    # Deadlock recovery state. The scheduler-visible ``phase`` never changes
    # while yielding: a retreating robot keeps its order and resumes it.
    mission_destination: tuple[float,float] | None = None
    retreating: bool = False
    hold: int = 0                    # ticks left waiting at a refuge / corridor entry
    hold_reason: str = ''
    yields: int = 0                  # completed yields: ages priority so nobody starves
    escalation: int = 0              # recovery ladder rung since last progress
    best_remaining: float = float('inf')
    best_heading: float = math.pi
    progress_tick: int = 0
    refuges: list = field(default_factory=list)
    corridor_waited: int = 0
    corridor_waived: int = 0
    wait_on: str = ''
    navigation_state: str = 'READY'
    blocked_signature: tuple | None = None
    recovery_budget: int = 0
    mission_best_distance: float = float('inf')
    mission_progress_tick: int = 0
    esc_mark: float = float('inf')   # remaining route length at the last stuck event

    def __post_init__(self):
        self.ekf = PoseEKF(self.pose)


@dataclass
class DynamicObstacle:
    name: str
    x: float
    y: float
    radius: float = 0.20
    is_static: bool = True


class Stage79Engine:
    def __init__(self, env, grid: OccupancyGrid, robot_count: int = 2,
                 pedestrians: AgentProvider | None = None, seed: int = 11,
                 traffic_mode: str = "standard", safety_margin: float = 0.18):
        if not 1 <= robot_count <= 4:
            raise ValueError('Demo supports 1–4 robots')
        if traffic_mode not in ("standard", "exclusive", "coordinated"):
            raise ValueError("traffic_mode must be standard, exclusive, or coordinated")
        self.traffic_mode = traffic_mode
        self.safety_margin = safety_margin
        self.deadlock_manager = DeadlockManager(TrafficPolicy(margin=safety_margin))
        self.dynamic_obstacles: dict[str, DynamicObstacle] = {}
        self.dynamic_replans = 0
        self.env, self.grid = env, grid
        self.lanes = _lane_map(grid)
        self.wait_graph = WaitForGraph()
        self.robot_envelope = 2 * math.hypot(.2, .18) + safety_margin + .04  # = DWA's gap to a robot
        self.deadlocks_resolved = 0
        self.retreats = 0
        self.corridor_waits = 0
        self.stuck_events = 0
        self._veto: dict[int, int] = {}
        self.checker = CollisionChecker(env)
        self.clock = SimClock(50_000)
        self.bus = EventBus(record=True)
        self.orders = OrderManager(self.bus,self.clock)
        # A customer owns exactly one immutable order identity after payment.
        self.order_customers: dict[int,int] = {}
        self.customer_orders: dict[int,int] = {}
        self.fleet = RobotRegistry(self.bus,self.clock,robot_count)
        self.scheduler = GlobalScheduler(self.bus,self.clock,self.orders,self.fleet,
                                         release_on_delivery=False)
        self.pedestrians = pedestrians or (lambda t: ())
        self.walkers = None  # LivePedestrians (patrol segments), if the host has them
        self.crowd = None   # CustomerFlow registers itself: gives recovery the customers' routes
        self.seed=seed
        self.paused=True
        self.estopped=False
        self.speed=1.
        self.failures: list[str] = []
        self.events: list[dict] = []
        self.routes: list[dict] = []
        self.yield_events=0
        self.min_robot_clearance=float('inf')
        self.min_pedestrian_clearance=float('inf')
        self.history: list[dict] = []
        self.waypoints = WaypointManager(env).generate().valid
        self.pickups = {}
        self.drops = {}
        for w in self.waypoints:
            if not grid.is_free(*grid.world_to_grid(w.x,w.y)):
                continue
            if w.kind is WaypointKind.PICKUP:
                key = w.label.removeprefix('pickup_').split('_0')[0]
                self.pickups.setdefault(key,[]).append(w)
            elif w.kind is WaypointKind.DELIVERY:
                key=w.label.split('_')[1]
                self.drops.setdefault(key,[]).append(w)
        if not self.pickups or not self.drops:
            raise ValueError('No validated pickup or delivery waypoints')
        self.robots = self._create_robots(robot_count)

    def _create_robots(self, count):
        # Prefer actual valid Stage 3 spawn candidates, but allocate separate
        # collision-free base slots around the validated primary base camp.
        spawns = [w for w in self.waypoints if w.kind is WaypointKind.SPAWN
                  and self.grid.is_free(*self.grid.world_to_grid(w.x,w.y))]
        if not spawns:
            raise ValueError('No validated robot spawn positions')
        center=spawns[0]
        candidates=[]
        for row in range(self.grid.height):
            for col in range(self.grid.width):
                x,y=self.grid.grid_to_world(col,row)
                if math.dist((x,y),(center.x,center.y)) > 2.1:
                    continue
                if not self.grid.is_free(col,row): continue
                if not self.checker.is_pose_free(x,y,center.heading):continue
                candidates.append((math.dist((x,y),(center.x,center.y)),x,y))
        candidates.sort()
        chosen=[Pose(center.x,center.y,center.heading)]
        for _,x,y in candidates:
            if len(chosen)>=count:break
            if all(math.dist((x,y),(b.x,b.y))>=1.0 for b in chosen):
                if find_path(self.grid,(x,y),(center.x,center.y)):
                    chosen.append(Pose(x,y,center.heading))
        if len(chosen)<count:
            raise ValueError('Insufficient separated, traversable base slots')
        return {i+1:RobotRuntime(i+1,p,p) for i,p in enumerate(chosen)}

    def add_robot(self) -> int:
        """Add a robot at a separately validated base while the clock is running.

        This V14 routing implementation currently supports at most four robots.
        Existing poses, assigned orders and base slots are never changed.
        """
        if len(self.robots) >= 4:
            raise ValueError('V14 live simulator supports at most four robots')
        candidate = self._create_robots(len(self.robots) + 1)[len(self.robots) + 1]
        # Reject if an active robot is currently passing the new base slot.
        safe_gap = 2 * (math.hypot(.2,.18) + .05)
        if any(math.dist((candidate.pose.x,candidate.pose.y), (r.pose.x,r.pose.y)) <= safe_gap
               for r in self.robots.values()):
            raise ValueError('New robot base is temporarily occupied; retry when clear')
        rid = int(self.fleet.add_robot())
        self.robots[rid] = candidate
        self._event('ROBOT_ADDED', robot_id=rid, x=candidate.pose.x, y=candidate.pose.y)
        return rid

    def _event(self, name, **fields):
        self.events.append({'tick':self.clock.tick,'event':name,**fields})

    def create_order(self, table: str, station: str, ready: bool = True,
                     customer_id: int | None = None) -> int:
        table=table.upper();station=station.upper()
        if table not in self.drops or station not in self.pickups:
            raise ValueError(f'Unknown or unreachable table/station: {table}/{station}')
        if customer_id is not None and customer_id in self.customer_orders:
            raise ValueError(f'Customer {customer_id} has already paid for order {self.customer_orders[customer_id]}')
        o=self.orders.create_order(table,station)
        if customer_id is not None:
            self.customer_orders[customer_id]=int(o.order_id)
            self.order_customers[int(o.order_id)]=customer_id
        self.orders.start_preparing(o.order_id)
        if ready:self.orders.mark_ready(o.order_id)
        self._event('ORDER_CREATED',order_id=int(o.order_id),customer_id=customer_id,table=table,station=station,ready=ready)
        return int(o.order_id)

    def ready_order(self, order_id: int, customer_id: int | None = None):
        if customer_id is not None and self.order_customers.get(int(order_id)) != customer_id:
            raise ValueError('Customer/order identity mismatch while marking ready')
        self.orders.mark_ready(order_id)
        self._event('ORDER_READY',order_id=order_id)

    def add_dynamic_obstacle(self, x: float, y: float, radius: float = 0.20, name: str | None = None) -> DynamicObstacle:
        if name is None:
            name = f"OBSTACLE_{len(self.dynamic_obstacles) + 1}"
        obs = DynamicObstacle(name=name, x=x, y=y, radius=radius)
        self.dynamic_obstacles[name] = obs
        self._event('OBSTACLE_ADDED', obstacle=name, x=round(x, 3), y=round(y, 3), radius=round(radius, 3))
        return obs

    def remove_dynamic_obstacle(self, name: str) -> None:
        if name in self.dynamic_obstacles:
            del self.dynamic_obstacles[name]
            self._event('OBSTACLE_REMOVED', obstacle=name)

    def clear_dynamic_obstacles(self) -> None:
        self.dynamic_obstacles.clear()
        self._event('OBSTACLES_CLEARED')

    def _best_waypoint(self, options, start):
        ranked=[]
        for w in options:
            path=find_path(self.grid,start,(w.x,w.y))
            if path:
                ranked.append((len(path),w.label,(w.x,w.y)))
        if not ranked:
            if not options:
                raise ValueError('No waypoint configured')
            # Preserve a concrete mission goal so route setup can enter BLOCKED
            # without losing the assignment or aborting the live timer.
            waypoint = min(options, key=lambda w: (math.dist(start, (w.x, w.y)), w.label))
            return waypoint.x, waypoint.y
        return min(ranked)[2]

    def _best_unblocked_waypoint(self, options, start, grid: OccupancyGrid):
        ranked = []
        for w in options:
            col, row = grid.world_to_grid(w.x, w.y)
            if not grid.is_free(col, row):
                continue
            path = find_path(grid, start, (w.x, w.y))
            if path:
                ranked.append((len(path), w.label, (w.x, w.y)))
        if not ranked:
            return None
        return min(ranked)[2]

    def _traffic_grid(self, robot: RobotRuntime, extra_blocked: Sequence[tuple[float, float, float]] = (), *, reserve_patrol: bool = True, reserve_agents: bool = True) -> OccupancyGrid:
        """Temporarily reserve occupied base slots, other stationary robots, and dynamic obstacles."""
        g = self.grid
        rows = [list(row) for row in g.cells]

        # 1. Other robots: reserve parked, waiting, or other robots in transit
        other_robots = [other for other in self.robots.values() if other.robot_id != robot.robot_id] if reserve_agents else []
        block_dist = self.robot_envelope + .02
        for other in other_robots:
            positions = [(other.pose.x, other.pose.y)]
            # If the other robot is in active transit, also mark its immediate upcoming trajectory waypoints
            if other.phase in ('TO_PICKUP', 'TO_CUSTOMER', 'RETURNING') and other.planner and other.planner.path:
                idx = other.planner.index
                for p in other.planner.path[idx : min(len(other.planner.path), idx + 2)]:
                    positions.append(p)
            for px, py in positions:
                min_c, min_r = g.world_to_grid(px - block_dist, py - block_dist)
                max_c, max_r = g.world_to_grid(px + block_dist, py + block_dist)
                for row in range(max(0, min_r), min(g.height, max_r + 1)):
                    for col in range(max(0, min_c), min(g.width, max_c + 1)):
                        x, y = g.grid_to_world(col, row)
                        if math.hypot(x - px, y - py) <= block_dist:
                            rows[row][col] = 1

        # 2. Dynamic obstacles (e.g. moved chairs)
        for obs in self.dynamic_obstacles.values():
            block_r = obs.radius + math.hypot(.2, .18) + self.safety_margin + 0.04
            min_c, min_r = g.world_to_grid(obs.x - block_r, obs.y - block_r)
            max_c, max_r = g.world_to_grid(obs.x + block_r, obs.y + block_r)
            for row in range(max(0, min_r), min(g.height, max_r + 1)):
                for col in range(max(0, min_c), min(g.width, max_c + 1)):
                    x, y = g.grid_to_world(col, row)
                    if math.hypot(x - obs.x, y - obs.y) <= block_r:
                        rows[row][col] = 1

        # 3. Active pedestrians and customers
        for p in (self.pedestrians(self.clock.now() / 1e6) if reserve_agents else ()):
            ped_dist = p.radius + math.hypot(.2, .18) + self.safety_margin
            min_c, min_r = g.world_to_grid(p.x - ped_dist, p.y - ped_dist)
            max_c, max_r = g.world_to_grid(p.x + ped_dist, p.y + ped_dist)
            for row in range(max(0, min_r), min(g.height, max_r + 1)):
                for col in range(max(0, min_c), min(g.width, max_c + 1)):
                    x, y = g.grid_to_world(col, row)
                    if math.hypot(x - p.x, y - p.y) <= ped_dist:
                        rows[row][col] = 1

        # 3b. Known patrol lines of non-yielding walkers: keep routes off the whole line.
        for (ax, ay), (bx, by), prad in (self._patrol_segments() if reserve_patrol else ()):
            line_r = prad + math.hypot(.2, .18) + self.safety_margin + .05
            n = max(1, math.ceil(math.dist((ax, ay), (bx, by)) / .2))
            for k in range(n + 1):
                px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                min_c, min_r = g.world_to_grid(px - line_r, py - line_r)
                max_c, max_r = g.world_to_grid(px + line_r, py + line_r)
                for row in range(max(0, min_r), min(g.height, max_r + 1)):
                    for col in range(max(0, min_c), min(g.width, max_c + 1)):
                        x, y = g.grid_to_world(col, row)
                        if math.hypot(x - px, y - py) <= line_r:
                            rows[row][col] = 1

        # 4. Extra blocked positions
        for ex, ey, er in extra_blocked:
            min_c, min_r = g.world_to_grid(ex - er, ey - er)
            max_c, max_r = g.world_to_grid(ex + er, ey + er)
            for row in range(max(0, min_r), min(g.height, max_r + 1)):
                for col in range(max(0, min_c), min(g.width, max_c + 1)):
                    x, y = g.grid_to_world(col, row)
                    if math.hypot(x - ex, y - ey) <= er:
                        rows[row][col] = 1

        # Ensure robot's current pose and immediate escape neighborhood are free on static map
        c_cur, r_cur = g.world_to_grid(robot.pose.x, robot.pose.y)
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                rr, cc = r_cur + dr, c_cur + dc
                if 0 <= rr < g.height and 0 <= cc < g.width:
                    if g.is_free(cc, rr):
                        rows[rr][cc] = 0

        return OccupancyGrid(g.origin_x, g.origin_y, g.resolution, g.width, g.height,
                             tuple(tuple(row) for row in rows))

    def _set_route(self, robot: RobotRuntime, destination, purpose, extra_blocked=()):
        origin=(robot.pose.x,robot.pose.y)
        routing_grid = self._traffic_grid(robot, extra_blocked=extra_blocked, reserve_patrol=purpose != 'RETREAT')
        seed=self.seed+robot.robot_id+self.clock.tick
        try:
            route=ALNSRouteOptimizer(routing_grid,seed=seed).optimize(origin,destination,iterations=50)
        except ValueError:
            if purpose == 'RETREAT':
                raise
            # No lane is free *right now* (another robot / person is in the way). Plan
            # without transient agents, retaining persistent patrol reservations.
            # DWA and centralized reservations still arbitrate access.
            routing_grid = self._traffic_grid(robot, reserve_agents=False)
            try:
                route=ALNSRouteOptimizer(routing_grid,seed=seed).optimize(origin,destination,iterations=50)
            except ValueError:
                routing_grid = self.grid
                try:
                    route=ALNSRouteOptimizer(routing_grid,seed=seed).optimize(origin,destination,iterations=50)
                except ValueError:
                    robot.destination = robot.mission_destination = destination
                    self._block_navigation(robot, 'Destination unreachable on static map')
                    return
        # Agent positions are planning reservations, not permanent local walls.
        # DWA uses live agents; retain persistent geometry and patrol keep-outs.
        control_grid = (self._traffic_grid(robot, reserve_agents=False, reserve_patrol=purpose != 'RETREAT')
                        if routing_grid is not self.grid else self.grid)
        planner=RouteDWA(control_grid,DWAConfig(safety_margin=self.safety_margin, allow_reverse=purpose == 'RETREAT'))
        planner.set_path(route.path)
        if purpose not in ('RETREAT', 'RECOVERY', 'RESUME'):
            robot.recovery_budget = 0
            robot.mission_best_distance = math.dist(origin, destination)
            robot.mission_progress_tick = self.clock.tick
        robot.navigation_state = 'READY'
        robot.planner=planner
        robot.destination=destination
        if purpose != 'RETREAT':
            if robot.mission_destination != destination:
                # Alternative service waypoints change the distance baseline,
                # but selecting one is not physical mission progress.
                robot.mission_best_distance = math.dist(origin, destination)
            robot.mission_destination=destination
        robot.stalls=0
        robot.blocked_reason=''
        robot.tracker.reset()
        self._reset_progress(robot)
        self.routes.append({'robot_id':robot.robot_id,'purpose':purpose,
                            'start':origin,'goal':destination,
                            'initial_cost':route.initial_cost,
                            'optimized_cost':route.optimized_cost,
                            'path':route.path,'iterations':route.iterations,
                            'parked_robot_reservations': routing_grid is not self.grid})
        self._event('ROUTE',robot_id=robot.robot_id,purpose=purpose,
                    path_points=len(route.path),initial_cost=round(route.initial_cost,3),
                    optimized_cost=round(route.optimized_cost,3))

    def _launch(self, r):
        order = self.orders.get(r.order_id)
        r.phase = 'TO_PICKUP'
        self.orders.advance(order.order_id, OrderStatus.TO_PICKUP)
        routing_grid = self._traffic_grid(r)
        pickup = self._best_unblocked_waypoint(self.pickups[str(order.station_id)],
                                              (r.pose.x, r.pose.y), routing_grid)
        if pickup is None:
            pickup = self._best_waypoint(self.pickups[str(order.station_id)],
                                         (r.pose.x, r.pose.y))
        self._set_route(r, pickup, 'PICKUP')
        self._event('TRAFFIC_ADMITTED', robot_id=r.robot_id, order_id=r.order_id)

    def _admit_waiting(self):
        waiting = [r for r in self.robots.values() if r.phase == 'WAITING_FOR_AISLE']
        if not waiting:
            return
        active = [r for r in self.robots.values() if r.phase in
                  ('TO_PICKUP', 'TO_CUSTOMER', 'LOADING', 'UNLOADING', 'RETURNING')]
        if self.traffic_mode == "exclusive":
            if active:
                return
            limit = 1
        elif self.traffic_mode == "standard":
            if active:
                return
            limit = 2
        else:  # "coordinated"
            limit = len(self.robots)

        for r in sorted(waiting, key=lambda x: x.robot_id)[:limit]:
            if self.traffic_mode == "coordinated" and active:
                if any(math.dist((r.pose.x, r.pose.y), (o.pose.x, o.pose.y)) < 0.65 for o in active):
                    continue
            self._launch(r)

    def _dispatch(self):
        if self.traffic_mode != "coordinated":
            if any(r.phase == 'WAITING_FOR_AISLE' for r in self.robots.values()):
                return
        for assignment in self.scheduler.dispatch():
            rid = int(assignment.robot_id)
            r = self.robots[rid]
            order = self.orders.get(assignment.order_id)
            if any(other.order_id == int(order.order_id) for other in self.robots.values() if other.robot_id != rid):
                raise RuntimeError('Duplicate assignment of one order to multiple robots')
            r.order_id = int(order.order_id)
            self._event('ASSIGNED', robot_id=rid, order_id=r.order_id)
            active = sum(o.phase in ('TO_PICKUP', 'TO_CUSTOMER', 'LOADING',
                                     'UNLOADING', 'RETURNING') for o in self.robots.values())
            limit = 1 if self.traffic_mode == "exclusive" else (2 if self.traffic_mode == "standard" else len(self.robots))
            if active >= limit:
                r.phase = 'WAITING_FOR_AISLE'
                self._event('TRAFFIC_WAIT', robot_id=rid, order_id=r.order_id)
            else:
                self._launch(r)

    ACTIVE_PHASES = ('TO_PICKUP', 'TO_CUSTOMER', 'RETURNING')
    STUCK_TICKS = 80          # no measurable progress for 4 s => stuck
    CYCLE_TICKS = 24          # robots in a wait cycle must have stalled this long
    CORRIDOR_PATIENCE = 500   # ticks a robot waits at a lane entry before the claim is waived

    def _reset_progress(self, r: RobotRuntime):
        r.best_remaining = self._remaining(r)
        r.best_heading = math.pi
        r.progress_tick = self.clock.tick

    @staticmethod
    def _remaining(r: RobotRuntime) -> float:
        p = r.planner
        if p is None or not p.path:
            return 0.
        idx = min(p.index, len(p.path) - 1)
        total = math.dist((r.pose.x, r.pose.y), p.path[idx])
        for a, b in zip(p.path[idx:], p.path[idx + 1:]):
            total += math.dist(a, b)
        return total

    def _track_progress(self, r: RobotRuntime):
        """Progress = shorter remaining route or a better heading toward the target."""
        rem = self._remaining(r)
        if rem < r.best_remaining - .04:
            r.best_remaining = rem
            r.best_heading = math.pi
            r.progress_tick = self.clock.tick
            r.recovery_attempts = 0
            if rem < r.esc_mark - .6:      # clearly moving again: forget past escalation
                r.escalation = 0
            return
        tgt = r.planner.last_target if r.planner else None
        if tgt:
            err = abs(math.atan2(tgt[1] - r.pose.y, tgt[0] - r.pose.x) - r.pose.theta)
            err = min(err % (2 * math.pi), 2 * math.pi - err % (2 * math.pi))
            if err < r.best_heading - .25:
                r.best_heading = err
                r.progress_tick = self.clock.tick

    def _priority(self, r: RobotRuntime):
        return DeadlockManager.robot_priority(r)

    def _people(self):
        """(x, y, radius) of customers, pedestrians and loose obstacles."""
        t = self.clock.now() / 1e6
        out = [(p.x, p.y, p.radius) for p in self.pedestrians(t)]
        out += [(o.x, o.y, o.radius) for o in self.dynamic_obstacles.values()]
        return out

    def _identify_blocker(self, r: RobotRuntime):
        """('robot', other) | ('person'|'obstacle'|'static', None) for a stalled robot."""
        oid = self._veto.get(r.robot_id)
        if oid is not None:
            return 'robot', self.robots[oid]
        near = [(math.dist((r.pose.x, r.pose.y), (o.pose.x, o.pose.y)), o)
                for o in self.robots.values() if o is not r]
        near = [(d, o) for d, o in near if d < self.robot_envelope + .45]
        if near:
            return 'robot', min(near, key=lambda x: x[0])[1]
        reach = math.hypot(.2, .18) + self.safety_margin + .6
        t = self.clock.now() / 1e6
        if any(math.hypot(r.pose.x - p.x, r.pose.y - p.y) < p.radius + reach for p in self.pedestrians(t)):
            return 'person', None
        if any(math.hypot(r.pose.x - o.x, r.pose.y - o.y) < o.radius + reach
               for o in self.dynamic_obstacles.values()):
            return 'obstacle', None
        return 'static', None

    def _replan(self, robot: RobotRuntime, reason: str) -> bool:
        """Re-run A*/ALNS to the mission goal around current traffic; keeps the order intact."""
        robot.blocked_reason = reason
        destination = robot.mission_destination or robot.destination
        if destination is None:
            return False
        self._event('RECOVERY_REQUESTED', robot_id=robot.robot_id, order_id=robot.order_id,
                    attempt=robot.escalation, reason=reason)
        try:
            routing_grid = self._traffic_grid(robot)
            c_dest, r_dest = routing_grid.world_to_grid(*destination)
            if robot.order_id:
                order = self.orders.get(robot.order_id)
                path_to_dest = find_path(routing_grid, (robot.pose.x, robot.pose.y), destination)
                if (not path_to_dest) or (not routing_grid.is_free(c_dest, r_dest)):
                    alt = None
                    if robot.phase == 'TO_CUSTOMER':
                        alt = self._best_unblocked_waypoint(self.drops.get(str(order.table_id), []),
                                                            (robot.pose.x, robot.pose.y), routing_grid)
                    elif robot.phase == 'TO_PICKUP':
                        alt = self._best_unblocked_waypoint(self.pickups.get(str(order.station_id), []),
                                                            (robot.pose.x, robot.pose.y), routing_grid)
                    if alt is not None:
                        destination = alt
            self._set_route(robot, destination, 'RECOVERY')
            if robot.navigation_state == 'BLOCKED':
                return False
            self.dynamic_replans += 1
            self._event('DYNAMIC_REPLAN', robot_id=robot.robot_id,
                        order_id=robot.order_id, destination=destination)
            return True
        except (ValueError, RuntimeError) as exc:
            robot.blocked_reason = f'{reason}; replan failed: {exc}'
            return False

    def _attempt_stationary_recovery(self, robot: RobotRuntime, reason: str) -> bool:
        """Bounded (2 attempts) replan, only from a stationary state; kept for the live API."""
        robot.blocked_reason = reason
        if robot.destination is None or robot.recovery_attempts >= 2:
            return False
        if abs(robot.linear) > 1e-4 or abs(robot.angular) > 1e-4:
            return False
        robot.recovery_attempts += 1
        return self._replan(robot, reason)

    def _avoid_discs(self, r: RobotRuntime):
        """Keep-outs for a refuge: other robots, their upcoming lanes, nearby people."""
        lane = 2 * math.hypot(.2, .18) + .15
        avoid = []
        for o in self.robots.values():
            if o is r:
                continue
            avoid.append((o.pose.x, o.pose.y, self.robot_envelope + .15))
            if o.phase in self.ACTIVE_PHASES and o.planner and o.planner.path and not o.retreating:
                idx = o.planner.index
                travelled, prev = 0., (o.pose.x, o.pose.y)
                for pt in o.planner.path[idx:]:
                    travelled += math.dist(prev, pt)
                    prev = pt
                    if travelled > 4.5:
                        break
                    avoid.append((pt[0], pt[1], lane))
        for x, y, rad in self._people():
            avoid.append((x, y, rad + math.hypot(.2, .18) + self.safety_margin + .5))
        for seg in self._patrol_segments():
            (ax, ay), (bx, by), rad = seg
            n = max(1, math.ceil(math.dist((ax, ay), (bx, by)) / .25))
            for k in range(n + 1):
                avoid.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n,
                              rad + math.hypot(.2, .18) + .2))
        for p in self.pedestrians(self.clock.now() / 1e6):
            speed = math.hypot(p.vx, p.vy)
            if speed > .05:   # a walker that cannot yield: keep off its whole patrol line
                ux, uy = p.vx / speed, p.vy / speed
                for k in range(-7, 8):
                    avoid.append((p.x + ux * .25 * k, p.y + uy * .25 * k,
                                  p.radius + math.hypot(.2, .18) + .25))
        crowd = self.crowd if self.crowd is not None else self.pedestrians
        for cust in getattr(crowd, 'customers', {}).values():
            if cust.phase in ('TO_CASH', 'TO_TABLE'):
                for pt in cust.route[cust.route_index:cust.route_index + 12]:
                    avoid.append((pt[0], pt[1], cust.radius + math.hypot(.2, .18) + self.safety_margin + .2))
        return avoid

    def _begin_retreat(self, r: RobotRuntime, reason: str) -> bool:
        """Drive to a nearby passing bay so the blocking traffic can go through."""
        if r.retreating or r.phase not in self.ACTIVE_PHASES:
            return False
        grid = self._traffic_grid(r, reserve_patrol=False)
        refuge = find_refuge(grid, self.lanes, (r.pose.x, r.pose.y), self._avoid_discs(r),
                             exclude=r.refuges[-3:])
        if refuge is None:
            return False
        keep = (r.mission_destination, r.destination)
        try:
            self._set_route(r, refuge, 'RETREAT')
        except ValueError:
            r.mission_destination, r.destination = keep
            return False
        r.mission_destination = keep[0] or keep[1]
        r.retreating = True
        r.refuges.append(refuge)
        r.refuges = r.refuges[-6:]
        r.blocked_reason = reason
        self.retreats += 1
        self._event('RETREAT', robot_id=r.robot_id, order_id=r.order_id, reason=reason,
                    refuge=(round(refuge[0], 2), round(refuge[1], 2)))
        return True

    def _hold_ticks(self, r: RobotRuntime) -> int:
        # Backoff grows with repeated escalation; the robot-id offset breaks symmetry
        # so two robots released together do not collide again in lock-step.
        return 50 + 25 * min(r.escalation, 6) + 9 * r.robot_id

    def _end_hold(self, r: RobotRuntime):
        r.hold_reason = ''
        if r.planner is None and r.mission_destination is not None and r.phase in self.ACTIVE_PHASES:
            try:
                self._set_route(r, r.mission_destination, 'RESUME')
                if r.navigation_state != 'BLOCKED':
                    self._event('RESUME', robot_id=r.robot_id, order_id=r.order_id)
            except (ValueError, RuntimeError) as exc:
                r.blocked_reason = f'resume failed: {exc}'
                r.hold = 40  # try again shortly; never fault for a temporarily blocked route

    def _navigation_signature(self):
        # Only persistent geometry changes release a sealed route. Moving people
        # are handled by predictive avoidance, not by retrying a sealed mission.
        return (id(self.grid), tuple(sorted((o.name, o.x, o.y, o.radius)
                                           for o in self.dynamic_obstacles.values())),
                tuple(sorted(self._patrol_segments())))

    def _block_navigation(self, r, reason):
        r.navigation_state = 'BLOCKED'
        r.blocked_signature = self._navigation_signature()
        r.blocked_reason = reason
        r.planner = None
        r.hold = 0
        r.retreating = False
        self._event('NAVIGATION_BLOCKED', robot_id=r.robot_id, order_id=r.order_id,
                    reason=reason, attempts=r.recovery_budget)

    def retry_blocked(self):
        """Operator-requested retry; no automatic retry loop or custody release."""
        if self.estopped:
            raise ValueError('Reset after emergency stop')
        resumed = 0
        for r in self.robots.values():
            if r.navigation_state != 'BLOCKED' or abs(r.linear) > 1e-4 or abs(r.angular) > 1e-4:
                continue
            r.navigation_state = 'READY'
            r.recovery_budget = 0
            r.mission_progress_tick = self.clock.tick
            if self._replan(r, 'Operator retry'):
                resumed += 1
        return resumed

    def _persistent_grid(self, r):
        # Deliberately omit transient robots, customers and patrol predictions.
        g = self.grid
        rows = [list(row) for row in g.cells]
        for o in self.dynamic_obstacles.values():
            radius = o.radius + math.hypot(.2, .18) + self.safety_margin + .04
            c0, r0 = g.world_to_grid(o.x-radius, o.y-radius)
            c1, r1 = g.world_to_grid(o.x+radius, o.y+radius)
            for row in range(max(0, r0), min(g.height, r1+1)):
                for col in range(max(0, c0), min(g.width, c1+1)):
                    if math.dist(g.grid_to_world(col, row), (o.x, o.y)) <= radius:
                        rows[row][col] = 1
        return OccupancyGrid(g.origin_x, g.origin_y, g.resolution, g.width, g.height,
                             tuple(tuple(row) for row in rows))

    def _handle_stuck(self, r: RobotRuntime):
        """Recovery ladder for a robot with no progress: replan, yield, back off. Never FAULT."""
        r.recovery_budget += 1
        if r.recovery_budget >= 12 and self.clock.tick - r.mission_progress_tick >= 2400:
            self._block_navigation(r, 'Recovery budget exhausted without mission progress')
            return
        if r.recovery_budget >= 4:
            destination = r.mission_destination or r.destination
            persistent = self._persistent_grid(r)
            options = [destination] if destination is not None else []
            if r.order_id and not r.retreating:
                order = self.orders.get(r.order_id)
                waypoints = (self.pickups.get(str(order.station_id), []) if r.phase == 'TO_PICKUP'
                             else self.drops.get(str(order.table_id), []) if r.phase == 'TO_CUSTOMER' else [])
                options += [(w.x, w.y) for w in waypoints]
            if not any(find_path(persistent, (r.pose.x, r.pose.y), goal) for goal in options):
                self._block_navigation(r, 'Persistent geometry seals mission destination')
                return
        r.escalation += 1
        self.stuck_events += 1
        kind, other = self._identify_blocker(r)
        r.progress_tick = self.clock.tick
        r.esc_mark = r.best_remaining
        r.wait_on = kind if other is None else f'robot:{other.robot_id}'
        self._event('STUCK', robot_id=r.robot_id, level=r.escalation, blocker=r.wait_on)
        if r.retreating:
            # The retreat itself is blocked: pick another refuge.
            r.retreating = False
            if not self._begin_retreat(r, 'retreat blocked'):
                r.planner, r.destination = None, None
                r.hold, r.hold_reason = self._hold_ticks(r), 'BACKOFF'
            return
        # Robots give way to people at once; the resume after the hold re-plans around them.
        if r.escalation == 1 and kind != 'person':
            if self._replan(r, f'stuck behind {r.wait_on}'):
                return
        yields_to_other = (other is not None and
                           (self._priority(r) < self._priority(other)
                            or other.phase not in self.ACTIVE_PHASES))
        must_yield = (kind == 'person' or yields_to_other or r.escalation >= 3
                      or (kind == 'robot' and r.escalation >= 2))
        if must_yield and self._begin_retreat(r, f'yield to {r.wait_on}'):
            return
        if r.escalation >= 2:
            self._replan(r, f'persistent block by {r.wait_on}')
        if r.escalation >= 4 and r.hold <= 0:
            r.hold, r.hold_reason = self._hold_ticks(r), 'BACKOFF'

    def _resolve_cycles(self):
        """Wait-for cycles: the lowest-priority robot (aged by past yields) retreats."""
        for cycle in self.wait_graph.cycles():
            members = [self.robots[i] for i in cycle]
            if any(self.clock.tick - m.progress_tick < self.CYCLE_TICKS or m.retreating
                   or m.hold > 0 for m in members):
                continue
            victim = min(members, key=self._priority)
            self.deadlocks_resolved += 1
            self._event('DEADLOCK_DETECTED', cycle=list(cycle), victim=victim.robot_id)
            victim.escalation += 1
            victim.progress_tick = self.clock.tick
            if not self._begin_retreat(victim, f'deadlock cycle {cycle}'):
                self._replan(victim, f'deadlock cycle {cycle}')

    def _patrol_segments(self):
        agents = getattr(self.walkers, 'agents', None)
        return [(a.start, a.end, a.radius) for a in agents.values()] if agents else []

    @staticmethod
    def _segment_distance(p, a, b) -> float:
        dx, dy = b[0] - a[0], b[1] - a[1]
        d2 = dx * dx + dy * dy
        t = 0. if d2 < 1e-12 else max(0., min(1., ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d2))
        return math.hypot(p[0] - a[0] - dx * t, p[1] - a[1] - dy * t)

    def _evade_pedestrians(self):
        """Leave the path of a walker that will not give way (it ignores robots).

        Extrapolates relative robot/pedestrian motion 4 s ahead; a robot (even one parked in a
        refuge) that would be within reach retreats to a bay off the walker's line
        instead of waiting for DWA to freeze in front of it.
        """
        if self.clock.tick % 4:
            return
        t = self.clock.now() / 1e6
        walkers = [p for p in self.pedestrians(t) if math.hypot(p.vx, p.vy) > .05]
        segments = self._patrol_segments()
        if not walkers and not segments:
            return
        for r in self.robots.values():
            if r.phase not in self.ACTIVE_PHASES or r.retreating:
                continue
            # A robot standing still within reach of a known patrol line will be walked into.
            if abs(r.linear) < .05 and any(
                    self._segment_distance((r.pose.x, r.pose.y), a, b) < rad + math.hypot(.2, .18) + .15
                    for a, b, rad in segments):
                saved = (r.hold, r.hold_reason)
                r.hold, r.hold_reason = 0, ''
                if self._begin_retreat(r, 'leave pedestrian patrol line'):
                    continue
                r.hold, r.hold_reason = saved
            for p in walkers:
                reach = p.radius + math.hypot(.2, .18) + .12
                if any(math.hypot(r.pose.x + r.linear * math.cos(r.pose.theta) * k * .05 - (p.x + p.vx * k * .05), r.pose.y + r.linear * math.sin(r.pose.theta) * k * .05 - (p.y + p.vy * k * .05)) < reach
                       for k in range(0, 81, 4)):
                    saved = (r.hold, r.hold_reason)
                    r.hold, r.hold_reason = 0, ''
                    if self._begin_retreat(r, 'evade walking pedestrian'):
                        break
                    r.hold, r.hold_reason = saved
    def _lane_decisions(self):
        """Proactive single-lane reservation for head-on routes."""
        claims = {}
        for rid, r in self.robots.items():
            if r.phase in self.ACTIVE_PHASES and r.planner and not r.hold:
                claims[rid] = lane_claim(r.planner.path, r.planner.index,
                                         (r.pose.x, r.pose.y), self.lanes)
        for rid, mine in claims.items():
            r = self.robots[rid]
            if not mine or r.retreating or self.clock.tick < r.corridor_waived:
                continue
            for oid, theirs in claims.items():
                o = self.robots[oid]
                if oid == rid or not theirs or o.retreating or self._priority(o) <= self._priority(r):
                    continue
                if not corridor_conflict(mine, theirs):
                    continue
                gap = math.dist((r.pose.x, r.pose.y), (o.pose.x, o.pose.y))
                if gap > 4.5:
                    continue
                # Am I already inside the other robot's lane? Then waiting would block it:
                # leave to a passing bay now instead of meeting nose to nose.
                in_lane = any(math.hypot(r.pose.x - b.x, r.pose.y - b.y) < 2 * math.hypot(.2, .18) + .15
                              for b in theirs)
                if in_lane:
                    if self._begin_retreat(r, f'lane yield to robot:{oid}'):
                        self.corridor_waits += 1
                        break
                    continue
                r.hold, r.hold_reason = 20, 'CORRIDOR'
                r.corridor_waited += 20
                r.wait_on = f'robot:{oid}'
                self.corridor_waits += 1
                if r.corridor_waited > self.CORRIDOR_PATIENCE:
                    r.corridor_waived = self.clock.tick + 400
                    r.corridor_waited = 0
                break

    def _arrived(self,r):
        r.linear=r.angular=0.
        if r.retreating:
            # Reached the passing bay: wait (backoff) for the traffic to clear, then resume.
            r.retreating=False
            r.planner=None
            r.destination=None
            r.yields+=1
            r.hold,r.hold_reason=self._hold_ticks(r),'REFUGE'
            self._event('REFUGE_REACHED',robot_id=r.robot_id,order_id=r.order_id,hold=r.hold)
            return
        r.planner=None
        r.destination=None
        r.stalls=0
        r.recovery_attempts=0
        if r.phase=='TO_PICKUP':
            self.orders.advance(r.order_id,OrderStatus.LOADING)
            r.phase='LOADING';r.dwell=8
            self._event('LOADING',robot_id=r.robot_id,order_id=r.order_id)
        elif r.phase=='TO_CUSTOMER':
            if not self._custody_valid(r):
                self._fail(r,'Order/customer custody mismatch at drop-off')
                return
            self.orders.advance(r.order_id,OrderStatus.UNLOADING)
            r.phase='UNLOADING';r.dwell=8
            self._event('UNLOADING',robot_id=r.robot_id,order_id=r.order_id)
        elif r.phase=='RETURNING':
            if math.dist((r.pose.x,r.pose.y),(r.base.x,r.base.y)) > .12:
                self._fail(r,'Return arrival tolerance violated')
                return
            if not self.checker.is_pose_free(r.pose.x,r.pose.y,r.pose.theta):
                self._fail(r,'Return footprint not clear')
                return
            self.scheduler.complete_return(r.robot_id)
            self._event('RETURNED_TO_BASE',robot_id=r.robot_id,order_id=r.order_id)
            r.phase='IDLE';r.order_id=None

    def _custody_valid(self, r):
        if r.order_id is None:
            return False
        order = self.orders.get(r.order_id)
        if int(order.robot_id or 0) != r.robot_id:
            return False
        customer = self.order_customers.get(r.order_id)
        return customer is None or self.customer_orders.get(customer) == r.order_id

    def _fail(self,r,reason):
        if r.phase == 'FAULT':
            return
        self.failures.append(f'Robot {r.robot_id}: {reason}')
        self._event('FAULT',robot_id=r.robot_id,reason=reason)
        r.phase='FAULT';r.linear=r.angular=0.
        r.blocked_reason=reason
        self.fleet.mark_faulted(r.robot_id)

    @staticmethod
    def _swept_clearance(a0: Pose,a1: Pose,b0: Pose,b1: Pose, min_radius: float) -> float:
        rx,ry=a0.x-b0.x,a0.y-b0.y
        vx,vy=(a1.x-a0.x)-(b1.x-b0.x),(a1.y-a0.y)-(b1.y-b0.y)
        denom=vx*vx+vy*vy
        t=max(0.,min(1.,-(rx*vx+ry*vy)/denom)) if denom>1e-12 else 0.
        return math.hypot(rx+vx*t,ry+vy*t)-min_radius

    def _stationary_proposal(self, r: RobotRuntime, dt: float):
        """Hold position; brake first if still rolling (never an instantaneous stop)."""
        if abs(r.linear) > 1e-4 or abs(r.angular) > 1e-4:
            lin = math.copysign(max(0., abs(r.linear) - .8 * dt), r.linear)
            return (advance(r.pose, lin, 0., dt), lin, 0., True)
        return (r.pose, 0., 0., True)

    def _note_wait(self, r: RobotRuntime):
        """Add a wait-for edge to the nearest robot crowding a motionless robot."""
        best = None
        for o in self.robots.values():
            if o is r:
                continue
            d = math.dist((r.pose.x, r.pose.y), (o.pose.x, o.pose.y))
            if d < self.robot_envelope + .45 and (best is None or d < best[0]):
                best = (d, o.robot_id)
        if best:
            self.wait_graph.add(r.robot_id, best[1])

    def step(self):
        if self.paused or self.estopped:return
        self._admit_waiting()
        self._dispatch()
        dt=.05;t=self.clock.now()/1e6
        pedestrian_now=tuple(self.pedestrians(t))
        previous_poses = {rid: r.pose for rid, r in self.robots.items()}
        all_dyn_list = list(self.dynamic_obstacles.values())
        boxes = [(o.x-o.radius,o.y-o.radius,o.x+o.radius,o.y+o.radius) for o in pedestrian_now]
        boxes += [(o.x-o.radius,o.y-o.radius,o.x+o.radius,o.y+o.radius) for o in all_dyn_list]
        dyn_moving = [MovingObstacle(o.x, o.y, o.radius) for o in all_dyn_list]
        proposals={}
        self.wait_graph.clear()
        self._lane_decisions()
        self._evade_pedestrians()
        for rid,r in self.robots.items():
            if r.mission_destination is not None and not r.retreating:
                distance = math.dist((r.pose.x, r.pose.y), r.mission_destination)
                if distance < r.mission_best_distance - .25:
                    r.mission_best_distance = distance
                    r.mission_progress_tick = self.clock.tick
                    r.recovery_budget = 0
            if r.phase in ('IDLE','FAULT','LOADING','UNLOADING','WAITING_FOR_AISLE'):
                proposals[rid]=(r.pose,0.,0.,False)
                continue
            if r.navigation_state == 'BLOCKED':
                proposals[rid] = self._stationary_proposal(r, dt)
                if r.blocked_signature != self._navigation_signature():
                    r.navigation_state = 'READY'
                    r.recovery_budget = 0
                    self._replan(r, 'Persistent geometry changed')
                continue
            if r.hold > 0 or r.planner is None:
                # Yielding (refuge / lane entry / backoff): stay put, then resume the mission.
                proposals[rid]=self._stationary_proposal(r,dt)
                r.progress_tick=self.clock.tick
                if r.hold > 0:
                    r.hold-=1
                if r.hold == 0:
                    self._end_hold(r)
                continue
            others=[MovingObstacle(o.pose.x,o.pose.y,math.hypot(.2,.18)+.04,
                                   o.linear*math.cos(o.pose.theta),o.linear*math.sin(o.pose.theta))
                    for oid,o in self.robots.items() if oid!=rid]
            ranges=lidar_scan(r.pose,boxes,rays=72,max_range=4.,step=.04)
            perceived=r.tracker.observe(r.ekf.pose,ranges,t)
            if perceived:r.detected_ticks+=1
            untracked_dyn = [d for d in dyn_moving if not any(math.hypot(d.x - p.x, d.y - p.y) < 0.5 for p in perceived)]
            command=r.planner.plan(r.ekf.pose,r.linear,r.angular,tuple(pedestrian_now)+tuple(perceived)+tuple(others)+tuple(untracked_dyn))
            if command.reached:
                self._arrived(r)
                proposals[rid]=(r.pose,0.,0.,False)
                continue
            if not command.safe:
                # Blocked: brake smoothly (only fully stop from a stationary state).
                r.blocked_reason = command.reason
                r.stalls += 1
                proposals[rid] = self._stationary_proposal(r, dt)
                if abs(r.linear) < 1e-4:
                    self._note_wait(r)
            else:
                nxt=advance(r.pose,command.linear,command.angular,dt)
                if (not self.grid.is_free(*self.grid.world_to_grid(nxt.x,nxt.y))
                        or not self.checker.is_motion_free((r.pose.x,r.pose.y,r.pose.theta),
                                                           (nxt.x,nxt.y,nxt.theta))):
                    self._fail(r,'Exact footprint / static map collision guard')
                    proposals[rid]=(r.pose,0.,0.,False)
                    continue
                if abs(command.linear) < .02 and abs(r.linear) < .02 and abs(command.angular) < .05:
                    r.stalls += 1
                    self._note_wait(r)
                elif abs(command.linear) > .02 or abs(command.angular) > .02:
                    r.stalls = 0
                proposals[rid]=(nxt,command.linear,command.angular,False)
            self._track_progress(r)
            if self.clock.tick - r.progress_tick >= self.STUCK_TICKS:
                self._handle_stuck(r)
                proposals[rid] = self._stationary_proposal(r, dt)

        # Centralized reservation with priority ranking
        robot_order = sorted(self.robots.keys(), key=lambda i: DeadlockManager.robot_priority(self.robots[i]), reverse=True)
        accepted={}
        blockers: dict[int,int] = {}
        physical = 2 * math.hypot(.2, .18)
        for rid in robot_order:
            r = self.robots[rid]
            nxt,v,w,waiting=proposals[rid]
            if r.phase in ('IDLE', 'FAULT'):
                accepted[rid] = (r.pose, 0., 0.)
                continue
            conflict=False
            for oid in robot_order:
                if oid==rid:continue
                other = self.robots[oid]
                other_nxt=accepted[oid][0] if oid in accepted else other.pose
                if self.deadlock_manager.robot_pair_conflict(r.pose,nxt,other.pose,other_nxt,
                                                             self.robot_envelope,physical):
                    conflict=True
                    blockers[rid]=oid
                    self.wait_graph.add(rid,oid)
                    break
            if conflict:
                r.stalls += 1
                self.yield_events += 1
                if abs(r.linear)>0.0001 or abs(r.angular)>0.0001:
                    v = math.copysign(max(0., abs(r.linear) - 0.8 * dt), r.linear)
                    nxt = advance(r.pose, v, 0., dt)
                    w = 0.
                else:
                    nxt,v,w=r.pose,0.,0.
            accepted[rid]=(nxt,v,w)
        self._veto = blockers
        self._resolve_cycles()
        for rid,r in self.robots.items():
            nxt,v,w=accepted[rid]
            if r.phase=='FAULT':continue
            # Validate pedestrian clearance independently of LiDAR estimates.
            pedestrian_next = tuple(self.pedestrians(t+dt))
            for index, p in enumerate(pedestrian_next):
                old = pedestrian_now[index] if index < len(pedestrian_now) else p
                gap=self._swept_clearance(r.pose,nxt,Pose(old.x,old.y),Pose(p.x,p.y),
                                          math.hypot(.2,.18)+p.radius)
                self.min_pedestrian_clearance=min(self.min_pedestrian_clearance,gap)
                if gap<=0:
                    self._fail(r,'Pedestrian collision guard')
                    break
            if r.phase=='FAULT':continue
            for obs in self.dynamic_obstacles.values():
                obs_gap = math.hypot(nxt.x - obs.x, nxt.y - obs.y) - math.hypot(.2, .18) - obs.radius
                if obs_gap <= 0:
                    self._fail(r, f'Dynamic obstacle collision guard: {obs.name}')
                    break
            if r.phase=='FAULT':continue
            r.pose=nxt;r.linear=v;r.angular=w
            if r.phase not in ('IDLE','LOADING','UNLOADING','WAITING_FOR_AISLE'):
                r.nav_ticks+=1
                od=r.sensor.odometry(v,w)
                r.ekf.predict(od.linear,od.angular,dt)
                if r.nav_ticks%10==0:
                    lm=r.sensor.landmark(r.pose)
                    r.ekf.correct_xy(lm.x,lm.y)
            if r.phase in ('LOADING','UNLOADING'):
                r.dwell-=1
                if r.dwell<=0:
                    if r.phase=='LOADING':
                        self.orders.advance(r.order_id,OrderStatus.TO_CUSTOMER)
                        order=self.orders.get(r.order_id)
                        routing_grid = self._traffic_grid(r)
                        dest = self._best_unblocked_waypoint(self.drops[str(order.table_id)],
                                                             (r.pose.x, r.pose.y), routing_grid)
                        if dest is None:
                            dest = self._best_waypoint(self.drops[str(order.table_id)], (r.pose.x, r.pose.y))
                        r.phase='TO_CUSTOMER';self._set_route(r,dest,'DELIVERY')
                    else:
                        if not self._custody_valid(r):
                            self._fail(r,'Order/customer custody mismatch during delivery confirmation')
                            continue
                        self.orders.advance(r.order_id,OrderStatus.DELIVERED)
                        if not self.scheduler.is_return_pending(r.robot_id):
                            self._fail(r,'Scheduler released before return')
                            continue
                        r.phase='RETURNING'
                        self._event('DELIVERED_RETURN_PENDING',robot_id=rid,order_id=r.order_id)
                        self._set_route(r,(r.base.x,r.base.y),'RETURN')
            r.trace.append({'tick':self.clock.tick,'x':round(r.pose.x,5),'y':round(r.pose.y,5),
                            'theta':round(r.pose.theta,5),'phase':r.phase,
                            'order_id':r.order_id,'v':round(r.linear,5),'w':round(r.angular,5)})
        for i,a in self.robots.items():
            for j,b in self.robots.items():
                if j<=i:continue
                physical_distance = 2 * math.hypot(.2, .18)
                gap = self._swept_clearance(previous_poses[i], a.pose, previous_poses[j], b.pose,
                                            physical_distance)
                self.min_robot_clearance = min(self.min_robot_clearance, gap)
                if gap <= 0:
                    self.failures.append(f'Robots {i}/{j} overlap')
        self.clock.advance()
        if self.clock.tick%10==0:self.history.append(self.snapshot(compact=True))

    def run(self,max_ticks=8000):
        self.paused=False
        for _ in range(max_ticks):
            self.step()
            if self.failures:break
            if self.orders.all_orders() and all(o.status in (OrderStatus.DELIVERED,OrderStatus.FAILED)
                                                for o in self.orders.all_orders()) and all(
                   r.phase in ('IDLE','FAULT') for r in self.robots.values()):
                break
        return self.report()

    def snapshot(self,compact=False):
        result={'tick':self.clock.tick,'time_s':round(self.clock.now()/1e6,2),
                'paused':self.paused,'estopped':self.estopped,'speed':self.speed,
                'robots':[{'id':r.robot_id,'x':round(r.pose.x,3),'y':round(r.pose.y,3),
                           'theta':round(r.pose.theta,3),'phase':r.phase,
                           'order_id':r.order_id,'base':[r.base.x,r.base.y],
                           'fleet_available':self.fleet.is_available(r.robot_id),
                           'blocked_reason':r.blocked_reason,
                           'navigation_state':r.navigation_state,
                           'recovery_budget':r.recovery_budget,
                           'recovery_attempts':r.recovery_attempts,
                           'wait_on':r.wait_on,'hold':r.hold_reason,
                           'retreating':r.retreating,'yields':r.yields}
                          for r in self.robots.values()],
                'orders':[{'id':int(o.order_id),'table':str(o.table_id),'station':str(o.station_id),
                           'status':o.status.value,'robot_id':int(o.robot_id) if o.robot_id else None,
                           'ready_seq':o.ready_seq,
                           'customer_id':self.order_customers.get(int(o.order_id))}
                          for o in self.orders.all_orders()],
                'queue':[int(e) for e in self.scheduler.queue.order_ids()],
                'traffic_mode':self.traffic_mode,
                'safety_margin':self.safety_margin,
                'dynamic_obstacles':[{'name':o.name,'x':round(o.x,3),'y':round(o.y,3),'radius':o.radius}
                                     for o in self.dynamic_obstacles.values()],
                'dynamic_replans':self.dynamic_replans,
                'pedestrian_yield_events':self.deadlock_manager.customer_yields,
                'pedestrian_reroutes':self.deadlock_manager.customer_reroutes,
                'order_customer_links':len(self.order_customers),
                'deadlock_stats':{'resolved':self.deadlocks_resolved,'retreats':self.retreats,
                                  'corridor_waits':self.corridor_waits,'stuck_events':self.stuck_events},
                'failures':list(self.failures)}
        if not compact:
            result['recent_events']=self.events[-20:]
            result['yield_events']=self.yield_events
        return result

    def report(self):
        orders=self.orders.all_orders()
        all_delivered=bool(orders) and all(o.status is OrderStatus.DELIVERED for o in orders)
        returned=all(r.phase=='IDLE' and self.fleet.is_available(r.robot_id)
                     for r in self.robots.values())
        pending: set[int] = set()
        return_ok = True
        for event in self.events:
            rid = event.get('robot_id')
            if event['event'] == 'DELIVERED_RETURN_PENDING':
                if rid in pending:
                    return_ok = False
                pending.add(rid)
            elif event['event'] == 'RETURNED_TO_BASE':
                if rid not in pending:
                    return_ok = False
                pending.discard(rid)
            elif event['event'] == 'ASSIGNED' and rid in pending:
                return_ok = False
        if pending:
            return_ok = False
        return {'success':all_delivered and returned and not self.failures and return_ok,
                'ticks':self.clock.tick,'time_s':self.clock.now()/1e6,
                'entrances':[e.to_dict() for e in self.env.entrances],
                'entrance_grid_center_free':{
                    e.name:self.grid.is_free(*self.grid.world_to_grid(e.center_x,e.center_y))
                    for e in self.env.entrances},
                'orders':[{'id':int(o.order_id),'status':o.status.value,
                           'robot_id':int(o.robot_id) if o.robot_id else None,
                           'ready_seq':o.ready_seq} for o in orders],
                'robots':[{'id':r.robot_id,'phase':r.phase,'base':[r.base.x,r.base.y],
                           'final_pose':[r.pose.x,r.pose.y],
                           'available':self.fleet.is_available(r.robot_id),
                           'navigation_state':r.navigation_state,
                           'blocked_reason':r.blocked_reason,
                           'lidar_detection_ticks':r.detected_ticks} for r in self.robots.values()],
                'return_before_reassignment':return_ok,
                'alns_routes':len(self.routes),
                'alns_total_initial_cost':round(sum(r['initial_cost'] for r in self.routes),3),
                'alns_total_optimized_cost':round(sum(r['optimized_cost'] for r in self.routes),3),
                'yield_events':self.yield_events,
                'min_robot_clearance':None if not math.isfinite(self.min_robot_clearance) else self.min_robot_clearance,
                'min_pedestrian_clearance':None if not math.isfinite(self.min_pedestrian_clearance) else self.min_pedestrian_clearance,
                'failures':self.failures,'events':self.events}
