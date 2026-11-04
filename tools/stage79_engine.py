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

AgentProvider = Callable[[float], Sequence[MovingObstacle]]


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
            raise ValueError('No reachable waypoint')
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

    def _traffic_grid(self, robot: RobotRuntime, extra_blocked: Sequence[tuple[float, float, float]] = ()) -> OccupancyGrid:
        """Temporarily reserve occupied base slots, other stationary robots, and dynamic obstacles."""
        g = self.grid
        rows = [list(row) for row in g.cells]

        # 1. Other robots: reserve parked, waiting, or other robots in transit
        other_robots = [other for other in self.robots.values() if other.robot_id != robot.robot_id]
        block_dist = math.hypot(.2, .18) + self.safety_margin + 0.05
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
        ped_dist = 0.14 + math.hypot(.2, .18) + self.safety_margin
        for p in self.pedestrians(self.clock.now() / 1e6):
            min_c, min_r = g.world_to_grid(p.x - ped_dist, p.y - ped_dist)
            max_c, max_r = g.world_to_grid(p.x + ped_dist, p.y + ped_dist)
            for row in range(max(0, min_r), min(g.height, max_r + 1)):
                for col in range(max(0, min_c), min(g.width, max_c + 1)):
                    x, y = g.grid_to_world(col, row)
                    if math.hypot(x - p.x, y - p.y) <= ped_dist:
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
        routing_grid = self._traffic_grid(robot, extra_blocked=extra_blocked)
        route=ALNSRouteOptimizer(routing_grid,seed=self.seed+robot.robot_id+self.clock.tick).optimize(origin,destination,iterations=50)
        planner=RouteDWA(routing_grid,DWAConfig(safety_margin=self.safety_margin))
        planner.set_path(route.path)
        robot.planner=planner
        robot.destination=destination
        robot.stalls=0
        robot.blocked_reason=''
        robot.tracker.reset()
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

    def _attempt_stationary_recovery(self, robot: RobotRuntime, reason: str) -> bool:
        """Bounded route replan around dynamic obstacles; keep assigned order intact."""
        robot.blocked_reason=reason
        if robot.destination is None or robot.recovery_attempts >= 2:
            return False
        if abs(robot.linear)>1e-4 or abs(robot.angular)>1e-4:
            return False
        robot.recovery_attempts += 1
        self._event('RECOVERY_REQUESTED',robot_id=robot.robot_id,
                    order_id=robot.order_id,attempt=robot.recovery_attempts,
                    reason=reason)
        try:
            destination=robot.destination
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
            self._set_route(robot,destination,'RECOVERY')
            self.dynamic_replans += 1
            self._event('DYNAMIC_REPLAN', robot_id=robot.robot_id,
                        order_id=robot.order_id, destination=destination)
            return True
        except (ValueError,RuntimeError) as exc:
            robot.blocked_reason=f'{reason}; replan failed: {exc}'
            return False

    def _arrived(self,r):
        r.linear=r.angular=0.
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

    def step(self):
        if self.paused or self.estopped:return
        self._admit_waiting()
        self._dispatch()
        dt=.05;t=self.clock.now()/1e6
        pedestrian_now=self.pedestrians(t)
        all_dyn_list = list(self.dynamic_obstacles.values())
        boxes = [(o.x-o.radius,o.y-o.radius,o.x+o.radius,o.y+o.radius) for o in pedestrian_now]
        boxes += [(o.x-o.radius,o.y-o.radius,o.x+o.radius,o.y+o.radius) for o in all_dyn_list]
        dyn_moving = [MovingObstacle(o.x, o.y, o.radius) for o in all_dyn_list]
        proposals={}
        for rid,r in self.robots.items():
            if r.phase in ('IDLE','FAULT','LOADING','UNLOADING','WAITING_FOR_AISLE'):
                proposals[rid]=(r.pose,0.,0.,False)
                continue
            assert r.planner is not None
            others=[MovingObstacle(o.pose.x,o.pose.y,math.hypot(.2,.18)+.04,
                                   o.linear*math.cos(o.pose.theta),o.linear*math.sin(o.pose.theta))
                    for oid,o in self.robots.items() if oid!=rid]
            ranges=lidar_scan(r.pose,boxes,rays=72,max_range=4.,step=.04)
            perceived=r.tracker.observe(r.ekf.pose,ranges,t)
            if perceived:r.detected_ticks+=1
            untracked_dyn = [d for d in dyn_moving if not any(math.hypot(d.x - p.x, d.y - p.y) < 0.5 for p in perceived)]
            command=r.planner.plan(r.ekf.pose,r.linear,r.angular,tuple(perceived)+tuple(others)+tuple(untracked_dyn))
            if not command.safe:
                # When an occupied corridor is blocked, wait only from a
                # stationary state. Never fake an instantaneous stop at speed.
                if abs(r.linear)<1e-4 and abs(r.angular)<1e-4:
                    proposals[rid]=(r.pose,0.,0.,True)
                    r.stalls+=1
                    if r.stalls>15:
                        if not self._attempt_stationary_recovery(r,'DWA: '+command.reason):
                            if r.stalls>150:
                                self._fail(r,'DWA deadlock: '+command.reason)
                    continue
                brake_linear = max(0.0, r.linear - 0.8 * dt)
                nxt = advance(r.pose, brake_linear, 0., dt)
                proposals[rid] = (nxt, brake_linear, 0., True)
                r.stalls += 1
                continue
            if command.reached:
                self._arrived(r)
                proposals[rid]=(r.pose,0.,0.,False)
                continue
            nxt=advance(r.pose,command.linear,command.angular,dt)
            if (not self.grid.is_free(*self.grid.world_to_grid(nxt.x,nxt.y))
                    or not self.checker.is_motion_free((r.pose.x,r.pose.y,r.pose.theta),
                                                       (nxt.x,nxt.y,nxt.theta))):
                self._fail(r,'Exact footprint / static map collision guard')
                proposals[rid]=(r.pose,0.,0.,False)
                continue

            # Detect if robot is brought to a stop by an obstacle
            if (abs(command.linear) < 1e-4 and abs(r.linear) < 1e-4
                    and abs(command.angular) < 1e-3 and abs(r.angular) < 1e-3
                    and not command.reached):
                dist_to_dest = math.dist((r.pose.x, r.pose.y), r.destination) if r.destination else 0.0
                if dist_to_dest > 0.15:
                    r.stalls += 1
                    if r.stalls > 15:
                        self._attempt_stationary_recovery(r, 'Obstacle in corridor: ' + command.reason)
            else:
                if r.stalls > 0 and (abs(command.linear) > 0.02 or abs(command.angular) > 0.02):
                    r.stalls = 0
                if abs(r.linear) > 0.03 or abs(r.angular) > 0.05:
                    r.recovery_attempts = 0

            proposals[rid]=(nxt,command.linear,command.angular,False)

        # Centralized reservation with priority ranking
        robot_order = sorted(self.robots.keys(), key=lambda i: DeadlockManager.robot_priority(self.robots[i]), reverse=True)
        accepted={}
        safety_distance = 2 * (math.hypot(.2, .18) + self.safety_margin)
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
                req_safety = (math.hypot(.2, .18) * 2 + self.safety_margin) if other.phase in ('IDLE', 'FAULT') else safety_distance
                gap=self._swept_clearance(r.pose,nxt,other.pose,other_nxt,req_safety)
                if gap<=0:
                    conflict=True
                    break
            if conflict:
                if r.linear>0.0001 or abs(r.angular)>0.0001:
                    brake_linear = max(0., r.linear - 0.8 * dt)
                    nxt = advance(r.pose, brake_linear, 0., dt)
                    v, w = brake_linear, 0.
                    r.stalls += 1
                    self.yield_events += 1
                else:
                    r.stalls+=1;self.yield_events+=1
                    if r.stalls>15:
                        if not self._attempt_stationary_recovery(r,'Traffic conflict in shared aisle'):
                            if r.stalls>250:
                                self._fail(r,'Traffic deadlock timeout')
                    nxt,v,w=r.pose,0.,0.
            elif not waiting and r.phase not in ('FAULT','IDLE','LOADING','UNLOADING','WAITING_FOR_AISLE'):
                if abs(v) > 0.02:
                    r.stalls=0
            accepted[rid]=(nxt,v,w)
        for rid,r in self.robots.items():
            nxt,v,w=accepted[rid]
            if r.phase=='FAULT':continue
            # Validate pedestrian clearance independently of LiDAR estimates.
            for p in self.pedestrians(t+dt):
                gap=math.hypot(nxt.x-p.x,nxt.y-p.y)-math.hypot(.2,.18)-p.radius
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
                gap = math.dist((a.pose.x, a.pose.y), (b.pose.x, b.pose.y)) - physical_distance
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
                           'recovery_attempts':r.recovery_attempts}
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
                           'lidar_detection_ticks':r.detected_ticks} for r in self.robots.values()],
                'return_before_reassignment':return_ok,
                'alns_routes':len(self.routes),
                'alns_total_initial_cost':round(sum(r['initial_cost'] for r in self.routes),3),
                'alns_total_optimized_cost':round(sum(r['optimized_cost'] for r in self.routes),3),
                'yield_events':self.yield_events,
                'min_robot_clearance':None if not math.isfinite(self.min_robot_clearance) else self.min_robot_clearance,
                'min_pedestrian_clearance':None if not math.isfinite(self.min_pedestrian_clearance) else self.min_pedestrian_clearance,
                'failures':self.failures,'events':self.events}
