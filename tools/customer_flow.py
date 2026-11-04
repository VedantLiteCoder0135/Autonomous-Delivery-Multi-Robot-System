"""Customer lifecycle for the embedded AIFR Blender simulation (no HTTP server).

Entrances, CASH and table locations come from the exported V14 geometry.
Movement happens on a grid-inflated, validated A* route. Customer bills create
existing Stage 2 orders; orders are marked READY after the customer sits.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from collections import deque
from typing import Callable

from aifr.navigation.astar import find_path
from aifr.navigation.dwa import MovingObstacle


@dataclass
class Customer:
    customer_id: int
    entrance: str
    table: str
    station: str
    x: float
    y: float
    phase: str = 'TO_CASH'  # QUEUED at a busy entrance until admission
    route: list[tuple[float, float]] = field(default_factory=list)
    route_index: int = 0
    order_id: int | None = None
    cash_wait: float = 0.0
    radius: float = .14
    seat: tuple[float, float] | None = None
    blocked_ticks: int = 0


class CustomerFlow:
    def __init__(self, engine, *, walking_speed: float = .65, payment_seconds: float = 1.5):
        self.engine = engine
        self.grid = engine.grid
        self.env = engine.env
        self.walking_speed = walking_speed
        self.payment_seconds = payment_seconds
        self.customers: dict[int, Customer] = {}
        self.next_id = 1
        self.max_per_table = 4
        self.max_customers = 72  # 18 tables x 4; queued entrants count toward capacity
        cash = next((c for c in self.env.counters if c.name.upper() == 'CASH'), None)
        if cash is None:
            raise ValueError('Recovered cafeteria scene has no CASH counter')
        self.cash = self._nearest_free((cash.rect.min_x+cash.rect.max_x)/2,
                                       cash.rect.max_y, radius=1.6)
        self.reachable = self._reachable_from_cash()
        self.entrance_points = {}
        for ent in self.env.entrances:
            self.entrance_points[ent.name] = self._near_entrance(ent.center_x, ent.center_y)

    def _reachable_from_cash(self):
        g=self.grid
        root=g.world_to_grid(*self.cash)
        seen={root}
        queue=deque([root])
        while queue:
            c,r=queue.popleft()
            for dc,dr in ((0,1),(0,-1),(1,0),(-1,0)):
                other=(c+dc,r+dr)
                if other not in seen and g.is_free(*other):
                    seen.add(other);queue.append(other)
        return seen

    def _nearby_free(self, x, y, radius):
        g = self.grid
        col, row = g.world_to_grid(x, y)
        rings = int(math.ceil(radius / g.resolution)) + 2
        possibilities = []
        for r in range(max(0,row-rings), min(g.height,row+rings+1)):
            for c in range(max(0,col-rings), min(g.width,col+rings+1)):
                if not g.is_free(c,r):
                    continue
                px,py = g.grid_to_world(c,r)
                d = math.hypot(px-x,py-y)
                if d<=radius:
                    possibilities.append((d,px,py))
        return sorted(possibilities)

    def _nearest_free(self,x,y,*,radius=1.4):
        options=self._nearby_free(x,y,radius)
        if not options:raise ValueError(f'No walkable position near ({x:.2f},{y:.2f})')
        return options[0][1:]

    def _near_entrance(self,x,y):
        # Door metadata positions lie *on the wall*. Use an inward floor cell,
        # rather than feeding these occupied coordinates directly to A*.
        for _,px,py in self._nearby_free(x,y,1.2):
            if self.grid.world_to_grid(px,py) in self.reachable:
                return (px,py)
        raise ValueError(f'No reachable interior approach to entrance at {x,y}')

    def _seat_point(self,table):
        info=next((t for t in self.env.tables if t.table_id==table),None)
        if info is None:raise ValueError('Unknown table: '+table)
        cx=(info.rect.min_x+info.rect.max_x)/2
        cy=(info.rect.min_y+info.rect.max_y)/2
        drops=self.engine.drops[table]
        # Sit near the requested table, without occupying the robot's delivery
        # waypoint. Chair geometry is static, so we represent sitting by a
        # nearby *free* pedestrian anchor, not a physical chair mesh.
        choices=[]
        for d,x,y in self._nearby_free(cx,cy,2.0):
            if min(math.hypot(x-w.x,y-w.y) for w in drops)<.83:continue
            if self.grid.world_to_grid(x,y) not in self.reachable:continue
            choices.append((d,x,y))
        reserved=[c.seat for c in self.customers.values()
                  if c.table == table and c.phase != 'LEFT' and c.seat is not None]
        # Spatially distinct pedestrian anchors, including while a customer is
        # still walking to the cashier. Do not place multiple avatars together.
        for _,x,y in choices:
            if all(math.hypot(x-sx,y-sy)>=.40 for sx,sy in reserved):
                return (x,y)
        raise ValueError(f'No fourth collision-separated seat anchor available for {table}')

    def _set_route(self,customer,destination):
        route=find_path(self.grid,(customer.x,customer.y),destination)
        if not route:raise ValueError('No pedestrian route through the cafeteria')
        customer.route=list(route)
        customer.route_index=1 if len(route)>1 else len(route)

    def _entrance_available(self, entrance):
        x,y=self.entrance_points[entrance]
        if any(math.hypot(x-c.x,y-c.y)<.48 for c in self.customers.values()
               if c.phase not in ('QUEUED','LEFT')):
            return False
        if any(math.hypot(x-r.pose.x,y-r.pose.y)<.65 for r in self.engine.robots.values()):
            return False
        # CASH is a single-server resource. Allow one customer in the cashier
        # corridor at a time, including arrivals from different entrances.
        if any(c.phase in ('TO_CASH','PAYING') for c in self.customers.values()):
            return False
        return True

    def _admit(self, c):
        c.x,c.y=self.entrance_points[c.entrance]
        self._set_route(c,self.cash)
        c.phase='TO_CASH'
        self.engine._event('CUSTOMER_ENTERED',customer_id=c.customer_id,
                           entrance=c.entrance,table=c.table)

    def add(self, *, entrance: str, table: str, station: str):
        if len(self.customers)>=self.max_customers:
            raise ValueError(f'Maximum of {self.max_customers} active customers reached')
        table=table.upper();station=station.upper()
        if table not in self.engine.drops or station not in self.engine.pickups:
            raise ValueError('Choose a valid table and food station')
        if entrance not in self.entrance_points:
            raise ValueError('Choose one of the four verified cafeteria entrances')
        occupants=sum(c.table==table and c.phase!='LEFT' for c in self.customers.values())
        if occupants>=self.max_per_table:
            raise ValueError(f'Table {table} is full (maximum {self.max_per_table} customers)')
        seat=self._seat_point(table)
        x,y=self.entrance_points[entrance]
        c=Customer(self.next_id,entrance,table,station,x,y,phase='QUEUED',seat=seat)
        self.customers[c.customer_id]=c
        self.next_id+=1
        # Preserve arrival order at each door; don't let a new entrant jump
        # in front of people queued at that same entrance.
        earlier=any(other.customer_id<c.customer_id and other.entrance==entrance
                    and other.phase=='QUEUED' for other in self.customers.values())
        if not earlier and self._entrance_available(entrance):
            self._admit(c)
        else:
            self.engine._event('CUSTOMER_QUEUED',customer_id=c.customer_id,
                               entrance=entrance,table=table)
        return c.customer_id

    def __call__(self,time_s):
        return tuple(MovingObstacle(c.x,c.y,c.radius) for c in self.customers.values()
                     if c.phase not in ('QUEUED','LEFT'))

    def _walk(self,c,dt):
        manager=self.engine.deadlock_manager
        remaining=self.walking_speed*dt
        blocked=False
        while remaining>1e-8 and c.route_index<len(c.route):
            tx,ty=c.route[c.route_index]
            length=math.hypot(tx-c.x,ty-c.y)
            if length<1e-8:
                c.route_index+=1
                continue
            travel=min(remaining,length)
            nx=c.x+(tx-c.x)*travel/length
            ny=c.y+(ty-c.y)*travel/length
            dyn_obs = tuple(getattr(self.engine, 'dynamic_obstacles', {}).values())
            if (not manager.pedestrian_step_safe((c.x,c.y),(nx,ny),
                                                 self.engine.robots,c.radius)
                or not manager.customer_step_safe((c.x,c.y),(nx,ny),
                                                   self.customers,c.customer_id,c.radius)
                or not manager.dynamic_obstacle_step_safe((c.x,c.y),(nx,ny),
                                                           dyn_obs,c.radius)):
                blocked=True
                break
            c.x,c.y=nx,ny
            remaining-=travel
            if travel>=length-1e-8:
                c.route_index+=1
        if blocked:
            c.blocked_ticks=manager.record_block(c.customer_id)
            if c.blocked_ticks % manager.policy.pedestrian_replan_after_ticks == 0:
                goal=self.cash if c.phase=='TO_CASH' else c.seat
                dyn_obs = tuple(getattr(self.engine, 'dynamic_obstacles', {}).values())
                new_path=manager.reroute_around_agents(
                    self.grid,(c.x,c.y),goal,self.engine.robots,
                    self.customers,c.customer_id,c.radius,
                    dynamic_obstacles=dyn_obs)
                if new_path:
                    c.route=list(new_path)
                    c.route_index=1 if len(new_path)>1 else len(new_path)
                    manager.customer_reroutes+=1
                    self.engine._event('CUSTOMER_REROUTED', customer_id=c.customer_id)
            if (c.blocked_ticks >= manager.policy.pedestrian_warning_after_ticks
                    and c.customer_id not in manager.warnings):
                manager.warnings.add(c.customer_id)
                self.engine._event('CUSTOMER_TRAFFIC_BLOCKED',customer_id=c.customer_id,
                                   reason='No collision-free pedestrian passage; waiting')
            # When blocked in a narrow corridor by an oncoming robot, yield backward along approach path
            if c.blocked_ticks > 15 and c.route_index > 1:
                prev_pt = c.route[c.route_index - 1]
                dx = prev_pt[0] - c.x
                dy = prev_pt[1] - c.y
                dist_prev = math.hypot(dx, dy)
                if dist_prev > 1e-4:
                    step_back = min(self.walking_speed * dt, dist_prev)
                    bx = c.x + (dx / dist_prev) * step_back
                    by = c.y + (dy / dist_prev) * step_back
                    dyn_obs = tuple(getattr(self.engine, 'dynamic_obstacles', {}).values())
                    if (manager.pedestrian_step_safe((c.x, c.y), (bx, by), self.engine.robots, c.radius)
                            and manager.customer_step_safe((c.x, c.y), (bx, by), self.customers, c.customer_id, c.radius)
                            and manager.dynamic_obstacle_step_safe((c.x, c.y), (bx, by), dyn_obs, c.radius)):
                        c.x, c.y = bx, by
                        manager.customer_yields += 1
                        if dist_prev <= step_back + 1e-4:
                            c.route_index -= 1
            return False
        c.blocked_ticks=0
        manager.clear_block(c.customer_id)
        return c.route_index>=len(c.route)

    def step(self,dt=.05):
        # Admit at most one queued customer per entrance, in FIFO order.
        # Existing customers move first; admissions cannot overlap them.
        for c in self.customers.values():
            if c.phase in ('SEATED','QUEUED','LEFT'):continue
            if c.phase=='TO_CASH':
                if self._walk(c,dt):
                    c.phase='PAYING'
                    self.engine._event('CUSTOMER_AT_CASH',customer_id=c.customer_id)
            elif c.phase=='PAYING':
                c.cash_wait+=dt
                if c.cash_wait>=self.payment_seconds:
                    # Payment creates the actual existing Stage 2 order.
                    # Keep it PREPARING until the customer has sat down.
                    c.order_id=self.engine.create_order(c.table,c.station,ready=False,
                                                        customer_id=c.customer_id)
                    self.engine._event('BILL_PAID',customer_id=c.customer_id,order_id=c.order_id)
                    self._set_route(c,c.seat)
                    c.phase='TO_TABLE'
            elif c.phase=='TO_TABLE':
                if self._walk(c,dt):
                    c.phase='SEATED'
                    self.engine.ready_order(c.order_id,customer_id=c.customer_id)
                    self.engine._event('CUSTOMER_SEATED',customer_id=c.customer_id,order_id=c.order_id,table=c.table)
        # One CASH approach slot; admit the oldest globally queued customer.
        # This prevents a perpetually busy door from starving other entrances.
        for waiting in sorted((c for c in self.customers.values()
                               if c.phase=='QUEUED'), key=lambda c:c.customer_id):
            if self._entrance_available(waiting.entrance):
                self._admit(waiting)
                break

    def snapshot(self):
        return [{'id':c.customer_id,'x':round(c.x,3),'y':round(c.y,3),
                 'phase':c.phase,'entrance':c.entrance,'table':c.table,
                 'order_id':c.order_id,'radius':c.radius} for c in self.customers.values()]
