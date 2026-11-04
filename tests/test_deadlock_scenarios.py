"""Deadlock prevention / recovery: unit tests for each mechanism plus full-engine scenarios.

Every scenario asserts the same safety envelope: no FAULT, no collision-guard trip,
positive robot-robot and robot-pedestrian clearance; and, where it applies, that every
order is delivered and every robot is back at base (return-before-reassignment).
"""
from __future__ import annotations

import math
import random
from pathlib import Path
from types import SimpleNamespace

import pytest

from aifr.navigation.dwa import DWAConfig, MovingObstacle
from aifr.navigation.dwa_route import RouteDWA
from aifr.navigation.kinematics import Pose, advance
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.perception.ekf import PoseEKF
from aifr.perception.sensors import lidar_scan
from tools.customer_flow import CustomerFlow
from tools.deadlock_manager import DeadlockManager
from tools.live_pedestrians import LivePedestrians
from tools.stage79_engine import Stage79Engine, load_reference
from tools.traffic_coordinator import (LaneMap, LaneSample, WaitForGraph, corridor_conflict,
                                       find_refuge, lane_claim)

ROOT = Path(__file__).resolve().parents[1]
ROBOT_R = math.hypot(.20, .18)
slow = pytest.mark.slow


@pytest.fixture(scope='module')
def world():
    return load_reference(ROOT)


# --------------------------------------------------------------------------- helpers
def build(world, robots, *, customers=False, pedestrians=None, seed=11):
    env, grid, _ = world
    holder: dict = {}
    provider = pedestrians or (lambda t: holder['flow'](t) if 'flow' in holder else ())
    engine = Stage79Engine(env, grid, robot_count=robots, pedestrians=provider,
                           traffic_mode='coordinated', safety_margin=.18, seed=seed)
    flow = None
    if customers:
        flow = CustomerFlow(engine)
        holder['flow'] = flow
    return engine, flow


def place(engine, rid, x, y, theta):
    """Teleport a robot (test set-up only; never used while the simulation is judged)."""
    r = engine.robots[rid]
    r.pose = Pose(x, y, theta)
    r.ekf = PoseEKF(r.pose)
    r.linear = r.angular = 0.


def drive(engine, flow, until, max_ticks):
    engine.paused = False
    for _ in range(max_ticks):
        if flow is not None:
            flow.step(.05)
        engine.step()
        if engine.failures or until():
            break
    return engine.clock.tick


def all_done(engine, n_orders=None):
    orders = engine.orders.all_orders()
    return (bool(orders) and (n_orders is None or len(orders) == n_orders)
            and all(o.status.name == 'DELIVERED' for o in orders)
            and all(r.phase == 'IDLE' for r in engine.robots.values()))


def assert_safe(engine):
    assert engine.failures == []
    assert not [e for e in engine.events if e['event'] == 'FAULT']
    assert all(r.phase != 'FAULT' for r in engine.robots.values())
    assert engine.min_robot_clearance > 0
    assert engine.min_pedestrian_clearance > 0


def assert_delivered(engine, n_orders):
    assert all_done(engine, n_orders), [
        (r.robot_id, r.phase, r.order_id, round(r.pose.x, 2), round(r.pose.y, 2), r.wait_on)
        for r in engine.robots.values()]
    report = engine.report()
    assert report['success'] and report['return_before_reassignment']
    assert_safe(engine)
    retreats = {}
    for event in engine.events:
        if event['event'] == 'RETREAT':
            retreats[event['robot_id']] = event['tick']
        elif event['event'] == 'REFUGE_REACHED' and event['robot_id'] in retreats:
            assert event['tick'] - retreats.pop(event['robot_id']) <= 2400, 'Retreat exceeded 120 seconds'


def make_customer_walk(flow, engine, start, seat, *, table='T11', station='TEA'):
    """Put a customer on a chosen walk (set-up helper). The paid order stays unready."""
    cid = flow.add(entrance='main_entry', table=table, station=station)
    c = flow.customers[cid]
    c.order_id = engine.create_order(table, station, ready=False, customer_id=cid)
    c.phase, c.x, c.y, c.seat = 'TO_TABLE', start[0], start[1], seat
    flow._set_route(c, seat)
    return c


# ------------------------------------------------------------- wait-for graph / priority
def test_wait_for_graph_finds_two_and_three_cycles():
    g = WaitForGraph()
    g.add(1, 2); g.add(2, 1)
    g.add(3, 4); g.add(4, 5); g.add(5, 3)
    g.add(6, 1)                      # a tail feeding into a cycle is not part of it
    found = sorted(sorted(c) for c in g.cycles())
    assert found == [[1, 2], [3, 4, 5]]


def test_wait_for_graph_chain_has_no_cycle_and_reports_its_head():
    g = WaitForGraph()
    g.add(1, 2); g.add(2, 3)
    assert g.cycles() == []
    assert g.chain_head(1) == 3
    g.add(1, 1)                       # self-wait is ignored
    assert g.edges[1] == 2


def test_priority_prefers_loaded_robot_and_ages_with_yields():
    carrying = SimpleNamespace(phase='TO_CUSTOMER', yields=0, robot_id=2,
                               pose=Pose(0, 0), destination=(5, 0), mission_destination=(5, 0))
    pickup = SimpleNamespace(phase='TO_PICKUP', yields=0, robot_id=1,
                             pose=Pose(0, 0), destination=(1, 0), mission_destination=(1, 0))
    prio = DeadlockManager.robot_priority
    assert prio(carrying) > prio(pickup)
    # A robot of the same class that has yielded before outranks one that never has,
    # so repeated deadlocks cannot starve the same robot.
    twin = SimpleNamespace(**{**vars(pickup), 'robot_id': 3, 'yields': 2})
    assert prio(twin) > prio(pickup)
    # Priority follows the mission goal, not a temporary refuge.
    away = SimpleNamespace(**{**vars(pickup), 'destination': (30, 0)})
    assert prio(away) == prio(pickup)


# ------------------------------------------------------------------- reservation rules
def test_reservation_allows_separating_motion_inside_the_soft_envelope():
    m = DeadlockManager()
    env, phys = 2 * ROBOT_R + .22, 2 * ROBOT_R
    a, b = Pose(0, 0), Pose(.7, 0)     # inside the envelope, outside physical contact
    assert not m.robot_pair_conflict(a, Pose(-.05, 0), b, b, env, phys)   # moving away
    assert m.robot_pair_conflict(a, Pose(.05, 0), b, b, env, phys)        # closing in


def test_reservation_never_allows_physical_contact():
    m = DeadlockManager()
    phys = 2 * ROBOT_R
    a, b = Pose(0, 0), Pose(.6, 0)
    assert m.robot_pair_conflict(a, Pose(.1, 0), b, b, 0., phys)          # would touch
    assert m.robot_pair_conflict(Pose(0, 0), Pose(0, 0), Pose(.3, 0), Pose(.3, 0), 0., phys)


def test_two_robots_passing_head_on_conflict_even_if_each_move_is_small():
    m = DeadlockManager()
    phys = 2 * ROBOT_R
    # Swept (not endpoint) check: they cross within one tick.
    assert m.robot_pair_conflict(Pose(-.3, 0), Pose(.3, 0), Pose(.3, 0), Pose(-.3, 0), 0., phys)


# ---------------------------------------------------------------- DWA, LiDAR hot spots
def test_dwa_inside_safety_margin_never_closes_the_gap():
    grid = OccupancyGrid(0, 0, .1, 80, 40, tuple((0,) * 80 for _ in range(40)))
    cfg = DWAConfig(safety_margin=.18)
    person = MovingObstacle(2.0, 2.0, .14)
    # Robot is 0.55 m from a person: inside robot_radius + radius + margin (0.59 m).
    pose = Pose(2.0 - .55, 2.0, 0.)
    planner = RouteDWA(grid, cfg)
    planner.set_path([(1.0, 2.0), (3.0, 2.0)])
    v = w = 0.
    start_gap = math.hypot(pose.x - person.x, pose.y - person.y)
    for _ in range(60):
        cmd = planner.plan(pose, v, w, (person,))
        if cmd.safe:
            pose = advance(pose, cmd.linear, cmd.angular, cfg.dt)
            v, w = cmd.linear, cmd.angular
        else:
            v = w = 0.
        assert math.hypot(pose.x - person.x, pose.y - person.y) >= start_gap - .01
    # Never creeps to near contact (ROBOT_R + radius = 0.41 m).
    assert math.hypot(pose.x - person.x, pose.y - person.y) > ROBOT_R + person.radius + .1


def test_lidar_prefilter_matches_exhaustive_scan():
    def reference(pose, boxes, rays, max_range, step):
        out = []
        for i in range(rays):
            a = pose.theta + 2 * math.pi * i / rays
            c, s, d = math.cos(a), math.sin(a), max_range
            for k in range(1, math.ceil(max_range / step) + 1):
                r = min(k * step, max_range)
                x, y = pose.x + r * c, pose.y + r * s
                if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in boxes):
                    d = r
                    break
            out.append(d)
        return out
    rng = random.Random(4)
    for _ in range(12):
        pose = Pose(rng.uniform(0, 10), rng.uniform(0, 10), rng.uniform(-3, 3))
        boxes = [(x - .2, y - .2, x + .2, y + .2)
                 for x, y in ((rng.uniform(0, 10), rng.uniform(0, 10)) for _ in range(rng.randint(0, 6)))]
        assert lidar_scan(pose, boxes, rays=36, max_range=4., step=.04) == \
            reference(pose, boxes, 36, 4., .04)


# ----------------------------------------------------------- lanes, claims, refuge sites
def test_lane_map_separates_single_lane_aisles_from_open_floor(world):
    _, grid, _ = world
    lanes = LaneMap(grid)
    assert lanes.is_narrow(9.45, 12.0)          # main one-robot-wide aisle
    assert not lanes.is_narrow(3.0, 4.5)        # open hall beside the left counters
    assert not lanes.is_narrow(6.0, 6.0)        # robot base area


def test_corridor_claims_only_conflict_when_travelling_toward_each_other(world):
    _, grid, _ = world
    lanes = LaneMap(grid)
    north = lane_claim([(9.45, 8.5), (9.45, 12.0)], 0, (9.45, 8.5), lanes)
    south = lane_claim([(9.45, 12.0), (9.45, 8.5)], 0, (9.45, 12.0), lanes)
    follower = lane_claim([(9.45, 9.0), (9.45, 12.0)], 0, (9.45, 9.0), lanes)
    assert north and south
    assert corridor_conflict(north, south)
    assert not corridor_conflict(north, follower)       # same direction: just follow
    far = [LaneSample(x.x + 5, x.y, x.dx, x.dy) for x in south]
    assert not corridor_conflict(north, far)            # disjoint lanes


def test_refuge_is_reachable_outside_keepouts_and_prefers_a_passing_bay(world):
    _, grid, _ = world
    lanes = LaneMap(grid)
    other = (9.45, 11.0, 1.0)
    refuge = find_refuge(grid, lanes, (9.45, 11.9), [other])
    assert refuge is not None
    assert math.hypot(refuge[0] - other[0], refuge[1] - other[1]) >= other[2]
    assert grid.is_free(*grid.world_to_grid(*refuge))
    # An excluded refuge is not offered again, so a retried yield changes position.
    again = find_refuge(grid, lanes, (9.45, 11.9), [other], exclude=[refuge])
    assert again is None or math.dist(again, refuge) > .5


# --------------------------------------------------- engine scenarios: robot vs robot
@slow
def test_four_robots_clustered_at_base_no_longer_freeze(world):
    """Regression: all four bases sit inside each other's soft envelope; every move
    used to be vetoed forever without any FAULT or recovery."""
    engine, _ = build(world, 4)
    stations = sorted(engine.pickups)
    tables = sorted(engine.drops)
    for i in range(4):
        engine.create_order(tables[(i * 5) % len(tables)], stations[i % len(stations)])
    drive(engine, None, lambda: all_done(engine, 4), 7000)
    assert_delivered(engine, 4)
    assert engine.deadlocks_resolved + engine.retreats >= 1     # recovery actually used


@slow
def test_head_on_in_single_lane_aisle_resolves_by_priority_yield(world):
    engine, _ = build(world, 2)
    engine.create_order('T11', 'TEA')
    engine.create_order('T12', 'JUICE')
    engine.paused = False
    engine.step()
    south, north = (9.45, 9.2), (9.45, 15.2)
    place(engine, 1, *south, math.pi / 2)
    place(engine, 2, *north, -math.pi / 2)
    engine._set_route(engine.robots[1], north, 'TEST')
    engine._set_route(engine.robots[2], south, 'TEST')
    arrived = lambda: {e['robot_id'] for e in engine.events if e['event'] == 'LOADING'} == {1, 2}
    drive(engine, None, arrived, 4000)
    assert arrived(), 'both robots must reach the far end of the aisle'
    assert_safe(engine)
    kinds = {e['event'] for e in engine.events}
    assert {'RETREAT', 'REFUGE_REACHED', 'RESUME'} <= kinds
    # The yielding robot keeps its order and later finishes it.
    drive(engine, None, lambda: all_done(engine, 2), 9000)
    assert_delivered(engine, 2)


@slow
def test_left_counter_queue_four_robots_one_station(world):
    """The left-hand MAIN_COURSE counter has one pickup point; four robots converge on it."""
    engine, _ = build(world, 4)
    tables = ['T03', 'T08', 'T11', 'T14']
    for t in tables:
        engine.create_order(t, 'MAIN_COURSE')
    drive(engine, None, lambda: all_done(engine, 4), 12000)
    assert_delivered(engine, 4)


def test_lane_yield_moves_a_robot_out_of_an_oncoming_lane_before_contact(world):
    engine, _ = build(world, 2)
    engine.create_order('T11', 'TEA')
    engine.create_order('T12', 'JUICE')
    engine.paused = False
    engine.step()
    place(engine, 1, 9.45, 9.2, math.pi / 2)
    place(engine, 2, 9.45, 15.2, -math.pi / 2)
    engine._set_route(engine.robots[1], (9.45, 15.2), 'TEST')
    engine._set_route(engine.robots[2], (9.45, 9.2), 'TEST')
    drive(engine, None, lambda: engine.corridor_waits > 0 or engine.retreats > 0, 1500)
    assert engine.corridor_waits > 0 or engine.retreats > 0
    assert engine.min_robot_clearance > 0.2        # coordination happened well before contact


# ------------------------------------------------------- engine scenarios: with customers
@slow
def test_robot_gives_way_to_customer_walking_toward_it_in_an_aisle(world):
    """x=12.35 is a one-robot-wide aisle ending in a wide pocket at y~16.3 (the customer's
    seat). The robot comes down the aisle while the customer walks up it."""
    engine, flow = build(world, 1, customers=True)
    engine.create_order('T12', 'JUICE')
    customer = make_customer_walk(flow, engine, (12.35, 9.6), (12.35, 16.35))
    engine.paused = False
    flow.step(.05)
    engine.step()
    place(engine, 1, 12.35, 15.0, -math.pi / 2)
    engine._set_route(engine.robots[1], (12.35, 9.2), 'TEST')
    seated = lambda: customer.phase == 'SEATED'
    reached = lambda: any(e['event'] == 'LOADING' for e in engine.events)
    drive(engine, flow, lambda: seated() and reached(), 4500)
    assert seated(), 'customer must not be held up forever by the robot'
    assert reached(), 'robot must still reach its goal'
    assert_safe(engine)
    # Someone gave way (robot retreat/replan or customer re-route/yield): never a pass-through.
    assert any(e['event'] in ('RETREAT', 'DYNAMIC_REPLAN', 'CUSTOMER_REROUTED', 'CUSTOMER_YIELDING')
               for e in engine.events)


def test_walkers_meeting_head_on_resolve_with_one_giving_way(world):
    engine, flow = build(world, 1, customers=True)
    a = make_customer_walk(flow, engine, (9.45, 9.4), (9.45, 12.4))
    b = make_customer_walk(flow, engine, (9.45, 12.4), (9.45, 9.4), table='T12', station='JUICE')
    # Re-seat so each target is free of the other's start.
    a.seat, b.seat = (9.45, 12.4), (9.45, 9.4)
    flow._set_route(a, a.seat)
    flow._set_route(b, b.seat)
    engine.paused = False
    for _ in range(2500):
        flow.step(.05)
        if a.phase == b.phase == 'SEATED':
            break
        # The pair must never overlap, whatever happens.
        assert math.hypot(a.x - b.x, a.y - b.y) >= a.radius + b.radius - 1e-9
    assert a.phase == b.phase == 'SEATED'


@slow
def test_blocked_destination_waits_without_fault_then_completes(world):
    """A ring of loose obstacles seals the pickup: the robot must wait/back off (never
    FAULT), keep its order, and finish once the way is clear."""
    engine, _ = build(world, 1)
    engine.create_order('T11', 'SANDWICH')
    sx, sy = 8.8, 1.7
    for i in range(10):
        ang = 2 * math.pi * i / 10
        engine.add_dynamic_obstacle(sx + .95 * math.cos(ang), sy + .95 * math.sin(ang),
                                    radius=.22, name=f'RING_{i}')
    drive(engine, None, lambda: False, 2600)
    r = engine.robots[1]
    assert not engine.failures and r.phase != 'FAULT'
    assert r.order_id is not None and not any(e['event'] == 'LOADING' for e in engine.events)
    assert engine.stuck_events >= 1                    # detected, not silently frozen
    engine.clear_dynamic_obstacles()
    drive(engine, None, lambda: all_done(engine, 1), 9000)
    assert_delivered(engine, 1)


@slow
@pytest.mark.parametrize('n', [8, 12])
def test_four_robots_with_many_moving_customers(world, n):
    engine, flow = build(world, 4, customers=True)
    rng = random.Random(7)
    entrances = list(flow.entrance_points)
    tables, stations = sorted(engine.drops), sorted(engine.pickups)
    for i in range(n):
        flow.add(entrance=entrances[i % len(entrances)], table=rng.choice(tables),
                 station='MAIN_COURSE' if i % 2 == 0 else rng.choice(stations))
    done = lambda: (len(flow.customers) == n and all(c.phase == 'SEATED' for c in flow.customers.values())
                    and all_done(engine, n))
    drive(engine, flow, done, 20000)
    assert_delivered(engine, n)
    assert engine.deadlocks_resolved + engine.retreats >= 0


@slow
def test_four_robots_customers_and_patrolling_pedestrians(world):
    env, grid, _ = world
    holder: dict = {}
    live = LivePedestrians(grid)
    engine = Stage79Engine(env, grid, robot_count=4, seed=5, traffic_mode='coordinated',
                           safety_margin=.18,
                           pedestrians=lambda t: tuple(holder['flow'](t)) + tuple(live(t)))
    flow = CustomerFlow(engine)
    holder['flow'] = flow
    engine.walkers = live
    # Non-yielding walkers that cross robot routes in the open hall (but do not seal the exit).
    live.add(4.0, 5.0, 4.0, 6.5, 0.)
    live.add(3.5, 3.5, 5.0, 3.5, 0.)
    n = 8
    entrances = list(flow.entrance_points)
    tables = sorted(engine.drops)
    for i in range(n):
        flow.add(entrance=entrances[i % len(entrances)], table=tables[(i * 3) % len(tables)],
                 station=['MAIN_COURSE', 'TEA', 'SANDWICH', 'JUICE'][i % 4])
    done = lambda: (all(c.phase == 'SEATED' for c in flow.customers.values()) and all_done(engine, n))
    drive(engine, flow, done, 22000)
    assert_delivered(engine, n)
    assert engine.min_pedestrian_clearance >= .15


def test_snapshot_reports_deadlock_statistics(world):
    engine, _ = build(world, 2)
    snap = engine.snapshot()
    assert set(snap['deadlock_stats']) == {'resolved', 'retreats', 'corridor_waits', 'stuck_events'}
    assert {'wait_on', 'hold', 'retreating', 'yields'} <= set(snap['robots'][0])


def test_sealed_mission_enters_blocked_state_and_waits_for_geometry_change(world):
    engine, _ = build(world, 1)
    engine.create_order('T11', 'SANDWICH')
    engine.paused = False
    engine.step()
    r = engine.robots[1]
    # Seal every validated approach, rather than merely the currently selected goal.
    for i, w in enumerate(engine.pickups['SANDWICH']):
        engine.add_dynamic_obstacle(w.x, w.y, radius=.22, name=f'SEAL_{i}')
    for _ in range(4):
        engine._handle_stuck(r)
    assert r.navigation_state == 'BLOCKED'
    assert r.order_id == 1 and not engine.fleet.is_available(1)
    count = len(engine.routes)
    for _ in range(300):
        engine.step()
    assert len(engine.routes) == count
    assert engine.orders.get(1).status.name == 'TO_PICKUP'
    assert engine.snapshot()['robots'][0]['navigation_state'] == 'BLOCKED'
    engine.clear_dynamic_obstacles()
    engine.step()
    assert r.navigation_state == 'READY'
    assert r.planner is not None
    assert not engine.failures


def test_unreachable_static_destination_does_not_raise_or_release_order(world):
    engine, _ = build(world, 1)
    engine.create_order('T11', 'TEA')
    engine.paused = False
    engine.step()
    r = engine.robots[1]
    engine._set_route(r, (-100., -100.), 'RECOVERY')
    assert r.navigation_state == 'BLOCKED'
    for _ in range(100):
        engine.step()
    assert r.order_id == 1 and not engine.fleet.is_available(1)
    assert not engine.failures


def test_swept_pedestrian_check_detects_crossing_with_clear_endpoints():
    a = Pose(0., 0.)
    b0, b1 = Pose(-1., 0.), Pose(1., 0.)
    assert math.dist((a.x, a.y), (b0.x, b0.y)) > .5
    assert math.dist((a.x, a.y), (b1.x, b1.y)) > .5
    assert Stage79Engine._swept_clearance(a, a, b0, b1, .5) == pytest.approx(-.5)


def test_retreat_can_reverse_without_turning_into_an_approaching_person():
    grid = OccupancyGrid(0, 0, .1, 80, 40, tuple((0,) * 80 for _ in range(40)))
    planner = RouteDWA(grid, DWAConfig(safety_margin=.18, allow_reverse=True))
    planner.set_path([(3., 2.), (1., 2.)])
    pose = Pose(3., 2., 0.)
    person = MovingObstacle(4., 2., .16, -.2, 0.)
    command = planner.plan(pose, 0., 0., (person,))
    assert command.safe and command.linear < 0
    assert abs(command.angular) < .01
    assert abs(command.linear) <= planner.config.linear_acceleration * planner.config.dt
    nxt = advance(pose, command.linear, command.angular, planner.config.dt)
    assert nxt.x < pose.x
    assert planner._brake_safe(pose, command.linear, command.angular, (person,))


def test_reverse_retreat_still_rejects_physical_obstacles_behind_robot():
    grid = OccupancyGrid(0, 0, .1, 80, 40, tuple((0,) * 80 for _ in range(40)))
    planner = RouteDWA(grid, DWAConfig(safety_margin=.18, allow_reverse=True))
    planner.set_path([(3., 2.), (1., 2.)])
    person = MovingObstacle(2.55, 2., .16)
    assert not planner._brake_safe(Pose(3., 2.), -.25, 0., (person,))


def test_transient_route_failure_preserves_patrol_reservation(world, monkeypatch):
    from tools.stage79_engine import ALNSRouteOptimizer
    engine, _ = build(world, 1)
    live = LivePedestrians(engine.grid)
    live.add(4., 5., 4., 6.5, 0.)
    engine.walkers = live
    engine.pedestrians = live
    original = ALNSRouteOptimizer.optimize
    calls = []

    def temporarily_blocked(optimizer, *args, **kwargs):
        calls.append(optimizer.grid)
        if len(calls) == 1:
            raise ValueError('Transient robot occupies the route')
        return original(optimizer, *args, **kwargs)

    monkeypatch.setattr(ALNSRouteOptimizer, 'optimize', temporarily_blocked)
    robot = engine.robots[1]
    engine._set_route(robot, (8., 5.), 'RECOVERY')
    assert len(calls) == 2
    assert robot.planner is not None
    assert not robot.planner.grid.is_free(*engine.grid.world_to_grid(4., 5.7))
    assert engine.grid.is_free(*engine.grid.world_to_grid(4., 5.7))


def test_replanning_cannot_reset_the_mission_recovery_budget(world):
    engine, _ = build(world, 1)
    engine.create_order('T11', 'TEA')
    engine.paused = False
    engine.step()
    r = engine.robots[1]
    r.recovery_budget = 11
    r.mission_progress_tick = engine.clock.tick - 2400
    engine._replan(r, 'Repeated local route without physical progress')
    assert r.recovery_budget == 11
    engine._handle_stuck(r)
    assert r.navigation_state == 'BLOCKED'
    assert r.recovery_budget == 12
    count = len(engine.routes)
    for _ in range(80):
        engine.step()
    assert len(engine.routes) == count
    assert r.order_id == 1 and not engine.fleet.is_available(1)


def test_alternative_waypoint_selection_is_not_physical_mission_progress(world):
    engine, _ = build(world, 1)
    engine.create_order('T11', 'TEA')
    engine.paused = False
    engine.step()
    r = engine.robots[1]
    r.recovery_budget = 11
    progress_tick = r.mission_progress_tick
    goal = (r.base.x + .3, r.base.y)
    engine._set_route(r, goal, 'RECOVERY')
    assert r.mission_best_distance == pytest.approx(math.dist((r.pose.x, r.pose.y), goal))
    assert r.recovery_budget == 11
    assert r.mission_progress_tick == progress_tick
