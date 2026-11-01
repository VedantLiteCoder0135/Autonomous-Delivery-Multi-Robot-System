"""Real cafeteria Stages 7–9 Python integration regression tests."""
from __future__ import annotations

import json
import math
from http.server import ThreadingHTTPServer
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from aifr.common.clock import SimClock
from aifr.common.events import EventBus
from aifr.environment.environment_mapper import EnvironmentMapper
from aifr.fleet.registry import RobotRegistry
from aifr.navigation.alns import ALNSRouteOptimizer
from aifr.navigation.dwa_route import RouteDWA, line_clear
from aifr.navigation.kinematics import Pose
from aifr.orders.manager import OrderManager
from aifr.orders.model import OrderStatus
from aifr.scheduling.scheduler import GlobalScheduler
from tools.stage79_dashboard import Stage79App, make_handler
from tools.stage79_engine import Stage79Engine, load_reference
from tools.stage79_scenarios import v14_pedestrian

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def reference():
    return load_reference(ROOT)


def test_four_entrances_are_wall_verified(reference):
    env, grid, report = reference
    assert set(e.name for e in env.entrances) == {
        'main_entry', 'side_exit_1', 'side_exit_2', 'sliding_entry_t11'}
    assert all(max(e.width, e.depth) >= .55 for e in env.entrances)
    assert report['v14_door_metadata']
    # A doorway being classified does not mean its centre is navigable on the
    # conservatively inflated Stage 4 grid, especially at a map boundary.
    assert all(not grid.is_free(*grid.world_to_grid(e.center_x, e.center_y))
               for e in env.entrances)


def test_door_metadata_cannot_override_physical_wall(reference):
    env, _, report = reference
    walls = [{'name':w.name, 'bbox':w.rect.to_dict()} for w in env.walls]
    walls.append({'name':'WALL_BLOCK_TEST', 'bbox':{
        'min_x':4.0,'max_x':4.2,'min_y':7.40,'max_y':7.60}})
    with pytest.raises(ValueError, match='blocked|valid wall gap'):
        EnvironmentMapper({'objects':walls,
                           'custom_props':{'AIFR_doors':json.dumps(report['v14_door_metadata'])}}).map()


def test_route_dwa_does_not_skip_grid_corner(reference):
    _, grid, _ = reference
    path = [(14.95,5.25),(15.05,5.25),(15.05,5.35),(15.05,5.45)]
    p = RouteDWA(grid)
    p.set_path(path)
    p.index = 1
    target = p._target(Pose(14.9805337,5.2920802,1.156387))
    assert line_clear(grid, (14.9805337,5.2920802), target)
    assert target != path[-1], 'Do not cut a blocked grid-cell corner'


def test_alns_keeps_collision_free_route_and_improves_cost(reference):
    _, grid, report = reference
    start = report['mission']['spawn'];goal = report['mission']['pickup']
    a = (start['x'], start['y']);b = (goal['x'], goal['y'])
    route = ALNSRouteOptimizer(grid, seed=12).optimize(a,b,iterations=50)
    assert route.initial_cost >= route.optimized_cost - 1e-9
    assert math.dist(route.path[0],a) < .15
    assert math.dist(route.path[-1],b) < .15
    assert all(line_clear(grid,x,y) for x,y in zip(route.path,route.path[1:]))


def test_scheduler_does_not_release_delivered_robot_before_return():
    clock=SimClock(50_000);bus=EventBus(record=True)
    orders=OrderManager(bus,clock);fleet=RobotRegistry(bus,clock,1)
    scheduler=GlobalScheduler(bus,clock,orders,fleet,release_on_delivery=False)
    a=orders.create_order('T11','SANDWICH');orders.start_preparing(a.order_id);orders.mark_ready(a.order_id)
    b=orders.create_order('T12','JUICE');orders.start_preparing(b.order_id);orders.mark_ready(b.order_id)
    assert [int(x.order_id) for x in scheduler.dispatch()] == [int(a.order_id)]
    for status in (OrderStatus.TO_PICKUP,OrderStatus.LOADING,OrderStatus.TO_CUSTOMER,
                   OrderStatus.UNLOADING,OrderStatus.DELIVERED):
        orders.advance(a.order_id,status)
    assert scheduler.is_return_pending(1)
    assert not fleet.is_available(1)
    assert scheduler.dispatch() == []
    scheduler.complete_return(1)
    assert fleet.is_available(1)
    assert [int(x.order_id) for x in scheduler.dispatch()] == [int(b.order_id)]


def test_four_robots_have_separate_bases_and_traffic_admission(reference):
    env,grid,_=reference
    e=Stage79Engine(env,grid,robot_count=4)
    bases=[r.base for r in e.robots.values()]
    assert all(math.dist((a.x,a.y),(b.x,b.y)) > .65
               for i,a in enumerate(bases) for b in bases[i+1:])
    for table,station in [('T11','SANDWICH'),('T12','JUICE'),
                          ('T13','TEA'),('T14','MAIN_COURSE')]:
        e.create_order(table,station)
    e.paused=False;e.step()
    assert [e.robots[i].phase for i in (3,4)] == ['WAITING_FOR_AISLE']*2
    assert all(not e.fleet.is_available(i) for i in (1,2,3,4))
    assert len(e.orders.by_status(OrderStatus.ASSIGNED, OrderStatus.TO_PICKUP)) == 4


def test_one_robot_responds_to_moving_pedestrian(reference):
    env,grid,_=reference
    baseline=Stage79Engine(env,grid,robot_count=1)
    baseline.create_order('T11','SANDWICH')
    r0=baseline.run(max_ticks=3000)
    moving=Stage79Engine(env,grid,robot_count=1,pedestrians=v14_pedestrian(grid))
    moving.create_order('T11','SANDWICH')
    r1=moving.run(max_ticks=3000)
    assert r0['success'] and r1['success']
    assert r1['robots'][0]['lidar_detection_ticks'] > 0
    assert r1['min_pedestrian_clearance'] > 0
    t0=baseline.robots[1].trace;t1=moving.robots[1].trace
    assert max(math.dist((a['x'],a['y']),(b['x'],b['y']))
               for a,b in zip(t0,t1)) > .02
    assert any(a['v'] != b['v'] or a['w'] != b['w'] for a,b in zip(t0,t1))


def test_two_robots_three_orders_complete_with_return_before_reassignment(reference):
    env,grid,_=reference
    e=Stage79Engine(env,grid,robot_count=2)
    for table,station in [('T11','SANDWICH'),('T12','JUICE'),('T13','TEA')]:
        e.create_order(table,station)
    report=e.run(max_ticks=6500)
    assert report['success'], report['failures']
    assert len(report['entrances']) == 4
    assert len(report['orders']) == 3
    assert all(o['status'] == 'DELIVERED' for o in report['orders'])
    assert all(r['phase'] == 'IDLE' and r['available'] for r in report['robots'])
    assert report['return_before_reassignment']
    assert report['alns_total_optimized_cost'] <= report['alns_total_initial_cost']
    assert report['min_robot_clearance'] > 0
    assert any(x['event']=='RETURNED_TO_BASE' for x in report['events'])
    assert any(x['event']=='ASSIGNED' and x['order_id']==3 for x in report['events'])


def test_dashboard_http_controls_and_order_creation():
    app=Stage79App(ROOT,robot_count=2)
    server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(app))
    thread=threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    def get(path):
        with urlopen(base+path,timeout=4) as response:
            return json.load(response)
    def post(path,data):
        req=Request(base+path, data=json.dumps(data).encode(),
                    headers={'Content-Type':'application/json'},method='POST')
        with urlopen(req,timeout=4) as response:
            return json.load(response)
    try:
        assert len(get('/api/map')['entrances']) == 4
        assert get('/api/state')['paused']
        assert post('/api/order',{'table':'T11','station':'SANDWICH'})['order_id'] == 1
        assert post('/api/control',{'action':'speed','value':4})['message']
        assert post('/api/control',{'action':'step'})['message']
        state=get('/api/state')
        assert state['tick'] == 1 and state['speed'] == 4
        assert post('/api/control',{'action':'estop'})['message']
        with pytest.raises(HTTPError) as exc:
            post('/api/control',{'action':'start'})
        assert exc.value.code == 400
        assert post('/api/control',{'action':'reset'})['message']
        assert get('/api/state')['orders'] == []
    finally:
        server.shutdown();server.server_close();thread.join(timeout=2)
