"""Member 3 regression: real scene geometry and real order identities."""
from pathlib import Path
import random

import pytest

from aifr.orders.model import OrderStatus
from tools.customer_delivery_adapter import CustomerDeliveryBridge
from tools.customer_flow import CustomerFlow
from tools.customer_lifecycle import MealClock
from tools.customer_pose_adapter import seating_fraction
from tools.customer_seating import FREE, OCCUPIED, RESERVED, SeatRegistry
from tools.stage79_engine import Stage79Engine, load_reference

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def world():
    env, grid, _ = load_reference(ROOT)
    return env, grid


def build(world, seed=19):
    env, grid = world
    engine = Stage79Engine(env, grid, 1)
    wall = [100.0]
    flow = CustomerFlow(engine, customer_seating_enabled=True, seed=seed,
                        now_real_s=lambda: wall[0])
    return engine, flow, wall


def until_seated(flow, customer, wall, *, limit=4500):
    for _ in range(limit):
        flow.step(.05, now_real_s=wall[0])
        if customer.phase == 'SEATED':
            return
    pytest.fail(f'Customer never sat on {customer.seat_id}: {customer.phase}')


def test_actual_scene_geometry_maps_every_chair(world):
    _, flow, _ = build(world)
    audit = flow.seating.audit()
    assert audit['scene_chair_count'] == audit['mapped_chairs'] == 72
    assert audit['table_count'] == 18
    assert audit['expected_count_matches']
    assert not audit['unreachable_or_unmapped_chairs']
    assert set(audit['seats_per_table'].values()) == {4}
    for seat in flow.seating.seats.values():
        g = flow.engine.grid
        assert g.is_free(*g.world_to_grid(*seat.approach))
        assert seat.table_id in flow.engine.drops
        assert any((w.x,w.y) == (seat.delivery_x,seat.delivery_y)
                   for w in flow.engine.drops[seat.table_id])


def test_random_table_then_chair_is_seeded_and_no_double_booking(world):
    _, one, _ = build(world, seed=44)
    _, two, _ = build(world, seed=44)
    a = [one.seating.reserve_random_seat(cid).seat_id for cid in range(1, 73)]
    b = [two.seating.reserve_random_seat(cid).seat_id for cid in range(1, 73)]
    assert a == b
    assert len(set(a)) == 72
    assert max(one.seating.counts().values()) == 4
    assert min(one.seating.counts().values()) == 4
    assert one.seating.reserve_random_seat(1000) is None
    assert one.seating.reserve_random_seat(10).seat_id == a[9]
    chosen = one.seating.get(10)
    assert chosen.status == RESERVED
    one.seating.occupy(10)
    assert chosen.status == OCCUPIED
    assert one.seating.release(10) is chosen
    assert chosen.status == FREE
    assert one.seating.reserve_random_seat(1000) is chosen
    assert one.seating.release(1000) is chosen
    assert one.seating.release(1000) is None


def test_specific_table_capacity_and_seat_reuse(world):
    _, flow, _ = build(world)
    chairs = [flow.seating.reserve_random_seat(i, 'T11') for i in range(1,5)]
    assert len({c.seat_id for c in chairs}) == 4
    assert flow.seating.reserve_random_seat(5, 'T11') is None
    flow.seating.release(2)
    assert flow.seating.reserve_random_seat(5, 'T11').seat_id == chairs[1].seat_id


def test_full_capacity_customer_uses_existing_queued_phase(world):
    _, flow, _ = build(world)
    for i in range(100, 172):
        assert flow.seating.reserve_random_seat(i) is not None
    cid = flow.add(entrance='main_entry', station='SANDWICH')
    c = flow.customers[cid]
    assert c.phase == 'QUEUED' and c.seat_id is None
    flow.step(.05)
    assert c.phase == 'QUEUED'
    flow.seating.release(107)
    flow.step(.05)
    assert c.seat_id is not None and c.phase == 'TO_CASH'


def test_actual_chair_seating_and_order_ready_gate(world):
    engine, flow, wall = build(world)
    cid = flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    c = flow.customers[cid]
    assert c.seat_id.startswith('T11_')
    assert c.seat is not None and c.approach is not None
    assert flow.seating.get(cid).status == RESERVED
    until_seated(flow,c,wall)
    assert (c.x,c.y) == c.seat
    assert flow.seating.get(cid).status == OCCUPIED
    assert engine.orders.get(c.order_id).status is OrderStatus.READY
    assert engine.order_customers[c.order_id] == cid
    assert c.phase == 'SEATED' and flow.meals.meals[cid].deadline_real_s is None


def test_correct_serving_real_clock_and_exit_reuse(world):
    engine, flow, wall = build(world, seed=17)
    cid = flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    c = flow.customers[cid]
    until_seated(flow,c,wall)
    seat = flow.seating.get(cid)
    oid = c.order_id
    bridge = CustomerDeliveryBridge(clock=lambda: wall[0])
    assert not flow.on_food_delivered(oid, cid, now_real_s=wall[0])
    assert not flow.on_food_served(oid, cid, now_real_s=wall[0])
    # Simulate the scheduler's verified terminal order event after its actual
    # state transition, without needing an expensive full robot journey.
    engine.orders.get(oid).status = OrderStatus.DELIVERED
    engine._event('DELIVERED_RETURN_PENDING', order_id=oid, robot_id=1)
    assert bridge.consume(engine,flow,now_real_s=wall[0]) == 1
    m = flow.meals.meals[cid]
    assert 20 <= m.eating_duration_s <= 40
    assert m.delivered_at_real_s == m.served_at_real_s == wall[0]
    assert m.deadline_real_s == wall[0] + m.eating_duration_s
    assert bridge.consume(engine,flow,now_real_s=wall[0]) == 0
    assert not flow.on_food_served(oid, cid, now_real_s=wall[0] + 10)
    assert not flow.on_food_delivered(oid, cid+100, now_real_s=wall[0])
    wall[0] = m.deadline_real_s - .001
    flow.step(.05)
    assert c.phase == 'EATING'
    assert flow.seating.get(cid).status == OCCUPIED
    wall[0] = m.deadline_real_s
    flow.step(.05)
    assert c.phase == 'STANDING'
    for _ in range(50):
        flow.step(.05)
        if c.phase == 'TO_EXIT':
            break
    assert c.phase == 'TO_EXIT'
    assert seat.status == FREE
    assert flow.seating.get(cid) is None
    assert flow.seating.reserve_random_seat(2000,'T11') is not None
    for _ in range(6000):
        flow.step(.05)
        if cid not in flow.customers:
            break
    assert cid not in flow.customers
    assert flow.completed[0]['customer_id'] == cid


def test_speed_independent_real_timer_and_pause_policy():
    for speed in (1,2,4,8,16):
        clock = MealClock(rng=random.Random(5),clock=lambda:120.)
        clock.track(1, 8)
        assert clock.delivered(8,1, now_real_s=120.)
        assert clock.served(8,1, now_real_s=120.)
        meal = clock.meals[1]
        assert not meal.expired(120 + meal.eating_duration_s -.001)
        # Regardless of number of simulation ticks, only monotonic seconds count.
        for _ in range(speed*10):
            assert not meal.expired(120.)
        assert meal.expired(120 + meal.eating_duration_s)
        assert not clock.served(8,1, now_real_s=999.)
        assert not clock.delivered(8,1, now_real_s=999.)


def test_pose_interpolation_and_default_off(world):
    assert seating_fraction('TO_TABLE') == 0
    assert seating_fraction('SITTING', .3) == pytest.approx(.5)
    assert seating_fraction('SEATED') == 1
    assert seating_fraction('EATING') == 1
    assert seating_fraction('STANDING', .3) == pytest.approx(.5)
    assert seating_fraction('TO_EXIT') == 0
    env,grid=world
    e=Stage79Engine(env,grid,1)
    legacy=CustomerFlow(e)
    assert not legacy.customer_seating_enabled
    assert legacy.seating is None


def test_chair_approaches_avoid_other_furniture(world):
    _, flow, _ = build(world)
    env = flow.env

    def inside(rect,x,y,margin=.01):
        return (rect.min_x-margin <= x <= rect.max_x+margin
                and rect.min_y-margin <= y <= rect.max_y+margin)

    for seat in flow.seating.seats.values():
        for i in range(1,21):
            f=i/20
            x=seat.approach_x*(1-f)+seat.x*f
            y=seat.approach_y*(1-f)+seat.y*f
            assert not any(inside(t.rect,x,y) for t in env.tables)
            assert not any(inside(ch.rect,x,y) for ch in env.chairs
                           if ch.name != seat.seat_id)


def test_cancel_and_external_order_cancellation_release_seat(world):
    engine, flow, wall = build(world)
    cid = flow.add(entrance='main_entry', table='T11', station='SANDWICH')
    seat_id = flow.customers[cid].seat_id
    assert flow.remove_customer(cid, reason='cancel at entrance')
    assert seat_id in flow.seating.seats and flow.seating.seats[seat_id].status == FREE
    assert not flow.remove_customer(cid)
    cid2 = flow.add(entrance='main_entry', table='T11', station='SANDWICH')
    c = flow.customers[cid2]
    until_seated(flow,c,wall)
    engine.orders.cancel(c.order_id, reason='test cancellation')
    flow.step(.05)
    assert cid2 not in flow.customers
    assert flow.seating.seats[c.seat_id].status == FREE


def test_observations_show_actual_seated_and_moving_positions(world):
    engine, flow, wall = build(world)
    cid=flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    c=flow.customers[cid]
    flow.step(.05)
    obs=flow.get_customer_observations()
    assert len(obs)==1
    assert obs[0]['id']==cid
    assert 'vx' in obs[0] and 'timestamp_s' in obs[0]
    until_seated(flow,c,wall)
    assert flow.get_customer_observations()[0]['x'] == c.seat[0]
    assert flow.get_customer_observations()[0]['y'] == c.seat[1]
    assert flow.get_customer_observations()[0]['phase']=='SEATED'
    assert flow.seating.get(cid).seat_id==c.seat_id
    # Static furniture already occupies the chair; do not double-count the
    # same seated body as a second mobile obstacle in DWA.
    assert len(flow(0.0)) == 0


def test_real_engine_delivery_event_starts_meal(world):
    env,grid=world
    wall=[100.]
    flow=None
    engine=Stage79Engine(env,grid,1,pedestrians=lambda t: flow(t) if flow is not None else (),
                         traffic_mode='coordinated')
    flow=CustomerFlow(engine, customer_seating_enabled=True,seed=17,
                      now_real_s=lambda:wall[0])
    cid=flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    bridge=CustomerDeliveryBridge(clock=lambda:wall[0])
    engine.paused=False
    for _ in range(3500):
        flow.step(.05)
        engine.step()
        bridge.consume(engine,flow)
        if cid in flow.meals.meals and flow.meals.meals[cid].served:
            break
    else:
        pytest.fail('Real engine did not finish the linked robot order')
    c=flow.customers[cid]
    meal=flow.meals.meals[cid]
    assert c.phase == 'SEATED'
    assert engine.orders.get(c.order_id).status is OrderStatus.DELIVERED
    assert len([v for v in engine.events if v['event']=='CUSTOMER_FOOD_SERVED'])==1
    assert meal.deadline_real_s > wall[0]
    assert engine.failures == []


@pytest.mark.slow
def test_two_robots_three_customers_all_served_and_exit(world):
    """Real multi-robot/customer regression for a shared T11 table."""
    env,grid=world
    wall=[100.]
    flow=None
    engine=Stage79Engine(env,grid,2,
                         pedestrians=lambda t: flow(t) if flow else (),
                         traffic_mode='coordinated')
    flow=CustomerFlow(engine, customer_seating_enabled=True,seed=7,
                      now_real_s=lambda:wall[0])
    ids=[
        flow.add(entrance='main_entry', table='T11', station='SANDWICH'),
        flow.add(entrance='side_exit_1',table='T12',station='SANDWICH'),
        flow.add(entrance='side_exit_2',table='T11',station='SANDWICH'),
    ]
    bridge=CustomerDeliveryBridge(clock=lambda:wall[0])
    engine.paused=False
    all_served=False
    for _ in range(5500):
        flow.step(.05)
        engine.step()
        bridge.consume(engine,flow)
        if not all_served and sum(meal.served for meal in flow.meals.meals.values())==3:
            all_served=True
            wall[0]=max(meal.deadline_real_s for meal in flow.meals.meals.values())+.05
        if all_served and all(cid not in flow.customers for cid in ids):
            break
    assert not engine.failures
    assert all_served
    assert all(cid not in flow.customers for cid in ids)
    assert len(flow.completed)==3
    assert all(count==0 for count in flow.seating.counts().values())


def test_four_customers_physically_reach_four_chairs_at_same_table(world):
    _, flow, wall = build(world, seed=42)
    ids=[flow.add(entrance='main_entry',table='T11',station='SANDWICH')
         for _ in range(4)]
    for _ in range(3000):
        flow.step(.05,now_real_s=wall[0])
        if all(flow.customers[i].phase == 'SEATED' for i in ids):
            break
    assert all(flow.customers[i].phase == 'SEATED' for i in ids)
    assert len({flow.customers[i].seat_id for i in ids}) == 4
    assert flow.seating.counts()['T11'] == 4
