"""Regression guards for phases 1-7: identities, geometry, pedestrians, live controls."""
from pathlib import Path
from types import SimpleNamespace
import math
import pytest

from tools.stage79_engine import Stage79Engine,load_reference
from tools.customer_flow import CustomerFlow
from tools.deadlock_manager import DeadlockManager
from tools.geometry_hardening import audit_egress_and_waiting
from aifr.navigation.kinematics import Pose
from aifr.orders.model import OrderStatus

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def world():
    e,g,_=load_reference(ROOT)
    return e,g

def make(world,n=1):
    env,grid=world
    engine=Stage79Engine(env,grid,n,traffic_mode='exclusive')
    return engine,CustomerFlow(engine)

def test_v14_pickup_egress_and_waiting_nonblocking(world):
    env,grid=world
    cells=grid.cells
    a=audit_egress_and_waiting(env,grid)
    assert a['entrances']==4
    assert {x['station'] for x in a['pickups']}=={'TEA','SANDWICH','JUICE','MAIN_COURSE'}
    assert all(x['approach_reachable'] and x['egress_reachable'] for x in a['pickups'])
    assert len(a['waiting'])>=5
    assert all(x['nonblocking_for_pickup_access'] for x in a['waiting'])
    assert grid.cells==cells

def test_identical_orders_have_immutable_distinct_customer_ids(world):
    engine,flow=make(world)
    ids=[flow.add(entrance='main_entry',table='T11',station='SANDWICH') for _ in range(4)]
    for i in range(5000):
        flow.step()
        if all(flow.customers[x].phase=='SEATED' for x in ids):break
    assert all(flow.customers[x].phase=='SEATED' for x in ids)
    order_ids=[flow.customers[x].order_id for x in ids]
    assert len(set(order_ids))==4
    assert {engine.order_customers[x] for x in order_ids}==set(ids)
    assert {engine.customer_orders[x] for x in ids}==set(order_ids)
    assert all(engine.orders.get(oid).status is OrderStatus.READY for oid in order_ids)
    with pytest.raises(ValueError,match='already paid'):
        engine.create_order('T11','SANDWICH',customer_id=ids[0])
    with pytest.raises(ValueError,match='identity mismatch'):
        engine.ready_order(order_ids[0],customer_id=ids[1])

def test_different_shops_same_table_have_distinct_orders(world):
    engine,_=make(world)
    a=engine.create_order('T11','SANDWICH',customer_id=101)
    b=engine.create_order('T11','MAIN_COURSE',customer_id=102)
    assert a!=b
    assert engine.snapshot()['orders']==[
        {'id':1,'table':'T11','station':'SANDWICH','status':'READY',
         'robot_id':None,'ready_seq':1,'customer_id':101},
        {'id':2,'table':'T11','station':'MAIN_COURSE','status':'READY',
         'robot_id':None,'ready_seq':2,'customer_id':102}]

def test_cashier_fifo_across_different_doors(world):
    engine,flow=make(world)
    names=list(flow.entrance_points)
    ids=[flow.add(entrance=names[i%len(names)],table='T11',station='SANDWICH') for i in range(3)]
    for _ in range(4500):
        flow.step()
        if all(c.phase=='SEATED' for c in flow.customers.values()):break
    assert all(flow.customers[i].phase=='SEATED' for i in ids)
    entries=[event['customer_id'] for event in engine.events if event['event']=='CUSTOMER_ENTERED']
    assert entries==ids

def test_pedestrian_cannot_cross_other_pedestrian_or_robot():
    m=DeadlockManager()
    people={1:SimpleNamespace(customer_id=1,x=1.,y=2.,radius=.14,phase='TO_TABLE'),
            2:SimpleNamespace(customer_id=2,x=2.,y=2.,radius=.14,phase='SEATED')}
    assert not m.customer_step_safe((1,2),(3,2),people,1)
    assert m.customer_step_safe((1,3),(3,3),people,1)
    people[2].phase='QUEUED'
    assert m.customer_step_safe((1,2),(3,2),people,1)
    assert not m.pedestrian_step_safe((1,2),(3,2),{1:SimpleNamespace(pose=Pose(2,2))})

def test_recovery_bounded_and_keeps_order_identity(world):
    engine,_=make(world)
    r=engine.robots[1]
    assert not engine._attempt_stationary_recovery(r,'No destination')
    oid=engine.create_order('T11','MAIN_COURSE',customer_id=91)
    engine.paused=False
    engine.step()
    assert r.order_id==oid and r.destination is not None
    r.linear=0.;r.angular=0.
    assert engine._attempt_stationary_recovery(r,'Synthetic static blockage')
    assert r.order_id==oid
    assert engine.customer_orders[91]==oid
    assert any(x['event']=='RECOVERY_REQUESTED' and x['order_id']==oid for x in engine.events)
    assert r.recovery_attempts==1
    r.recovery_attempts=2
    assert not engine._attempt_stationary_recovery(r,'Exceeded attempts')

def test_customer_cannot_spawn_at_full_table(world):
    _,flow=make(world)
    for _ in range(4):flow.add(entrance='main_entry',table='T11',station='TEA')
    with pytest.raises(ValueError,match='full'):
        flow.add(entrance='main_entry',table='T11',station='TEA')

def test_pause_emergency_stop_do_not_advance_or_duplicate_orders(world):
    engine,_=make(world)
    oid=engine.create_order('T11','TEA',customer_id=25)
    engine.step()
    assert engine.clock.tick==0
    engine.paused=False;engine.step()
    assert engine.clock.tick==1
    engine.estopped=True
    engine.step()
    assert engine.clock.tick==1
    assert len(engine.orders.all_orders())==1
    assert engine.order_customers[oid]==25

def test_fixed_step_speed_controls_and_invalid_settings():
    from tools.runtime_timing import requested_steps
    assert [requested_steps(x) for x in (1,2,4,8,16)]==[2,4,8,16,32]
    with pytest.raises(ValueError,match='speed'):
        requested_steps(3)
    with pytest.raises(ValueError,match='positive'):
        requested_steps(4,refresh_interval=0)
