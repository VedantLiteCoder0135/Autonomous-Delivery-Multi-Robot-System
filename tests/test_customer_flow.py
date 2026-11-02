"""Regression checks for door->CASH->table->automatic order lifecycle."""
from pathlib import Path
import pytest
from tools.stage79_engine import Stage79Engine,load_reference
from tools.customer_flow import CustomerFlow

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def world():
    env,grid,_=load_reference(ROOT)
    return env,grid

def new_flow(world):
    env,grid=world
    engine=Stage79Engine(env,grid,1)
    return engine,CustomerFlow(engine)

def test_four_entrances_reachable(world):
    engine,flow=new_flow(world)
    assert len(flow.entrance_points)==4
    assert all(engine.grid.is_free(*engine.grid.world_to_grid(*p)) for p in flow.entrance_points.values())

def test_cash_is_real_nonfood_counter(world):
    engine,flow=new_flow(world)
    assert any(c.name=='CASH' and not c.is_food_station for c in engine.env.counters)
    assert engine.grid.is_free(*engine.grid.world_to_grid(*flow.cash))

@pytest.mark.parametrize('entrance',['main_entry','side_exit_1','side_exit_2','sliding_entry_t11'])
def test_paid_customer_creates_order_only_after_cash(world,entrance):
    engine,flow=new_flow(world)
    pid=flow.add(entrance=entrance,table='T11',station='SANDWICH')
    customer=flow.customers[pid]
    assert customer.order_id is None
    for _ in range(5000):
        flow.step()
        if customer.phase=='SEATED':break
    assert customer.phase=='SEATED'
    assert customer.order_id is not None
    order=engine.orders.get(customer.order_id)
    assert str(order.table_id)=='T11'
    assert str(order.station_id)=='SANDWICH'
    assert any(event['event']=='BILL_PAID' for event in engine.events)
    assert any(event['event']=='ORDER_READY' for event in engine.events)
    events=[x['event'] for x in engine.events]
    assert events.index('BILL_PAID') < events.index('CUSTOMER_SEATED')

def test_four_at_one_table_and_fifth_rejected(world):
    _,flow=new_flow(world)
    for _ in range(4):
        flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    assert len(flow.customers)==4
    assert len({c.seat for c in flow.customers.values()})==4
    assert sum(c.phase=='QUEUED' for c in flow.customers.values())==3
    with pytest.raises(ValueError, match='full'):
        flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    with pytest.raises(ValueError, match='four verified'):
        flow.add(entrance='fictional',table='T10',station='SANDWICH')


def test_same_entrance_admits_queued_people_in_order(world):
    engine,flow=new_flow(world)
    ids=[flow.add(entrance='main_entry',table='T11',station='SANDWICH') for _ in range(4)]
    assert flow.customers[ids[0]].phase=='TO_CASH'
    for _ in range(1200):
        flow.step()
        if all(flow.customers[i].phase!='QUEUED' for i in ids):break
    assert all(flow.customers[i].phase!='QUEUED' for i in ids)
    entries=[e['customer_id'] for e in engine.events if e['event']=='CUSTOMER_ENTERED']
    assert entries==ids


def test_multiple_customers_create_independent_orders(world):
    engine,flow=new_flow(world)
    ids=[flow.add(entrance='main_entry',table='T11',station='SANDWICH') for _ in range(4)]
    for _ in range(5000):
        flow.step()
        if all(flow.customers[i].phase=='SEATED' for i in ids):break
    assert all(flow.customers[i].phase=='SEATED' for i in ids)
    orders=[flow.customers[i].order_id for i in ids]
    assert len(set(orders))==4
    assert len(engine.orders.all_orders())==4

