"""Traffic arbitration and pedestrian/robot conflict tests."""
from pathlib import Path
from types import SimpleNamespace
from aifr.navigation.kinematics import Pose
from aifr.navigation.occupancy_grid import OccupancyGrid
from tools.deadlock_manager import DeadlockManager
from tools.stage79_engine import Stage79Engine, load_reference
from tools.customer_flow import CustomerFlow

ROOT = Path(__file__).resolve().parents[1]


def fake_robot(x, y, phase='TO_PICKUP'):
    return SimpleNamespace(pose=Pose(x,y), phase=phase)


def open_grid():
    return OccupancyGrid(0,0,.1,80,80,tuple((0,)*80 for _ in range(80)))


def test_traffic_admission_refuses_any_second_active_robot():
    manager=DeadlockManager()
    robots={1:fake_robot(1,1,'TO_PICKUP'),2:fake_robot(4,4,'WAITING_FOR_AISLE')}
    assert not manager.can_admit_robot(robots)
    robots[1].phase='IDLE'
    assert manager.can_admit_robot(robots)


def test_approaching_customer_cannot_step_through_robot():
    manager=DeadlockManager()
    robots={1:fake_robot(2,2)}
    assert not manager.pedestrian_step_safe((1.0,2.0),(3.0,2.0),robots)
    assert manager.pedestrian_step_safe((1.0,3.0),(3.0,3.0),robots)


def test_customer_can_reroute_around_stationary_robot():
    manager=DeadlockManager()
    grid=open_grid()
    robots={1:fake_robot(2,2)}
    route=manager.reroute_around_robots(grid,(1.05,2.05),(3.05,2.05),robots)
    assert route and len(route)>2
    assert all(manager.clearance(x,y,robots) > -.09 for x,y in route)


def test_customer_keeps_position_when_robot_blocks_path():
    env,grid,_=load_reference(ROOT)
    engine=Stage79Engine(env,grid,robot_count=1,traffic_mode='exclusive')
    flow=CustomerFlow(engine)
    pid=flow.add(entrance='main_entry',table='T11',station='SANDWICH')
    customer=flow.customers[pid]
    # Place robot on the pedestrian's immediate route, but not on the customer.
    target=customer.route[1]
    engine.robots[1].pose=Pose(*target)
    before=(customer.x,customer.y)
    flow.step(.05)
    assert (customer.x,customer.y)==before
    assert customer.blocked_ticks==1
    assert engine.deadlock_manager.customer_yields==1


def test_blender_policy_serializes_traffic_without_losing_assigned_orders():
    env,grid,_=load_reference(ROOT)
    engine=Stage79Engine(env,grid,robot_count=2,traffic_mode='exclusive')
    engine.create_order('T11','SANDWICH')
    engine.create_order('T12','JUICE')
    engine.paused=False
    engine.step()
    phases=[r.phase for r in engine.robots.values()]
    assert phases.count('TO_PICKUP')==1
    assert phases.count('WAITING_FOR_AISLE')==1
    assert len([r for r in engine.robots.values() if r.order_id])==2


def test_invalid_mode_rejected():
    env,grid,_=load_reference(ROOT)
    import pytest
    with pytest.raises(ValueError,match='traffic_mode'):
        Stage79Engine(env,grid,robot_count=1,traffic_mode='magic')
