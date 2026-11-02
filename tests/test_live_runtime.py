"""Live event acceptance and API regression tests; no Blender required."""
from __future__ import annotations
import math
from pathlib import Path
import pytest

from aifr.fleet.registry import RobotRegistry
from aifr.common.clock import SimClock
from aifr.common.events import EventBus
from tools.live_pedestrians import LivePedestrians
from tools.stage79_dashboard import Stage79App
from tools.stage79_engine import load_reference

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def reference():
    return load_reference(ROOT)


def test_registration_is_incremental():
    registry=RobotRegistry(EventBus(),SimClock(50000),1)
    assert int(registry.add_robot())==2
    assert [int(robot.robot_id) for robot in registry.all()]==[1,2]
    assert registry.is_available(1) and registry.is_available(2)


def test_add_robot_keeps_existing_base_and_assignments(reference):
    env,grid,_=reference
    from tools.stage79_engine import Stage79Engine
    engine=Stage79Engine(env,grid,1)
    engine.create_order('T11','SANDWICH')
    old_base=engine.robots[1].base
    engine.paused=False
    engine.step()
    assert engine.robots[1].order_id==1
    assert engine.add_robot()==2
    assert engine.robots[1].order_id==1
    assert engine.robots[1].base==old_base
    assert engine.fleet.is_available(2)
    assert math.dist((engine.robots[1].pose.x,engine.robots[1].pose.y),
                     (engine.robots[2].pose.x,engine.robots[2].pose.y))>0.6


def test_live_app_accepts_orders_after_running(reference):
    app=Stage79App(ROOT,1)
    app.control({'action':'start'})
    app.engine.step()
    created=app.order({'table':'T11','station':'SANDWICH'})
    assert created['order_id']==1
    app.engine.step()
    assert app.engine.robots[1].order_id==1


def test_live_app_adds_robot_without_reset(reference):
    app=Stage79App(ROOT,1)
    app.order({'table':'T11','station':'SANDWICH'})
    result=app.control({'action':'add_robot'})
    assert result['robot_id']==2
    assert app.state()['orders'][0]['id']==1
    assert len(app.state()['robots'])==2


def test_live_pedestrians_reject_obstacles(reference):
    _,grid,_=reference
    pedestrians=LivePedestrians(grid)
    with pytest.raises(ValueError):
        pedestrians.add(-100,-100,-100,-100,0)
    with pytest.raises(ValueError):
        pedestrians.add(float('nan'),1,1,1,0)
    with pytest.raises(ValueError):
        pedestrians.add(8,5,12,5,0)


def test_dynamic_pedestrian_moves_without_teleporting(reference):
    _,grid,_=reference
    pedestrians=LivePedestrians(grid)
    # V14 validated pedestrian area (from the original Stage 7 scenario).
    pid=pedestrians.add(8.28,5.95,8.29,5.95,0)
    first=pedestrians(0)[0]
    middle=pedestrians(4)[0]
    final=pedestrians(8)[0]
    assert pid==1
    assert first.x==pytest.approx(8.28)
    assert middle.x==pytest.approx(8.29)
    assert final.x==pytest.approx(8.28)
    assert middle.vx==pytest.approx(0,abs=1e-8)


def test_pedestrians_visible_in_snapshot(reference):
    app=Stage79App(ROOT,1)
    added=app.pedestrian({'x':8.28,'y':5.95,'end_x':8.29,'end_y':5.95})
    state=app.state()
    assert added['pedestrian_id']==1
    assert state['pedestrians'][0]['id']==1


def test_reset_clears_live_agents(reference):
    app=Stage79App(ROOT,1)
    app.pedestrian({'x':8.28,'y':5.95})
    app.control({'action':'reset'})
    assert app.state()['pedestrians']==[]


def test_runtime_rejects_unsafe_robot_spawns(reference):
    app=Stage79App(ROOT,4)
    with pytest.raises(ValueError,match='four'):
        app.control({'action':'add_robot'})
