from dataclasses import replace
import math
from pathlib import Path

import pytest

from aifr.battery import BatteryConfig
from aifr.navigation.kinematics import Pose
from aifr.navigation.astar import find_path
from aifr.orders.model import OrderStatus
from aifr.perception.ekf import PoseEKF
from tools.stage79_engine import Stage79Engine, load_reference


@pytest.fixture
def engine():
    env, grid, _ = load_reference(Path(__file__).resolve().parents[1])
    # Low initial SOC uses the live thresholds. High test-only charge power
    # keeps the complete physical cycle short without changing timestep.
    config = replace(BatteryConfig(), dispatch_soc_fraction=.15, charging_power_w=54000)
    runtime = Stage79Engine(env, grid, robot_count=1, traffic_mode='coordinated',
                            battery_enabled=True, charging_enabled=True, battery_config=config)
    runtime.paused = False
    return runtime


def test_battery_override_and_consumption_share_manager_and_feasibility(engine):
    manager = engine.batteries
    state = manager.get(1)
    state.soh = .9
    manager.set_soc_percent(1, 15)
    assert state.remaining_wh == pytest.approx(16.2)
    assert state.soc_percent == pytest.approx(15)
    for invalid in (-1, 101, math.nan):
        with pytest.raises(ValueError):
            manager.set_soc_percent(1, invalid)
    order = engine.create_order('T11', 'SANDWICH')
    engine._dispatch()
    assert engine.orders.get(order).status is OrderStatus.READY
    manager.set_soc_percent(1, 100)
    original = engine.battery_config
    cost = engine._pair_metrics([1], [order])[(1, order)]['mission_wh']
    engine.set_battery_consumption_multiplier(2)
    assert manager.config is engine.battery_config
    assert engine._pair_metrics([1], [order])[(1, order)]['mission_wh'] == pytest.approx(cost * 2)
    engine.step()
    assert state.remaining_wh < state.usable_capacity_wh
    assert state.soh <= .9
    remaining = state.remaining_wh
    engine.set_battery_consumption_multiplier(1)
    assert manager.config == original
    assert state.remaining_wh == remaining


def test_physical_return_charge_and_release_on_departure(engine):
    robot = engine.robots[1]
    base = robot.base.x, robot.base.y
    # Start an idle robot off its pad in a reachable, footprint-clear cell.
    # Every subsequent movement uses the unmodified engine navigation loop.
    grid = engine.grid
    start = next(
        (x, y) for row in range(grid.height) for col in range(grid.width)
        if grid.is_free(col, row)
        for x, y in [grid.grid_to_world(col, row)]
        if .75 < math.dist(base, (x, y)) < 1.0
        and engine.checker.is_pose_free(x, y, robot.base.theta)
        and find_path(grid, (x, y), base)
    )
    robot.pose = Pose(*start, robot.base.theta)
    robot.ekf = PoseEKF(robot.pose)
    engine.batteries.set_soc_percent(1, 14)
    state = engine.batteries.get(1)
    engine.step()
    assert robot.phase == 'RETURNING_TO_CHARGE'
    pad = engine.charging.robot_station[1]
    assert engine.charging.owners[pad] == {1}
    assert engine.routes[-1]['purpose'] == 'CHARGE_RETURN'
    assert state.charged_wh == 0

    for _ in range(500):
        engine.step()
        if robot.phase == 'CHARGING':
            break
        assert state.charged_wh == 0
    assert robot.phase == 'CHARGING', (robot.blocked_reason, engine.failures)
    assert math.dist(base, (robot.pose.x, robot.pose.y)) <= .12
    assert robot.nav_ticks > 0
    docked_pose = robot.pose
    before = state.soc_percent
    engine.step()
    assert robot.pose == docked_pose
    assert state.soc_percent > before
    for _ in range(180):
        engine.step()
        if robot.phase == 'IDLE':
            break
    assert robot.phase == 'IDLE'
    assert engine.snapshot()['robots'][0]['fleet_available']
    assert engine.charging.owners[pad] == {1}

    # Reapplying a low level on the still occupied dock runs the same logic.
    engine.batteries.set_soc_percent(1, 14)
    engine.step()
    assert robot.phase == 'CHARGING'
    engine.batteries.set_soc_percent(1, 90)
    engine.step()
    assert robot.phase == 'IDLE'

    order = engine.create_order('T11', 'SANDWICH')
    for _ in range(500):
        engine.step()
        if not engine.charging.owns(1):
            break
    assert int(engine.orders.get(order).robot_id) == 1
    assert not engine.charging.owns(1)
    assert any(event['event'] == 'CHARGER_UNDOCKED' for event in engine.events)
    assert not engine.failures


def test_queued_robot_inherits_reserved_pad_only_after_physical_departure(engine):
    engine.add_robot()
    first, second = [engine._station_for_robot[rid] for rid in (1, 2)]
    engine.charging.fail_station(second)
    for rid in (1, 2):
        engine.batteries.set_soc_percent(rid, 14)
    engine.step()
    assert engine.robots[1].phase == 'CHARGING'
    assert engine.robots[2].phase == 'WAITING_FOR_CHARGER'
    assert list(engine.charging.queue) == [2]
    assert engine.charging.owners[first] == {1}
    assert engine.batteries.get(2).charged_wh == 0

    engine.batteries.set_soc_percent(1, 90)
    engine.step()
    assert engine.robots[1].phase == 'IDLE'
    assert list(engine.charging.queue) == [2]
    assert engine.charging.owners[first] == {1}

    engine.create_order('T11', 'SANDWICH')
    for _ in range(500):
        engine.step()
        if engine.robots[2].phase == 'RETURNING_TO_CHARGE':
            break
    assert not engine.charging.owns(1)
    assert engine.charging.owners[first] == {2}
    assert not engine.charging.queue
    assert engine.robots[2].phase == 'RETURNING_TO_CHARGE'
    assert engine.batteries.get(2).charged_wh == 0

    robot = engine.robots[2]
    for _ in range(700):
        engine.step()
        if robot.phase == 'CHARGING':
            break
    assert robot.phase == 'CHARGING', (robot.blocked_reason, engine.failures)
    engine.batteries.set_soc_percent(2, 90)
    engine.step()
    assert robot.phase == 'RETURNING'
    assert not engine.snapshot()['robots'][1]['fleet_available']
    for _ in range(700):
        engine.step()
        if robot.phase == 'IDLE':
            break
    assert robot.phase == 'IDLE', (robot.blocked_reason, engine.failures)
    assert math.dist((robot.pose.x, robot.pose.y), (robot.base.x, robot.base.y)) <= .12
    assert not engine.charging.owns(2)
    assert engine.snapshot()['robots'][1]['fleet_available']
    assert not engine.failures


def test_low_override_during_infeasible_delivery_returns_safely(engine):
    order = engine.create_order('T11', 'SANDWICH')
    engine.step()
    robot = engine.robots[1]
    assert robot.phase == 'TO_PICKUP'
    engine.batteries.set_soc_percent(1, 14)
    for _ in range(200):
        engine.step()
        if robot.phase == 'CHARGING':
            break
    assert engine.orders.get(order).status is OrderStatus.FAILED
    assert any(event['event'] == 'BATTERY_SAFE_RETURN' for event in engine.events)
    assert robot.phase == 'CHARGING'
    assert math.dist((robot.pose.x, robot.pose.y), (robot.base.x, robot.base.y)) <= .12
    assert not engine.failures


def test_blocked_dock_keeps_reservation_and_retries_when_clear(engine):
    robot = engine.robots[1]
    engine.batteries.set_soc_percent(1, 0)
    # Outside the collision guard but inside the dock's safety clearance.
    engine.add_dynamic_obstacle(robot.base.x + .51, robot.base.y, radius=.08)
    for _ in range(3):
        engine.step()
    pad = engine.charging.robot_station[1]
    assert engine.charging.owners[pad] == {1}
    assert robot.phase == 'WAITING_FOR_CHARGER'
    assert engine.batteries.get(1).charged_wh == 0
    assert sum(event['event'] == 'DOCKING_BLOCKED' for event in engine.events) == 1

    engine.clear_dynamic_obstacles()
    engine.step()
    assert robot.phase == 'CHARGING'
    assert robot.pose == robot.base
    assert engine.batteries.get(1).soc_percent > 0
    assert not engine.failures


@pytest.mark.parametrize('percent', [15, 25])
def test_waiting_order_does_not_start_charging_at_or_above_cutoff(engine, percent):
    engine.batteries.set_soc_percent(1, percent)
    order = engine.create_order('T11', 'SANDWICH')
    engine.step()
    assert engine.orders.get(order).status is OrderStatus.READY
    assert engine.robots[1].phase == 'IDLE'
    assert not engine.charging.owns(1)

    engine.batteries.set_soc_percent(1, 56.9)
    engine.step()
    assert engine.orders.get(order).status is OrderStatus.TO_PICKUP


def test_feasible_order_interrupts_charging_below_full_target(engine):
    engine.batteries.set_soc_percent(1, 14)
    engine.step()
    robot = engine.robots[1]
    assert robot.phase == 'CHARGING'
    order = engine.create_order('T11', 'SANDWICH')
    engine.step()
    assert engine.orders.get(order).status is OrderStatus.READY

    engine.batteries.set_soc_percent(1, 56.9)
    pad = engine.charging.robot_station[1]
    charged = engine.batteries.get(1).charged_wh
    engine.step()
    assert engine.orders.get(order).status is OrderStatus.TO_PICKUP
    assert engine.charging.owners[pad] == {1}
    assert engine.batteries.get(1).charged_wh == charged
    assert any(event['event'] == 'CHARGING_INTERRUPTED_FOR_ORDER'
               for event in engine.events)
