from __future__ import annotations

import math

import pytest

from aifr.battery import (BatteryConfig, BatteryManager, ChargingManager,
                          ChargingStation, audit_base_chargers)
from aifr.battery.estimation import estimate_leg, path_metrics
from aifr.navigation.kinematics import Pose
from aifr.navigation.occupancy_grid import OccupancyGrid


def config(**changes):
    values = dict(nominal_capacity_wh=100.0, initial_soc=1.0,
                  movement_wh_per_m=1.0, rotation_wh_per_rad=0.5,
                  idle_power_w=3600.0, cycle_fade_per_efc=0.01,
                  calendar_fade_per_day=0.02)
    values.update(changes)
    return BatteryConfig(**values)


def test_motion_energy_uses_observed_distance_rotation_idle_and_cycle_fade():
    manager = BatteryManager([1], config())
    used = manager.update_motion(1, 2.0, math.pi, 1.0, 1.0)
    expected = 2.0 + math.pi * 0.5
    state = manager.get(1)
    assert used == pytest.approx(expected)
    assert state.cumulative_discharged_wh == pytest.approx(expected)
    assert state.equivalent_full_cycles == pytest.approx(expected / 100.0)
    assert state.soh == pytest.approx(1.0 - 0.01 * expected / 100.0)
    assert state.remaining_wh == pytest.approx(100.0 - expected)


def test_stationary_idle_energy_is_not_added_while_robot_is_moving_or_turning():
    manager = BatteryManager([1], config())
    manager.update_motion(1, 0.0, 0.0, 1.0, 1.0)
    manager.update_motion(1, 1.0, 0.0, 1.0, 2.0)
    manager.update_motion(1, 0.0, 1.0, 1.0, 3.0)
    assert manager.get(1).cumulative_discharged_wh == pytest.approx(2.5)


def test_calendar_fade_clamps_energy_to_new_usable_capacity():
    manager = BatteryManager([1], config(initial_soc=1.0, calendar_fade_per_day=0.1,
                                         cycle_fade_per_efc=0.0, idle_power_w=0.0))
    manager.update_motion(1, 0.0, 0.0, 0.0, 86400.0)
    state = manager.get(1)
    assert state.soh == pytest.approx(0.9)
    assert state.usable_capacity_wh == pytest.approx(90.0)
    assert state.remaining_wh == pytest.approx(90.0)


def test_mission_feasibility_uses_remaining_energy_and_usable_reserve():
    manager = BatteryManager([1], config(initial_soc=0.5, reserve_fraction=0.15,
                                         protected_arrival_fraction=0.05))
    assert manager.can_complete(1, 30.0)
    assert not manager.can_complete(1, 36.0)
    assert manager.return_soc_threshold(1, 10.0) == pytest.approx(0.15)


def test_energy_estimator_and_runtime_accounting_agree_for_controlled_astar_path():
    grid = OccupancyGrid(0, 0, 1, 4, 4,
                         tuple(tuple(0 for _ in range(4)) for _ in range(4)))
    leg = estimate_leg(grid, (0.5, 0.5), (3.5, 2.5))
    assert leg is not None
    distance, rotation = path_metrics(leg[3])
    cfg = config(idle_power_w=0.0, cycle_fade_per_efc=0.0)
    estimated = cfg.movement_wh_per_m * distance + cfg.rotation_wh_per_rad * rotation
    manager = BatteryManager([1], cfg)
    actual = manager.update_motion(1, distance, rotation, 0.0, 0.0)
    assert actual == pytest.approx(estimated)


def test_charge_requires_docking_stationarity_and_clamps_to_soh_capacity():
    manager = BatteryManager([1], config(initial_soc=0.5, charging_power_w=100.0,
                                         charging_efficiency=0.8))
    assert manager.charge(1, 3600, docked=False, stationary=True) == 0
    assert manager.charge(1, 3600, docked=True, stationary=False) == 0
    assert manager.charge(1, 3600, docked=True, stationary=True) == pytest.approx(50.0)
    assert manager.get(1).remaining_wh == pytest.approx(100.0)


def test_charger_ownership_is_exclusive_and_released_only_after_undock():
    station = ChargingStation("A", Pose(0.0, 0.0, 0.0), capacity=1)
    chargers = ChargingManager([station])
    assert chargers.request(1, "A") == "DOCKING"
    assert chargers.request(2, "A") == "WAITING_FOR_CHARGER"
    with pytest.raises(ValueError):
        chargers.undock(1)
    chargers.mark_docked(1)
    chargers.mark_complete(1)
    assert chargers.owns(1)
    chargers.undock(1)
    assert not chargers.owns(1)
    assert chargers.owns(2)
    assert chargers.phase[2] == "DOCKING"


def test_failed_charger_stays_reserved_until_safe_evacuation():
    chargers = ChargingManager([ChargingStation("A", Pose(0, 0, 0))])
    assert chargers.request(1, "A") == "DOCKING"
    chargers.mark_docked(1)
    chargers.fail_station("A")
    assert chargers.owns(1)
    assert chargers.request(2, "A") == "WAITING_FOR_CHARGER"
    with pytest.raises(ValueError):
        chargers.undock_failed(1, safely_undocked=False)
    chargers.undock_failed(1, safely_undocked=True)
    assert not chargers.owns(1)
    assert not chargers.owns(2)


def test_reference_base_slots_are_audited_without_inventing_geometry():
    from tools.stage79_engine import Stage79Engine, load_reference
    from pathlib import Path

    project = Path(__file__).resolve().parents[1]
    env, grid, _ = load_reference(project)
    engine = Stage79Engine(env, grid, robot_count=4)
    stations, report = audit_base_chargers(
        env, grid, engine.checker, [robot.base for robot in engine.robots.values()])
    assert stations
    assert report["candidate_source"] == "existing validated robot base slots"
    assert report["pedestrian_routes_available"] is False
    assert len(stations) == report["maximum_simultaneous_robots"]


def test_engine_intelligent_dispatch_keeps_return_gate_and_records_costs():
    from pathlib import Path
    from tools.stage79_engine import Stage79Engine, load_reference

    project = Path(__file__).resolve().parents[1]
    env, grid, _ = load_reference(project)
    engine = Stage79Engine(env, grid, robot_count=2, battery_enabled=True,
                           intelligent_allocation_enabled=True,
                           charging_enabled=True)
    engine.create_order("T11", "SANDWICH")
    engine._dispatch()
    assert engine.robots[1].phase in ("TO_PICKUP", "WAITING_FOR_AISLE")
    history = [status.value for _, status in engine.orders.all_orders()[0].history]
    assert "ASSIGNED" in history and history[-1] == "TO_PICKUP"
    assert engine.scheduler.is_return_pending(1) is False
    assert engine.allocation_log and any(item["selected"] for item in engine.allocation_log)
    assert engine.charging_enabled

