import pytest

from aifr.common.errors import ConfigError, InvalidRobotStateError, UnknownRobotError
from aifr.fleet import RobotAvailable, RobotFaulted, RobotStatus, RobotStatusChanged

from helpers import build_core, deliver, make_ready


@pytest.mark.parametrize("n", range(1, 9))
def test_registry_creates_n_robots_numbered_from_one(n):
    core = build_core(n)
    assert len(core.fleet) == n
    assert [r.robot_id for r in core.fleet.all()] == list(range(1, n + 1))
    assert core.fleet.available_ids() == list(range(1, n + 1))
    assert all(r.status is RobotStatus.IDLE and r.active_order_id is None for r in core.fleet.all())


@pytest.mark.parametrize("bad", [0, -2, 65, 2.0, "3", True])
def test_registry_rejects_bad_counts(bad):
    with pytest.raises(ConfigError):
        build_core(bad)


def test_unknown_robot():
    with pytest.raises(UnknownRobotError):
        build_core(2).fleet.get(3)  # type: ignore[arg-type]


def test_assign_makes_robot_busy_and_unavailable():
    core = build_core(3, record=True)
    core.fleet.assign(2, 10)  # type: ignore[arg-type]
    assert core.fleet.get(2).status is RobotStatus.BUSY  # type: ignore[arg-type]
    assert core.fleet.available_ids() == [1, 3]
    assert [(e.old, e.new) for e in core.bus.history if isinstance(e, RobotStatusChanged)] == [
        (RobotStatus.IDLE, RobotStatus.BUSY)
    ]


def test_cannot_assign_busy_or_faulted_robot():
    core = build_core(2)
    core.fleet.assign(1, 1)  # type: ignore[arg-type]
    with pytest.raises(InvalidRobotStateError):
        core.fleet.assign(1, 2)  # type: ignore[arg-type]
    core.fleet.mark_faulted(2)  # type: ignore[arg-type]
    with pytest.raises(InvalidRobotStateError):
        core.fleet.assign(2, 2)  # type: ignore[arg-type]


def test_release_makes_robot_available_immediately_and_announces_it():
    core = build_core(1, record=True)
    core.fleet.assign(1, 5)  # type: ignore[arg-type]
    core.fleet.release(1)  # type: ignore[arg-type]
    assert core.fleet.available_ids() == [1]
    assert sum(isinstance(e, RobotAvailable) for e in core.bus.history) == 1


def test_release_without_order_raises():
    with pytest.raises(InvalidRobotStateError):
        build_core(1).fleet.release(1)  # type: ignore[arg-type]


def test_fault_is_idempotent_and_blocks_availability_until_recovery():
    core = build_core(3, record=True)
    core.fleet.mark_faulted(2)  # type: ignore[arg-type]
    core.fleet.mark_faulted(2)  # type: ignore[arg-type]
    assert sum(isinstance(e, RobotFaulted) for e in core.bus.history) == 1
    assert core.fleet.available_ids() == [1, 3]
    core.fleet.recover(2)  # type: ignore[arg-type]
    assert core.fleet.available_ids() == [1, 2, 3]
    with pytest.raises(InvalidRobotStateError):
        core.fleet.recover(2)  # type: ignore[arg-type]


def test_robot_is_available_right_after_delivery_without_any_return_trip():
    core = build_core(1, record=True)
    order = make_ready(core)
    core.scheduler.dispatch()
    assert core.fleet.available_ids() == []
    deliver(core, order.order_id)
    # Same instant, no extra status or event between DELIVERED and availability.
    assert core.fleet.available_ids() == [1]
    assert core.fleet.get(1).status is RobotStatus.IDLE  # type: ignore[arg-type]
    assert core.clock.now() == 0


def test_registry_available_helpers_and_listing():
    core = build_core(3)
    core.fleet.assign(2, 1)  # type: ignore[arg-type]
    assert core.fleet.is_available(1) and not core.fleet.is_available(2)  # type: ignore[arg-type]
    assert [r.robot_id for r in core.fleet.all()] == [1, 2, 3]


def test_recover_refuses_a_robot_that_still_holds_an_order():
    core = build_core(1)
    core.fleet.assign(1, 5)  # type: ignore[arg-type]
    core.fleet.mark_faulted(1)  # type: ignore[arg-type]
    with pytest.raises(InvalidRobotStateError):
        core.fleet.recover(1)  # type: ignore[arg-type]
