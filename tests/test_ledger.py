import pytest

from aifr.common.errors import SchedulingInvariantError
from aifr.scheduling import AssignmentLedger


def test_assign_and_lookup():
    ledger = AssignmentLedger()
    a = ledger.assign(1, 2, at=50)
    assert (a.order_id, a.robot_id, a.at) == (1, 2, 50)
    assert ledger.robot_of(1) == 2 and ledger.order_of(2) == 1
    assert ledger.has_order(1) and ledger.is_robot_busy(2)
    assert ledger.robot_of(9) is None and ledger.order_of(9) is None
    assert len(ledger) == 1 and ledger.active() == [a]
    ledger.check_consistency()


def test_same_order_cannot_be_assigned_twice_while_active():
    ledger = AssignmentLedger()
    ledger.assign(1, 1, 0)
    with pytest.raises(SchedulingInvariantError):
        ledger.assign(1, 2, 0)
    assert len(ledger) == 1 and not ledger.is_robot_busy(2)


def test_robot_cannot_hold_two_orders():
    ledger = AssignmentLedger()
    ledger.assign(1, 1, 0)
    with pytest.raises(SchedulingInvariantError):
        ledger.assign(2, 1, 0)
    assert not ledger.has_order(2)


def test_order_can_never_be_assigned_again_even_after_release():
    ledger = AssignmentLedger()
    ledger.assign(1, 1, 0)
    ledger.release(1)
    with pytest.raises(SchedulingInvariantError):
        ledger.assign(1, 2, 10)


def test_release_frees_robot_for_new_order():
    ledger = AssignmentLedger()
    ledger.assign(1, 1, 0)
    released = ledger.release(1)
    assert released.robot_id == 1 and len(ledger) == 0
    ledger.assign(2, 1, 5)
    assert ledger.order_of(1) == 2
    ledger.check_consistency()


def test_release_unknown_order_raises():
    with pytest.raises(SchedulingInvariantError):
        AssignmentLedger().release(5)


def test_history_is_append_only_audit_trail():
    ledger = AssignmentLedger()
    ledger.assign(1, 1, 0)
    ledger.assign(2, 2, 0)
    ledger.release(1)
    assert [(a.order_id, a.robot_id) for a in ledger.history] == [(1, 1), (2, 2)]


def test_check_consistency_detects_corruption():
    ledger = AssignmentLedger()
    ledger.assign(1, 1, 0)
    ledger.assign(2, 2, 0)
    ledger._by_robot.pop(2)                      # simulate index corruption
    with pytest.raises(SchedulingInvariantError):
        ledger.check_consistency()
    ledger._by_robot[2] = ledger._by_order[1]    # same size, wrong content
    with pytest.raises(SchedulingInvariantError):
        ledger.check_consistency()
