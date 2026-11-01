import itertools

import pytest

from aifr.common.errors import InvalidTransitionError, OrderError, UnknownOrderError
from aifr.common.types import OrderId
from aifr.orders import ALLOWED_TRANSITIONS, FLOW, TERMINAL_STATUSES, OrderStatus
from aifr.orders.events import OrderCreated, OrderStatusChanged

from helpers import build_orders as build_core

S = OrderStatus


def test_exact_lifecycle_order():
    assert [s.value for s in FLOW] == [
        "CREATED", "PREPARING", "READY", "ASSIGNED", "TO_PICKUP",
        "LOADING", "TO_CUSTOMER", "UNLOADING", "DELIVERED",
    ]


def test_create_order_defaults_and_ids():
    core = build_core(record=True)
    a = core.orders.create_order("T03", "TEA")
    b = core.orders.create_order("T04", "JUICE")
    assert (a.order_id, b.order_id) == (1, 2)
    assert a.status is S.CREATED and a.ready_at is None and a.ready_seq is None
    assert [type(e) for e in core.bus.history] == [OrderCreated, OrderCreated]


@pytest.mark.parametrize("table,station", [("", "TEA"), ("  ", "TEA"), ("T03", ""), ("T03", " ")])
def test_create_order_validates_input(table, station):
    with pytest.raises(OrderError):
        build_core().orders.create_order(table, station)


def test_full_happy_path_and_history():
    core = build_core()
    o = core.orders.create_order("T03", "TEA")
    core.orders.start_preparing(o.order_id)
    core.clock.advance(2)
    core.orders.mark_ready(o.order_id)
    core.orders.mark_assigned(o.order_id, 1)  # type: ignore[arg-type]
    for status in (S.TO_PICKUP, S.LOADING, S.TO_CUSTOMER, S.UNLOADING, S.DELIVERED):
        core.orders.advance(o.order_id, status)
    assert [s for _, s in o.history] == list(FLOW)
    assert o.ready_at == 100_000 and o.ready_seq == 1 and o.robot_id == 1
    assert o.is_terminal and o.closed_at is not None


def test_ready_sequence_numbers_increase_in_mark_order():
    core = build_core()
    ids = [core.orders.create_order("T03", "TEA").order_id for _ in range(3)]
    for i in ids:
        core.orders.start_preparing(i)
    for i in (ids[2], ids[0], ids[1]):
        core.orders.mark_ready(i)
    assert [core.orders.get(i).ready_seq for i in ids] == [2, 3, 1]


def test_failed_ready_does_not_consume_a_sequence_number():
    core = build_core()
    a = core.orders.create_order("T03", "TEA")
    with pytest.raises(InvalidTransitionError):
        core.orders.mark_ready(a.order_id)           # still CREATED
    core.orders.start_preparing(a.order_id)
    assert core.orders.mark_ready(a.order_id).ready_seq == 1


@pytest.mark.parametrize(
    "old,new",
    [(o, n) for o, n in itertools.product(S, S) if n not in ALLOWED_TRANSITIONS[o]],
)
def test_every_disallowed_transition_is_rejected(old, new):
    """Exhaustive: drive an order into ``old`` by the legal path, then try ``new``."""
    core = build_core()
    o = core.orders.create_order("T03", "TEA")
    oid = o.order_id
    if old in FLOW:
        for status in FLOW[1 : FLOW.index(old) + 1]:
            if status is S.READY:
                core.orders.mark_ready(oid)
            elif status is S.ASSIGNED:
                core.orders.mark_assigned(oid, 1)  # type: ignore[arg-type]
            else:
                core.orders.advance(oid, status)
    elif old is S.FAILED:
        core.orders.fail(oid)
    else:
        core.orders.cancel(oid)
    assert o.status is old
    with pytest.raises(InvalidTransitionError):
        if new is S.READY:
            core.orders.mark_ready(oid)
        elif new is S.ASSIGNED:
            core.orders.mark_assigned(oid, 1)  # type: ignore[arg-type]
        elif new is S.FAILED:
            core.orders.fail(oid)
        elif new is S.CANCELLED:
            core.orders.cancel(oid)
        else:
            core.orders.advance(oid, new)
    assert o.status is old                         # unchanged after a rejected transition


@pytest.mark.parametrize("status", [S.CREATED, S.READY, S.ASSIGNED, S.FAILED, S.CANCELLED])
def test_advance_refuses_statuses_with_dedicated_methods(status):
    core = build_core()
    o = core.orders.create_order("T03", "TEA")
    with pytest.raises(InvalidTransitionError):
        core.orders.advance(o.order_id, status)


def test_cancel_allowed_only_before_loading_and_fail_from_any_non_terminal():
    cancellable = {S.CREATED, S.PREPARING, S.READY, S.ASSIGNED, S.TO_PICKUP}
    for s in FLOW[:-1]:
        assert (S.CANCELLED in ALLOWED_TRANSITIONS[s]) == (s in cancellable)
        assert S.FAILED in ALLOWED_TRANSITIONS[s]
    for t in TERMINAL_STATUSES:
        assert ALLOWED_TRANSITIONS[t] == frozenset()


def test_cancel_and_fail_record_reason_and_time():
    core = build_core()
    a = core.orders.create_order("T03", "TEA")
    b = core.orders.create_order("T04", "TEA")
    core.clock.advance()
    core.orders.cancel(a.order_id, "customer left")
    core.orders.fail(b.order_id, "kitchen error")
    assert (a.status, a.close_reason, a.closed_at) == (S.CANCELLED, "customer left", 50_000)
    assert (b.status, b.close_reason) == (S.FAILED, "kitchen error")


def test_status_change_events_carry_old_new_and_robot():
    core = build_core(record=True)
    o = core.orders.create_order("T03", "TEA")
    core.orders.start_preparing(o.order_id)
    changes = [e for e in core.bus.history if isinstance(e, OrderStatusChanged)]
    assert [(e.old, e.new) for e in changes] == [(S.CREATED, S.PREPARING)]


def test_unknown_order():
    with pytest.raises(UnknownOrderError):
        build_core().orders.get(OrderId(99))


def test_manager_len_and_listing_helpers():
    core = build_core()
    a = core.orders.create_order("T03", "TEA")
    b = core.orders.create_order("T04", "TEA")
    core.orders.start_preparing(b.order_id)
    assert len(core.orders) == 2
    assert [o.order_id for o in core.orders.all_orders()] == [1, 2]
    assert core.orders.by_status(S.PREPARING) == [b]
    assert core.orders.by_status(S.CREATED, S.PREPARING) == [a, b]
