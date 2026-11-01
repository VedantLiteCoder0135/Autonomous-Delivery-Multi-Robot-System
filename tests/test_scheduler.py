"""Behavioural tests for the global FCFS scheduler."""

import pytest

from aifr.orders import OrderStatus as S
from aifr.fleet import RobotAvailable, RobotStatus

from helpers import advance_to, build_core, deliver, make_ready, pairs


# ------------------------------------------------------------------ core policy
def test_oldest_ready_order_goes_to_lowest_numbered_robot():
    core = build_core(3)
    a, b, c = (make_ready(core) for _ in range(3))
    assert pairs(core.scheduler.dispatch()) == [
        (a.order_id, 1), (b.order_id, 2), (c.order_id, 3)
    ]


def test_dispatch_with_nothing_to_do_is_a_noop():
    core = build_core(3)
    assert core.scheduler.dispatch() == []
    assert len(core.scheduler.ledger) == 0 and len(core.scheduler.queue) == 0


def test_orders_ordered_by_ready_time_not_creation_order():
    core = build_core(2)
    first = core.orders.create_order("T03", "TEA")
    second = core.orders.create_order("T04", "TEA")
    core.orders.start_preparing(first.order_id)
    core.orders.start_preparing(second.order_id)
    core.orders.mark_ready(second.order_id)       # created later, ready earlier
    core.clock.advance()
    core.orders.mark_ready(first.order_id)
    assert pairs(core.scheduler.dispatch()) == [(second.order_id, 1), (first.order_id, 2)]


def test_ready_timestamp_tie_resolved_by_sequence_number():
    core = build_core(3)
    orders = [core.orders.create_order("T03", "TEA") for _ in range(3)]
    for o in orders:
        core.orders.start_preparing(o.order_id)
    # all three become READY at the same simulation time, in the order 3, 1, 2
    for idx in (2, 0, 1):
        core.orders.mark_ready(orders[idx].order_id)
    assert {core.orders.get(o.order_id).ready_at for o in orders} == {0}
    assert pairs(core.scheduler.dispatch()) == [
        (orders[2].order_id, 1), (orders[0].order_id, 2), (orders[1].order_id, 3)
    ]


def test_excess_orders_wait_in_fcfs_order():
    core = build_core(2)
    ids = [make_ready(core).order_id for _ in range(5)]
    assert pairs(core.scheduler.dispatch()) == [(ids[0], 1), (ids[1], 2)]
    assert core.scheduler.queue.order_ids() == ids[2:]


# ----------------------------------------------------- simultaneous availability
def test_simultaneous_availability_is_processed_as_a_batch_sorted_by_robot_id():
    core = build_core(3)
    first = [make_ready(core) for _ in range(3)]
    core.scheduler.dispatch()                      # robots 1,2,3 all busy
    waiting = [make_ready(core) for _ in range(3)]  # A, B, C queue up (oldest first)

    # Robots free up in the order 3, 1, 2 within the same tick, with no dispatch in between.
    deliver(core, first[2].order_id)
    deliver(core, first[0].order_id)
    deliver(core, first[1].order_id)
    assert core.fleet.available_ids() == [1, 2, 3]

    got = pairs(core.scheduler.dispatch())
    # Arrival order of availability must not matter: oldest order -> robot 1, etc.
    assert got == [(waiting[0].order_id, 1), (waiting[1].order_id, 2), (waiting[2].order_id, 3)]


def test_partial_batch_gives_oldest_order_to_lowest_free_robot():
    core = build_core(4)
    first = [make_ready(core) for _ in range(4)]
    core.scheduler.dispatch()
    waiting = [make_ready(core) for _ in range(2)]
    deliver(core, first[3].order_id)               # robot 4 frees first
    deliver(core, first[1].order_id)               # robot 2 frees second
    assert pairs(core.scheduler.dispatch()) == [
        (waiting[0].order_id, 2), (waiting[1].order_id, 4)
    ]


def test_scheduler_never_dispatches_on_its_own_from_events():
    core = build_core(2, record=True)
    a = make_ready(core)
    assert core.scheduler.ledger.history == ()      # READY alone does not assign
    core.scheduler.dispatch()
    deliver(core, a.order_id)
    b = make_ready(core)
    assert len(core.scheduler.ledger.history) == 1  # RobotAvailable / READY did not assign
    assert pairs(core.scheduler.dispatch()) == [(b.order_id, 1)]


def test_dispatch_events_are_published_only_after_the_whole_batch_is_committed():
    core = build_core(3)
    snapshots = []

    def observe(event):  # type: ignore[no-untyped-def]
        snapshots.append(len(core.scheduler.ledger))

    from aifr.orders.events import OrderStatusChanged
    core.bus.subscribe(OrderStatusChanged, observe)
    for _ in range(3):
        make_ready(core)
    snapshots.clear()
    core.scheduler.dispatch()
    assigned_events = snapshots[-3:]
    assert assigned_events == [3, 3, 3]            # every subscriber saw the finished batch


# ------------------------------------------------------ immediate (re)assignment
def test_robot_gets_waiting_order_in_the_same_tick_it_becomes_available():
    core = build_core(1)
    a, b = make_ready(core), make_ready(core)
    core.scheduler.dispatch()
    tick_before = core.clock.tick
    deliver(core, a.order_id)
    assert pairs(core.scheduler.dispatch()) == [(b.order_id, 1)]
    assert core.clock.tick == tick_before          # no time had to pass


def test_no_return_to_base_needed_between_orders():
    core = build_core(1, record=True)
    a, b = make_ready(core), make_ready(core)
    core.scheduler.dispatch()
    deliver(core, a.order_id)
    core.scheduler.dispatch()
    statuses = [s for _, s in core.orders.get(b.order_id).history]
    assert statuses[-1] is S.ASSIGNED
    assert core.fleet.get(1).active_order_id == b.order_id  # type: ignore[arg-type]
    assert sum(isinstance(e, RobotAvailable) for e in core.bus.history) == 1


# ---------------------------------------------- no duplicates / no multi-orders
def test_no_robot_ever_holds_two_active_orders():
    core = build_core(1)
    ids = [make_ready(core).order_id for _ in range(3)]
    assert len(core.scheduler.dispatch()) == 1
    assert core.scheduler.dispatch() == []
    assert len(core.scheduler.ledger) == 1
    assert core.scheduler.queue.order_ids() == ids[1:]


def test_repeated_dispatch_never_duplicates_an_assignment():
    core = build_core(3)
    for _ in range(2):
        make_ready(core)
    first = core.scheduler.dispatch()
    again = core.scheduler.dispatch()
    assert len(first) == 2 and again == []
    assert len(core.scheduler.ledger.history) == 2


def test_each_order_is_assigned_exactly_once_over_a_long_run():
    core = build_core(3)
    for _ in range(30):
        make_ready(core)
    seen: list[int] = []
    for _ in range(40):
        for a in core.scheduler.dispatch():
            seen.append(int(a.order_id))
            deliver(core, a.order_id)
        core.clock.advance()
    assert sorted(seen) == list(range(1, 31))
    assert len(set(seen)) == len(seen)
    assert len(core.orders.by_status(S.DELIVERED)) == 30


# ------------------------------------------------- cancellation / failure paths
def test_cancelling_a_queued_order_removes_it_from_the_queue():
    core = build_core(1)
    a, b, c = (make_ready(core) for _ in range(3))
    core.scheduler.dispatch()
    core.orders.cancel(b.order_id, "customer left")
    assert core.scheduler.queue.order_ids() == [c.order_id]
    deliver(core, a.order_id)
    assert pairs(core.scheduler.dispatch()) == [(c.order_id, 1)]


def test_cancelling_an_assigned_order_frees_the_robot():
    core = build_core(1)
    a, b = make_ready(core), make_ready(core)
    core.scheduler.dispatch()
    advance_to(core, a.order_id, S.TO_PICKUP)
    core.orders.cancel(a.order_id)
    assert core.fleet.available_ids() == [1] and len(core.scheduler.ledger) == 0
    assert pairs(core.scheduler.dispatch()) == [(b.order_id, 1)]


def test_failing_an_in_flight_order_frees_a_healthy_robot():
    core = build_core(2)
    a = make_ready(core)
    core.scheduler.dispatch()
    advance_to(core, a.order_id, S.TO_CUSTOMER)
    core.orders.fail(a.order_id, "dropped tray")
    assert core.fleet.available_ids() == [1, 2]


def test_cancel_or_fail_before_ready_never_touches_queue_or_ledger():
    core = build_core(2)
    o = core.orders.create_order("T03", "TEA")
    core.orders.cancel(o.order_id)
    p = core.orders.create_order("T03", "TEA")
    core.orders.start_preparing(p.order_id)
    core.orders.fail(p.order_id)
    assert len(core.scheduler.queue) == 0 and len(core.scheduler.ledger) == 0


# ------------------------------------------------------------------ robot faults
def test_fault_on_a_busy_robot_fails_its_order_and_keeps_robot_out_of_service():
    core = build_core(2)
    a, b = make_ready(core), make_ready(core)
    core.scheduler.dispatch()
    core.fleet.mark_faulted(1)  # type: ignore[arg-type]
    assert core.orders.get(a.order_id).status is S.FAILED
    assert core.orders.get(b.order_id).status is S.ASSIGNED
    assert core.fleet.get(1).status is RobotStatus.FAULT  # type: ignore[arg-type]
    assert core.fleet.get(1).active_order_id is None       # type: ignore[arg-type]
    assert core.fleet.available_ids() == []
    core.fleet.recover(1)  # type: ignore[arg-type]
    assert core.fleet.available_ids() == [1]


def test_faulted_robot_is_skipped_by_dispatch():
    core = build_core(3)
    core.fleet.mark_faulted(1)  # type: ignore[arg-type]
    a, b = make_ready(core), make_ready(core)
    assert pairs(core.scheduler.dispatch()) == [(a.order_id, 2), (b.order_id, 3)]


def test_dispatch_refuses_a_queued_order_that_is_not_ready():
    """Queue corruption must fail loudly, never silently assign a non-READY order."""
    from aifr.common.errors import SchedulingInvariantError

    core = build_core(2)
    order = core.orders.create_order("T03", "TEA")          # still CREATED
    core.scheduler.queue.push(order.order_id, 0, 1)
    with pytest.raises(SchedulingInvariantError):
        core.scheduler.dispatch()
    assert len(core.scheduler.ledger) == 0 and core.fleet.available_ids() == [1, 2]
