"""Scheduling behaves identically for every supported fleet size 1..8."""

import pytest

from helpers import build_core, deliver, make_ready, pairs

COUNTS = list(range(1, 9))


@pytest.mark.parametrize("n", COUNTS)
def test_fewer_orders_than_robots_use_lowest_ids(n):
    core = build_core(n)
    k = max(1, n // 2)
    ids = [make_ready(core).order_id for _ in range(k)]
    assert pairs(core.scheduler.dispatch()) == [(ids[i], i + 1) for i in range(k)]
    assert core.fleet.available_ids() == list(range(k + 1, n + 1))


@pytest.mark.parametrize("n", COUNTS)
def test_more_orders_than_robots_saturate_fleet_and_queue_the_rest(n):
    core = build_core(n)
    ids = [make_ready(core).order_id for _ in range(n + 3)]
    assert pairs(core.scheduler.dispatch()) == [(ids[i], i + 1) for i in range(n)]
    assert core.fleet.available_ids() == []
    assert core.scheduler.queue.order_ids() == ids[n:]
    assert len(core.scheduler.ledger) == n


@pytest.mark.parametrize("n", COUNTS)
def test_full_batch_release_reassigns_in_robot_id_order_regardless_of_finish_order(n):
    core = build_core(n)
    first = [make_ready(core).order_id for _ in range(n)]
    core.scheduler.dispatch()
    nxt = [make_ready(core).order_id for _ in range(n)]
    for order_id in reversed(first):               # robots free in descending id order
        deliver(core, order_id)
    assert pairs(core.scheduler.dispatch()) == [(nxt[i], i + 1) for i in range(n)]


@pytest.mark.parametrize("n", COUNTS)
def test_throughput_run_delivers_everything_exactly_once(n):
    core = build_core(n)
    total = 4 * n + 1
    for _ in range(total):
        make_ready(core)
    delivered: list[int] = []
    while len(delivered) < total:
        batch = core.scheduler.dispatch()
        assert batch, "scheduler stalled with waiting orders and idle robots"
        assert len({a.robot_id for a in batch}) == len(batch)       # one order per robot
        for a in batch:
            deliver(core, a.order_id)
            delivered.append(int(a.order_id))
        core.clock.advance()
    assert delivered == list(range(1, total + 1))                  # strict FCFS overall
