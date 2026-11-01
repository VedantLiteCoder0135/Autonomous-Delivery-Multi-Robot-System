import random

import pytest

from aifr.common.errors import SchedulingInvariantError
from aifr.scheduling import FcfsQueue

from helpers import build_core, make_ready


def oid(n):
    return n  # OrderId is a NewType over int


def test_empty_queue():
    q = FcfsQueue()
    assert len(q) == 0 and q.peek() is None and q.order_ids() == []
    with pytest.raises(IndexError):
        q.pop_head()


def test_orders_by_ready_timestamp_first_then_sequence():
    q = FcfsQueue()
    q.push(oid(1), ready_at=500, ready_seq=1)
    q.push(oid(2), ready_at=300, ready_seq=2)      # earlier timestamp wins despite higher seq
    q.push(oid(3), ready_at=300, ready_seq=3)      # same timestamp: sequence breaks the tie
    assert q.order_ids() == [2, 3, 1]


def test_ties_on_timestamp_follow_sequence_not_order_id():
    q = FcfsQueue()
    q.push(oid(1), 100, ready_seq=3)
    q.push(oid(2), 100, ready_seq=1)
    q.push(oid(3), 100, ready_seq=2)
    assert [q.pop_head().order_id for _ in range(3)] == [2, 3, 1]


def test_pop_peek_contains_len():
    q = FcfsQueue()
    q.push(oid(7), 10, 1)
    q.push(oid(8), 20, 2)
    assert q.peek().order_id == 7 and len(q) == 2 and 7 in q and 9 not in q
    assert q.pop_head().order_id == 7
    assert 7 not in q and len(q) == 1


def test_remove_anywhere():
    q = FcfsQueue()
    for i in range(1, 6):
        q.push(oid(i), i * 10, i)
    assert q.remove(oid(3)) is True
    assert q.remove(oid(3)) is False
    assert q.remove(oid(99)) is False
    assert q.order_ids() == [1, 2, 4, 5]


def test_duplicate_push_rejected():
    q = FcfsQueue()
    q.push(oid(1), 10, 1)
    with pytest.raises(SchedulingInvariantError):
        q.push(oid(1), 20, 2)
    assert len(q) == 1


def test_requeue_after_pop_restores_original_position():
    q = FcfsQueue()
    q.push(oid(1), 10, 1)
    q.push(oid(2), 20, 2)
    head = q.pop_head()
    q.push(head.order_id, head.ready_at, head.ready_seq)
    assert q.order_ids() == [1, 2]


@pytest.mark.parametrize("seed", range(20))
def test_matches_sorted_reference_under_random_ops(seed):
    rng = random.Random(seed)
    q, ref = FcfsQueue(), {}
    for n in range(1, 120):
        q.push(oid(n), rng.randint(0, 15), rng.randint(0, 10_000) * 1000 + n)
        ref[n] = (q._by_order[n].ready_at, q._by_order[n].ready_seq)
        if rng.random() < 0.3 and ref:
            victim = rng.choice(sorted(ref))
            assert q.remove(oid(victim))
            del ref[victim]
        if rng.random() < 0.2 and ref:
            head = q.pop_head().order_id
            assert head == min(ref, key=lambda k: (ref[k], k))
            del ref[head]
        assert q.order_ids() == sorted(ref, key=lambda k: (ref[k], k))


def test_queue_is_fed_in_ready_order_by_scheduler_events():
    core = build_core(2)
    a, b, c = (make_ready(core) for _ in range(3))
    assert core.scheduler.queue.order_ids() == [a.order_id, b.order_id, c.order_id]
