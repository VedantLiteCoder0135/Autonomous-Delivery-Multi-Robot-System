from dataclasses import dataclass

import pytest

from aifr.common.events import Event, EventBus


@dataclass(frozen=True, slots=True)
class A(Event):
    n: int = 0


@dataclass(frozen=True, slots=True)
class B(Event):
    pass


def test_exact_type_subscription_and_order():
    bus, seen = EventBus(), []
    bus.subscribe(A, lambda e: seen.append(("first", e.n)))
    bus.subscribe(A, lambda e: seen.append(("second", e.n)))
    bus.subscribe(B, lambda e: seen.append(("b", 0)))
    bus.publish(A(0, 1))
    assert seen == [("first", 1), ("second", 1)]


def test_subscribe_all_sees_everything():
    bus, seen = EventBus(), []
    bus.subscribe_all(lambda e: seen.append(type(e).__name__))
    bus.publish(A(0))
    bus.publish(B(0))
    assert seen == ["A", "B"]


def test_publish_inside_handler_is_queued_fifo():
    bus, log = EventBus(), []

    def on_a(e: A) -> None:
        log.append(f"a{e.n}-start")
        if e.n < 2:
            bus.publish(A(0, e.n + 1))
        bus.publish(B(0))
        log.append(f"a{e.n}-end")

    bus.subscribe(A, on_a)
    bus.subscribe(B, lambda e: log.append("b"))
    bus.publish(A(0, 0))
    # Each handler finishes completely; what it published is delivered afterwards, FIFO.
    assert log == [
        "a0-start", "a0-end",
        "a1-start", "a1-end",
        "b",
        "a2-start", "a2-end",
        "b", "b",
    ]


def test_defer_holds_delivery_until_block_exits():
    bus, seen = EventBus(), []
    bus.subscribe(A, lambda e: seen.append(e.n))
    with bus.defer():
        bus.publish(A(0, 1))
        bus.publish(A(0, 2))
        assert seen == []
        with bus.defer():
            bus.publish(A(0, 3))
        assert seen == []          # inner exit does not drain while outer is open
    assert seen == [1, 2, 3]


def test_record_history():
    bus = EventBus(record=True)
    bus.publish(A(0, 1))
    bus.publish(B(5))
    assert [type(e) for e in bus.history] == [A, B]
    assert EventBus().history == ()


def test_handler_exception_propagates_and_bus_recovers():
    bus, seen = EventBus(), []

    def boom(e: A) -> None:
        raise RuntimeError("handler bug")

    bus.subscribe(A, boom)
    bus.subscribe(B, lambda e: seen.append("b"))
    with pytest.raises(RuntimeError):
        bus.publish(A(0))
    bus.publish(B(0))              # bus is usable again, stale events were discarded
    assert seen == ["b"]
