"""Synchronous, deterministic in-process event bus.

Guarantees
* Handlers run in subscription order.
* Events are delivered FIFO. A ``publish`` made from inside a handler is queued and
  delivered after the current handler returns (no re-entrant surprises).
* ``defer()`` holds delivery until the block exits, so a multi-step state change
  (e.g. one dispatch batch) is observed by subscribers only once it is complete.
* Subscription is by exact event type (or all events via ``subscribe_all``).
* If a handler raises, undelivered events are discarded and the error propagates;
  a failing handler is a programming error and system state should be treated as undefined.
"""

from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Deque, Iterator, TypeVar

from aifr.common.types import SimUs


@dataclass(frozen=True, slots=True)
class Event:
    """Base class for all events. ``at`` is the simulation time of occurrence."""

    at: SimUs


E = TypeVar("E", bound=Event)
_Handler = Callable[[Any], None]


class EventBus:
    def __init__(self, *, record: bool = False) -> None:
        self._subs: list[tuple[type[Event] | None, _Handler]] = []
        self._pending: Deque[Event] = deque()
        self._defer_depth = 0
        self._draining = False
        self._record = record
        self._history: list[Event] = []

    def subscribe(self, event_type: type[E], handler: Callable[[E], None]) -> None:
        """Call ``handler`` for every event whose type is exactly ``event_type``."""
        self._subs.append((event_type, handler))

    def subscribe_all(self, handler: Callable[[Event], None]) -> None:
        """Call ``handler`` for every event."""
        self._subs.append((None, handler))

    def publish(self, event: Event) -> None:
        self._pending.append(event)
        self._drain()

    @contextmanager
    def defer(self) -> Iterator[None]:
        """Hold event delivery until the (possibly nested) block exits."""
        self._defer_depth += 1
        try:
            yield
        finally:
            self._defer_depth -= 1
            self._drain()

    @property
    def history(self) -> tuple[Event, ...]:
        """Delivered events in order (only populated when ``record=True``)."""
        return tuple(self._history)

    def _drain(self) -> None:
        if self._defer_depth > 0 or self._draining:
            return
        self._draining = True
        try:
            while self._pending:
                event = self._pending.popleft()
                if self._record:
                    self._history.append(event)
                for event_type, handler in tuple(self._subs):
                    if event_type is None or type(event) is event_type:
                        handler(event)
        except BaseException:
            self._pending.clear()
            raise
        finally:
            self._draining = False
