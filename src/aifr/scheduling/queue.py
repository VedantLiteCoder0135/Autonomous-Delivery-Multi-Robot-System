"""The single global FCFS queue.

Entries are ordered by ``(ready_at, ready_seq, order_id)``. The queue knows nothing about
tables, counters, robots, distances or costs. It is a sorted list: queue sizes in this
simulation are small, and sorted-list semantics keep removal and requeue trivially correct.
"""

from __future__ import annotations

from bisect import bisect_left, insort
from dataclasses import dataclass

from aifr.common.errors import SchedulingInvariantError
from aifr.common.types import OrderId, SimUs


@dataclass(frozen=True, slots=True, order=True)
class QueueEntry:
    ready_at: SimUs
    ready_seq: int
    order_id: OrderId


class FcfsQueue:
    def __init__(self) -> None:
        self._entries: list[QueueEntry] = []
        self._by_order: dict[OrderId, QueueEntry] = {}

    def push(self, order_id: OrderId, ready_at: SimUs, ready_seq: int) -> QueueEntry:
        if order_id in self._by_order:
            raise SchedulingInvariantError(f"order {order_id} is already queued")
        entry = QueueEntry(ready_at, ready_seq, order_id)
        insort(self._entries, entry)
        self._by_order[order_id] = entry
        return entry

    def peek(self) -> QueueEntry | None:
        return self._entries[0] if self._entries else None

    def pop_head(self) -> QueueEntry:
        if not self._entries:
            raise IndexError("pop from empty FcfsQueue")
        entry = self._entries.pop(0)
        del self._by_order[entry.order_id]
        return entry

    def remove(self, order_id: OrderId) -> bool:
        """Remove an order (e.g. cancelled while waiting). Returns whether it was queued."""
        entry = self._by_order.pop(order_id, None)
        if entry is None:
            return False
        del self._entries[bisect_left(self._entries, entry)]
        return True

    def order_ids(self) -> list[OrderId]:
        """Queued order ids in FCFS order (head first)."""
        return [e.order_id for e in self._entries]

    def __len__(self) -> int:
        return len(self._entries)

    def __contains__(self, order_id: object) -> bool:
        return order_id in self._by_order
