"""Order data model and the lifecycle transition table.

Happy path (no skipping):
CREATED -> PREPARING -> READY -> ASSIGNED -> TO_PICKUP -> LOADING
        -> TO_CUSTOMER -> UNLOADING -> DELIVERED

CANCELLED is allowed until pickup travel is complete (before LOADING).
FAILED is allowed from any non-terminal status. DELIVERED, CANCELLED and FAILED are terminal.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping

from aifr.common.types import OrderId, RobotId, SimUs, StationId, TableId


class OrderStatus(Enum):
    CREATED = "CREATED"
    PREPARING = "PREPARING"
    READY = "READY"
    ASSIGNED = "ASSIGNED"
    TO_PICKUP = "TO_PICKUP"
    LOADING = "LOADING"
    TO_CUSTOMER = "TO_CUSTOMER"
    UNLOADING = "UNLOADING"
    DELIVERED = "DELIVERED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


S = OrderStatus

FLOW: tuple[OrderStatus, ...] = (
    S.CREATED, S.PREPARING, S.READY, S.ASSIGNED, S.TO_PICKUP,
    S.LOADING, S.TO_CUSTOMER, S.UNLOADING, S.DELIVERED,
)
TERMINAL_STATUSES: frozenset[OrderStatus] = frozenset({S.DELIVERED, S.FAILED, S.CANCELLED})
_CANCELLABLE: frozenset[OrderStatus] = frozenset(
    {S.CREATED, S.PREPARING, S.READY, S.ASSIGNED, S.TO_PICKUP}
)


def _build_transitions() -> dict[OrderStatus, frozenset[OrderStatus]]:
    table: dict[OrderStatus, frozenset[OrderStatus]] = {}
    for current, following in zip(FLOW, FLOW[1:]):
        extra = {S.FAILED} | ({S.CANCELLED} if current in _CANCELLABLE else set())
        table[current] = frozenset({following} | extra)
    for terminal in TERMINAL_STATUSES:
        table[terminal] = frozenset()
    return table


ALLOWED_TRANSITIONS: Mapping[OrderStatus, frozenset[OrderStatus]] = _build_transitions()


@dataclass(slots=True)
class Order:
    """A customer order. Mutated only by :class:`aifr.orders.manager.OrderManager`.

    ``(ready_at, ready_seq)`` is the FCFS key: ``ready_at`` is the simulation time of the
    READY transition, ``ready_seq`` a global increasing counter that breaks timestamp ties.
    """

    order_id: OrderId
    table_id: TableId
    station_id: StationId
    created_at: SimUs
    status: OrderStatus = OrderStatus.CREATED
    ready_at: SimUs | None = None
    ready_seq: int | None = None
    robot_id: RobotId | None = None
    assigned_at: SimUs | None = None
    closed_at: SimUs | None = None
    close_reason: str | None = None
    history: list[tuple[SimUs, OrderStatus]] = field(default_factory=list)

    @property
    def is_terminal(self) -> bool:
        return self.status in TERMINAL_STATUSES
