"""Order lifecycle manager: the single writer of order state."""

from __future__ import annotations

from aifr.common.clock import Clock
from aifr.common.errors import InvalidTransitionError, OrderError, UnknownOrderError
from aifr.common.events import EventBus
from aifr.common.types import OrderId, RobotId, StationId, TableId
from aifr.orders.events import OrderCreated, OrderStatusChanged
from aifr.orders.model import (
    ALLOWED_TRANSITIONS,
    TERMINAL_STATUSES,
    Order,
    OrderStatus,
)

# Statuses that carry extra data or are not part of the plain forward flow, so they
# have dedicated methods instead of going through ``advance``.
_NOT_ADVANCEABLE = frozenset(
    {
        OrderStatus.CREATED,
        OrderStatus.READY,
        OrderStatus.ASSIGNED,
        OrderStatus.FAILED,
        OrderStatus.CANCELLED,
    }
)


class OrderManager:
    def __init__(self, bus: EventBus, clock: Clock) -> None:
        self._bus = bus
        self._clock = clock
        self._orders: dict[OrderId, Order] = {}
        self._next_order_id = 1
        self._next_ready_seq = 1

    # ------------------------------------------------------------------ queries
    def get(self, order_id: OrderId) -> Order:
        try:
            return self._orders[order_id]
        except KeyError:
            raise UnknownOrderError(f"unknown order {order_id}") from None

    def all_orders(self) -> list[Order]:
        """All orders sorted by id."""
        return [self._orders[i] for i in sorted(self._orders)]

    def by_status(self, *statuses: OrderStatus) -> list[Order]:
        """Orders currently in any of ``statuses``, sorted by id."""
        wanted = set(statuses)
        return [o for o in self.all_orders() if o.status in wanted]

    def __len__(self) -> int:
        return len(self._orders)

    # ---------------------------------------------------------------- lifecycle
    def create_order(self, table_id: str, station_id: str) -> Order:
        if not isinstance(table_id, str) or not table_id.strip():
            raise OrderError("table_id must be a non-empty string")
        if not isinstance(station_id, str) or not station_id.strip():
            raise OrderError("station_id must be a non-empty string")
        now = self._clock.now()
        order = Order(
            order_id=OrderId(self._next_order_id),
            table_id=TableId(table_id),
            station_id=StationId(station_id),
            created_at=now,
            history=[(now, OrderStatus.CREATED)],
        )
        self._next_order_id += 1
        self._orders[order.order_id] = order
        self._bus.publish(OrderCreated(now, order.order_id, order.table_id, order.station_id))
        return order

    def start_preparing(self, order_id: OrderId) -> Order:
        return self._apply(self.get(order_id), OrderStatus.PREPARING)

    def mark_ready(self, order_id: OrderId) -> Order:
        """PREPARING -> READY. Stamps the FCFS key ``(ready_at, ready_seq)``."""
        order = self.get(order_id)
        self._check(order, OrderStatus.READY)       # validate before consuming a sequence number
        order.ready_at = self._clock.now()
        order.ready_seq = self._next_ready_seq
        self._next_ready_seq += 1
        return self._apply(order, OrderStatus.READY)

    def mark_assigned(self, order_id: OrderId, robot_id: RobotId) -> Order:
        """READY -> ASSIGNED. Intended to be called by the scheduler only."""
        order = self.get(order_id)
        self._check(order, OrderStatus.ASSIGNED)
        order.robot_id = robot_id
        order.assigned_at = self._clock.now()
        return self._apply(order, OrderStatus.ASSIGNED)

    def advance(self, order_id: OrderId, new_status: OrderStatus) -> Order:
        """Move along the delivery flow: PREPARING, TO_PICKUP ... DELIVERED."""
        if new_status in _NOT_ADVANCEABLE:
            raise InvalidTransitionError(
                f"{new_status.value} cannot be set via advance(); use its dedicated method"
            )
        return self._apply(self.get(order_id), new_status)

    def cancel(self, order_id: OrderId, reason: str = "") -> Order:
        order = self.get(order_id)
        self._check(order, OrderStatus.CANCELLED)
        order.close_reason = reason or None
        return self._apply(order, OrderStatus.CANCELLED)

    def fail(self, order_id: OrderId, reason: str = "") -> Order:
        order = self.get(order_id)
        self._check(order, OrderStatus.FAILED)
        order.close_reason = reason or None
        return self._apply(order, OrderStatus.FAILED)

    # ----------------------------------------------------------------- internals
    @staticmethod
    def _check(order: Order, new: OrderStatus) -> None:
        if new not in ALLOWED_TRANSITIONS[order.status]:
            raise InvalidTransitionError(
                f"order {order.order_id}: {order.status.value} -> {new.value} is not allowed"
            )

    def _apply(self, order: Order, new: OrderStatus) -> Order:
        self._check(order, new)
        now = self._clock.now()
        old = order.status
        order.status = new
        order.history.append((now, new))
        if new in TERMINAL_STATUSES:
            order.closed_at = now
        self._bus.publish(OrderStatusChanged(now, order.order_id, old, new, order.robot_id))
        return order
