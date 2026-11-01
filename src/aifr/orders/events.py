"""Order events."""

from __future__ import annotations

from dataclasses import dataclass

from aifr.common.events import Event
from aifr.common.types import OrderId, RobotId, StationId, TableId
from aifr.orders.model import OrderStatus


@dataclass(frozen=True, slots=True)
class OrderCreated(Event):
    order_id: OrderId
    table_id: TableId
    station_id: StationId


@dataclass(frozen=True, slots=True)
class OrderStatusChanged(Event):
    """Published for every status change, including READY and ASSIGNED."""

    order_id: OrderId
    old: OrderStatus
    new: OrderStatus
    robot_id: RobotId | None = None
