"""Shared test utilities: core wiring and lifecycle drivers."""

from __future__ import annotations

from dataclasses import dataclass

from aifr.common.clock import SimClock
from aifr.common.events import EventBus
from aifr.common.types import OrderId
from aifr.fleet.registry import RobotRegistry
from aifr.orders.manager import OrderManager
from aifr.orders.model import FLOW, Order, OrderStatus
from aifr.scheduling.scheduler import GlobalScheduler


@dataclass
class Core:
    bus: EventBus
    clock: SimClock
    orders: OrderManager
    fleet: RobotRegistry
    scheduler: GlobalScheduler


def build_core(robot_count: int = 3, *, record: bool = False, dt_us: int = 50_000) -> Core:
    bus = EventBus(record=record)
    clock = SimClock(dt_us)
    orders = OrderManager(bus, clock)
    fleet = RobotRegistry(bus, clock, robot_count)
    scheduler = GlobalScheduler(bus, clock, orders, fleet)
    return Core(bus, clock, orders, fleet, scheduler)


@dataclass
class OrdersOnly:
    bus: EventBus
    clock: SimClock
    orders: OrderManager


def build_orders(*, record: bool = False) -> OrdersOnly:
    """Order manager with no scheduler attached, for pure lifecycle-rule tests."""
    bus = EventBus(record=record)
    clock = SimClock(50_000)
    return OrdersOnly(bus, clock, OrderManager(bus, clock))


def make_ready(core: Core, table: str = "T03", station: str = "TEA") -> Order:
    """Create an order and take it CREATED -> PREPARING -> READY at the current time."""
    order = core.orders.create_order(table, station)
    core.orders.start_preparing(order.order_id)
    return core.orders.mark_ready(order.order_id)


def advance_to(core: Core, order_id: OrderId, target: OrderStatus) -> None:
    """Walk an assigned order forward along the happy path until it reaches ``target``."""
    while core.orders.get(order_id).status is not target:
        current = core.orders.get(order_id).status
        core.orders.advance(order_id, FLOW[FLOW.index(current) + 1])


def deliver(core: Core, order_id: OrderId) -> None:
    advance_to(core, order_id, OrderStatus.DELIVERED)


def pairs(assignments: list) -> list[tuple[int, int]]:  # type: ignore[type-arg]
    return [(int(a.order_id), int(a.robot_id)) for a in assignments]
