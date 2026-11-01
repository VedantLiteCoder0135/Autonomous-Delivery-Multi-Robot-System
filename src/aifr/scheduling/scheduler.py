"""Global FCFS scheduler.

Policy (and nothing else):
  * One global queue ordered by READY ``(timestamp, sequence)``.
  * On ``dispatch()``, take the batch of currently available robots sorted by id and pair the
    oldest READY order with the lowest-numbered robot, the next-oldest with the next robot, ...
  * No distance, route length, position or cost is read anywhere in this module.

Batching: ``dispatch()`` is a barrier. The engine calls it once per tick, after all events of
that tick (deliveries, cancellations, READY transitions) have been applied, so robots that
became available "simultaneously" are always processed together in robot-id order, never in
event-arrival order. The scheduler never dispatches on its own from an event handler.

It keeps queue and ledger in step with order events:
  READY                      -> enqueue with the order's (ready_at, ready_seq)
  CANCELLED / FAILED         -> dequeue if waiting, else release the assignment
  DELIVERED                  -> release the assignment (robot becomes available at once)
  robot FAULT with an order  -> that order is failed (requeue is a later, explicit decision)
"""

from __future__ import annotations

from aifr.common.clock import Clock
from aifr.common.errors import SchedulingInvariantError
from aifr.common.events import EventBus
from aifr.common.types import OrderId
from aifr.fleet.events import RobotFaulted
from aifr.fleet.registry import RobotRegistry
from aifr.orders.events import OrderStatusChanged
from aifr.orders.manager import OrderManager
from aifr.orders.model import OrderStatus
from aifr.scheduling.ledger import Assignment, AssignmentLedger
from aifr.scheduling.queue import FcfsQueue


class GlobalScheduler:
    def __init__(
        self,
        bus: EventBus,
        clock: Clock,
        orders: OrderManager,
        fleet: RobotRegistry,
        queue: FcfsQueue | None = None,
        ledger: AssignmentLedger | None = None,
        release_on_delivery: bool = True,
    ) -> None:
        self.release_on_delivery = release_on_delivery
        self._return_pending: set = set()
        self._bus = bus
        self._clock = clock
        self._orders = orders
        self._fleet = fleet
        self.queue = queue if queue is not None else FcfsQueue()
        self.ledger = ledger if ledger is not None else AssignmentLedger()
        bus.subscribe(OrderStatusChanged, self._on_order_status)
        bus.subscribe(RobotFaulted, self._on_robot_faulted)

    # ----------------------------------------------------------------- dispatch
    def dispatch(self) -> list[Assignment]:
        """Assign waiting orders to available robots. Returns the assignments made.

        All state changes of one call are committed before any subscriber sees an event.
        """
        now = self._clock.now()
        assignments: list[Assignment] = []
        with self._bus.defer():
            for robot_id in self._fleet.available_ids():        # ascending robot id
                if len(self.queue) == 0:
                    break
                entry = self.queue.pop_head()                    # oldest READY order
                order = self._orders.get(entry.order_id)
                if order.status is not OrderStatus.READY:
                    raise SchedulingInvariantError(
                        f"queued order {order.order_id} is {order.status.value}, expected READY"
                    )
                assignments.append(self.ledger.assign(entry.order_id, robot_id, now))
                self._orders.mark_assigned(entry.order_id, robot_id)
                self._fleet.assign(robot_id, entry.order_id)
        return assignments

    # ------------------------------------------------------------ event handlers
    def _on_order_status(self, event: OrderStatusChanged) -> None:
        new = event.new
        if new is OrderStatus.READY:
            order = self._orders.get(event.order_id)
            if order.ready_at is None or order.ready_seq is None:
                raise SchedulingInvariantError(f"order {order.order_id} is READY without a key")
            self.queue.push(order.order_id, order.ready_at, order.ready_seq)
        elif new in (OrderStatus.CANCELLED, OrderStatus.FAILED):
            if not self.queue.remove(event.order_id) and self.ledger.has_order(event.order_id):
                self._release(event.order_id)
        elif new is OrderStatus.DELIVERED:
            if self.release_on_delivery:
                self._release(event.order_id)
            else:
                self._return_pending.add(event.order_id)


    def _on_robot_faulted(self, event: RobotFaulted) -> None:
        order_id = self.ledger.order_of(event.robot_id)
        if order_id is not None:
            if self._orders.get(order_id).status is OrderStatus.DELIVERED:
                # Fault during return: the delivered order stays delivered.
                self._return_pending.discard(order_id)
                self._release(order_id)
            else:
                self._orders.fail(order_id, reason=f"robot {event.robot_id} faulted")

    def _release(self, order_id: OrderId) -> None:
        assignment = self.ledger.release(order_id)
        self._fleet.release(assignment.robot_id)

    def complete_return(self, robot_id) -> None:
        """Stage 8: release a delivered robot only after its base arrival is validated.

        Caller is responsible for verifying the physical robot is at base.
        This method intentionally does not infer location from fleet status.
        """
        if self.release_on_delivery:
            raise SchedulingInvariantError("Return gating is not enabled")
        order_id = self.ledger.order_of(robot_id)
        if order_id is None or order_id not in self._return_pending:
            raise SchedulingInvariantError(f"robot {robot_id} has no pending return")
        if self._orders.get(order_id).status is not OrderStatus.DELIVERED:
            raise SchedulingInvariantError("Cannot release robot before delivery")
        self._return_pending.remove(order_id)
        self._release(order_id)

    def is_return_pending(self, robot_id) -> bool:
        order_id = self.ledger.order_of(robot_id)
        return order_id is not None and order_id in self._return_pending
