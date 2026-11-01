"""Strict assignment ledger: the authority on who is doing what.

Invariants
* Active assignments form a bijection: one robot per order and one order per robot.
* An order can be assigned at most once, ever (even after release).
"""

from __future__ import annotations

from dataclasses import dataclass

from aifr.common.errors import SchedulingInvariantError
from aifr.common.types import OrderId, RobotId, SimUs


@dataclass(frozen=True, slots=True)
class Assignment:
    order_id: OrderId
    robot_id: RobotId
    at: SimUs


class AssignmentLedger:
    def __init__(self) -> None:
        self._by_order: dict[OrderId, Assignment] = {}
        self._by_robot: dict[RobotId, Assignment] = {}
        self._ever_assigned: set[OrderId] = set()
        self._history: list[Assignment] = []

    def assign(self, order_id: OrderId, robot_id: RobotId, at: SimUs) -> Assignment:
        if order_id in self._ever_assigned:
            raise SchedulingInvariantError(f"order {order_id} was already assigned")
        if robot_id in self._by_robot:
            raise SchedulingInvariantError(
                f"robot {robot_id} already has active order {self._by_robot[robot_id].order_id}"
            )
        assignment = Assignment(order_id, robot_id, at)
        self._by_order[order_id] = assignment
        self._by_robot[robot_id] = assignment
        self._ever_assigned.add(order_id)
        self._history.append(assignment)
        return assignment

    def release(self, order_id: OrderId) -> Assignment:
        assignment = self._by_order.pop(order_id, None)
        if assignment is None:
            raise SchedulingInvariantError(f"order {order_id} has no active assignment")
        del self._by_robot[assignment.robot_id]
        return assignment

    def has_order(self, order_id: OrderId) -> bool:
        return order_id in self._by_order

    def is_robot_busy(self, robot_id: RobotId) -> bool:
        return robot_id in self._by_robot

    def robot_of(self, order_id: OrderId) -> RobotId | None:
        a = self._by_order.get(order_id)
        return a.robot_id if a else None

    def order_of(self, robot_id: RobotId) -> OrderId | None:
        a = self._by_robot.get(robot_id)
        return a.order_id if a else None

    def active(self) -> list[Assignment]:
        """Active assignments sorted by order id."""
        return [self._by_order[o] for o in sorted(self._by_order)]

    @property
    def history(self) -> tuple[Assignment, ...]:
        """Every assignment ever made, in order."""
        return tuple(self._history)

    def __len__(self) -> int:
        return len(self._by_order)

    def check_consistency(self) -> None:
        """Raise if the two indexes disagree (used by tests and debug runs)."""
        if len(self._by_order) != len(self._by_robot):
            raise SchedulingInvariantError("ledger indexes differ in size")
        for order_id, a in self._by_order.items():
            if self._by_robot.get(a.robot_id) is not a or a.order_id != order_id:
                raise SchedulingInvariantError(f"ledger indexes disagree on order {order_id}")
