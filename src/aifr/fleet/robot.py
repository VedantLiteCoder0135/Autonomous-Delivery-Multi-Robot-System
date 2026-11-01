"""Robot model (Stage 2: identity and availability only; no pose or kinematics yet)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from aifr.common.types import OrderId, RobotId


class RobotStatus(Enum):
    IDLE = "IDLE"      # healthy, may be assigned
    BUSY = "BUSY"      # holds exactly one active order
    FAULT = "FAULT"    # out of service until recovered


@dataclass(slots=True)
class Robot:
    """Mutated only by :class:`aifr.fleet.registry.RobotRegistry`."""

    robot_id: RobotId
    status: RobotStatus = RobotStatus.IDLE
    active_order_id: OrderId | None = None

    @property
    def is_available(self) -> bool:
        """Available == healthy, idle and holding no order. Location is irrelevant."""
        return self.status is RobotStatus.IDLE and self.active_order_id is None
