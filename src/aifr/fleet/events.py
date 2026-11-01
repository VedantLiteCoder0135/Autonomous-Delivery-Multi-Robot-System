"""Fleet events."""

from __future__ import annotations

from dataclasses import dataclass

from aifr.common.events import Event
from aifr.common.types import RobotId
from aifr.fleet.robot import RobotStatus


@dataclass(frozen=True, slots=True)
class RobotStatusChanged(Event):
    robot_id: RobotId
    old: RobotStatus
    new: RobotStatus


@dataclass(frozen=True, slots=True)
class RobotAvailable(Event):
    """A robot just became available (after delivery, cancellation or recovery)."""

    robot_id: RobotId


@dataclass(frozen=True, slots=True)
class RobotFaulted(Event):
    robot_id: RobotId
