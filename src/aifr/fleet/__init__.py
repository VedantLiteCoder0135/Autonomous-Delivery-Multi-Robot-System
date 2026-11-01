"""Robot model and registry."""

from aifr.fleet.events import RobotAvailable, RobotFaulted, RobotStatusChanged
from aifr.fleet.registry import RobotRegistry
from aifr.fleet.robot import Robot, RobotStatus

__all__ = [
    "Robot",
    "RobotAvailable",
    "RobotFaulted",
    "RobotRegistry",
    "RobotStatus",
    "RobotStatusChanged",
]
