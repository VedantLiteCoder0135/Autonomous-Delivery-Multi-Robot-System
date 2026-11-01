"""Exception hierarchy. Every AIFR-raised error derives from :class:`AifrError`."""

from __future__ import annotations


class AifrError(Exception):
    """Base class for all AIFR errors."""


class ConfigError(AifrError, ValueError):
    """Invalid or unknown configuration."""


class OrderError(AifrError):
    """Base class for order-related errors."""


class UnknownOrderError(OrderError):
    """No order exists with the requested id."""


class InvalidTransitionError(OrderError):
    """The requested order status change is not allowed by the lifecycle."""


class RobotError(AifrError):
    """Base class for robot-related errors."""


class UnknownRobotError(RobotError):
    """No robot exists with the requested id."""


class InvalidRobotStateError(RobotError):
    """The requested robot operation is not valid in its current state."""


class SchedulingInvariantError(AifrError):
    """A scheduling invariant was violated (duplicate assignment, queue corruption...)."""
