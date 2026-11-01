"""Typed identifiers and simulation-time helpers.

Simulation time is an integer number of microseconds (``SimUs``). Integers keep
READY-timestamp comparisons exact; floats are never used for ordering.
"""

from __future__ import annotations

from typing import NewType

RobotId = NewType("RobotId", int)      # 1-based, so "lowest-numbered robot" == min()
OrderId = NewType("OrderId", int)      # monotonic creation sequence, starts at 1
TableId = NewType("TableId", str)      # e.g. "T03"; validated against the environment in Stage 3
StationId = NewType("StationId", str)  # e.g. "TEA"; validated against the environment in Stage 3

SimUs = int                            # simulation time in integer microseconds
US_PER_SECOND = 1_000_000


def seconds_to_us(seconds: float) -> SimUs:
    """Convert seconds to integer microseconds (rounded to nearest)."""
    return round(seconds * US_PER_SECOND)


def us_to_seconds(us: SimUs) -> float:
    """Convert integer microseconds to seconds."""
    return us / US_PER_SECOND
