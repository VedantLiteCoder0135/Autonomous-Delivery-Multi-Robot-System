"""Simulation clock abstraction.

Domain code depends on the :class:`Clock` protocol only, so the same logic runs
under a fixed-step simulation clock, a test clock, or (later) a Blender-driven one.
"""

from __future__ import annotations

from typing import Protocol

from aifr.common.errors import ConfigError
from aifr.common.types import SimUs


class Clock(Protocol):
    """Read-only view of simulation time."""

    def now(self) -> SimUs:
        """Current simulation time in microseconds."""
        ...


class SimClock:
    """Deterministic fixed-timestep clock. Time only moves when ``advance`` is called."""

    def __init__(self, dt_us: SimUs, start_us: SimUs = 0) -> None:
        if dt_us <= 0:
            raise ConfigError(f"dt_us must be positive, got {dt_us}")
        if start_us < 0:
            raise ConfigError(f"start_us must be >= 0, got {start_us}")
        self._dt_us = dt_us
        self._now = start_us
        self._tick = 0

    @property
    def dt_us(self) -> SimUs:
        return self._dt_us

    @property
    def tick(self) -> int:
        """Number of ticks advanced since construction."""
        return self._tick

    def now(self) -> SimUs:
        return self._now

    def advance(self, ticks: int = 1) -> SimUs:
        """Advance by ``ticks`` whole ticks and return the new time."""
        if ticks < 1:
            raise ValueError(f"ticks must be >= 1, got {ticks}")
        self._tick += ticks
        self._now += ticks * self._dt_us
        return self._now
