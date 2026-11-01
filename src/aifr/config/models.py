"""Immutable, validated configuration models.

Values are validated on construction; ``from_dict`` rejects unknown keys so typos
fail loudly. The default tick length is a proposal, not a V14 fact.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
from typing import Any, Mapping, TypeVar

from aifr.common.errors import ConfigError

DEFAULT_ROBOT_COUNT = 3
MAX_ROBOTS = 64                 # sanity cap, not a design limit
DEFAULT_TICK_DT_US = 50_000     # 20 Hz


def _check_int(name: str, value: object, minimum: int, maximum: int | None = None) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{name} must be an integer, got {value!r}")
    if value < minimum:
        raise ConfigError(f"{name} must be >= {minimum}, got {value}")
    if maximum is not None and value > maximum:
        raise ConfigError(f"{name} must be <= {maximum}, got {value}")


@dataclass(frozen=True, slots=True)
class FleetConfig:
    robot_count: int = DEFAULT_ROBOT_COUNT

    def __post_init__(self) -> None:
        _check_int("fleet.robot_count", self.robot_count, 1, MAX_ROBOTS)


@dataclass(frozen=True, slots=True)
class ClockConfig:
    tick_dt_us: int = DEFAULT_TICK_DT_US

    def __post_init__(self) -> None:
        _check_int("clock.tick_dt_us", self.tick_dt_us, 1)


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    seed: int = 0

    def __post_init__(self) -> None:
        _check_int("simulation.seed", self.seed, 0)


T = TypeVar("T")


def _build(cls: type[T], data: Any, section: str) -> T:
    if not isinstance(data, Mapping):
        raise ConfigError(f"section '{section}' must be a mapping, got {type(data).__name__}")
    unknown = set(data) - {f.name for f in fields(cls)}  # type: ignore[arg-type]
    if unknown:
        raise ConfigError(f"unknown key(s) in '{section}': {sorted(unknown)}")
    return cls(**data)


@dataclass(frozen=True, slots=True)
class AifrConfig:
    fleet: FleetConfig = field(default_factory=FleetConfig)
    clock: ClockConfig = field(default_factory=ClockConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> AifrConfig:
        """Build from a plain mapping, e.g. parsed JSON. Missing sections use defaults."""
        if not isinstance(data, Mapping):
            raise ConfigError(f"config must be a mapping, got {type(data).__name__}")
        unknown = set(data) - {"fleet", "clock", "simulation"}
        if unknown:
            raise ConfigError(f"unknown config section(s): {sorted(unknown)}")
        return cls(
            fleet=_build(FleetConfig, data.get("fleet", {}), "fleet"),
            clock=_build(ClockConfig, data.get("clock", {}), "clock"),
            simulation=_build(SimulationConfig, data.get("simulation", {}), "simulation"),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
