import pytest

from aifr.common.clock import SimClock
from aifr.common.errors import ConfigError
from aifr.common.types import seconds_to_us, us_to_seconds


def test_clock_starts_at_zero_and_only_moves_on_advance():
    clock = SimClock(50_000)
    assert clock.now() == 0 and clock.tick == 0
    assert clock.now() == 0


def test_clock_advance_is_fixed_step():
    clock = SimClock(50_000)
    assert clock.advance() == 50_000
    assert clock.advance(3) == 200_000
    assert clock.tick == 4 and clock.dt_us == 50_000


def test_clock_rejects_bad_arguments():
    with pytest.raises(ConfigError):
        SimClock(0)
    with pytest.raises(ConfigError):
        SimClock(10, start_us=-1)
    with pytest.raises(ValueError):
        SimClock(10).advance(0)


def test_time_conversions_roundtrip():
    assert seconds_to_us(0.05) == 50_000
    assert us_to_seconds(1_500_000) == 1.5
