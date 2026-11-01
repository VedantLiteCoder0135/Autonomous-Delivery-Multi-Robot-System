import pytest

from aifr.common.errors import ConfigError
from aifr.config import DEFAULT_ROBOT_COUNT, MAX_ROBOTS, AifrConfig, FleetConfig


def test_default_robot_count_is_three():
    assert DEFAULT_ROBOT_COUNT == 3
    assert FleetConfig().robot_count == 3
    assert AifrConfig().fleet.robot_count == 3


@pytest.mark.parametrize("n", [1, 2, 3, 4, 5, 6, 7, 8, MAX_ROBOTS])
def test_valid_robot_counts(n):
    assert FleetConfig(robot_count=n).robot_count == n


@pytest.mark.parametrize("bad", [0, -1, MAX_ROBOTS + 1, 2.5, "3", True, None])
def test_invalid_robot_counts(bad):
    with pytest.raises(ConfigError):
        FleetConfig(robot_count=bad)


def test_from_dict_and_roundtrip():
    cfg = AifrConfig.from_dict({"fleet": {"robot_count": 5}, "simulation": {"seed": 7}})
    assert cfg.fleet.robot_count == 5 and cfg.simulation.seed == 7
    assert cfg.clock.tick_dt_us == 50_000
    assert AifrConfig.from_dict(cfg.to_dict()) == cfg
    assert AifrConfig.from_dict({}) == AifrConfig()


@pytest.mark.parametrize(
    "data",
    [
        {"fleeet": {}},
        {"fleet": {"robots": 3}},
        {"fleet": 3},
        {"clock": {"tick_dt_us": 0}},
        {"simulation": {"seed": -1}},
    ],
)
def test_from_dict_rejects_bad_input(data):
    with pytest.raises(ConfigError):
        AifrConfig.from_dict(data)


def test_config_is_immutable():
    with pytest.raises(Exception):
        FleetConfig().robot_count = 9  # type: ignore[misc]


@pytest.mark.parametrize("data", [[], "fleet", 3, None])
def test_from_dict_rejects_non_mapping_top_level(data):
    with pytest.raises(ConfigError):
        AifrConfig.from_dict(data)
