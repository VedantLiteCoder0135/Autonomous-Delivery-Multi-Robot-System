"""A* mission distance, time, and energy estimates in SI units."""
from __future__ import annotations

from dataclasses import dataclass
import math

from aifr.navigation.astar import find_path
from .manager import BatteryConfig


@dataclass(frozen=True, slots=True)
class MissionEstimate:
    distance_m: float
    rotation_rad: float
    travel_time_s: float
    stationary_time_s: float
    energy_wh: float
    legs: tuple[tuple[tuple[float, float], ...], ...]


def path_metrics(path):
    """Return metre length and absolute wrapped heading change of a grid route."""
    if not path:
        return None
    distance = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(path, path[1:]) if math.dist(a, b) > 1e-9]
    rotation = sum(abs(math.atan2(math.sin(b - a), math.cos(b - a)))
                   for a, b in zip(headings, headings[1:]))
    return distance, rotation


def estimate_leg(grid, start, goal, *, max_speed_mps: float = 0.35,
                 max_accel_mps2: float = 0.8, max_angular_rps: float = 1.5):
    if min(max_speed_mps, max_accel_mps2, max_angular_rps) <= 0:
        raise ValueError("motion limits must be positive")
    path = find_path(grid, start, goal)
    metrics = path_metrics(path)
    if metrics is None:
        return None
    distance, rotation = metrics
    if distance <= 1e-9:
        travel_s = 0.0
    else:
        # A* path length plus a conservative start/stop ramp; turns are charged
        # explicitly because the differential-drive model can rotate in place.
        travel_s = distance / max_speed_mps + max_speed_mps / max_accel_mps2
    travel_s += rotation / max_angular_rps
    return distance, rotation, travel_s, tuple(path)


def estimate_mission(grid, legs, config: BatteryConfig, *, stationary_time_s: float,
                     max_speed_mps: float = 0.35, max_accel_mps2: float = 0.8,
                     max_angular_rps: float = 1.5) -> MissionEstimate | None:
    """Estimate all legs; ``legs`` must include the final charger/base return leg."""
    if not math.isfinite(stationary_time_s) or stationary_time_s < 0:
        raise ValueError("stationary time must be finite and nonnegative")
    distance = rotation = travel_time = 0.0
    paths = []
    for start, goal in legs:
        metrics = estimate_leg(grid, start, goal, max_speed_mps=max_speed_mps,
                               max_accel_mps2=max_accel_mps2,
                               max_angular_rps=max_angular_rps)
        if metrics is None:
            return None
        leg_distance, leg_rotation, leg_time, path = metrics
        distance += leg_distance
        rotation += leg_rotation
        travel_time += leg_time
        paths.append(path)
    energy = (config.movement_wh_per_m * distance
              + config.rotation_wh_per_rad * rotation
              + config.idle_power_w * stationary_time_s / 3600.0)
    return MissionEstimate(distance, rotation, travel_time, stationary_time_s,
                           energy, tuple(paths))
