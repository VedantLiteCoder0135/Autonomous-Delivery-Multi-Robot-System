"""Stage 7 Part 2: turn simulated 2D LiDAR ranges into DWA obstacles.

This adapter uses only scan ranges, the robot's estimated pose and timestamps.
It does not read the true positions or velocities of moving agents.

The input scan is the 360-degree, equally spaced, counter-clockwise output of
``aifr.perception.sensors.lidar_scan``. A finite return shorter than max_range
is treated as a hit. Neighboring returns form conservative circular obstacle
estimates; nearest-neighbor tracking supplies approximate velocities.

IMPORTANT LIMITATIONS:
- The tracker does not classify static versus dynamic objects. The Part 2 demo
  scans simulated moving agents only; permanent furniture remains in the
  Stage 4 occupancy grid. A full-scene scan needs static-map filtering later.
- The maximum obstacle radius assumes pedestrian-sized moving objects; larger
  objects are NOT supported without changing this configuration and detector.
- Sparse scans can miss thin obstacles. These circles are not a safety
  certification; the Stage 4 footprint checker remains essential.
- Robot pose must be in the same world frame as the Stage 4 grid.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from aifr.navigation.dwa import MovingObstacle
from aifr.navigation.kinematics import Pose


@dataclass(frozen=True)
class LiDARAdapterConfig:
    max_range: float = 4.0
    cluster_distance: float = 0.28
    minimum_radius: float = 0.16
    maximum_radius: float = 0.24  # demo assumes pedestrian-sized obstacles
    measurement_padding: float = 0.04
    hit_epsilon: float = 0.025
    association_distance: float = 0.38
    max_track_age: float = 0.20
    velocity_smoothing: float = 0.25
    max_estimated_speed: float = 1.0

    def __post_init__(self) -> None:
        values = (self.max_range, self.cluster_distance, self.minimum_radius,
                  self.maximum_radius, self.measurement_padding, self.hit_epsilon,
                  self.association_distance, self.max_track_age,
                  self.max_estimated_speed)
        if any(not math.isfinite(x) or x <= 0 for x in values):
            raise ValueError("LiDAR ranges, distances and radii must be positive and finite")
        if self.maximum_radius < self.minimum_radius:
            raise ValueError("maximum_radius must be >= minimum_radius")
        if self.hit_epsilon >= self.max_range:
            raise ValueError("hit_epsilon must be smaller than max_range")
        if not 0 < self.velocity_smoothing <= 1:
            raise ValueError("velocity_smoothing must be in (0, 1]")


@dataclass(frozen=True)
class ScanCluster:
    """One group of adjacent LiDAR hits, in world coordinates."""
    x: float
    y: float
    radius: float
    hit_count: int


@dataclass
class _Track:
    x: float
    y: float
    radius: float
    vx: float
    vy: float
    last_seen: float


def _hit_points(pose: Pose, ranges: Sequence[float],
                cfg: LiDARAdapterConfig) -> list[tuple[float, float] | None]:
    if not ranges:
        raise ValueError("LiDAR scan must contain at least one range")
    if not all(math.isfinite(x) for x in (pose.x, pose.y, pose.theta)):
        raise ValueError("Robot pose must be finite")
    n = len(ranges)
    points: list[tuple[float, float] | None] = []
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance < 0 or distance > cfg.max_range + 1e-9:
            raise ValueError("LiDAR ranges must be finite and in [0, max_range]")
        if distance >= cfg.max_range - cfg.hit_epsilon:
            points.append(None)
        else:
            theta = pose.theta + 2.0 * math.pi * i / n
            points.append((pose.x + distance * math.cos(theta),
                           pose.y + distance * math.sin(theta)))
    return points


def scan_clusters(pose: Pose, ranges: Sequence[float],
                  config: LiDARAdapterConfig | None = None) -> tuple[ScanCluster, ...]:
    """Cluster adjacent scan hits, including a cluster across the 0/360 seam."""
    cfg = config or LiDARAdapterConfig()
    points = _hit_points(pose, ranges, cfg)
    groups: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for point in points:
        if point is None:
            if current:
                groups.append(current)
                current = []
            continue
        if current and math.dist(current[-1], point) > cfg.cluster_distance:
            groups.append(current)
            current = []
        current.append(point)
    if current:
        groups.append(current)

    # A full 360-degree scan wraps: first and last rays are neighbors.
    if (len(groups) > 1 and points[0] is not None and points[-1] is not None
            and math.dist(groups[-1][-1], groups[0][0]) <= cfg.cluster_distance):
        merged = groups[-1] + groups[0]
        groups = [merged] + groups[1:-1]

    observations: list[ScanCluster] = []
    for group in groups:
        x = sum(p[0] for p in group) / len(group)
        y = sum(p[1] for p in group) / len(group)
        radius = min(cfg.maximum_radius,
                     max(cfg.minimum_radius,
                         max(math.hypot(px - x, py - y) for px, py in group)
                         + cfg.measurement_padding))
        # A scan measures the *visible surface*, not the obstacle center.
        # For pedestrian-sized targets, shift the center estimate away from
        # the sensor by the configured minimum radius. Keep the additional
        # measurement padding in the reported radius.
        bearing_x, bearing_y = x - pose.x, y - pose.y
        bearing_norm = math.hypot(bearing_x, bearing_y)
        if bearing_norm > 1e-12:
            x += cfg.minimum_radius * bearing_x / bearing_norm
            y += cfg.minimum_radius * bearing_y / bearing_norm
        observations.append(ScanCluster(x, y, radius, len(group)))
    return tuple(observations)


class LiDARObstacleTracker:
    """Associates successive LiDAR clusters and predicts short-term motion.

    A new obstacle begins with zero estimated velocity. Subsequent observations
    estimate velocity from displacement, capped and smoothed for noisy scans.
    Tracks not seen for ``max_track_age`` seconds are removed.
    """

    def __init__(self, config: LiDARAdapterConfig | None = None) -> None:
        self.config = config or LiDARAdapterConfig()
        self._tracks: list[_Track] = []
        self._last_time: float | None = None

    def reset(self) -> None:
        self._tracks.clear()
        self._last_time = None

    def observe(self, pose: Pose, ranges: Sequence[float],
                timestamp: float) -> tuple[MovingObstacle, ...]:
        if not math.isfinite(timestamp):
            raise ValueError("Scan timestamp must be finite")
        if self._last_time is not None and timestamp <= self._last_time:
            raise ValueError("Scan timestamps must be strictly increasing")
        self._last_time = timestamp
        cfg = self.config
        observations = scan_clusters(pose, ranges, cfg)
        old = [t for t in self._tracks if timestamp - t.last_seen <= cfg.max_track_age]

        # One-to-one greedy matching using predicted centers. Restrict to
        # plausible displacement so a different pedestrian isn't confused
        # with an existing one. This is a simple prototype tracker.
        candidates: list[tuple[float, int, int]] = []
        for i, track in enumerate(old):
            dt = timestamp - track.last_seen
            for j, obs in enumerate(observations):
                predicted_x = track.x + track.vx * dt
                predicted_y = track.y + track.vy * dt
                distance = math.hypot(obs.x - predicted_x, obs.y - predicted_y)
                if distance <= cfg.association_distance:
                    candidates.append((distance, i, j))
        candidates.sort()
        matched_old: set[int] = set()
        matched_obs: set[int] = set()
        new_tracks: list[_Track] = []
        for _, i, j in candidates:
            if i in matched_old or j in matched_obs:
                continue
            matched_old.add(i)
            matched_obs.add(j)
            track, obs = old[i], observations[j]
            dt = timestamp - track.last_seen
            raw_vx = (obs.x - track.x) / dt
            raw_vy = (obs.y - track.y) / dt
            raw_speed = math.hypot(raw_vx, raw_vy)
            if raw_speed > cfg.max_estimated_speed:
                scale = cfg.max_estimated_speed / raw_speed
                raw_vx *= scale
                raw_vy *= scale
            a = cfg.velocity_smoothing
            new_tracks.append(_Track(obs.x, obs.y,
                                     max(obs.radius, track.radius),
                                     (1 - a) * track.vx + a * raw_vx,
                                     (1 - a) * track.vy + a * raw_vy,
                                     timestamp))
        for j, obs in enumerate(observations):
            if j not in matched_obs:
                new_tracks.append(_Track(obs.x, obs.y, obs.radius, 0.0, 0.0, timestamp))
        for i, track in enumerate(old):
            if i not in matched_old:
                new_tracks.append(track)
        self._tracks = new_tracks
        return tuple(
            MovingObstacle(t.x + t.vx * (timestamp - t.last_seen),
                           t.y + t.vy * (timestamp - t.last_seen),
                           t.radius, t.vx, t.vy)
            for t in self._tracks
        )
