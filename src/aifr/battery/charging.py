"""Exclusive charger ownership and conservative V14 base-slot auditing."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from aifr.navigation.astar import find_path
from aifr.navigation.kinematics import Pose


@dataclass(frozen=True, slots=True)
class ChargingStation:
    station_id: str
    pose: Pose
    capacity: int = 1

    def __post_init__(self) -> None:
        if not self.station_id or self.capacity < 1:
            raise ValueError("station id and positive capacity are required")


def audit_base_chargers(env, grid, checker, bases, *, safety_margin: float = 0.18):
    """Use only existing, validated base slots; never invent charger coordinates.

    OccupancyGrid is already inflated by the conservative robot footprint and
    clearance. Static approach/departure and entrance checks happen here.
    The reference has no pedestrian route graph, so active pedestrian clearance
    is enforced again at docking time and the report calls out that limitation.
    """
    entrances = tuple(env.entrances)
    stations: list[ChargingStation] = []
    rejected: dict[str, list[str]] = {}
    pad_radius = math.hypot(0.20, 0.18) + safety_margin
    for index, base in enumerate(bases, 1):
        name = f"BASE_{index:02d}"
        reasons: list[str] = []
        col, row = grid.world_to_grid(base.x, base.y)
        if not grid.is_free(col, row) or not checker.is_pose_free(base.x, base.y, base.theta):
            reasons.append("dock footprint is not statically clear")
        if entrances and min(math.dist((base.x, base.y), (e.center_x, e.center_y))
                             for e in entrances) < pad_radius + 0.25:
            reasons.append("dock is too close to a verified entrance")

        approach_ok = False
        departure_ok = False
        if not reasons:
            # Try a ring of real, footprint-clear approach poses. A* paths must
            # exist in both directions; the robot can dock without teleporting.
            for k in range(16):
                angle = 2 * math.pi * k / 16
                for offset in (0.65, 0.85, 1.05):
                    point = (base.x + offset * math.cos(angle),
                             base.y + offset * math.sin(angle))
                    pc, pr = grid.world_to_grid(*point)
                    if (not grid.is_free(pc, pr)
                            or not checker.is_pose_free(point[0], point[1], base.theta)):
                        continue
                    if find_path(grid, point, (base.x, base.y)):
                        approach_ok = True
                    if find_path(grid, (base.x, base.y), point):
                        departure_ok = True
                    if approach_ok and departure_ok:
                        break
                if approach_ok and departure_ok:
                    break
            if not approach_ok or not departure_ok:
                reasons.append("no footprint-clear A* approach and departure pair")

        if reasons:
            rejected[name] = reasons
        else:
            stations.append(ChargingStation(name, base, capacity=1))

    # Ensure distinct pads are not closer than two inflated robot envelopes.
    spacing = 2 * pad_radius
    for i, first in enumerate(tuple(stations)):
        for second in tuple(stations)[i + 1:]:
            if math.dist((first.pose.x, first.pose.y), (second.pose.x, second.pose.y)) < spacing:
                rejected.setdefault(first.station_id, []).append(
                    f"overlaps charger envelope of {second.station_id}")
                rejected.setdefault(second.station_id, []).append(
                    f"overlaps charger envelope of {first.station_id}")
    stations = [s for s in stations if s.station_id not in rejected]
    report = {
        "enabled": bool(stations),
        "candidate_source": "existing validated robot base slots",
        "stations": [{"station_id": s.station_id,
                      "position": [round(s.pose.x, 3), round(s.pose.y, 3)],
                      "capacity": s.capacity} for s in stations],
        "rejected": rejected,
        "obstacle_inflation": "inherited from 0.1 m Stage 4 occupancy grid",
        "entrance_clearance_checked": True,
        "static_approach_departure_checked": True,
        "pedestrian_routes_available": False,
        "pedestrian_clearance": "checked dynamically before docking; no static path graph exists",
        "maximum_simultaneous_robots": sum(s.capacity for s in stations),
    }
    return tuple(stations), report


class ChargingManager:
    """FIFO reservations with an explicit post-charge undocking release."""

    def __init__(self, stations) -> None:
        self.stations = {station.station_id: station for station in stations}
        self.owners: dict[str, set[int]] = {key: set() for key in self.stations}
        self.robot_station: dict[int, str] = {}
        self.queue: deque[int] = deque()
        self.phase: dict[int, str] = {}
        self.failed_stations: set[str] = set()

    def register_station(self, station: ChargingStation) -> None:
        existing = self.stations.get(station.station_id)
        if existing is not None and existing != station:
            raise ValueError(f"charger {station.station_id} is already registered differently")
        if existing is None:
            self.stations[station.station_id] = station
            self.owners[station.station_id] = set()

    def request(self, robot_id: int, station_id: str | None = None) -> str:
        robot_id = int(robot_id)
        if not self.stations:
            self.phase[robot_id] = "UNAVAILABLE_NO_CHARGER"
            return self.phase[robot_id]
        if robot_id in self.robot_station:
            if self.phase.get(robot_id) == "CHARGE_COMPLETE":
                station_id = self.robot_station[robot_id]
                self.phase[robot_id] = ("CHARGER_FAILED" if station_id in self.failed_stations
                                        else "DOCKING")
            return self.phase.get(robot_id, "RESERVED")
        if robot_id not in self.queue:
            self.queue.append(robot_id)
        candidates = ([station_id] if station_id in self.stations else [])
        candidates.extend(key for key in self.stations if key not in candidates)
        for candidate in candidates:
            if candidate in self.failed_stations:
                continue
            station = self.stations[candidate]
            if len(self.owners[candidate]) < station.capacity:
                self.owners[candidate].add(robot_id)
                self.robot_station[robot_id] = candidate
                self.queue.remove(robot_id)
                self.phase[robot_id] = "DOCKING"
                return self.phase[robot_id]
        self.phase[robot_id] = "WAITING_FOR_CHARGER"
        return self.phase[robot_id]

    def mark_docked(self, robot_id: int) -> None:
        if robot_id not in self.robot_station:
            raise ValueError("robot has no charger reservation")
        if self.phase.get(robot_id) != "DOCKING":
            raise ValueError("robot is not docking")
        self.phase[robot_id] = "CHARGING"

    def mark_complete(self, robot_id: int) -> None:
        if self.phase.get(robot_id) != "CHARGING":
            raise ValueError("only a charging robot can complete charging")
        self.phase[robot_id] = "CHARGE_COMPLETE"

    def undock(self, robot_id: int) -> None:
        """Release ownership only after the robot has physically left the pad."""
        if self.phase.get(robot_id) != "CHARGE_COMPLETE":
            raise ValueError("charge must be complete before undocking")
        station_id = self.robot_station.pop(robot_id)
        self.owners[station_id].remove(robot_id)
        self.phase[robot_id] = "IDLE"
        self._serve_queue(station_id)

    def fail_station(self, station_id: str) -> None:
        if station_id not in self.stations:
            raise KeyError(f"unknown charging station {station_id}")
        self.failed_stations.add(station_id)
        for robot_id in self.owners[station_id]:
            if self.phase.get(robot_id) != "CHARGE_COMPLETE":
                self.phase[robot_id] = "CHARGER_FAILED"
        self._serve_queue(station_id)

    def undock_failed(self, robot_id: int, *, safely_undocked: bool) -> None:
        """Release a failed pad only after external motion checks confirm clearance."""
        if not safely_undocked:
            raise ValueError("failed charger reservation requires verified safe undocking")
        if self.phase.get(robot_id) != "CHARGER_FAILED":
            raise ValueError("robot is not reserved at a failed charger")
        station_id = self.robot_station.pop(robot_id)
        self.owners[station_id].remove(robot_id)
        self.phase[robot_id] = "FAULT"
        self._serve_queue(station_id)

    def cancel(self, robot_id: int) -> None:
        """Release only a not-yet-docked reservation, e.g. charger failure."""
        if self.phase.get(robot_id) in ("CHARGING", "CHARGE_COMPLETE", "CHARGER_FAILED"):
            raise ValueError("cannot cancel a docked reservation without safe undocking")
        if robot_id in self.queue:
            self.queue.remove(robot_id)
        station_id = self.robot_station.pop(robot_id, None)
        if station_id is not None:
            self.owners[station_id].discard(robot_id)
            self._serve_queue(station_id)
        self.phase[robot_id] = "CHARGER_FAILED"

    def _serve_queue(self, station_id: str) -> None:
        if station_id in self.failed_stations:
            return
        station = self.stations[station_id]
        while self.queue and len(self.owners[station_id]) < station.capacity:
            robot_id = self.queue.popleft()
            if robot_id in self.robot_station:
                continue
            self.owners[station_id].add(robot_id)
            self.robot_station[robot_id] = station_id
            self.phase[robot_id] = "DOCKING"

    def owns(self, robot_id: int) -> bool:
        return int(robot_id) in self.robot_station

    def telemetry(self) -> dict:
        return {"stations": {key: sorted(value) for key, value in self.owners.items()},
                "queue": list(self.queue),
                "robot_phase": dict(sorted(self.phase.items())),
                "failed_stations": sorted(self.failed_stations)}
