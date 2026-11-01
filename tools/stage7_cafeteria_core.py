"""Stage 7 Part 4: reproduce a DWA mission using Stage 1–6 Blender exports.

Pure Python: Blender is used only to extract the authoritative scene (earlier
Stage 1–6 bridge) and to display the resulting closed-loop simulation trace.

This is deliberately opt-in. It does not modify the Stage 2 scheduler, the
Stage 5 delivery runner, or the approved Blender scene. No success is reported
unless both mission legs complete and all recorded footprints remain clear.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

from aifr.environment.environment_mapper import (
    CafeEnvironment, ChairInfo, CounterInfo, EntranceInfo, FloorRegion,
    PillarInfo, Rect2D, TableInfo, WallSegment,
)
from aifr.navigation.astar import find_path
from aifr.navigation.collision_checker import CollisionChecker
from aifr.navigation.dwa import DWAConfig, MovingObstacle
from aifr.navigation.kinematics import Pose
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.perception.sensors import SensorSuite
from aifr.simulation.dwa_delivery import DWAIntegratedRobot, TravelResult


@dataclass(frozen=True)
class MissionInputs:
    scene: Path
    spawn: Pose
    pickup: tuple[float, float]
    delivery: tuple[float, float]
    labels: dict[str, str]
    grid: OccupancyGrid
    environment: CafeEnvironment


@dataclass(frozen=True)
class PedestrianScenario:
    """Small sinusoidal walk along a verified free segment near the route."""
    x: float
    y: float
    direction_x: float
    direction_y: float
    radius: float = 0.16
    amplitude: float = 0.16
    frequency_rad_s: float = 0.28

    def at(self, time_s: float) -> tuple[MovingObstacle, ...]:
        offset = self.amplitude * math.sin(self.frequency_rad_s * time_s)
        velocity = self.amplitude * self.frequency_rad_s * math.cos(self.frequency_rad_s * time_s)
        return (MovingObstacle(
            self.x + self.direction_x * offset,
            self.y + self.direction_y * offset,
            self.radius,
            self.direction_x * velocity,
            self.direction_y * velocity,
        ),)


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}. Run the Stages 1–6 Blender bridge first.")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return data


def _rect(d: dict[str, Any]) -> Rect2D:
    return Rect2D(*(float(d[k]) for k in ("min_x", "max_x", "min_y", "max_y")))


def _environment_from_json(data: dict[str, Any]) -> CafeEnvironment:
    """Deserialize the actual Stage 3 geometry; never replace it with defaults."""
    return CafeEnvironment(
        floor_regions=[FloorRegion(str(v["name"]), _rect(v["rect"]))
                       for v in data["floor_regions"]],
        walls=[WallSegment(str(v["name"]), _rect(v["rect"]))
               for v in data["walls"]],
        pillars=[PillarInfo(str(v["name"]), _rect(v["rect"]))
                 for v in data["pillars"]],
        tables=[TableInfo(str(v["table_id"]), _rect(v["rect"]))
                for v in data["tables"]],
        chairs=[ChairInfo(str(v["name"]), str(v["parent_table"]), _rect(v["rect"]))
                for v in data["chairs"]],
        counters=[CounterInfo(str(v["name"]), _rect(v["rect"]), bool(v["is_food_station"]))
                  for v in data["counters"]],
        entrances=[EntranceInfo(**v) for v in data.get("entrances", [])],
        unknown_objects=[str(x) for x in data.get("unknown_objects", [])],
        skipped_objects=[str(x) for x in data.get("skipped_objects", [])],
    )


def _grid_from_json(data: dict[str, Any]) -> OccupancyGrid:
    width, height = int(data["width"]), int(data["height"])
    cells = tuple(tuple(int(c) for c in row) for row in data["cells"])
    if width <= 0 or height <= 0 or len(cells) != height or any(len(r) != width for r in cells):
        raise ValueError("Stage 4 occupancy grid has invalid dimensions")
    if any(c not in (0, 1) for row in cells for c in row):
        raise ValueError("Stage 4 occupancy grid has invalid cell values")
    origin = data["origin"]
    grid = OccupancyGrid(float(origin[0]), float(origin[1]), float(data["resolution"]),
                         width, height, cells)
    if grid.resolution <= 0 or not all(math.isfinite(v) for v in
                                      (grid.origin_x, grid.origin_y, grid.resolution)):
        raise ValueError("Stage 4 grid origin/resolution is invalid")
    return grid


def load_mission(project: Path) -> MissionInputs:
    project = project.expanduser().resolve()
    outputs = project / "outputs"
    stage36 = _read_json(outputs / "stage36_report.json")
    if stage36.get("success") is not True or stage36.get("mission", {}).get("completed") is not True:
        raise ValueError("Stages 1–6 bridge did not report a completed mission")
    if stage36["mission"].get("trace_footprint_clear") is not True:
        raise ValueError("Stage 1–6 mission did not pass its footprint check")
    scene = project / "AIFR_Simulation_Stage3.blend"
    if not scene.is_file():
        raise FileNotFoundError(f"Approved Blender scene not found: {scene}")
    # Prevent silently replaying old geometry after moving or editing the scene.
    reported_scene = Path(str(stage36.get("scene", ""))).expanduser().resolve()
    if reported_scene != scene.resolve():
        raise ValueError("Stage 1–6 report points to a different scene. Rerun the Blender bridge.")
    if scene.stat().st_mtime > (outputs / "stage36_report.json").stat().st_mtime + 1.0:
        raise ValueError("Scene was modified after Stage 1–6 extraction. Rerun the Blender bridge.")
    env = _environment_from_json(_read_json(outputs / "stage3_environment.json"))
    grid = _grid_from_json(_read_json(outputs / "stage4_occupancy_grid.json"))
    mission = stage36["mission"]
    spawn_data, pickup_data, drop_data = mission["spawn"], mission["pickup"], mission["drop"]
    spawn = Pose(float(spawn_data["x"]), float(spawn_data["y"]), float(spawn_data["heading"]))
    pickup = (float(pickup_data["x"]), float(pickup_data["y"]))
    drop = (float(drop_data["x"]), float(drop_data["y"]))
    checker = CollisionChecker(env)
    for name, pose in (("spawn", spawn),
                       ("pickup", Pose(*pickup, float(pickup_data["heading"]))),
                       ("delivery", Pose(*drop, float(drop_data["heading"])))):
        if not grid.is_free(*grid.world_to_grid(pose.x, pose.y)):
            raise ValueError(f"{name} from Stage 1–6 report is blocked on the extracted grid")
        if not checker.is_pose_free(pose.x, pose.y, pose.theta):
            raise ValueError(f"{name} from Stage 1–6 report fails exact footprint validation")
    return MissionInputs(scene, spawn, pickup, drop,
                         {"spawn": str(spawn_data["label"]),
                          "pickup": str(pickup_data["label"]),
                          "delivery": str(drop_data["label"])}, grid, env)


def _pedestrian_free(grid: OccupancyGrid, x: float, y: float, radius: float) -> bool:
    # The Stage 4 grid already has a conservative robot envelope inflated.
    # Extra probes ensure the animated pedestrian never crosses blocked cells.
    for i in range(16):
        a = i * math.tau / 16
        px, py = x + radius * math.cos(a), y + radius * math.sin(a)
        if not grid.is_free(*grid.world_to_grid(px, py)):
            return False
    return grid.is_free(*grid.world_to_grid(x, y))


def select_pedestrian(grid: OccupancyGrid,
                      route: list[tuple[float, float]],
                      avoid_route: list[tuple[float, float]] | None = None) -> PedestrianScenario | None:
    """Place a walking pedestrian beside an actual A* route, never on furniture.

    Fails explicitly if the real cafeteria has no safe nearby placement. The
    obstacle is not fabricated at a hard-coded cafeteria coordinate.
    """
    if len(route) < 8:
        return None
    indices = sorted(set(max(3, min(len(route)-4, round((len(route)-1)*fraction)))
                         for fraction in (0.75, 0.65, 0.85, 0.5, 0.35)))
    indices.sort(key=lambda i: abs(i - 0.75 * (len(route)-1)))
    for index in indices:
        p0, p1 = route[index-3], route[index+3]
        dx, dy = p1[0]-p0[0], p1[1]-p0[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        dx, dy = dx/length, dy/length
        for distance in (0.95, 1.10, 1.25, 1.40, 0.85):
            for side in (1, -1):
                x = route[index][0] - dy * distance * side
                y = route[index][1] + dx * distance * side
                candidate = PedestrianScenario(x, y, dx, dy)
                if any(not _pedestrian_free(grid, candidate.at(t)[0].x,
                                            candidate.at(t)[0].y, candidate.radius)
                       for t in (0, 3, 6, 9, 12, 15, 18, 21, 24)):
                    continue
                # Keep a clear corridor from the nominal A* route at all phases.
                min_dist = min(math.hypot(candidate.at(t)[0].x-qx,
                                          candidate.at(t)[0].y-qy)
                               for t in (0, 6, 12, 18, 24)
                               for qx, qy in route)
                if min_dist < 0.55:
                    continue
                if avoid_route and min(math.hypot(candidate.at(t)[0].x-qx,
                                                 candidate.at(t)[0].y-qy)
                                       for t in (0, 6, 12, 18, 24)
                                       for qx, qy in avoid_route) < 1.05:
                    continue
                return candidate
    return None


def _pose_dict(pose: Pose) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "theta": pose.theta}


def _result_dict(result: TravelResult | None) -> dict[str, Any] | None:
    if result is None:
        return None
    out = asdict(result)
    if not math.isfinite(out["min_dynamic_clearance"]):
        out["min_dynamic_clearance"] = None
    return out


def run_cafeteria(project: Path, *, with_pedestrian: bool = True,
                  max_ticks_per_leg: int = 6000, noise_sigma: float = 0.0) -> dict[str, Any]:
    """Run a real-map closed-loop Python simulation; write trace + report.

    Failure is recorded in the report and returned, never silently replaced by
    the Stage 5 playback trace. Callers should exit nonzero when success=False.
    """
    if max_ticks_per_leg < 1 or not math.isfinite(noise_sigma) or noise_sigma < 0:
        raise ValueError("Invalid max_ticks_per_leg or noise_sigma")
    project = project.expanduser().resolve()
    mission = load_mission(project)
    outputs = project / "outputs"
    outputs.mkdir(parents=True, exist_ok=True)
    path1 = find_path(mission.grid, (mission.spawn.x, mission.spawn.y), mission.pickup)
    path2 = find_path(mission.grid, mission.pickup, mission.delivery)
    if not path1 or not path2:
        raise ValueError("Extracted grid has no route for the validated Stage 1–6 mission")
    pedestrian = select_pedestrian(mission.grid, path2, path1) if with_pedestrian else None
    if with_pedestrian and pedestrian is None:
        raise ValueError("No safe pedestrian location found beside the actual cafeteria route. "
                         "Inspect the map or run --no-pedestrian for a static baseline.")
    robot = DWAIntegratedRobot(
        mission.grid, mission.spawn,
        config=DWAConfig(),
        sensors=SensorSuite(seed=7, velocity_sigma=noise_sigma,
                            yaw_sigma=noise_sigma, position_sigma=noise_sigma),
        checker=CollisionChecker(mission.environment),
    )
    provider = pedestrian.at if pedestrian else None
    first, second = robot.deliver(mission.pickup, mission.delivery,
                                  agents_at=provider, max_ticks_per_leg=max_ticks_per_leg)
    exact_clear = all(robot.checker.is_pose_free(p.x, p.y, p.theta) for p in robot.trace)
    lidar_ticks = first.lidar_detection_ticks + (second.lidar_detection_ticks if second else 0)
    positive_clearance = all(sample.actual_clearance > 0 for sample in robot.controls)
    success = (first.reached and second is not None and second.reached and exact_clear
               and positive_clearance and (not with_pedestrian or lidar_ticks > 0))
    baseline_comparison: dict[str, Any] | None = None
    if with_pedestrian and success:
        # Compare against an identical no-pedestrian run. Detection alone does
        # not prove that the obstacle influenced navigation decisions.
        baseline = DWAIntegratedRobot(
            mission.grid, mission.spawn, config=DWAConfig(),
            sensors=SensorSuite(seed=7, velocity_sigma=noise_sigma,
                                yaw_sigma=noise_sigma, position_sigma=noise_sigma),
            checker=CollisionChecker(mission.environment),
        )
        base_first, base_second = baseline.deliver(
            mission.pickup, mission.delivery, max_ticks_per_leg=max_ticks_per_leg)
        base_reached = base_first.reached and base_second is not None and base_second.reached
        if base_reached:
            max_deviation = max((math.hypot(a.x-b.x, a.y-b.y)
                                 for a, b in zip(robot.trace, baseline.trace)), default=0.0)
            different_commands = sum(
                abs(a.linear-b.linear) > 1e-7 or abs(a.angular-b.angular) > 1e-7
                for a, b in zip(robot.controls, baseline.controls)
            )
            baseline_comparison = {
                "baseline_reached": True,
                "max_position_deviation_m": round(max_deviation, 6),
                "different_command_ticks": different_commands,
                "lidar_affected_motion": bool(max_deviation > .02 and different_commands > 0),
                "note": "Control differences demonstrate response, not that a collision was inevitable without DWA.",
            }
        else:
            baseline_comparison = {"baseline_reached": False,
                                   "lidar_affected_motion": None,
                                   "note": "No baseline comparison: no-pedestrian run failed."}
    warnings = [
        "Closed-loop DWA runs in Python; Blender displays recorded keyframes, not live physics.",
        "LiDAR sees simulated pedestrian AABBs, not static-map occlusion; not a safety-certified controller.",
        "Stage 2 scheduling is not re-executed here; mission waypoints come from the verified Stage 1–6 run.",
    ]
    if noise_sigma == 0:
        warnings.append("Ideal odometry and XY landmarks used to isolate navigation behavior.")
    if baseline_comparison and baseline_comparison["lidar_affected_motion"] is False:
        warnings.append("LiDAR detected the pedestrian, but no meaningful motion change was observed versus baseline.")
    if not exact_clear:
        warnings.append("Recorded robot footprint intersects static cafeteria geometry.")
    if with_pedestrian and lidar_ticks == 0:
        warnings.append("Pedestrian was not detected by LiDAR; dynamic avoidance is not demonstrated.")
    report: dict[str, Any] = {
        "success": bool(success),
        "scene": str(mission.scene),
        "simulation": "Python 20 Hz closed-loop DWA; offline Blender playback",
        "source": "Stage 1–6 extracted environment, occupancy grid and verified waypoints",
        "mission_labels": mission.labels,
        "pickup": _result_dict(first),
        "delivery": _result_dict(second),
        "footprint_clear": exact_clear,
        "dynamic_clearance_positive": positive_clearance,
        "lidar_detection_ticks": lidar_ticks,
        "pedestrian_enabled": with_pedestrian,
        "baseline_comparison": baseline_comparison,
        "pedestrian": asdict(pedestrian) if pedestrian else None,
        "trace_points": len(robot.trace),
        "dt_s": robot.config.dt,
        "warnings": warnings,
    }
    if not success:
        report["failure_reason"] = (first.reason if not first.reached else
                                    second.reason if second is not None and not second.reached else
                                    "; ".join(warnings[-2:]))
    trace = {
        "source_scene": str(mission.scene),
        "dt_s": robot.config.dt,
        "mission": {"spawn": _pose_dict(mission.spawn),
                    "pickup": list(mission.pickup),
                    "delivery": list(mission.delivery),
                    "labels": mission.labels},
        "global_paths": {"to_pickup": [list(p) for p in path1],
                         "to_delivery": [list(p) for p in path2]},
        "samples": [],
    }
    for i, pose in enumerate(robot.trace):
        t = i * robot.config.dt
        control = robot.controls[i-1] if i > 0 else None
        person = pedestrian.at(t)[0] if pedestrian else None
        trace["samples"].append({
            "time_s": round(t, 8),
            "phase": "pickup" if i <= first.ticks else "delivery",
            "robot": _pose_dict(pose),
            "ekf": _pose_dict(control.estimated_pose) if control else _pose_dict(mission.spawn),
            "linear": control.linear if control else 0.0,
            "angular": control.angular if control else 0.0,
            "tracked_obstacles": control.tracked_obstacles if control else 0,
            "pedestrian": {"x": person.x, "y": person.y, "radius": person.radius} if person else None,
        })
    (outputs / "stage7_cafeteria_trace.json").write_text(
        json.dumps(trace, indent=2, allow_nan=False), encoding="utf-8")
    (outputs / "stage7_cafeteria_report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    return report
