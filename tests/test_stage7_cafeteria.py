"""Stage 7 Part 4 pure-Python tests; Blender executable is not required."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import pytest

from aifr.environment.environment_mapper import CafeEnvironment, FloorRegion, Rect2D
from aifr.navigation.astar import find_path
from aifr.navigation.occupancy_grid import OccupancyGrid
from stage7_cafeteria_core import (
    load_mission, run_cafeteria, select_pedestrian,
)


def make_project(tmp_path: Path, *, completed: bool = True) -> Path:
    scene = tmp_path / "AIFR_Simulation_Stage3.blend"
    scene.write_bytes(b"test fixture; not a real .blend")
    outputs = tmp_path / "outputs"
    outputs.mkdir()
    env = CafeEnvironment(floor_regions=[FloorRegion("test_floor", Rect2D(-2, 4, -2, 4))])
    grid = OccupancyGrid.from_environment(env, resolution=.1)
    (outputs / "stage3_environment.json").write_text(json.dumps(env.to_dict()))
    (outputs / "stage4_occupancy_grid.json").write_text(json.dumps(grid.to_dict()))
    (outputs / "stage36_report.json").write_text(json.dumps({
        "scene": str(scene.resolve()), "success": True,
        "mission": {
            "completed": completed, "trace_footprint_clear": True,
            "spawn": {"x": 0, "y": 0, "heading": 0, "label": "spawn_01"},
            "pickup": {"x": 1.4, "y": 0, "heading": 0, "label": "pickup_SANDWICH"},
            "drop": {"x": 1.4, "y": 1.4, "heading": 1.57, "label": "delivery_T11_S"},
        },
    }))
    return tmp_path


def test_reads_authoritative_export_and_waypoints(tmp_path):
    project = make_project(tmp_path)
    mission = load_mission(project)
    assert mission.labels["delivery"] == "delivery_T11_S"
    assert mission.grid.is_free(*mission.grid.world_to_grid(*mission.pickup))
    assert mission.scene.name == "AIFR_Simulation_Stage3.blend"


def test_select_pedestrian_only_in_verified_free_space(tmp_path):
    mission = load_mission(make_project(tmp_path))
    route = find_path(mission.grid, mission.pickup, mission.delivery)
    person = select_pedestrian(mission.grid, route)
    assert person is not None
    for t in range(0, 25, 3):
        pos = person.at(float(t))[0]
        assert mission.grid.is_free(*mission.grid.world_to_grid(pos.x, pos.y))
        assert math.hypot(pos.vx, pos.vy) < .06


def test_static_baseline_produces_trace_and_report(tmp_path):
    project = make_project(tmp_path)
    report = run_cafeteria(project, with_pedestrian=False, max_ticks_per_leg=700)
    assert report["success"], report
    assert report["footprint_clear"]
    assert report["pedestrian_enabled"] is False
    trace = json.loads((project / "outputs/stage7_cafeteria_trace.json").read_text())
    assert len(trace["samples"]) == report["trace_points"]
    assert trace["samples"][-1]["robot"]["y"] > 1.2
    assert all(s["pedestrian"] is None for s in trace["samples"])


def test_pedestrian_is_sensed_in_real_map_runner(tmp_path):
    project = make_project(tmp_path)
    report = run_cafeteria(project, with_pedestrian=True, max_ticks_per_leg=700)
    assert report["success"], report
    assert report["lidar_detection_ticks"] > 0
    assert report["dynamic_clearance_positive"]
    assert report["baseline_comparison"]["baseline_reached"]
    assert report["baseline_comparison"]["lidar_affected_motion"]
    trace = json.loads((project / "outputs/stage7_cafeteria_trace.json").read_text())
    assert all(s["pedestrian"] for s in trace["samples"])


def test_failure_does_not_get_reported_as_success(tmp_path):
    project = make_project(tmp_path)
    report = run_cafeteria(project, with_pedestrian=False, max_ticks_per_leg=1)
    assert not report["success"]
    assert "failure_reason" in report
    saved = json.loads((project / "outputs/stage7_cafeteria_report.json").read_text())
    assert saved["success"] is False


def test_rejects_unfinished_stage36(tmp_path):
    project = make_project(tmp_path, completed=False)
    with pytest.raises(ValueError, match="completed mission"):
        load_mission(project)


def test_rejects_changed_scene(tmp_path):
    project = make_project(tmp_path)
    report = json.loads((project / "outputs/stage36_report.json").read_text())
    report["scene"] = str(project / "different.blend")
    (project / "outputs/stage36_report.json").write_text(json.dumps(report))
    with pytest.raises(ValueError, match="different scene"):
        load_mission(project)


def test_blender_script_is_syntactically_valid():
    script = Path(__file__).resolve().parents[1] / "tools/blender_stage7_bridge.py"
    compile(script.read_text(), str(script), "exec")
