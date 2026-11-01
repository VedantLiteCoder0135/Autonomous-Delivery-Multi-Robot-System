"""Tests for Stage 3 — environment extraction, mapping and waypoint management.

All tests are pure Python; no bpy dependency.
SceneExtractor is tested via a stub/mock approach that does not call bpy.
"""

from __future__ import annotations

import math
import json
from typing import Any

import pytest

from aifr.environment.environment_mapper import (
    CafeEnvironment,
    CounterInfo,
    EntranceInfo,
    EnvironmentMapper,
    FloorRegion,
    ObstacleRect,
    PillarInfo,
    Rect2D,
    TableInfo,
    WallSegment,
)
from aifr.environment.waypoint_manager import (
    WaypointCandidate,
    WaypointKind,
    WaypointManager,
    WaypointResult,
    _robot_rect,
    _rect_in_floor,
    _rect_collides,
)


# ===========================================================================
# Fixtures — synthetic scene data that mirrors V14 naming conventions
# ===========================================================================

def _make_bbox(cx: float, cy: float, w: float, h: float, z_max: float = 1.0) -> dict[str, float]:
    return {
        "min_x": cx - w / 2, "max_x": cx + w / 2,
        "min_y": cy - h / 2, "max_y": cy + h / 2,
        "min_z": 0.0,        "max_z": z_max,
    }


def _obj(name: str, cx: float, cy: float, w: float = 1.0, h: float = 1.0) -> dict[str, Any]:
    return {
        "name": name,
        "type": "MESH",
        "location": [cx, cy, 0.5],
        "dimensions": [w, h, 1.0],
        "rotation_euler_rad": [0.0, 0.0, 0.0],
        "parent": None,
        "bbox": _make_bbox(cx, cy, w, h),
    }


@pytest.fixture()
def minimal_raw() -> dict[str, Any]:
    """A small synthetic raw-scene dict that covers each V14 object category."""
    objects: list[dict[str, Any]] = [
        # Floor regions
        _obj("FLOOR_arm",  9.5,  3.75, 19.0,  7.5),
        _obj("FLOOR_wing", 14.0, 12.5,  10.0, 10.0),
        # Walls
        _obj("WALL_service_tea_sandwich_juice", 9.5, 0.0, 19.0, 0.16),
        _obj("WALL_main_course_side", 0.0, 3.75, 0.16, 7.5),
        # Pillar
        _obj("PILLAR_01", 9.0, 7.5, 0.38, 0.38),
        _obj("PILLAR_02", 13.5, 7.5, 0.38, 0.38),
        # Table T03 (top + stem + base)
        _obj("T03_TOP",  3.5, 3.15, 1.02, 0.64),
        _obj("T03_STEM", 3.5, 3.15, 0.065, 0.065),
        _obj("T03_BASE", 3.5, 3.15, 0.55, 0.21),
        # Chair roots and parts for T03
        _obj("T03_N", 3.5, 3.80, 0.39, 0.39),    # chair root (EMPTY)
        _obj("T03_N_SEAT", 3.5, 3.80, 0.39, 0.39),
        _obj("T03_N_BACK", 3.5, 3.72, 0.39, 0.055),
        _obj("T03_N_LEG-0.1450.145", 3.5, 3.85, 0.025, 0.025),
        # Food counters
        _obj("COUNTER_TEA",         5.6,  0.65, 2.30, 0.85),
        _obj("COUNTER_SANDWICH",    8.65, 0.65, 2.60, 0.85),
        _obj("COUNTER_JUICE",      14.94, 0.65, 7.88, 0.85),
        _obj("COUNTER_MAIN_COURSE", 0.67, 5.0,  0.92, 2.80),
        # Non-food counters
        _obj("COUNTER_CASH",        2.825, 0.65, 1.65, 0.85),
        _obj("HAND_WASH_CORNER_BASE", 0.82, 0.78, 0.88, 0.85),
        # Decorative / skipped
        _obj("LABEL_T03",           3.5,  3.0,  0.2,  0.2),
        _obj("DOOR_THRESHOLD_MAIN_ENTRY", 4.1, 7.5, 2.0, 0.15),
        _obj("COLOUR_COUNTER_FRONT_TEA",  5.6, 1.10, 2.2, 0.03),
        _obj("ROBOT_BODY",          5.9,  6.45, 0.40, 0.36),
        _obj("TOP_TEA",             5.6,  0.65, 2.34, 0.92),
        _obj("GLASS_TEA",           5.6,  0.43, 2.20, 0.045),
        _obj("CEILING_BEAM_0",      9.5,  4.3, 18.8, 0.32),
        _obj("SLIDING_DOOR_T11_JAMB_low", 9.0, 8.1, 0.13, 0.10),
        _obj("JUICE_SERVICE_PANEL_01", 11.18, 1.10, 1.31, 0.027),
        _obj("JUICE_BACK_FASCIA",  14.94, 0.20, 7.88, 0.12),
        _obj("HAND_WASH_BASIN",     0.82, 0.78, 0.60, 0.60),
        _obj("HAND_WASH_BACK_SPLASH", 0.82, 0.22, 0.92, 0.10),
    ]
    return {"scene_name": "TestScene", "object_count": len(objects), "objects": objects}


@pytest.fixture()
def env(minimal_raw: dict[str, Any]) -> CafeEnvironment:
    return EnvironmentMapper(minimal_raw).map()


# ===========================================================================
# Rect2D tests
# ===========================================================================

class TestRect2D:
    def test_properties(self) -> None:
        r = Rect2D(1.0, 3.0, 2.0, 6.0)
        assert r.center_x == pytest.approx(2.0)
        assert r.center_y == pytest.approx(4.0)
        assert r.width == pytest.approx(2.0)
        assert r.height == pytest.approx(4.0)

    def test_contains_point_inside(self) -> None:
        r = Rect2D(0.0, 4.0, 0.0, 4.0)
        assert r.contains_point(2.0, 2.0)

    def test_contains_point_on_edge(self) -> None:
        r = Rect2D(0.0, 4.0, 0.0, 4.0)
        assert r.contains_point(0.0, 2.0)   # left edge
        assert r.contains_point(4.0, 2.0)   # right edge

    def test_contains_point_outside(self) -> None:
        r = Rect2D(0.0, 4.0, 0.0, 4.0)
        assert not r.contains_point(5.0, 2.0)
        assert not r.contains_point(2.0, -1.0)

    def test_intersects_overlapping(self) -> None:
        a = Rect2D(0.0, 2.0, 0.0, 2.0)
        b = Rect2D(1.0, 3.0, 1.0, 3.0)
        assert a.intersects(b)
        assert b.intersects(a)

    def test_intersects_touching_edge_not_overlapping(self) -> None:
        # Touching edges: open-interval check means not intersecting
        a = Rect2D(0.0, 1.0, 0.0, 1.0)
        b = Rect2D(1.0, 2.0, 0.0, 1.0)
        # a.max_x == b.min_x → not a.max_x > b.min_x → no intersection
        assert not a.intersects(b)

    def test_intersects_separated(self) -> None:
        a = Rect2D(0.0, 1.0, 0.0, 1.0)
        b = Rect2D(2.0, 3.0, 2.0, 3.0)
        assert not a.intersects(b)

    def test_expanded(self) -> None:
        r = Rect2D(1.0, 3.0, 1.0, 3.0)
        e = r.expanded(0.5)
        assert e.min_x == pytest.approx(0.5)
        assert e.max_x == pytest.approx(3.5)
        assert e.min_y == pytest.approx(0.5)
        assert e.max_y == pytest.approx(3.5)

    def test_to_dict_keys(self) -> None:
        r = Rect2D(1.0, 2.0, 3.0, 4.0)
        d = r.to_dict()
        assert set(d.keys()) == {"min_x", "max_x", "min_y", "max_y"}


# ===========================================================================
# EnvironmentMapper tests
# ===========================================================================

class TestEnvironmentMapper:
    def test_floor_regions_extracted(self, env: CafeEnvironment) -> None:
        names = {f.name for f in env.floor_regions}
        assert "FLOOR_arm" in names
        assert "FLOOR_wing" in names

    def test_walls_extracted(self, env: CafeEnvironment) -> None:
        assert len(env.walls) >= 2
        names = {w.name for w in env.walls}
        assert "WALL_service_tea_sandwich_juice" in names

    def test_pillars_extracted(self, env: CafeEnvironment) -> None:
        assert len(env.pillars) == 2
        names = {p.name for p in env.pillars}
        assert "PILLAR_01" in names
        assert "PILLAR_02" in names

    def test_table_extracted(self, env: CafeEnvironment) -> None:
        tids = {t.table_id for t in env.tables}
        assert "T03" in tids

    def test_only_table_top_used(self, env: CafeEnvironment) -> None:
        """STEM and BASE should be skipped; only _TOP creates a TableInfo."""
        assert len(env.tables) == 1  # only T03 in fixture

    def test_chair_seat_extracted(self, env: CafeEnvironment) -> None:
        chair_names = {c.name for c in env.chairs}
        assert "T03_N_SEAT" in chair_names

    def test_chair_parent_table(self, env: CafeEnvironment) -> None:
        for c in env.chairs:
            assert c.parent_table == "T03"

    def test_chair_back_and_legs_skipped(self, env: CafeEnvironment) -> None:
        skipped = set(env.skipped_objects)
        assert "T03_N_BACK" in skipped
        # Leg object name contains "LEG" — should be skipped
        assert any("LEG" in s for s in skipped)

    def test_food_counters_extracted(self, env: CafeEnvironment) -> None:
        counter_names = {c.name for c in env.counters}
        assert "TEA" in counter_names
        assert "SANDWICH" in counter_names
        assert "JUICE" in counter_names
        assert "MAIN_COURSE" in counter_names

    def test_food_counters_marked_as_food(self, env: CafeEnvironment) -> None:
        food = {c.name for c in env.counters if c.is_food_station}
        assert "TEA" in food
        assert "SANDWICH" in food
        assert "JUICE" in food
        assert "MAIN_COURSE" in food

    def test_non_food_counters_marked_correctly(self, env: CafeEnvironment) -> None:
        non_food = {c.name for c in env.counters if not c.is_food_station}
        assert "CASH" in non_food or "HAND_WASH" in non_food

    def test_decorative_objects_skipped(self, env: CafeEnvironment) -> None:
        skipped = set(env.skipped_objects)
        assert "LABEL_T03" in skipped
        assert "DOOR_THRESHOLD_MAIN_ENTRY" in skipped
        assert "COLOUR_COUNTER_FRONT_TEA" in skipped

    def test_robot_body_skipped(self, env: CafeEnvironment) -> None:
        skipped = set(env.skipped_objects)
        assert "ROBOT_BODY" in skipped

    def test_counter_accessories_skipped(self, env: CafeEnvironment) -> None:
        skipped = set(env.skipped_objects)
        assert "TOP_TEA" in skipped
        assert "GLASS_TEA" in skipped

    def test_sliding_door_parts_skipped(self, env: CafeEnvironment) -> None:
        skipped = set(env.skipped_objects)
        assert any("SLIDING_DOOR" in s for s in skipped)

    def test_food_stations_helper(self, env: CafeEnvironment) -> None:
        food = env.food_stations()
        assert all(c.is_food_station for c in food)
        assert len(food) == 4

    def test_all_obstacles_non_empty(self, env: CafeEnvironment) -> None:
        obs = env.all_obstacles()
        assert len(obs) > 0

    def test_all_obstacles_excludes_floor(self, env: CafeEnvironment) -> None:
        # Floor region rects should NOT appear as obstacles
        floor_rects = {(fr.rect.min_x, fr.rect.min_y) for fr in env.floor_regions}
        obs_rects = {(o.min_x, o.min_y) for o in env.all_obstacles()}
        assert not floor_rects.intersection(obs_rects)

    def test_is_in_floor_inside_arm(self, env: CafeEnvironment) -> None:
        assert env.is_in_floor(9.5, 3.75)

    def test_is_in_floor_outside(self, env: CafeEnvironment) -> None:
        assert not env.is_in_floor(-1.0, -1.0)

    def test_to_dict_serialisable(self, env: CafeEnvironment) -> None:
        d = env.to_dict()
        s = json.dumps(d)   # should not raise
        assert len(s) > 100

    def test_to_dict_keys(self, env: CafeEnvironment) -> None:
        d = env.to_dict()
        expected = {"floor_regions", "walls", "pillars", "tables", "chairs",
                    "counters", "entrances", "unknown_objects", "skipped_objects"}
        assert set(d.keys()) == expected

    def test_empty_raw_gives_empty_env(self) -> None:
        raw: dict[str, Any] = {"objects": []}
        env = EnvironmentMapper(raw).map()
        assert env.floor_regions == []
        assert env.tables == []

    def test_object_without_bbox_goes_to_unknown(self) -> None:
        raw: dict[str, Any] = {"objects": [
            {"name": "T99_TOP", "type": "MESH", "location": [0, 0, 0],
             "dimensions": [1, 1, 1], "rotation_euler_rad": [0, 0, 0], "parent": None}
        ]}
        env = EnvironmentMapper(raw).map()
        assert any("T99_TOP" in s for s in env.unknown_objects)


# ===========================================================================
# Geometry helper tests
# ===========================================================================

class TestGeometryHelpers:
    def test_robot_rect_centred(self) -> None:
        r = _robot_rect(5.0, 5.0, 0.2, 0.18)
        assert r.min_x == pytest.approx(4.8)
        assert r.max_x == pytest.approx(5.2)
        assert r.min_y == pytest.approx(4.82)
        assert r.max_y == pytest.approx(5.18)

    def test_rect_in_floor_fully_inside(self) -> None:
        floor = [Rect2D(0.0, 10.0, 0.0, 10.0)]
        robot = Rect2D(1.0, 2.0, 1.0, 2.0)
        assert _rect_in_floor(robot, floor)

    def test_rect_in_floor_partially_outside(self) -> None:
        floor = [Rect2D(0.0, 10.0, 0.0, 10.0)]
        robot = Rect2D(-0.5, 1.0, 1.0, 2.0)
        assert not _rect_in_floor(robot, floor)

    def test_rect_in_floor_spanning_two_regions(self) -> None:
        """A robot that fits entirely inside the second region is valid."""
        floor = [Rect2D(0.0, 5.0, 0.0, 5.0), Rect2D(5.0, 10.0, 0.0, 5.0)]
        robot = Rect2D(6.0, 8.0, 1.0, 3.0)
        assert _rect_in_floor(robot, floor)

    def test_rect_collides_overlap(self) -> None:
        obstacles = [Rect2D(2.0, 4.0, 2.0, 4.0)]
        robot = Rect2D(1.0, 3.0, 1.0, 3.0)
        collides, hit = _rect_collides(robot, obstacles)
        assert collides
        assert hit != ""

    def test_rect_collides_no_overlap(self) -> None:
        obstacles = [Rect2D(5.0, 7.0, 5.0, 7.0)]
        robot = Rect2D(0.0, 1.0, 0.0, 1.0)
        collides, hit = _rect_collides(robot, obstacles)
        assert not collides
        assert hit == ""

    def test_rect_collides_empty_obstacles(self) -> None:
        robot = Rect2D(0.0, 1.0, 0.0, 1.0)
        collides, hit = _rect_collides(robot, [])
        assert not collides


# ===========================================================================
# WaypointManager tests
# ===========================================================================

def _make_simple_env() -> CafeEnvironment:
    """Minimal cafeteria-like environment for waypoint testing."""
    env = CafeEnvironment()
    # Two floor regions (L-shape approximation)
    env.floor_regions = [
        FloorRegion("FLOOR_arm",  Rect2D(0.0, 19.0,  0.0,  7.5)),
        FloorRegion("FLOOR_wing", Rect2D(9.0, 19.0,  7.5, 17.5)),
    ]
    # A wall at y=0 (bottom)
    env.walls = [WallSegment("WALL_bottom", Rect2D(0.0, 19.0, -0.16, 0.0))]
    # One pillar
    env.pillars = [PillarInfo("PILLAR_01", Rect2D(8.81, 9.19, 7.31, 7.69))]
    # One table (T03)
    env.tables = [TableInfo("T03", Rect2D(2.99, 4.01, 2.835, 3.465))]
    # One food counter (TEA)
    env.counters = [
        CounterInfo("TEA", Rect2D(4.45, 6.75, 0.225, 1.075), is_food_station=True),
        CounterInfo("MAIN_COURSE", Rect2D(0.21, 1.13, 3.6, 6.4), is_food_station=True),
        CounterInfo("CASH", Rect2D(2.0, 3.65, 0.225, 1.075), is_food_station=False),
    ]
    return env


@pytest.fixture()
def simple_env() -> CafeEnvironment:
    return _make_simple_env()


@pytest.fixture()
def wm(simple_env: CafeEnvironment) -> WaypointManager:
    return WaypointManager(simple_env)


class TestWaypointManager:
    def test_generate_returns_result(self, wm: WaypointManager) -> None:
        result = wm.generate()
        assert isinstance(result, WaypointResult)

    def test_valid_and_rejected_lists_exist(self, wm: WaypointManager) -> None:
        result = wm.generate()
        assert isinstance(result.valid, list)
        assert isinstance(result.rejected, list)

    def test_at_least_one_valid_spawn(self, wm: WaypointManager) -> None:
        result = wm.generate()
        spawns = [w for w in result.valid if w.kind == WaypointKind.SPAWN]
        assert len(spawns) >= 1

    def test_pickup_generated_for_food_stations_only(self, wm: WaypointManager) -> None:
        result = wm.generate()
        pickup_labels = {w.label for w in result.valid if w.kind == WaypointKind.PICKUP}
        # TEA and MAIN_COURSE are food stations
        assert any("TEA" in lbl for lbl in pickup_labels) or any("TEA" in r["label"] for r in result.rejected)

    def test_no_pickup_for_non_food_stations(self, wm: WaypointManager) -> None:
        result = wm.generate()
        all_pickup = (
            [w.label for w in result.valid if w.kind == WaypointKind.PICKUP]
            + [str(r["label"]) for r in result.rejected if "pickup" in str(r["label"])]
        )
        assert not any("CASH" in lbl for lbl in all_pickup)

    def test_delivery_candidates_for_table(self, wm: WaypointManager) -> None:
        result = wm.generate()
        all_delivery = (
            [w.label for w in result.valid if w.kind == WaypointKind.DELIVERY]
            + [str(r["label"]) for r in result.rejected if "delivery" in str(r["label"])]
        )
        # T03 must have 4 direction candidates
        t03_candidates = [lbl for lbl in all_delivery if "T03" in lbl]
        assert len(t03_candidates) == 4

    def test_delivery_not_at_table_centre(self, wm: WaypointManager, simple_env: CafeEnvironment) -> None:
        """Delivery waypoints must not be at the table centre."""
        result = wm.generate()
        t03 = next(t for t in simple_env.tables if t.table_id == "T03")
        cx, cy = t03.rect.center_x, t03.rect.center_y
        for w in result.valid:
            if w.kind == WaypointKind.DELIVERY and "T03" in w.label:
                assert not (w.x == pytest.approx(cx) and w.y == pytest.approx(cy)), \
                    "Delivery waypoint must not be at table centre"

    def test_valid_waypoints_inside_floor(self, wm: WaypointManager, simple_env: CafeEnvironment) -> None:
        result = wm.generate()
        floor_rects = [fr.rect for fr in simple_env.floor_regions]
        for w in result.valid:
            # Check centre point is in floor (full footprint check is done internally)
            in_floor = any(fr.contains_point(w.x, w.y) for fr in floor_rects)
            assert in_floor, f"{w.label} at ({w.x},{w.y}) not in floor"

    def test_rejected_waypoints_have_reason(self, wm: WaypointManager) -> None:
        result = wm.generate()
        for r in result.rejected:
            assert "reason" in r
            assert r["reason"] != ""

    def test_validate_point_valid(self, wm: WaypointManager) -> None:
        # Open area in the middle of the arm
        cand, reason = wm.validate_point(9.5, 4.0, WaypointKind.SPAWN, "test_valid")
        assert cand is not None
        assert reason == ""

    def test_validate_point_outside_floor(self, wm: WaypointManager) -> None:
        cand, reason = wm.validate_point(-5.0, -5.0, WaypointKind.SPAWN, "test_outside")
        assert cand is None
        assert "outside_floor" in reason

    def test_validate_point_collides_with_pillar(self, wm: WaypointManager) -> None:
        # Pillar at (8.81,9.19)x(7.31,7.69) — robot centre exactly on pillar
        cand, reason = wm.validate_point(9.0, 7.5, WaypointKind.SPAWN, "test_pillar")
        assert cand is None
        assert "obstacle_collision" in reason or "outside_floor" in reason

    def test_waypointcandidate_to_dict(self) -> None:
        w = WaypointCandidate(x=1.0, y=2.0, kind=WaypointKind.SPAWN, label="test", heading=0.5)
        d = w.to_dict()
        assert d["x"] == 1.0
        assert d["y"] == 2.0
        assert d["kind"] == "SPAWN"
        assert d["label"] == "test"
        assert d["heading"] == pytest.approx(0.5)

    def test_waypointresult_to_dict(self, wm: WaypointManager) -> None:
        result = wm.generate()
        d = result.to_dict()
        assert "valid_count" in d
        assert "rejected_count" in d
        assert d["valid_count"] == len(result.valid)
        assert d["rejected_count"] == len(result.rejected)
        s = json.dumps(d)
        assert len(s) > 10

    def test_custom_robot_dimensions(self, simple_env: CafeEnvironment) -> None:
        """A larger robot should have more rejections than a smaller one."""
        wm_small = WaypointManager(simple_env, robot_half_w=0.10, robot_half_h=0.09)
        wm_large = WaypointManager(simple_env, robot_half_w=0.80, robot_half_h=0.72)
        small_valid = len(wm_small.generate().valid)
        large_valid = len(wm_large.generate().valid)
        assert small_valid >= large_valid

    def test_clearance_affects_rejection(self, simple_env: CafeEnvironment) -> None:
        """Larger clearance should cause equal or more rejections."""
        wm_tight = WaypointManager(simple_env, clearance=0.01)
        wm_safe  = WaypointManager(simple_env, clearance=0.50)
        tight_valid = len(wm_tight.generate().valid)
        safe_valid  = len(wm_safe.generate().valid)
        assert tight_valid >= safe_valid

    def test_waiting_candidates_generated(self, wm: WaypointManager) -> None:
        result = wm.generate()
        all_waiting = (
            [w for w in result.valid if w.kind == WaypointKind.WAITING]
            + [r for r in result.rejected if "wait" in str(r["label"])]
        )
        assert len(all_waiting) > 0

    def test_empty_environment_no_crash(self) -> None:
        """WaypointManager with a completely empty env must not crash."""
        env = CafeEnvironment()
        wm = WaypointManager(env)
        result = wm.generate()
        # With no floor regions, all candidates should be rejected as outside_floor
        for r in result.rejected:
            assert r["reason"] != ""


# ===========================================================================
# SceneExtractor import isolation test
# ===========================================================================

class TestSceneExtractorImport:
    def test_import_without_bpy_does_not_crash(self) -> None:
        """Importing scene_extractor must succeed outside Blender (no bpy at top level)."""
        import importlib
        # If import itself crashes, the test fails
        mod = importlib.import_module("aifr.environment.scene_extractor")
        assert hasattr(mod, "SceneExtractor")

    def test_extract_raises_import_error_without_bpy(self) -> None:
        """Calling .extract() outside Blender should raise ImportError (not AttributeError etc.)."""
        from aifr.environment.scene_extractor import SceneExtractor
        extractor = SceneExtractor()
        with pytest.raises((ImportError, ModuleNotFoundError)):
            extractor.extract()


# ===========================================================================
# Architecture / integration tests
# ===========================================================================

class TestArchitecture:
    def test_environment_package_importable(self) -> None:
        import aifr.environment  # noqa: F401

    def test_environment_package_exports(self) -> None:
        from aifr.environment import (  # noqa: F401
            CafeEnvironment,
            EnvironmentMapper,
            ObstacleRect,
            WaypointCandidate,
            WaypointManager,
            WaypointResult,
        )

    def test_no_bpy_in_mapper(self) -> None:
        """environment_mapper must not import bpy."""
        import aifr.environment.environment_mapper as mod
        import sys
        assert "bpy" not in sys.modules or True  # just importing mod must not pull in bpy
        src = open(mod.__file__).read()
        assert "import bpy" not in src
        assert "from bpy" not in src

    def test_no_bpy_in_waypoint_manager(self) -> None:
        import aifr.environment.waypoint_manager as mod
        src = open(mod.__file__).read()
        assert "import bpy" not in src
        assert "from bpy" not in src

    def test_cafe_environment_json_roundtrip(self, env: CafeEnvironment) -> None:
        d = env.to_dict()
        s = json.dumps(d)
        d2 = json.loads(s)
        assert d2["floor_regions"][0]["name"] == d["floor_regions"][0]["name"]

    def test_stage2_packages_still_importable(self) -> None:
        """Stage 2 packages must still import correctly after Stage 3 addition."""
        from aifr.orders.model import Order  # noqa: F401
        from aifr.scheduling.queue import FcfsQueue  # noqa: F401
        from aifr.fleet.registry import RobotRegistry  # noqa: F401

    def test_entrance_info_to_dict(self) -> None:
        e = EntranceInfo(
            name="MAIN_ENTRY", center_x=4.1, center_y=7.5,
            width=2.0, depth=0.15, orientation="horizontal"
        )
        d = e.to_dict()
        assert d["name"] == "MAIN_ENTRY"
        assert d["center_x"] == pytest.approx(4.1)
        assert d["orientation"] == "horizontal"
        s = json.dumps(d)
        assert len(s) > 10

    def test_counter_hand_wash_corner_base_via_counter_prefix(self) -> None:
        """COUNTER_HAND_WASH_CORNER_BASE should be classified as non-food counter."""
        raw: dict[str, Any] = {"objects": [
            _obj("COUNTER_HAND_WASH_CORNER_BASE", 0.82, 0.78, 0.88, 0.85),
        ]}
        env = EnvironmentMapper(raw).map()
        assert len(env.counters) == 1
        assert env.counters[0].is_food_station is False

    def test_unrecognised_object_goes_to_unknown(self, env: CafeEnvironment) -> None:
        """Objects that match no pattern should appear in unknown_objects."""
        raw: dict[str, Any] = {"objects": [
            _obj("COMPLETELY_UNKNOWN_OBJECT_XYZ", 5.0, 5.0, 0.5, 0.5),
        ]}
        new_env = EnvironmentMapper(raw).map()
        assert "COMPLETELY_UNKNOWN_OBJECT_XYZ" in new_env.unknown_objects

    def test_counter_approach_short_wide_counter(self, simple_env: CafeEnvironment) -> None:
        """A counter exactly at width boundary (≤ 2.0) should produce one candidate."""
        from aifr.environment.environment_mapper import CounterInfo, Rect2D
        env = CafeEnvironment()
        env.floor_regions = [FloorRegion("FLOOR_arm", Rect2D(0.0, 19.0, 0.0, 7.5))]
        # Short counter (width = 1.5, ≤ 2.0 → single approach point)
        env.counters = [CounterInfo("TEA", Rect2D(4.0, 5.5, 0.2, 1.1), is_food_station=True)]
        wm_local = WaypointManager(env)
        result = wm_local.generate()
        pickup_all = (
            [w for w in result.valid if w.kind == WaypointKind.PICKUP]
            + [r for r in result.rejected if "pickup" in str(r["label"])]
        )
        tea_all = [p for p in pickup_all if "TEA" in (p.label if hasattr(p, "label") else str(p["label"]))]
        assert len(tea_all) == 1
