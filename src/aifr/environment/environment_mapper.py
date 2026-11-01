"""environment_mapper.py — Classify scene geometry into structured cafeteria data.

Input:  raw dict produced by SceneExtractor (or a hand-crafted equivalent for tests).
Output: CafeEnvironment — a fully serialisable, algorithm-independent description of
        the cafeteria: floor regions, walls, pillars, tables, chairs, food counters,
        entrances, and unknown/skipped objects.

This module is pure Python — no bpy dependency.
"""

from __future__ import annotations

import re
import json
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Sequence


# ---------------------------------------------------------------------------
# Value types
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Rect2D:
    """Axis-aligned rectangle in XY (top-down) plane."""
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def center_x(self) -> float:
        return (self.min_x + self.max_x) / 2.0

    @property
    def center_y(self) -> float:
        return (self.min_y + self.max_y) / 2.0

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_y - self.min_y

    def contains_point(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def intersects(self, other: "Rect2D") -> bool:
        return (
            self.min_x < other.max_x and self.max_x > other.min_x
            and self.min_y < other.max_y and self.max_y > other.min_y
        )

    def expanded(self, margin: float) -> "Rect2D":
        """Return a new Rect2D grown by *margin* on every side."""
        return Rect2D(
            self.min_x - margin, self.max_x + margin,
            self.min_y - margin, self.max_y + margin,
        )

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


# Alias used publicly (keeps future compatibility if we ever add rotation)
ObstacleRect = Rect2D


@dataclass(slots=True)
class FloorRegion:
    name: str
    rect: Rect2D

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "rect": self.rect.to_dict()}


@dataclass(slots=True)
class WallSegment:
    name: str
    rect: Rect2D         # bounding rectangle in XY — thin but non-zero

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "rect": self.rect.to_dict()}


@dataclass(slots=True)
class PillarInfo:
    name: str
    rect: Rect2D

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "rect": self.rect.to_dict()}


@dataclass(slots=True)
class TableInfo:
    table_id: str        # e.g. "T03"
    rect: Rect2D         # footprint of the table top

    def to_dict(self) -> dict[str, Any]:
        return {"table_id": self.table_id, "rect": self.rect.to_dict()}


@dataclass(slots=True)
class ChairInfo:
    name: str
    parent_table: str    # e.g. "T03"
    rect: Rect2D

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "parent_table": self.parent_table,
                "rect": self.rect.to_dict()}


@dataclass(slots=True)
class CounterInfo:
    name: str            # e.g. "TEA", "MAIN_COURSE"
    rect: Rect2D
    is_food_station: bool   # False for HAND_WASH and CASH

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "rect": self.rect.to_dict(),
            "is_food_station": self.is_food_station,
        }


@dataclass(slots=True)
class EntranceInfo:
    name: str
    center_x: float
    center_y: float
    width: float         # opening width (X direction for horizontal doors)
    depth: float         # opening depth (Y direction for horizontal doors)
    orientation: str     # "horizontal" (X-axis gap) or "vertical" (Y-axis gap)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "width": self.width,
            "depth": self.depth,
            "orientation": self.orientation,
        }


@dataclass
class CafeEnvironment:
    """Complete structured description of the cafeteria environment.

    All coordinates are in the same metric space as the V14 Blender scene
    (provisional metres, XY top-down, Z height).

    This object is the single authoritative environment representation for
    Stages 3 and beyond.  It deliberately carries no navigation algorithm
    specifics.
    """

    # Traversable floor polygons (modelled as rectangles for the L-shape)
    floor_regions: list[FloorRegion] = field(default_factory=list)

    # Physical walls (collision obstacles)
    walls: list[WallSegment] = field(default_factory=list)

    # Structural pillars
    pillars: list[PillarInfo] = field(default_factory=list)

    # Tables (18 in V14)
    tables: list[TableInfo] = field(default_factory=list)

    # Chairs (72 in V14)
    chairs: list[ChairInfo] = field(default_factory=list)

    # Food counters + non-food service fixtures
    counters: list[CounterInfo] = field(default_factory=list)

    # Actual openings (not blocked)
    entrances: list[EntranceInfo] = field(default_factory=list)

    # Objects we did not recognise — logged for review, not navigation
    unknown_objects: list[str] = field(default_factory=list)

    # Objects explicitly skipped (decorative, labels, lights …)
    skipped_objects: list[str] = field(default_factory=list)

    # ---------------------------------------------------------------
    # Convenience helpers
    # ---------------------------------------------------------------

    def food_stations(self) -> list[CounterInfo]:
        """Return only the counters that are actual food pickup stations."""
        return [c for c in self.counters if c.is_food_station]

    def all_obstacles(self) -> list[Rect2D]:
        """Return every XY rectangle that a robot must not overlap.

        Includes walls, pillars, tables, chairs, and all counters.
        Does NOT include floor or entrance objects.
        """
        rects: list[Rect2D] = []
        rects.extend(w.rect for w in self.walls)
        rects.extend(p.rect for p in self.pillars)
        rects.extend(t.rect for t in self.tables)
        rects.extend(c.rect for c in self.chairs)
        rects.extend(c.rect for c in self.counters)
        return rects

    def is_in_floor(self, x: float, y: float) -> bool:
        """Return True if (x, y) falls inside any floor region."""
        return any(fr.rect.contains_point(x, y) for fr in self.floor_regions)

    def to_dict(self) -> dict[str, Any]:
        return {
            "floor_regions": [f.to_dict() for f in self.floor_regions],
            "walls": [w.to_dict() for w in self.walls],
            "pillars": [p.to_dict() for p in self.pillars],
            "tables": [t.to_dict() for t in self.tables],
            "chairs": [c.to_dict() for c in self.chairs],
            "counters": [c.to_dict() for c in self.counters],
            "entrances": [e.to_dict() for e in self.entrances],
            "unknown_objects": list(self.unknown_objects),
            "skipped_objects": list(self.skipped_objects),
        }


# ---------------------------------------------------------------------------
# Name-based classification patterns (V14 naming conventions)
# ---------------------------------------------------------------------------

# Object name prefixes/patterns → category
_WALL_RE = re.compile(r"^WALL_", re.IGNORECASE)
_FLOOR_RE = re.compile(r"^FLOOR_", re.IGNORECASE)
_PILLAR_RE = re.compile(r"^PILLAR_\d+", re.IGNORECASE)
_TABLE_TOP_RE = re.compile(r"^(T\d{2})_TOP$", re.IGNORECASE)
_TABLE_STEM_RE = re.compile(r"^T\d{2}_STEM$", re.IGNORECASE)
_TABLE_BASE_RE = re.compile(r"^T\d{2}_BASE$", re.IGNORECASE)
_CHAIR_SEAT_RE = re.compile(r"^(T\d{2})_[NSEW]_SEAT$", re.IGNORECASE)
_CHAIR_BACK_RE = re.compile(r"^(T\d{2})_[NSEW]_BACK$", re.IGNORECASE)
_CHAIR_LEG_RE = re.compile(r"^T\d{2}_[NSEW]_LEG", re.IGNORECASE)
_CHAIR_ROOT_RE = re.compile(r"^(T\d{2})_[NSEW]$", re.IGNORECASE)

# Counter names in V14
_FOOD_COUNTER_NAMES = frozenset({"TEA", "SANDWICH", "JUICE", "MAIN_COURSE"})
_NON_FOOD_COUNTER_NAMES = frozenset({"HAND_WASH_CORNER", "CASH"})

_COUNTER_BODY_RE = re.compile(r"^COUNTER_(.+)$", re.IGNORECASE)

# Decorative / non-collision objects — skip entirely
_SKIP_PREFIXES = (
    "LABEL_", "DOOR_THRESHOLD_", "tile_band_", "COLOUR_",
    "CEILING_BEAM_", "ROBOT_FOOD_TRAY", "ROBOT_BODY",
    "ROBOT_START", "WP_",
    # Sliding door furniture (frame, glass, handle, track) — not traversal obstacles
    "SLIDING_DOOR_", "MAIN_ENTRY_SLIDING_", "SIDE_GATE_1_SLIDING_",
    "SIDE_GATE_2_SLIDING_",
    # Counter accessories that are not the main body
    "TOP_", "GLASS_", "CASH_COUNTER_METAL_TOP", "CASH_COUNTER_FRONT_COLOUR",
    "CASH_COUNTER_RIM", "CASH_REGISTER_", "CASH_PAYMENT_TERMINAL",
    "JUICE_SERVICE_PANEL_", "JUICE_PANEL_TRIM_", "JUICE_BACK_FASCIA",
    "HAND_WASH_BASIN", "HAND_WASH_DRAIN", "HAND_WASH_BACK_SPLASH",
    "HAND_WASH_LEFT_SPLASH", "HAND_WASH_CORNER_TOP",
    # Pillar overhead beams
    # (keep PILLAR_ — captured by _PILLAR_RE)
    # Lights, camera
    "LIGHT_", "Camera",
)


def _should_skip(name: str) -> bool:
    return any(name.startswith(p) for p in _SKIP_PREFIXES)


def _bbox_to_rect(bbox: dict[str, float]) -> Rect2D:
    return Rect2D(
        min_x=bbox["min_x"], max_x=bbox["max_x"],
        min_y=bbox["min_y"], max_y=bbox["max_y"],
    )


# ---------------------------------------------------------------------------
# Public mapper
# ---------------------------------------------------------------------------

class EnvironmentMapper:
    """Classify raw scene objects into a :class:`CafeEnvironment`.

    Parameters
    ----------
    raw:
        Dict returned by :class:`~aifr.environment.scene_extractor.SceneExtractor`
        or an equivalent hand-crafted test fixture.  Only the ``"objects"``
        key is used; ``"custom_props"`` are ignored.
    """

    def __init__(self, raw: dict[str, Any]) -> None:
        self._objects: list[dict[str, Any]] = raw.get("objects", [])
        self._custom_props: dict[str, Any] = raw.get("custom_props", {})

    def map(self) -> CafeEnvironment:  # noqa: A003  (shadowing built-in 'map' is fine here)
        """Process all objects and return a :class:`CafeEnvironment`."""
        env = CafeEnvironment()

        for obj in self._objects:
            name: str = obj.get("name", "")
            bbox: dict[str, float] | None = obj.get("bbox")

            if _should_skip(name):
                env.skipped_objects.append(name)
                continue

            if bbox is None:
                env.unknown_objects.append(f"{name} (no bbox)")
                continue

            rect = _bbox_to_rect(bbox)
            self._classify(name, rect, env)

        self._classify_entrances(env)
        return env

    def _classify_entrances(self, env: CafeEnvironment) -> None:
        """Read V14 door metadata and validate actual adjacent wall gaps.

        A label alone does not create a traversable entrance. Both flanking
        wall segments must exist and no wall may cross the opening.
        """
        value = self._custom_props.get("AIFR_doors")
        if value is None:
            return  # generic fixtures without door metadata remain valid
        try:
            doors = json.loads(value) if isinstance(value, str) else value
        except (ValueError, TypeError) as exc:
            raise ValueError("Invalid AIFR_doors metadata") from exc
        if not isinstance(doors, dict):
            raise ValueError("AIFR_doors must be a JSON object")
        required = {"main_entry": "horizontal", "side_exit_1": "vertical",
                    "side_exit_2": "vertical", "sliding_entry_t11": "vertical"}
        if set(doors) != set(required):
            raise ValueError(f"Expected four V14 entrances, found {sorted(doors)}")
        for name, orientation in required.items():
            coords = doors[name]
            if (not isinstance(coords, (list, tuple)) or len(coords) != 2
                    or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in coords)):
                raise ValueError(f"Invalid coordinates for entrance {name}")
            x, y = map(float, coords)
            horizontal = orientation == "horizontal"
            axis = y if horizontal else x
            center = x if horizontal else y
            # V14 wall thickness is 0.16 m. Restrict to walls on the same line.
            if horizontal:
                walls = [w.rect for w in env.walls
                         if w.rect.min_y - .20 <= axis <= w.rect.max_y + .20]
            else:
                walls = [w.rect for w in env.walls
                         if w.rect.min_x - .20 <= axis <= w.rect.max_x + .20]
            before = [r.max_x if horizontal else r.max_y for r in walls
                      if (r.max_x if horizontal else r.max_y) <= center + 1e-6]
            after = [r.min_x if horizontal else r.min_y for r in walls
                     if (r.min_x if horizontal else r.min_y) >= center - 1e-6]
            if not before or not after:
                raise ValueError(f"Entrance {name} has no flanking wall segments")
            low, high = max(before), min(after)
            width = high - low
            if not .55 <= width <= 4.0 or not low < center < high:
                raise ValueError(f"Entrance {name} does not match a valid wall gap")
            if any((r.min_x if horizontal else r.min_y) < high - 1e-5
                   and (r.max_x if horizontal else r.max_y) > low + 1e-5
                   for r in walls):
                raise ValueError(f"Entrance {name} is blocked by a wall")
            env.entrances.append(EntranceInfo(
                name, x, y, width if horizontal else .16,
                .16 if horizontal else width, orientation))

    # ------------------------------------------------------------------
    # Internal classification logic
    # ------------------------------------------------------------------

    def _classify(self, name: str, rect: Rect2D, env: CafeEnvironment) -> None:
        # --- Floor ---
        if _FLOOR_RE.match(name):
            env.floor_regions.append(FloorRegion(name=name, rect=rect))
            return

        # --- Walls ---
        if _WALL_RE.match(name):
            env.walls.append(WallSegment(name=name, rect=rect))
            return

        # --- Pillars ---
        if _PILLAR_RE.match(name):
            env.pillars.append(PillarInfo(name=name, rect=rect))
            return

        # --- Tables (top surface only — defines footprint) ---
        m = _TABLE_TOP_RE.match(name)
        if m:
            tid = m.group(1).upper()
            env.tables.append(TableInfo(table_id=tid, rect=rect))
            return

        # Ignore table stem and base — not traversal obstacles in their own right
        # (the table top covers the column footprint)
        if _TABLE_STEM_RE.match(name) or _TABLE_BASE_RE.match(name):
            env.skipped_objects.append(name)
            return

        # --- Chairs (seat is the significant horizontal obstacle) ---
        m = _CHAIR_SEAT_RE.match(name)
        if m:
            tid = m.group(1).upper()
            env.chairs.append(ChairInfo(name=name, parent_table=tid, rect=rect))
            return

        # Chair back / legs / root empty — skip
        if (_CHAIR_BACK_RE.match(name) or _CHAIR_LEG_RE.match(name)
                or _CHAIR_ROOT_RE.match(name)):
            env.skipped_objects.append(name)
            return

        # --- Counters ---
        m = _COUNTER_BODY_RE.match(name)
        if m:
            counter_key = m.group(1).upper()
            is_food = counter_key in _FOOD_COUNTER_NAMES
            # CASH counter — key is "CASH"
            if counter_key == "CASH":
                is_food = False
            # Hand wash base — name is HAND_WASH_CORNER_BASE
            if name.upper().startswith("COUNTER_HAND_WASH"):
                is_food = False
            env.counters.append(
                CounterInfo(name=counter_key, rect=rect, is_food_station=is_food)
            )
            return

        # Hand wash base captured as COUNTER_... above; raw base name fallthrough:
        if name.upper().startswith("HAND_WASH_CORNER_BASE"):
            env.counters.append(
                CounterInfo(name="HAND_WASH", rect=rect, is_food_station=False)
            )
            return

        # --- Everything else ---
        env.unknown_objects.append(name)
