"""waypoint_manager.py — Generate and validate robot waypoints for the cafeteria.

All candidate positions are checked against the complete robot footprint
(not just its centre point) to ensure the robot can physically fit.

This module is pure Python — no bpy dependency.

Waypoint categories
-------------------
spawn       Robot starting positions.
pickup      Approach positions beside food service counters.
delivery    Approach positions beside customer tables.
waiting     Positions where a robot can hold without blocking aisles.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Sequence

from aifr.environment.environment_mapper import (
    CafeEnvironment,
    CounterInfo,
    Rect2D,
    TableInfo,
)


# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------

class WaypointKind(Enum):
    SPAWN = auto()
    PICKUP = auto()
    DELIVERY = auto()
    WAITING = auto()


@dataclass(frozen=True, slots=True)
class WaypointCandidate:
    """A validated robot waypoint.

    Parameters
    ----------
    x, y:       World-space centre of the robot when parked here.
    kind:       Category of waypoint.
    label:      Human-readable identifier (e.g. "pickup_TEA", "delivery_T03_N").
    heading:    Robot heading in radians (0 = +X direction).
    """
    x: float
    y: float
    kind: WaypointKind
    label: str
    heading: float = 0.0

    def to_dict(self) -> dict[str, object]:
        return {
            "x": self.x,
            "y": self.y,
            "kind": self.kind.name,
            "label": self.label,
            "heading": self.heading,
        }


@dataclass
class WaypointResult:
    """Output of :meth:`WaypointManager.generate`."""
    valid: list[WaypointCandidate] = field(default_factory=list)
    rejected: list[dict[str, object]] = field(default_factory=list)   # {label, reason}

    def to_dict(self) -> dict[str, object]:
        return {
            "valid_count": len(self.valid),
            "rejected_count": len(self.rejected),
            "valid": [w.to_dict() for w in self.valid],
            "rejected": list(self.rejected),
        }


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _robot_rect(cx: float, cy: float, half_w: float, half_h: float) -> Rect2D:
    """Axis-aligned bounding box of a robot centred at (cx, cy)."""
    return Rect2D(cx - half_w, cx + half_w, cy - half_h, cy + half_h)


def _rect_in_floor(rect: Rect2D, floor_rects: list[Rect2D]) -> bool:
    """Return True if the rectangle is fully contained in at least one floor region."""
    for fr in floor_rects:
        if (fr.min_x <= rect.min_x and rect.max_x <= fr.max_x
                and fr.min_y <= rect.min_y and rect.max_y <= fr.max_y):
            return True
    return False


def _rect_collides(rect: Rect2D, obstacles: list[Rect2D]) -> tuple[bool, str]:
    """Return (collides, obstacle_name_or_empty)."""
    for obs in obstacles:
        if rect.intersects(obs):
            return True, f"[{obs.min_x:.2f},{obs.max_x:.2f}]x[{obs.min_y:.2f},{obs.max_y:.2f}]"
    return False, ""


# ---------------------------------------------------------------------------
# Default geometry constants — from V14 script
# ---------------------------------------------------------------------------

# L-shaped floor: lower arm + upper wing
_ARM_RECT = Rect2D(0.0, 19.0, 0.0, 7.5)
_WING_RECT = Rect2D(9.0, 19.0, 7.5, 17.5)

# Robot footprint half-extents (V14 placeholder: 0.40 × 0.36 m)
_ROBOT_HALF_W = 0.40 / 2.0   # 0.20 m
_ROBOT_HALF_H = 0.36 / 2.0   # 0.18 m

# Safety clearance margin around obstacles
_CLEARANCE = 0.15   # metres

# Standoff distance from a table/counter edge to robot centre
# 0.95m ensures the robot sits just outside the chair line beside the table.
_TABLE_STANDOFF = 0.95
_COUNTER_STANDOFF = 0.60  # counters are deeper, robot approaches from service side


# ---------------------------------------------------------------------------
# Main manager
# ---------------------------------------------------------------------------

class WaypointManager:
    """Generate and validate waypoints for all robot roles.

    Parameters
    ----------
    env:
        A :class:`~aifr.environment.environment_mapper.CafeEnvironment`
        already populated by :class:`~aifr.environment.environment_mapper.EnvironmentMapper`.
    robot_half_w, robot_half_h:
        Half-extents of the robot footprint.  Defaults match the V14 placeholder.
    clearance:
        Additional safety margin added to every obstacle before collision checking.
    table_standoff, counter_standoff:
        Distance from the edge of a table/counter to the robot centre for approach
        waypoints.
    """

    def __init__(
        self,
        env: CafeEnvironment,
        robot_half_w: float = _ROBOT_HALF_W,
        robot_half_h: float = _ROBOT_HALF_H,
        clearance: float = _CLEARANCE,
        table_standoff: float = _TABLE_STANDOFF,
        counter_standoff: float = _COUNTER_STANDOFF,
    ) -> None:
        self._env = env
        self._rhw = robot_half_w
        self._rhh = robot_half_h
        self._clearance = clearance
        self._table_standoff = table_standoff
        self._counter_standoff = counter_standoff

        # Pre-build inflated obstacle list once
        self._floor_rects: list[Rect2D] = [fr.rect for fr in env.floor_regions]
        self._obstacles: list[Rect2D] = [
            obs.expanded(clearance) for obs in env.all_obstacles()
        ]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate(self) -> WaypointResult:
        """Generate all waypoint candidates and validate each one."""
        result = WaypointResult()

        for cand, reason in self._generate_all():
            if reason:
                result.rejected.append({"label": cand.label, "reason": reason})
            else:
                result.valid.append(cand)

        return result

    def validate_point(
        self, x: float, y: float, kind: WaypointKind, label: str, heading: float = 0.0
    ) -> tuple[WaypointCandidate | None, str]:
        """Validate a single candidate position.

        Returns
        -------
        (WaypointCandidate, "")        if valid
        (None,             reason_str) if invalid
        """
        cand = WaypointCandidate(x=x, y=y, kind=kind, label=label, heading=heading)
        reason = self._check(cand)
        if reason:
            return None, reason
        return cand, ""

    # ------------------------------------------------------------------
    # Generation internals
    # ------------------------------------------------------------------

    def _generate_all(self) -> list[tuple[WaypointCandidate, str]]:
        out: list[tuple[WaypointCandidate, str]] = []
        out.extend(self._spawn_candidates())
        out.extend(self._pickup_candidates())
        out.extend(self._delivery_candidates())
        out.extend(self._waiting_candidates())
        return out

    def _check(self, cand: WaypointCandidate) -> str:
        """Return an empty string if the candidate is valid, or a reason string."""
        rect = _robot_rect(cand.x, cand.y, self._rhw, self._rhh)

        # Must be entirely inside traversable floor
        if self._floor_rects and not _rect_in_floor(rect, self._floor_rects):
            return "outside_floor"

        # Must not collide with any (inflated) obstacle
        collides, hit = _rect_collides(rect, self._obstacles)
        if collides:
            return f"obstacle_collision:{hit}"

        return ""

    # ------------------------------------------------------------------
    # Spawn candidates
    # ------------------------------------------------------------------

    def _spawn_candidates(self) -> list[tuple[WaypointCandidate, str]]:
        """Candidate robot spawn positions.

        V14 robot placeholder at (5.9, 6.45).  We generate a small grid
        around that area plus additional fallback positions in open zones.
        """
        raw: list[tuple[float, float, float]] = [
            # (x, y, heading_rad)
            # Near V14 placeholder — robot faces toward +Y (into cafeteria)
            (5.9, 6.45, math.pi / 2),
            (5.9, 5.80, math.pi / 2),
            # Open floor zone in lower arm (mid area)
            (7.5, 4.50, math.pi / 2),
            (7.5, 3.20, math.pi / 2),
            # Near main entrance corridor
            (4.1, 6.80, math.pi / 2),
            # Wing open area (between tables, right column)
            (11.5, 8.50, math.pi / 2),
            (15.0, 8.50, math.pi / 2),
        ]
        return [
            (WaypointCandidate(x=x, y=y, kind=WaypointKind.SPAWN,
                               label=f"spawn_{i+1:02d}", heading=h),
             self._check(WaypointCandidate(x=x, y=y, kind=WaypointKind.SPAWN,
                                           label=f"spawn_{i+1:02d}", heading=h)))
            for i, (x, y, h) in enumerate(raw)
        ]

    # ------------------------------------------------------------------
    # Pickup candidates
    # ------------------------------------------------------------------

    def _pickup_candidates(self) -> list[tuple[WaypointCandidate, str]]:
        """Approach positions beside each food service counter.

        The robot stands in front of (on the +Y / service side of) each counter.
        Hand Wash and Cash counters are excluded.
        """
        results: list[tuple[WaypointCandidate, str]] = []

        for counter in self._env.food_stations():
            cname = counter.name
            candidates = self._counter_approach_candidates(counter)
            for i, (x, y, h) in enumerate(candidates):
                label = f"pickup_{cname}_{i+1:02d}" if len(candidates) > 1 else f"pickup_{cname}"
                cand = WaypointCandidate(x=x, y=y, kind=WaypointKind.PICKUP,
                                        label=label, heading=h)
                results.append((cand, self._check(cand)))

        return results

    def _counter_approach_candidates(
        self, counter: CounterInfo
    ) -> list[tuple[float, float, float]]:
        """Return (x, y, heading) candidates in front of a counter.

        All V14 counters along the bottom wall (y ≈ 0.65) face toward +Y.
        Main Course is against the left wall (x ≈ 0) and faces toward +X.
        """
        r = counter.rect
        cx = r.center_x
        cy = r.center_y

        if counter.name == "MAIN_COURSE":
            # Counter against the left wall — robot approaches from the right (+X)
            approach_x = r.max_x + self._counter_standoff
            return [(approach_x, cy, math.pi)]   # heading: face left toward counter

        # All other food counters are along the bottom wall → robot approaches from +Y
        approach_y = r.max_y + self._counter_standoff

        # For long counters (JUICE), generate multiple approach positions
        if r.width > 2.0:
            step = 1.5
            xs = []
            x = r.min_x + step
            while x < r.max_x - step / 2:
                xs.append(x)
                x += step
            if not xs:
                xs = [cx]
            return [(xi, approach_y, -math.pi / 2) for xi in xs]  # face toward -Y (counter)

        return [(cx, approach_y, -math.pi / 2)]

    # ------------------------------------------------------------------
    # Delivery candidates
    # ------------------------------------------------------------------

    def _delivery_candidates(self) -> list[tuple[WaypointCandidate, str]]:
        """Approach positions beside each of the 18 customer tables.

        For each table we generate 4 candidates (N, S, E, W sides).
        The table centre is never used — the robot approaches from the side.
        """
        results: list[tuple[WaypointCandidate, str]] = []
        directions = [
            ("N",  0.0,  1.0, -math.pi / 2),   # north of table, facing south
            ("S",  0.0, -1.0,  math.pi / 2),   # south, facing north
            ("E",  1.0,  0.0,  math.pi),        # east, facing west
            ("W", -1.0,  0.0,  0.0),            # west, facing east
        ]
        for table in self._env.tables:
            tid = table.table_id
            r = table.rect
            cx, cy = r.center_x, r.center_y

            for dname, dx, dy, heading in directions:
                # Standoff from the table edge
                if dx != 0:
                    edge_x = r.max_x if dx > 0 else r.min_x
                    px = edge_x + dx * self._table_standoff
                    py = cy
                else:
                    edge_y = r.max_y if dy > 0 else r.min_y
                    px = cx
                    py = edge_y + dy * self._table_standoff

                label = f"delivery_{tid}_{dname}"
                cand = WaypointCandidate(x=px, y=py, kind=WaypointKind.DELIVERY,
                                        label=label, heading=heading)
                results.append((cand, self._check(cand)))

        return results

    # ------------------------------------------------------------------
    # Waiting candidates
    # ------------------------------------------------------------------

    def _waiting_candidates(self) -> list[tuple[WaypointCandidate, str]]:
        """Open holding positions where a robot can wait without blocking aisles.

        Positions are chosen manually from known open zones in the V14 layout.
        They are labelled as assumptions (not measured positions).
        """
        # NOTE: These are assumed open positions, not measured from the real cafeteria.
        raw: list[tuple[float, float, float, str]] = [
            # (x, y, heading, label)
            (7.5, 6.0, 0.0, "wait_arm_mid"),
            (7.5, 2.5, 0.0, "wait_arm_low"),
            (12.5, 8.5, 0.0, "wait_wing_entry"),
            (16.0, 8.5, 0.0, "wait_wing_right"),
            (12.5, 16.5, 0.0, "wait_wing_top"),
            (16.0, 12.5, 0.0, "wait_wing_far"),
        ]
        return [
            (WaypointCandidate(x=x, y=y, kind=WaypointKind.WAITING,
                               label=label, heading=h),
             self._check(WaypointCandidate(x=x, y=y, kind=WaypointKind.WAITING,
                                           label=label, heading=h)))
            for x, y, h, label in raw
        ]
