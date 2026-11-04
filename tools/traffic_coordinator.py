"""Multi-robot deadlock detection and recovery helpers for the Stage 7-9 engine.

Pure logic with no simulation state of its own: the engine feeds it poses,
paths and priorities and receives decisions back. Nothing here can move a robot
or relax a collision guard; every motion it proposes (a retreat or a resume) is
an ordinary route that still passes DWA, the central reservation and the exact
footprint checks.

* ``WaitForGraph``: who is waiting on whom; cycle detection.
* ``narrow_cells``: single-lane cells of the inflated grid (two robots cannot
  pass without one leaving the lane).
* ``lane_claim`` / ``corridor_conflict``: proactive single-lane reservation.
* ``find_refuge``: the nearest safe passing bay for a robot that must yield.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, Sequence

from aifr.navigation.occupancy_grid import OccupancyGrid

ROBOT_RADIUS = math.hypot(.20, .18)


class WaitForGraph:
    """Functional wait-for graph: each waiting robot waits on at most one agent."""

    def __init__(self) -> None:
        self.edges: dict[int, int] = {}

    def clear(self) -> None:
        self.edges.clear()

    def add(self, waiter: int, blocker: int) -> None:
        if waiter != blocker:
            self.edges[waiter] = blocker

    def cycles(self) -> list[list[int]]:
        """All distinct cycles (each node has out-degree <= 1)."""
        found: list[list[int]] = []
        state: dict[int, int] = {}
        for start in self.edges:
            if state.get(start):
                continue
            path: list[int] = []
            node: int | None = start
            while node is not None and state.get(node, 0) == 0:
                state[node] = 1
                path.append(node)
                node = self.edges.get(node)
            if node is not None and state.get(node) == 1:
                found.append(path[path.index(node):])
            for n in path:
                state[n] = 2
        return found

    def chain_head(self, robot: int) -> int:
        """Follow the wait chain to its end (or back to the start of a loop)."""
        seen = {robot}
        node = robot
        while node in self.edges:
            node = self.edges[node]
            if node in seen:
                break
            seen.add(node)
        return node


def distance_map(grid: OccupancyGrid, seeds: Iterable[tuple[int, int]], cap: float = 1.5) -> list[list[float]]:
    """Chamfer distance (m) from every cell to the nearest seed cell, capped."""
    w, h, res = grid.width, grid.height, grid.resolution
    dist = [[cap] * w for _ in range(h)]
    heap: list[tuple[float, int, int]] = []
    for c, r in seeds:
        dist[r][c] = 0.
        heap.append((0., c, r))
    heapq.heapify(heap)
    while heap:
        d, c, r = heapq.heappop(heap)
        if d > dist[r][c]:
            continue
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if not (dc or dr):
                    continue
                nc, nr = c + dc, r + dr
                if 0 <= nc < w and 0 <= nr < h:
                    nd = d + (1.4142 if dc and dr else 1.) * res
                    if nd < dist[nr][nc]:
                        dist[nr][nc] = nd
                        heapq.heappush(heap, (nd, nc, nr))
    return dist


def clearance_map(grid: OccupancyGrid, cap: float = 1.5) -> list[list[float]]:
    """Distance (m) from each cell to the nearest blocked cell, capped."""
    return distance_map(grid, [(c, r) for r in range(grid.height) for c in range(grid.width)
                               if not grid.is_free(c, r)], cap)


class LaneMap:
    """Cached single-lane / passing-bay classification of one static grid.

    ``clearance`` is the distance to the nearest obstacle. A *wide core* cell has
    room on both sides for two robots to pass (clearance >= SINGLE_LANE). A cell is
    single-lane only if no wide core lies within reach: hugging a wall of an open
    hall is not a corridor, but a one-robot-wide aisle is.
    """

    # Free space (in the inflated robot-centre grid) narrower than this cannot hold
    # two robots passing: 2 footprints + a minimal gap, halved per side.
    SINGLE_LANE = ROBOT_RADIUS + .18

    def __init__(self, grid: OccupancyGrid) -> None:
        self.grid = grid
        self.clearance = clearance_map(grid)
        core = [(c, r) for r in range(grid.height) for c in range(grid.width)
                if grid.is_free(c, r) and self.clearance[r][c] >= self.SINGLE_LANE]
        self.core_distance = distance_map(grid, core)

    def _cell(self, x: float, y: float):
        c, r = self.grid.world_to_grid(x, y)
        return (c, r) if 0 <= c < self.grid.width and 0 <= r < self.grid.height else None

    def clear_at(self, x: float, y: float) -> float:
        cell = self._cell(x, y)
        return 0. if cell is None else self.clearance[cell[1]][cell[0]]

    def is_narrow(self, x: float, y: float) -> bool:
        cell = self._cell(x, y)
        if cell is None:
            return True
        return self.core_distance[cell[1]][cell[0]] > self.SINGLE_LANE

    def is_wide(self, c: int, r: int) -> bool:
        return self.core_distance[r][c] <= self.SINGLE_LANE


@dataclass(frozen=True)
class LaneSample:
    x: float
    y: float
    dx: float
    dy: float


def lane_claim(path: Sequence[tuple[float, float]], index: int, pose: tuple[float, float],
               lanes: LaneMap, lookahead: float = 3.0, spacing: float = .25) -> list[LaneSample]:
    """Single-lane samples of the next ``lookahead`` metres of a route."""
    out: list[LaneSample] = []
    if not path:
        return out
    pts = [pose] + list(path[max(0, index):])
    travelled = 0.
    carry = 0.
    for a, b in zip(pts, pts[1:]):
        seg = math.dist(a, b)
        if seg < 1e-9:
            continue
        ux, uy = (b[0] - a[0]) / seg, (b[1] - a[1]) / seg
        pos = carry
        while pos <= seg:
            if travelled + pos > lookahead:
                return out
            x, y = a[0] + ux * pos, a[1] + uy * pos
            if lanes.is_narrow(x, y):
                out.append(LaneSample(x, y, ux, uy))
            pos += spacing
        carry = pos - seg
        travelled += seg
    return out


def corridor_conflict(mine: Sequence[LaneSample], theirs: Sequence[LaneSample],
                      reach: float = ROBOT_RADIUS * 2 + .1) -> bool:
    """True when two claims overlap a single lane while travelling toward each other."""
    for a in mine:
        for b in theirs:
            if (a.x - b.x) ** 2 + (a.y - b.y) ** 2 <= reach * reach and a.dx * b.dx + a.dy * b.dy < -.3:
                return True
    return False


def find_refuge(grid: OccupancyGrid, lanes: LaneMap, start: tuple[float, float],
                avoid: Iterable[tuple[float, float, float]],
                exclude: Iterable[tuple[float, float]] = (),
                max_path: float = 7.0) -> tuple[float, float] | None:
    """Nearest safe waiting spot reachable on ``grid`` (which already has keep-outs).

    ``avoid`` is ``(x, y, radius)`` discs the refuge must stay out of: other robots,
    their upcoming lanes, nearby people. A wide passing bay beats a single-lane
    cell; recently used refuges in ``exclude`` are skipped so a retried yield
    does not return to the same blocked spot.
    """
    avoid = list(avoid)
    exclude = list(exclude)
    res = grid.resolution
    sc, sr = grid.world_to_grid(*start)
    dist = {(sc, sr): 0.}
    heap = [(0., sc, sr)]
    best: tuple[float, tuple[float, float]] | None = None
    while heap:
        d, c, r = heapq.heappop(heap)
        if d > dist.get((c, r), 1e9):
            continue
        if d > max_path:
            break
        x, y = grid.grid_to_world(c, r)
        if d > .6 and all(math.hypot(x - ax, y - ay) >= rad for ax, ay, rad in avoid) \
                and all(math.hypot(x - ex, y - ey) > .5 for ex, ey in exclude):
            cost = d + (0. if lanes.is_wide(c, r) else 2.5)
            if best is None or cost < best[0]:
                best = (cost, (x, y))
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if not (dc or dr):
                    continue
                nc, nr = c + dc, r + dr
                if not (0 <= nc < grid.width and 0 <= nr < grid.height) or not grid.is_free(nc, nr):
                    continue
                if dc and dr and not (grid.is_free(c + dc, r) and grid.is_free(c, r + dr)):
                    continue
                nd = d + (1.4142 if dc and dr else 1.) * res
                if nd < dist.get((nc, nr), 1e9):
                    dist[(nc, nr)] = nd
                    heapq.heappush(heap, (nd, nc, nr))
        if best is not None and d > best[0] + 2.6:
            break
    return None if best is None else best[1]
