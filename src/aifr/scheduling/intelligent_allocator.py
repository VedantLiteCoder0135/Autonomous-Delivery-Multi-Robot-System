"""Pure Hungarian task matching with maximum-cardinality feasibility first."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class AllocationWeights:
    distance: float = 0.30
    time: float = 0.30
    battery: float = 0.20
    workload: float = 0.10
    degradation: float = 0.10

    def __post_init__(self) -> None:
        values = (self.distance, self.time, self.battery,
                  self.workload, self.degradation)
        if any(not math.isfinite(v) or v < 0 for v in values) or sum(values) <= 0:
            raise ValueError("allocation weights must be finite, nonnegative, and nonzero")


def hungarian_min_cost(costs: list[list[float]]) -> list[tuple[int, int]]:
    """Solve a rectangular finite matrix; rows and columns are zero-based."""
    if not costs:
        return []
    rows, cols = len(costs), len(costs[0])
    if cols == 0 or any(len(row) != cols for row in costs):
        raise ValueError("cost matrix must be nonempty and rectangular")
    if any(not math.isfinite(v) for row in costs for v in row):
        raise ValueError("Hungarian matrix must contain finite costs")
    transposed = rows > cols
    matrix = ([list(row) for row in zip(*costs)] if transposed else costs)
    n, m = len(matrix), len(matrix[0])  # n <= m
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [float("inf")] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = float("inf"), 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = matrix[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    pairs = [(p[j] - 1, j - 1) for j in range(1, m + 1) if p[j]]
    return [(col, row) for row, col in pairs] if transposed else pairs


def _normalized(values: dict[tuple[int, int], float]) -> dict[tuple[int, int], float]:
    lo, hi = min(values.values()), max(values.values())
    if hi - lo <= 1e-12:
        return {key: 0.0 for key in values}
    return {key: (value - lo) / (hi - lo) for key, value in values.items()}


def _maximum_matching_size(robot_ids, order_ids, feasible_pairs) -> int:
    """Small deterministic augmenting-path matcher used for fairness selection."""
    owner: dict[int, int] = {}

    def augment(order_id: int, seen: set[int]) -> bool:
        for robot_id in robot_ids:
            if (robot_id, order_id) not in feasible_pairs or robot_id in seen:
                continue
            seen.add(robot_id)
            if robot_id not in owner or augment(owner[robot_id], seen):
                owner[robot_id] = order_id
                return True
        return False

    count = 0
    for order_id in order_ids:
        if augment(order_id, set()):
            count += 1
    return count


class IntelligentAllocator:
    """Cost-based matching; all path and battery feasibility is supplied by caller."""

    def __init__(self, weights: AllocationWeights | None = None) -> None:
        self.weights = weights or AllocationWeights()
        self.last_decisions: list[dict] = []

    def allocate(self, robot_ids, order_ids, pair_metrics: dict[tuple[int, int], dict]) -> list[tuple[int, int]]:
        robots = sorted(map(int, robot_ids))
        orders = list(map(int, order_ids))  # caller supplies READY order age order
        if not robots or not orders:
            self.last_decisions = []
            return []
        keys = [(r, o) for r in robots for o in orders
                if (r, o) in pair_metrics and pair_metrics[(r, o)].get("feasible", False)]
        if not keys:
            self.last_decisions = [{"robot_id": r, "order_id": o,
                                    "feasible": False,
                                    "reason": pair_metrics.get((r, o), {}).get("reason", "infeasible")}
                                   for r in robots for o in orders]
            return []
        fields = ("distance_m", "time_s", "battery_fraction", "workload", "degradation_fraction")
        normalized = {field: _normalized({key: float(pair_metrics[key].get(field, 0.0))
                                          for key in keys}) for field in fields}
        w = self.weights
        weights = dict(zip(fields, (w.distance, w.time, w.battery, w.workload, w.degradation)))
        raw_cost = {key: sum(weights[field] * normalized[field][key] for field in fields)
                    for key in keys}

        # Matchable order subsets form a transversal matroid. Greedily adding
        # READY orders in queue order while preserving matchability yields a
        # maximum-cardinality, lexicographically oldest feasible order set.
        # The Hungarian stage then minimizes mission cost for that selected set.
        selected_orders: list[int] = []
        for order_id in orders:
            proposed = selected_orders + [order_id]
            if _maximum_matching_size(robots, proposed, set(keys)) == len(proposed):
                selected_orders.append(order_id)
            if len(selected_orders) >= len(robots):
                break

        # Make unmatched robots explicit. A dummy costs more than every possible
        # total real-edge cost; forbidden edges cost more than any dummy solution.
        max_real = max(raw_cost.values(), default=0.0)
        dummy_penalty = (len(robots) + 1) * (max_real + 1.0)
        forbidden_penalty = dummy_penalty * (len(robots) + 2)
        matrix = []
        for ri, robot_id in enumerate(robots):
            row = [raw_cost.get((robot_id, order_id), forbidden_penalty)
                   + abs(ri - oi) * 1e-9 + (ri + oi) * 1e-12
                   if order_id in selected_orders else forbidden_penalty
                   for oi, order_id in enumerate(orders)]
            row.extend(dummy_penalty + di * 1e-12 for di in range(len(robots)))
            matrix.append(row)
        assignment = hungarian_min_cost(matrix)
        matches = []
        for ri, ci in assignment:
            if ci < len(orders) and orders[ci] in selected_orders \
                    and (robots[ri], orders[ci]) in raw_cost:
                matches.append((robots[ri], orders[ci]))
        self.last_decisions = []
        selected = set(matches)
        for r in robots:
            for o in orders:
                metrics = pair_metrics.get((r, o), {})
                self.last_decisions.append({
                    "robot_id": r, "order_id": o,
                    "feasible": (r, o) in raw_cost,
                    "selected": (r, o) in selected,
                    "cost": round(raw_cost[(r, o)], 8) if (r, o) in raw_cost else None,
                    "reason": metrics.get("reason", ""),
                    **{k: metrics[k] for k in fields if k in metrics},
                })
        return sorted(matches, key=lambda pair: (orders.index(pair[1]), pair[0]))
