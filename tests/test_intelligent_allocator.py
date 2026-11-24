from aifr.scheduling.intelligent_allocator import IntelligentAllocator, hungarian_min_cost


def test_hungarian_known_three_by_three_matrix():
    matrix = [[18, 8, 25], [6, 21, 14], [17, 12, 9]]
    assignment = hungarian_min_cost(matrix)
    assert {(r + 1, c + 1) for r, c in assignment} == {(1, 2), (2, 1), (3, 3)}
    assert sum(matrix[r][c] for r, c in assignment) == 23


def test_hungarian_handles_both_rectangular_directions():
    assert len(hungarian_min_cost([[4, 1, 5], [2, 3, 9]])) == 2
    assert len(hungarian_min_cost([[4, 1], [2, 3], [0, 8]])) == 2


def test_allocator_maximizes_feasible_count_before_cost():
    metrics = {
        (1, 1): {"feasible": True, "distance_m": 1, "time_s": 1,
                 "battery_fraction": 0.1, "workload": 0, "degradation_fraction": 0.1},
        (1, 2): {"feasible": True, "distance_m": 10, "time_s": 10,
                 "battery_fraction": 0.9, "workload": 0, "degradation_fraction": 0.9},
        (2, 1): {"feasible": True, "distance_m": 2, "time_s": 2,
                 "battery_fraction": 0.2, "workload": 0, "degradation_fraction": 0.2},
    }
    matches = IntelligentAllocator().allocate([1, 2], [1, 2], metrics)
    assert len(matches) == 2
    assert set(matches) == {(1, 2), (2, 1)}


def test_allocator_returns_no_matches_when_every_pair_is_infeasible():
    metrics = {(robot, order): {"feasible": False, "reason": "reserve"}
               for robot in (1, 2) for order in (10, 11)}
    assert IntelligentAllocator().allocate([1, 2], [10, 11], metrics) == []


def test_allocator_skips_an_infeasible_competitor_but_keeps_oldest_matchable_orders():
    def pair(feasible):
        return {"feasible": feasible, "distance_m": 1, "time_s": 1,
                "battery_fraction": 0.1, "workload": 0, "degradation_fraction": 0.1}
    metrics = {
        (1, 1): pair(True), (2, 1): pair(False),
        (1, 2): pair(True), (2, 2): pair(False),
        (1, 3): pair(False), (2, 3): pair(True),
    }
    assert set(IntelligentAllocator().allocate([1, 2], [1, 2, 3], metrics)) == {
        (1, 1), (2, 3)}


def test_allocator_is_deterministic_for_equal_costs_and_oldest_order_list():
    metrics = {(robot, order): {"feasible": True, "distance_m": 1, "time_s": 1,
                 "battery_fraction": 0.1, "workload": 0, "degradation_fraction": 0.1}
               for robot in (2, 1) for order in (12, 11)}
    allocator = IntelligentAllocator()
    assert allocator.allocate([2, 1], [11, 12], metrics) == [(1, 11), (2, 12)]
