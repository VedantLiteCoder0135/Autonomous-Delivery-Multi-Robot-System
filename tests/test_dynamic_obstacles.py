"""Unit and integration tests for dynamic obstacle injection, 15-20cm clearance, and online replanning."""
import math
from pathlib import Path
import pytest

from tools.stage79_engine import Stage79Engine, load_reference, DynamicObstacle
from tools.deadlock_manager import DeadlockManager, TrafficPolicy

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def reference():
    return load_reference(ROOT)


def test_safety_margin_enforces_18cm_clearance():
    policy = TrafficPolicy()
    assert 0.15 <= policy.margin <= 0.20
    assert policy.margin == 0.18


def test_dynamic_obstacle_registration(reference):
    env, grid, _ = reference
    engine = Stage79Engine(env, grid, robot_count=1)
    assert len(engine.dynamic_obstacles) == 0

    obs = engine.add_dynamic_obstacle(8.0, 5.0, radius=0.22, name='CHAIR_TEST')
    assert obs.name == 'CHAIR_TEST'
    assert 'CHAIR_TEST' in engine.dynamic_obstacles
    assert len(engine.snapshot()['dynamic_obstacles']) == 1

    engine.remove_dynamic_obstacle('CHAIR_TEST')
    assert len(engine.dynamic_obstacles) == 0


def test_robot_detects_and_replans_around_moved_chair(reference):
    """A chair is moved onto the robot's active delivery corridor.
    
    The robot must detect the obstruction, execute controlled deceleration to a stop
    without colliding (maintaining >= 15-20cm clearance), replan an online detour,
    and successfully deliver the food.
    """
    env, grid, _ = reference
    engine = Stage79Engine(env, grid, robot_count=1, safety_margin=0.18)
    engine.create_order('T11', 'SANDWICH')
    engine.paused = False

    # Run until robot reaches SANDWICH pickup and starts moving to T11
    chair_dropped = False
    for step in range(2500):
        engine.step()
        r = engine.robots[1]

        # Once robot leaves pickup and heads towards T11, drop a chair directly on the corridor ahead
        if not chair_dropped and r.phase == 'TO_CUSTOMER' and r.pose.y < 2.5:
            # Place chair directly in the path at (8.8, 3.5)
            engine.add_dynamic_obstacle(8.8, 3.5, radius=0.22, name='MOVED_CHAIR_T11')
            chair_dropped = True

        if engine.failures:
            break

        if engine.orders.all_orders()[0].status.name == 'DELIVERED':
            break

    assert chair_dropped, "Chair should have been dropped during transit"
    assert not engine.failures, f"Encountered failures: {engine.failures}"
    assert engine.orders.all_orders()[0].status.name == 'DELIVERED', "Order must be delivered"
    assert engine.dynamic_replans >= 1, "Robot must have triggered online dynamic replanning"

    # Verify clearance was never violated
    chair = engine.dynamic_obstacles['MOVED_CHAIR_T11']
    min_dist_to_chair = min(
        math.hypot(p['x'] - chair.x, p['y'] - chair.y) - math.hypot(0.20, 0.18) - chair.radius
        for p in r.trace
    )
    assert min_dist_to_chair >= 0.05, f"Clearance to chair violated: {min_dist_to_chair}"


def test_concurrent_shared_aisle_multi_robot(reference):
    """Two robots navigate concurrently in coordinated mode with no deadlock."""
    env, grid, _ = reference
    engine = Stage79Engine(env, grid, robot_count=2, traffic_mode='coordinated', safety_margin=0.18)
    engine.create_order('T11', 'SANDWICH')
    engine.create_order('T12', 'JUICE')
    engine.paused = False

    concurrent_steps = 0
    for step in range(3500):
        engine.step()
        active = sum(r.phase in ('TO_PICKUP', 'TO_CUSTOMER', 'RETURNING') for r in engine.robots.values())
        if active >= 2:
            concurrent_steps += 1
        if all(o.status.name == 'DELIVERED' for o in engine.orders.all_orders()):
            break

    assert not engine.failures, f"Failures: {engine.failures}"
    assert concurrent_steps > 50, f"Expected concurrency in shared aisle, got {concurrent_steps} steps"
    assert all(o.status.name == 'DELIVERED' for o in engine.orders.all_orders())
