"""Headless integration smoke test; run with Blender --background --python.

Loads the native controller against the recovered scene without saving it.
Viewport redraw, sidebar interaction and timer responsiveness still need a UI.
"""
from pathlib import Path
import runpy

import bpy


def main():
    root = Path(__file__).resolve().parents[1]
    runtime = runpy.run_path(str(root / 'tools/blender_integration_live.py'))
    engine, flow = runtime['ENGINE'], runtime['FLOW']
    from tools.live_pedestrians import LivePedestrians
    walkers = LivePedestrians(engine.grid)
    walkers.add(4., 5., 4., 6.5, 0.)
    engine.walkers = walkers
    engine.pedestrians = lambda t: tuple(flow(t)) + tuple(walkers(t))
    for _ in range(3):
        engine.add_robot()
    flow.add(entrance='main_entry', table='T11', station='TEA')
    engine.paused = False
    for _ in range(20):
        runtime['one_step'](draw=False)
    runtime['render']()
    assert engine.clock.tick == 20 and not engine.failures
    for robot in engine.robots.values():
        obj = bpy.data.objects[f'AIFR_SIM_ROBOT_{robot.robot_id}']
        assert abs(obj.location.x - robot.pose.x) < 1e-5
        assert abs(obj.location.y - robot.pose.y) < 1e-5
        assert obj['navigation_state'] == robot.navigation_state
        assert obj['retreating'] == robot.retreating
    for customer in flow.customers.values():
        obj = bpy.data.objects[f'AIFR_SIM_CUSTOMER_{customer.customer_id}']
        assert abs(obj.location.x - customer.x) < 1e-5
        assert abs(obj.location.y - customer.y) < 1e-5
    patrol = walkers(engine.clock.now()/1e6)[0]
    obj = bpy.data.objects['AIFR_SIM_PATROL_1']
    assert abs(obj.location.x - patrol.x) < 1e-5
    assert abs(obj.location.y - patrol.y) < 1e-5
    robot = engine.robots[1]
    engine._block_navigation(robot, 'Headless synchronization check')
    runtime['render']()
    obj = bpy.data.objects['AIFR_SIM_ROBOT_1']
    assert obj['navigation_state'] == 'BLOCKED'
    assert obj['blocked_reason'] == robot.blocked_reason
    bridge = runpy.run_path(str(root / 'tools/blender_live_integration.py'))
    state = engine.snapshot()
    state['pedestrians'] = [{'id': 1, 'x': patrol.x, 'y': patrol.y, 'radius': patrol.radius}]
    bridge['apply_state'](state)
    obj = bpy.data.objects['AIFR_LIVE_ROBOT_1']
    assert obj['navigation_state'] == 'BLOCKED'
    assert obj['blocked_reason'] == robot.blocked_reason
    assert 'Blocked: 1' in bpy.context.scene.aifr_deadlock_stats
    obj = bpy.data.objects['AIFR_LIVE_PEDESTRIAN_1']
    assert abs(obj.location.y - patrol.y) < 1e-5
    print('AIFR_HEADLESS_PASS: native and HTTP bridges, four robots, customer, patrol and blocked state')


if __name__ == '__main__':
    main()
