"""Headless integration smoke test; run with Blender --background --python.

Loads the native controller against the recovered scene without saving it.
Viewport redraw, sidebar interaction and timer responsiveness still need a UI.
"""
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import runpy

import bpy


class SidebarRecorder:
    def __init__(self):
        self.labels = []
        self.properties = []
        self.progress_bars = []
        self.buttons = []

    def box(self):
        return self

    def row(self, **kwargs):
        return self

    column = row

    def separator(self):
        pass

    def label(self, *, text, **kwargs):
        self.labels.append(text)

    def prop(self, scene, name, **kwargs):
        self.properties.append(name)

    def progress(self, *, factor, text):
        self.progress_bars.append((factor, text))

    def operator(self, identifier, *, text, **kwargs):
        self.buttons.append(text)
        return SimpleNamespace()


def check_native_battery_and_seating(runtime):
    engine, flow = runtime['ENGINE'], runtime['FLOW']
    scene = bpy.context.scene
    assert engine.battery_enabled and engine.charging_enabled
    assert engine.battery_config.dispatch_soc_fraction == .15
    assert math.isclose(engine.battery_config.movement_wh_per_m, .825)
    for seat in flow.seating.seats.values():
        mesh = bpy.data.objects[seat.seat_id]
        assert mesh.type == 'MESH'
        assert math.dist(tuple(mesh.matrix_world.translation)[:2], seat.position) < .01
    assert len(flow.seating.seats) == 72

    control = bpy.ops.aifr.native_control
    assert control(action='add_robot') == {'FINISHED'}
    scene.aifr_battery_robot = '2'
    scene.aifr_battery_percent = 14
    robot = engine.robots[2]
    pose = robot.pose
    assert control(action='set_battery') == {'FINISHED'}
    assert robot.pose == pose and robot.phase == 'IDLE'
    assert math.isclose(engine.batteries.get(2).soc_percent, 14)
    sidebar = SidebarRecorder()
    runtime['AIFR_PT_native'].draw(SimpleNamespace(layout=sidebar), bpy.context)
    assert any(math.isclose(factor, .14) and label == 'R2  14.0%'
               for factor, label in sidebar.progress_bars)
    assert 'Low battery' in sidebar.labels
    assert 'Seats  72 total | 72 available' in sidebar.labels
    assert 'Random seat allocation active' in sidebar.labels
    assert 'aifr_live_table' not in sidebar.properties
    assert 'Add Manual Order' not in sidebar.buttons
    assert sidebar.buttons.index('Add Customer') < sidebar.buttons.index('Apply Battery Level')
    obj = bpy.data.objects['AIFR_SIM_ROBOT_2']
    assert math.isclose(obj['battery_percent'], 14)
    assert '14.0%' in bpy.data.objects['AIFR_SIM_ROBOT_2_BATTERY'].data.body

    scene.aifr_battery_multiplier = 2
    assert control(action='set_consumption') == {'FINISHED'}
    assert engine.battery_consumption_multiplier == 2
    assert control(action='restore_battery_defaults') == {'FINISHED'}
    assert engine.battery_consumption_multiplier == 1
    assert math.isclose(engine.batteries.get(2).soc_percent, 14)

    assert control(action='step') == {'FINISHED'}
    pad = engine.charging.robot_station[2]
    assert engine.charging.owners[pad] == {2}
    assert robot.phase == 'CHARGING' and robot.pose == pose
    assert engine.batteries.get(2).soc_percent > 14
    assert bpy.data.objects[f'AIFR_SIM_CHARGER_{pad}']['charger_state'] == 'Occupied'
    assert 'CHARGING' in bpy.data.objects['AIFR_SIM_ROBOT_2_BATTERY'].data.body
    assert control(action='start') == {'FINISHED'}
    for speed, steps in [('1', 2), ('4', 8), ('16', 32)]:
        assert bpy.ops.aifr.native_speed(multiplier=speed) == {'FINISHED'}
        start = engine.clock.tick
        # Ten 10 ms callbacks represent a tenth of a second at each setting.
        for tick in range(10):
            with patch('time.perf_counter', return_value=1 + .01 * tick):
                runtime['timer_update']()
        assert engine.clock.tick - start == steps
    assert control(action='pause') == {'FINISHED'}
    start = engine.clock.tick
    runtime['timer_update']()
    assert engine.clock.tick == start

    scene['aifr_live_table'] = 'STALE_LEGACY_VALUE'
    scene.aifr_live_entrance = 'main_entry'
    scene.aifr_live_station = 'SANDWICH'
    assert bpy.ops.aifr.native_add_customer() == {'FINISHED'}
    customer = next(iter(flow.customers.values()))
    seat = flow.seating.seats[customer.seat_id]
    assert customer.table == seat.table_id
    assert seat.status == 'RESERVED'
    assert control(action='start') == {'FINISHED'}
    for _ in range(5500):
        runtime['one_step'](draw=False)
        meal = flow.meals.meals.get(customer.customer_id)
        if meal and meal.served:
            break
    else:
        raise AssertionError((customer.phase, engine.failures, engine.snapshot()['orders']))
    assert customer.phase == 'SEATED' and seat.status == 'OCCUPIED'
    assert (customer.x, customer.y) == seat.position
    assert str(engine.orders.get(customer.order_id).table_id) == seat.table_id
    assert any(route['purpose'] == 'DELIVERY' and
               route['goal'] in [(w.x, w.y) for w in engine.drops[seat.table_id]]
               for route in engine.routes)
    assert 20 <= meal.deadline_real_s - meal.served_at_real_s <= 40
    runtime['render']()
    avatar = bpy.data.objects[f'AIFR_SIM_CUSTOMER_{customer.customer_id}']
    assert avatar['seat_id'] == seat.seat_id
    assert math.dist(tuple(avatar.location)[:2], seat.position) < .001
    flow.step(.05, now_real_s=meal.deadline_real_s - .01)
    assert seat.status == 'OCCUPIED'
    for _ in range(2500):
        flow.step(.05, now_real_s=meal.deadline_real_s + .01)
        if customer.customer_id not in flow.customers:
            break
    assert customer.customer_id not in flow.customers
    assert seat.status == 'FREE'
    assert not engine.failures
    assert robot.phase == 'CHARGING'
    scene.aifr_battery_percent = 56.9
    assert control(action='set_battery') == {'FINISHED'}
    ready_order = engine.create_order('T11', 'SANDWICH')
    assert control(action='step') == {'FINISHED'}
    assert int(engine.orders.get(ready_order).robot_id) == 2
    assert engine.charging.owners[pad] == {2}
    assert robot.phase == 'TO_PICKUP'
    assert control(action='reset') == {'FINISHED'}
    assert not bpy.data.objects.get('AIFR_SIM_ROBOT_2_BATTERY')
    current = runtime['render'].__globals__
    scene.aifr_live_entrance = 'AUTO'
    scene.aifr_rush_count = 36
    assert bpy.ops.aifr.native_add_customer(rush=True) == {'FINISHED'}
    rush = current['FLOW']
    assert len(rush.customers) == 36 and not current['ENGINE'].paused
    assert len({c.entrance for c in rush.customers.values()}) == 4
    assert any(c.phase == 'TO_CASH_QUEUE' for c in rush.customers.values())
    assert control(action='reset') == {'FINISHED'}
    print('AIFR_V13_PASS: battery, speed, seating, departure, and rush arrival controls')


def main():
    root = Path(__file__).resolve().parents[1]
    runtime = runpy.run_path(str(root / 'tools/blender_integration_live.py'))
    check_native_battery_and_seating(runtime)
    engine, flow = runtime['render'].__globals__['ENGINE'], runtime['render'].__globals__['FLOW']
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
