"""Complete AIFR Blender-native live control. No network or localhost service.

Launch Blender with recovered V13 .blend and --python tools/blender_integration_live.py.
In viewport press N > AIFR. Runtime is owned by Blender's main-thread timer;
the engine is the existing Stages 1–9 Python engine, not a second simulator.
The authoritative scene is never saved or modified by this controller.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[1]
for folder in (ROOT, ROOT/'src'):
    if str(folder) not in sys.path:
        sys.path.insert(0,str(folder))

from aifr.battery import BatteryConfig
from tools.stage79_engine import Stage79Engine, load_reference
from tools.customer_flow import CustomerFlow
from tools.customer_seating import FREE, OCCUPIED, RESERVED
from tools.customer_pose_adapter import update_customer_pose, clear_stale_poses
from tools.customer_delivery_adapter import CustomerDeliveryBridge

COLLECTION = 'AIFR_LIVE_CUSTOMER_SIM'
ENGINE = None
FLOW = None
ERR = ''
DELIVERY_BRIDGE = None
PHYSICS_STEP_S = .05
TIMER_INTERVAL_S = .01
TICK_BUDGET_S = .018
VIEWPORT_INTERVAL_S = .05
LAST_UPDATE_S = None
STEP_CREDIT = 0.0
NEXT_RENDER_S = 0.0
# Keep shared engine defaults for the other entry points.
LIVE_BATTERY_CONFIG = BatteryConfig(dispatch_soc_fraction=.15, movement_wh_per_m=.825,
                                   rotation_wh_per_rad=.066, idle_power_w=13.2)
ROBOT_CHOICES = tuple((str(i), f'Robot {i}', '', i) for i in range(1, 5))
ROBOT_COLORS = ((.10,.7,.45,1),(.20,.45,.93,1),(.98,.68,.14,1),(.65,.30,.90,1))


OBSTACLE_LOCATIONS = {
    'T11_AISLE': ('Table 11 Aisle', 8.2, 5.0),
    'MAIN_CORRIDOR': ('Main Central Corridor', 8.0, 3.2),
    'T12_APPROACH': ('Table 12 Approach', 13.5, 7.5),
    'CASH_CORRIDOR': ('Cashier Corridor', 2.8, 2.5),
}


def reset_runtime():
    global ENGINE, FLOW, ERR, DELIVERY_BRIDGE
    env, grid, _ = load_reference(ROOT)
    traffic_mode = getattr(bpy.context.scene, 'aifr_live_traffic_mode', 'coordinated') if hasattr(bpy.context, 'scene') else 'coordinated'
    # Forward reference means the same engine owns customer obstacle readings.
    ENGINE = Stage79Engine(env, grid, robot_count=1,
                            pedestrians=lambda t: FLOW(t) if FLOW is not None else (),
                            traffic_mode=traffic_mode,
                            safety_margin=0.18,
                            battery_enabled=True, charging_enabled=True,
                            battery_config=LIVE_BATTERY_CONFIG)
    FLOW = CustomerFlow(ENGINE, customer_seating_enabled=True)
    scene = bpy.context.scene
    scene.aifr_battery_robot = '1'
    scene.aifr_battery_percent = ENGINE.batteries.get(1).soc_percent
    scene.aifr_battery_multiplier = ENGINE.battery_consumption_multiplier
    DELIVERY_BRIDGE = CustomerDeliveryBridge()
    reset_tick_schedule()
    if FLOW.seating is not None:
        audit = FLOW.seating.audit()
        if not audit['expected_count_matches'] or audit['unreachable_or_unmapped_chairs']:
            print('AIFR customer seating audit warning:', audit)
        # Exported chair IDs name the seat mesh itself in the recovered scene.
        mismatched = []
        for seat in FLOW.seating.seats.values():
            mesh = bpy.data.objects.get(seat.seat_id)
            if (mesh is None or mesh.type != 'MESH'
                    or math.dist(tuple(mesh.matrix_world.translation)[:2], seat.position) > .01):
                mismatched.append(seat.seat_id)
        if mismatched:
            print('AIFR chair scene mismatch; inspect geometry before demo:', mismatched)
    ERR = ''
    return ENGINE


def entity_collection():
    coll = bpy.data.collections.get(COLLECTION)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION)
        bpy.context.scene.collection.children.link(coll)
    return coll


def material(name, color):
    mat=bpy.data.materials.get(name)
    if mat is None:
        mat=bpy.data.materials.new(name)
        mat.diffuse_color=color
    return mat


def marker(name, radius, color, *, depth=.24):
    obj=bpy.data.objects.get(name)
    if obj and obj.type == 'MESH':return obj
    if obj:
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.ops.mesh.primitive_cylinder_add(vertices=16,radius=radius,depth=depth,location=(0,0,.26))
    obj=bpy.context.object
    obj.name=name
    for c in tuple(obj.users_collection):
        c.objects.unlink(obj)
    entity_collection().objects.link(obj)
    obj.data.materials.append(material('AIFR_MAT_'+name,color))
    return obj


def robot_battery_readout(robot):
    battery = ENGINE.batteries.get(robot.robot_id)
    return (battery.soc_percent,
            ENGINE.charging.phase.get(robot.robot_id, 'IDLE' if ENGINE.charging_enabled else 'OFF'),
            ENGINE.charging.robot_station.get(robot.robot_id),
            battery.soc <= ENGINE.battery_config.low_soc_fraction)


def viewport_label(name, body, location):
    obj = bpy.data.objects.get(name)
    if obj is None:
        font = bpy.data.curves.new(name, 'FONT')
        font.align_x = 'CENTER'
        font.size = .14
        obj = bpy.data.objects.new(name, font)
        entity_collection().objects.link(obj)
        obj.rotation_euler.x = math.radians(35)
        obj.show_in_front = True
    obj.data.body = body
    obj.location = location
    return obj


def charger_status(station_id):
    owners = sorted(ENGINE.charging.owners[station_id])
    if station_id in ENGINE.charging.failed_stations:
        state = 'Offline'
    elif any(ENGINE.charging.phase[rid] in ('CHARGING', 'CHARGE_COMPLETE') for rid in owners):
        state = 'Occupied'
    elif owners:
        state = 'Reserved'
    else:
        station = ENGINE.charging.stations[station_id]
        parked = [f'R{robot.robot_id}' for robot in ENGINE.robots.values()
                  if math.dist((robot.pose.x, robot.pose.y), (station.pose.x, station.pose.y))
                  <= ENGINE._charger_radius]
        state = f'Pad occupied by {", ".join(parked)}' if parked else 'Available'
    return state, owners


def render():
    if ENGINE is None:return
    desired=set()
    for r in ENGINE.robots.values():
        name=f'AIFR_SIM_ROBOT_{r.robot_id}'
        desired.add(name)
        obj=marker(name,.22,ROBOT_COLORS[(r.robot_id-1)%len(ROBOT_COLORS)])
        obj.location=(r.pose.x,r.pose.y,.26)
        obj.rotation_euler.z=r.pose.theta
        soc, charging_state, station_id, low = robot_battery_readout(r)
        label = f'{name}_BATTERY'
        desired.add(label)
        status = r.phase.replace('_', ' ')
        if charging_state == 'CHARGE_COMPLETE' and r.phase == 'IDLE':
            status = 'IDLE / CHARGE COMPLETE'
        viewport_label(label, f'R{r.robot_id}  {soc:.1f}%{" ! LOW" if low else ""}\n'
                       f'{status}', (r.pose.x, r.pose.y, .85))
        obj['battery_percent'] = soc
        obj['charging_state'] = charging_state
        obj['charging_station'] = station_id or ''
        obj['phase']=r.phase
        obj['order_id']=r.order_id or 0
        obj['navigation_state']=r.navigation_state
        obj['blocked_reason']=r.blocked_reason
        obj['retreating']=r.retreating
        obj['hold']=r.hold_reason
        obj['yields']=r.yields
    for station_id, station in ENGINE.charging.stations.items():
        name = f'AIFR_SIM_CHARGER_{station_id}'
        label = f'{name}_LABEL'
        desired.update((name, label))
        state, owners = charger_status(station_id)
        pad = marker(name, .38, (.12, .78, .91, 1), depth=.035)
        pad.location = (station.pose.x, station.pose.y, .035)
        pad.color = (.12, .78, .91, 1)
        pad['charger_state'] = state
        pad['reservation'] = ', '.join(f'R{rid}' for rid in owners)
        viewport_label(label, f'CHARGER {station_id}\n{state}',
                       (station.pose.x, station.pose.y - .50, .12))
    visible_customer_ids = set()
    for c in FLOW.customers.values():
        if c.phase in ('QUEUED','LEFT'):
            continue
        visible_customer_ids.add(c.customer_id)
        name=f'AIFR_SIM_CUSTOMER_{c.customer_id}'
        desired.add(name)
        if FLOW.customer_seating_enabled:
            obj=update_customer_pose(c, entity_collection())
        else:
            obj=marker(name,.15,(.95,.38,.25,1))
            obj.location=(c.x,c.y,.32)
        obj['phase']=c.phase
        obj['table']=c.table
        obj['seat_id']=c.seat_id or ''
        obj['order_id']=c.order_id or 0
        obj['customer_id']=c.customer_id
    clear_stale_poses(visible_customer_ids if FLOW.customer_seating_enabled else set(),
                      entity_collection())
    walkers = getattr(ENGINE.walkers, 'agents', {})
    for walker in walkers.values():
        person = walker.at(ENGINE.clock.now()/1e6)
        name = f'AIFR_SIM_PATROL_{walker.pedestrian_id}'
        desired.add(name)
        obj = marker(name, person.radius, (.95,.55,.15,1))
        obj.location = (person.x, person.y, .32)
        obj['phase'] = 'PATROL'
    for ent in ENGINE.env.entrances:
        name='AIFR_SIM_ENTRANCE_'+ent.name
        desired.add(name)
        obj=marker(name,.13,(.95,.75,.18,1))
        obj.location=(ent.center_x,ent.center_y,.16)
    for obs in ENGINE.dynamic_obstacles.values():
        name=f'AIFR_SIM_OBSTACLE_{obs.name}'
        desired.add(name)
        obj=marker(name,obs.radius,(.85,.45,.15,1))
        obj.location=(obs.x,obs.y,.24)
        obj['obstacle_name']=obs.name
    for obj in tuple(entity_collection().objects):
        if obj.name.startswith('AIFR_SIM_') and obj.name not in desired:
            bpy.data.objects.remove(obj,do_unlink=True)
    scene=bpy.context.scene
    scene.aifr_live_status=f'{ENGINE.clock.now()/1e6:.1f}s | {len(ENGINE.robots)} robots | {len(FLOW.customers)} customers | {len(ENGINE.orders.all_orders())} orders'
    scene.aifr_live_fault='; '.join(ENGINE.failures[-2:]) or ERR


def one_step(*, draw=True):
    # Human observations precede robot safety checks. The existing scheduler
    # owns delivery; the bridge reads its confirmed events after this tick.
    FLOW.step(PHYSICS_STEP_S, now_real_s=time.monotonic())
    ENGINE.step()
    DELIVERY_BRIDGE.consume(ENGINE, FLOW, now_real_s=time.monotonic())
    if draw:
        render()


def reset_tick_schedule():
    global LAST_UPDATE_S, STEP_CREDIT, NEXT_RENDER_S
    LAST_UPDATE_S = None
    STEP_CREDIT = 0.0
    NEXT_RENDER_S = 0.0


def timer_update():
    global ERR, LAST_UPDATE_S, STEP_CREDIT, NEXT_RENDER_S
    try:
        if ENGINE is None or ENGINE.paused or ENGINE.estopped:
            reset_tick_schedule()
            return VIEWPORT_INTERVAL_S
        start = time.perf_counter()
        elapsed = TIMER_INTERVAL_S if LAST_UPDATE_S is None else start - LAST_UPDATE_S
        LAST_UPDATE_S = start
        speed = int(bpy.context.scene.aifr_live_speed)
        # Time spent computing and drawing counts towards the next batch.
        # Bound catch-up after a stalled frame so controls stay responsive.
        max_credit = .10 * speed / PHYSICS_STEP_S
        STEP_CREDIT = min(max_credit, STEP_CREDIT + elapsed * speed / PHYSICS_STEP_S)
        while STEP_CREDIT + 1e-9 >= 1:
            one_step(draw=False)
            STEP_CREDIT = max(0., STEP_CREDIT - 1)
            if time.perf_counter() - start >= TICK_BUDGET_S:
                break
        if start >= NEXT_RENDER_S:
            render()
            NEXT_RENDER_S = start + VIEWPORT_INTERVAL_S
        return .001 if STEP_CREDIT >= 1 else TIMER_INTERVAL_S
    except Exception as exc:
        ERR=f'{type(exc).__name__}: {exc}'
        if ENGINE is not None:
            ENGINE.paused = True
            bpy.context.scene.aifr_live_fault = ERR
        print('AIFR Blender runtime error:',ERR)
        return VIEWPORT_INTERVAL_S


def entrances(self,context):
    if FLOW is None:return [('AUTO','Automatic','Cycle through verified doors')]
    return [('AUTO','Automatic','Choose a verified entrance')] + [
        (key,key.replace('_',' ').title(),f'Walk in through {key}') for key in FLOW.entrance_points]


def stations(self,context):
    if ENGINE is None:return [('SANDWICH','SANDWICH','')]
    return [(x,x,'') for x in sorted(ENGINE.pickups)]


def battery_robots(self, context):
    return ROBOT_CHOICES[:len(ENGINE.robots) if ENGINE is not None else 0]


def update_battery_robot(self, context):
    if ENGINE is not None and self.aifr_battery_robot:
        self.aifr_battery_percent = ENGINE.batteries.get(int(self.aifr_battery_robot)).soc_percent


def apply_battery_controls(scene, action):
    if action == 'set_battery':
        ENGINE.batteries.set_soc_percent(int(scene.aifr_battery_robot), scene.aifr_battery_percent)
    elif action == 'set_consumption':
        ENGINE.set_battery_consumption_multiplier(scene.aifr_battery_multiplier)
    elif action == 'restore_battery_defaults':
        ENGINE.set_battery_consumption_multiplier(1.0)
        scene.aifr_battery_multiplier = ENGINE.battery_consumption_multiplier
        update_battery_robot(scene, None)


def draw_battery_panel(layout, scene):
    panel = layout.box()
    panel.label(text='Robot Battery & Charging')
    for robot in ENGINE.robots.values():
        soc, charging_state, station_id, low = robot_battery_readout(robot)
        row = panel.column(align=True)
        row.progress(factor=soc / 100, text=f'R{robot.robot_id}  {soc:.1f}%')
        row.label(text=f'{robot.phase.replace("_", " ")} | {charging_state.replace("_", " ")}'
                       + (f' | O{robot.order_id}' if robot.order_id else ''))
        if station_id:
            row.label(text=f'Assigned charger  {station_id}')
        if low:
            row.label(text='Low battery', icon='ERROR')
        if robot.retreating or robot.hold_reason:
            row.label(text=f'Yielding  {robot.hold_reason or "retreat"}', icon='TIME')
        elif robot.blocked_reason:
            row.label(text=robot.blocked_reason[:55], icon='ERROR')
    if not ENGINE.charging_enabled:
        panel.label(text=ENGINE.charging_blocker, icon='ERROR')
    for station_id in ENGINE.charging.stations:
        status, owners = charger_status(station_id)
        reservation = '  ' + ', '.join(f'R{rid}' for rid in owners) if owners else ''
        panel.label(text=f'{station_id}  {status}{reservation}')
    queue = ', '.join(f'R{rid}' for rid in ENGINE.charging.queue)
    panel.label(text=f'Charging queue  {queue or "Empty"}')
    panel.prop(scene, 'aifr_battery_robot')
    panel.prop(scene, 'aifr_battery_percent', slider=True)
    panel.operator('aifr.native_control', text='Apply Battery Level').action = 'set_battery'
    panel.prop(scene, 'aifr_battery_multiplier')
    panel.label(text=f'Active consumption  {ENGINE.battery_consumption_multiplier:g}×')
    panel.operator('aifr.native_control', text='Apply Consumption').action = 'set_consumption'
    panel.operator('aifr.native_control', text='Restore Battery Defaults').action = 'restore_battery_defaults'


class AIFR_OT_live_command(bpy.types.Operator):
    bl_idname='aifr.native_control'
    bl_label='AIFR simulation control'
    action:bpy.props.StringProperty()

    def execute(self,context):
        global ERR
        try:
            if self.action=='reset':
                reset_runtime()
            elif self.action=='start':
                if ENGINE.estopped:raise ValueError('Reset after emergency stop')
                ENGINE.paused=False
                reset_tick_schedule()
            elif self.action=='retry_blocked':
                ENGINE.retry_blocked()
            elif self.action=='pause':
                ENGINE.paused=True
            elif self.action=='step':
                if ENGINE.estopped:raise ValueError('Reset after emergency stop')
                was_paused=ENGINE.paused
                ENGINE.paused=False
                one_step()
                ENGINE.paused=was_paused
            elif self.action=='add_robot':
                ENGINE.add_robot()
            elif self.action in ('set_battery', 'set_consumption', 'restore_battery_defaults'):
                apply_battery_controls(context.scene, self.action)
            elif self.action=='estop':
                ENGINE.estopped=True
                ENGINE.paused=True
            else:
                raise ValueError('Unknown action')
            ERR=''
            render()
            return {'FINISHED'}
        except Exception as exc:
            ERR=str(exc)
            self.report({'ERROR'},ERR)
            return {'CANCELLED'}


class AIFR_OT_speed(bpy.types.Operator):
    bl_idname='aifr.native_speed'
    bl_label='Set simulation speed'
    multiplier:bpy.props.StringProperty(default='1')

    def execute(self,context):
        if self.multiplier not in ('1','2','4','8','16'):
            self.report({'ERROR'},'Invalid simulation speed')
            return {'CANCELLED'}
        context.scene.aifr_live_speed=self.multiplier
        reset_tick_schedule()
        return {'FINISHED'}


class AIFR_OT_add_customer(bpy.types.Operator):
    bl_idname='aifr.native_add_customer'
    bl_label='Add customer through entrance'
    rush:bpy.props.BoolProperty(default=False)

    def execute(self,context):
        global ERR
        s=context.scene
        try:
            if ENGINE.estopped:raise ValueError('Reset after emergency stop')
            entrance=s.aifr_live_entrance
            if self.rush:
                ids = FLOW.add_rush(count=s.aifr_rush_count, station=s.aifr_live_station,
                                    entrance=None if entrance == 'AUTO' else entrance)
                ENGINE.paused = False
                reset_tick_schedule()
                ERR = ''
                self.report({'INFO'}, f'Queued {len(ids)} rush customers.')
                render()
                return {'FINISHED'}
            if entrance=='AUTO':
                options=list(FLOW.entrance_points)
                entrance=min(options,key=lambda key:sum(c.phase=='QUEUED' and c.entrance==key for c in FLOW.customers.values()))
            pid=FLOW.add(entrance=entrance, station=s.aifr_live_station)
            ERR=''
            self.report({'INFO'},f'Customer {pid} entered via {entrance}. Payment auto-creates order.')
            render()
            return {'FINISHED'}
        except Exception as exc:
            ERR=str(exc)
            self.report({'ERROR'},ERR)
            return {'CANCELLED'}


class AIFR_OT_obstacle(bpy.types.Operator):
    bl_idname = 'aifr.native_obstacle'
    bl_label = 'Dynamic Obstacle Control'
    action: bpy.props.StringProperty(default='add')

    def execute(self, context):
        global ERR
        try:
            if ENGINE is None:
                raise ValueError('Engine not initialized')
            if self.action == 'add':
                loc_key = context.scene.aifr_live_obstacle_location
                info = OBSTACLE_LOCATIONS.get(loc_key, ('Custom', 8.2, 5.0))
                obs_name = f"CHAIR_{len(ENGINE.dynamic_obstacles) + 1}"
                ENGINE.add_dynamic_obstacle(info[1], info[2], radius=0.22, name=obs_name)
                self.report({'INFO'}, f"Moved Chair {obs_name} dropped at {info[0]} ({info[1]}, {info[2]})")
            elif self.action == 'clear':
                ENGINE.clear_dynamic_obstacles()
                self.report({'INFO'}, "All dynamic obstacles cleared")
            render()
            return {'FINISHED'}
        except Exception as exc:
            ERR = str(exc)
            self.report({'ERROR'}, ERR)
            return {'CANCELLED'}


class AIFR_PT_native(bpy.types.Panel):
    bl_label='AIFR Integrated Live Cafeteria'
    bl_idname='AIFR_PT_NATIVE'
    bl_space_type='VIEW_3D'
    bl_region_type='UI'
    bl_category='AIFR'

    def draw(self,context):
        box=self.layout
        s=context.scene
        box.label(text=s.aifr_live_status)
        if s.aifr_live_fault:
            box.label(text=s.aifr_live_fault[:65],icon='ERROR')
        row=box.row(align=True)
        for action in ('start','pause','step'):
            row.operator('aifr.native_control',text=action.title()).action=action
        row=box.row(align=True)
        row.operator('aifr.native_control',text='Add Robot').action='add_robot'
        row.operator('aifr.native_control',text='Reset').action='reset'
        box.operator('aifr.native_control',text='Retry Blocked').action='retry_blocked'
        box.operator('aifr.native_control',text='Emergency Stop',icon='CANCEL').action='estop'
        box.separator()
        box.label(text='Traffic & Aisle Mode:')
        box.prop(s, 'aifr_live_traffic_mode', text='Mode')
        box.separator()
        box.label(text='Simulation speed')
        speed_row=box.row(align=True)
        for value in ('1','2','4','8','16'):
            op=speed_row.operator('aifr.native_speed',text=value+'x',depress=(s.aifr_live_speed==value))
            op.multiplier=value
        box.label(text='Customers')
        box.prop(s,'aifr_live_entrance')
        box.prop(s,'aifr_live_station')
        box.operator('aifr.native_add_customer',text='Add Customer',icon='OUTLINER_OB_ARMATURE')
        rush_row = box.row(align=True)
        rush_row.prop(s, 'aifr_rush_count')
        rush_row.operator('aifr.native_add_customer', text='Start Rush').rush = True
        if FLOW is not None and FLOW.cash_queue_slots:
            waiting = sum(c.phase == 'QUEUED' for c in FLOW.customers.values())
            at_cash = sum(c.phase == 'CASH_QUEUE' for c in FLOW.customers.values())
            box.label(text=f'Entrance queue  {waiting} | CASH queue  {at_cash}')
        box.separator()
        if FLOW is not None and FLOW.seating is not None:
            seats = FLOW.seating.seats.values()
            available = sum(seat.status == FREE for seat in seats)
            reserved = sum(seat.status == RESERVED for seat in seats)
            occupied = sum(seat.status == OCCUPIED for seat in seats)
            box.label(text=f'Seats  {FLOW.seating.scene_chair_count} total | {available} available')
            box.label(text=f'{reserved} reserved | {occupied} occupied')
            box.label(text='Random seat allocation active' if available else 'Waiting for a free seat')
        box.label(text='Payment creates order automatically.')
        box.label(text='Order READY after sitting; eating begins after serving.')
        box.label(text='Customers / orders:')
        if FLOW is not None:
            for c in list(FLOW.customers.values())[-5:]:
                meal = FLOW.meals.meals.get(c.customer_id)
                remaining = (f" | eat {max(0., meal.deadline_real_s - time.monotonic()):.0f}s"
                             if meal and meal.deadline_real_s is not None
                             and c.phase in ('SEATED','EATING') else "")
                box.label(text=(f'C{c.customer_id} {c.phase} {c.table} '
                                f'{c.seat_id or "unassigned"} O{c.order_id or "—"}{remaining}')[:95])
        if ENGINE is not None:
            box.label(text='Order lifecycle:')
            for order in ENGINE.snapshot(compact=True)['orders'][-5:]:
                box.label(text=f"O{order['id']} C{order['customer_id'] or '-'} "
                               f"R{order['robot_id'] or '-'}: {order['status']}")
        if ENGINE is not None:
            draw_battery_panel(box, s)
        box.separator()
        box.label(text='Dynamic Obstacles (Moved Chair / Obstruction):')
        box.prop(s, 'aifr_live_obstacle_location', text='Location')
        obs_row = box.row(align=True)
        obs_row.operator('aifr.native_obstacle', text='Drop Moved Chair', icon='PHYSICS').action = 'add'
        obs_row.operator('aifr.native_obstacle', text='Clear Obstacles', icon='TRASH').action = 'clear'
        box.separator()
        box.label(text=f'Traffic: {ENGINE.traffic_mode.title()} (margin: {int(ENGINE.safety_margin*100)}cm)' if ENGINE else '')
        box.label(text=f'Obstacles: {len(ENGINE.dynamic_obstacles)} | Replans: {ENGINE.dynamic_replans}' if ENGINE else '')
        box.label(text=f'Blocked robots: {sum(r.navigation_state == "BLOCKED" for r in ENGINE.robots.values())}' if ENGINE else '')
        box.label(text=f'Pedestrian yields: {ENGINE.deadlock_manager.customer_yields}' if ENGINE else '')
        box.label(text=(f'Deadlocks resolved: {ENGINE.deadlocks_resolved} | Retreats: {ENGINE.retreats}'
                        f' | Lane waits: {ENGINE.corridor_waits}') if ENGINE else '')
        box.label(text='Speed target depends on available CPU.')
        box.label(text='Python engine runs inside Blender.')
        box.label(text='No localhost / server required.')


CLASSES=(AIFR_OT_live_command,AIFR_OT_speed,AIFR_OT_add_customer,AIFR_OT_obstacle,AIFR_PT_native)


def register():
    # Avoid simultaneously running the older HTTP client in the same scene.
    if bpy.app.timers.is_registered(timer_update):
        bpy.app.timers.unregister(timer_update)
    for cls in CLASSES:
        try:bpy.utils.register_class(cls)
        except ValueError:pass
    scene=bpy.types.Scene
    scene.aifr_live_status=bpy.props.StringProperty(default='Initializing')
    scene.aifr_live_speed=bpy.props.EnumProperty(name='Speed',items=[('1','1x','Normal speed'),('2','2x','Double'),('4','4x','Quadruple'),('8','8x','Eight times'),('16','16x','Sixteen times')],default='1')
    scene.aifr_live_fault=bpy.props.StringProperty(default='')
    scene.aifr_live_entrance=bpy.props.EnumProperty(name='Entrance',items=entrances)
    scene.aifr_live_station=bpy.props.EnumProperty(name='Food station',items=stations)
    scene.aifr_rush_count=bpy.props.IntProperty(name='Rush size', default=36, min=1, max=72)
    scene.aifr_battery_robot = bpy.props.EnumProperty(
        name='Robot', items=battery_robots, update=update_battery_robot)
    scene.aifr_battery_percent = bpy.props.FloatProperty(
        name='Battery %', min=0, max=100, precision=1)
    scene.aifr_battery_multiplier = bpy.props.FloatProperty(
        name='Consumption multiplier', default=1, min=.01, max=100,
        description='Scale driving, turning and standby energy use')
    def _on_traffic_mode_change(self, context):
        global ENGINE
        if ENGINE is not None:
            ENGINE.traffic_mode = self.aifr_live_traffic_mode

    scene.aifr_live_traffic_mode=bpy.props.EnumProperty(name='Traffic Mode',items=[('coordinated','Coordinated','Concurrent shared aisle'),('exclusive','Exclusive','Serialized')],default='coordinated',update=_on_traffic_mode_change)
    scene.aifr_live_obstacle_location=bpy.props.EnumProperty(name='Obstacle Location',items=[('T11_AISLE','Table 11 Aisle','Drop in front of T11'),('MAIN_CORRIDOR','Main Corridor','Drop in center'),('T12_APPROACH','Table 12 Approach','Drop near T12'),('CASH_CORRIDOR','Cashier Corridor','Drop near CASH')],default='T11_AISLE')
    reset_runtime()
    render()
    bpy.app.timers.register(timer_update, first_interval=TIMER_INTERVAL_S, persistent=False)
    print('AIFR NATIVE LIVE: N sidebar > AIFR; no HTTP server used')


register()
