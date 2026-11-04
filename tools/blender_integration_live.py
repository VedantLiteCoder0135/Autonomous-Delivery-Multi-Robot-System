"""Complete AIFR Blender-native live control. No network or localhost service.

Launch Blender with recovered V14 .blend and --python tools/blender_integration_live.py.
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

from tools.stage79_engine import Stage79Engine, load_reference
from tools.customer_flow import CustomerFlow
from tools.runtime_timing import requested_steps

COLLECTION = 'AIFR_LIVE_CUSTOMER_SIM'
ENGINE = None
FLOW = None
ERR = ''
ROBOT_COLORS = ((.10,.7,.45,1),(.20,.45,.93,1),(.98,.68,.14,1),(.65,.30,.90,1))


OBSTACLE_LOCATIONS = {
    'T11_AISLE': ('Table 11 Aisle', 8.2, 5.0),
    'MAIN_CORRIDOR': ('Main Central Corridor', 8.0, 3.2),
    'T12_APPROACH': ('Table 12 Approach', 13.5, 7.5),
    'CASH_CORRIDOR': ('Cashier Corridor', 2.8, 2.5),
}


def reset_runtime():
    global ENGINE, FLOW, ERR
    env, grid, _ = load_reference(ROOT)
    traffic_mode = getattr(bpy.context.scene, 'aifr_live_traffic_mode', 'coordinated') if hasattr(bpy.context, 'scene') else 'coordinated'
    # Forward reference means the same engine owns customer obstacle readings.
    ENGINE = Stage79Engine(env, grid, robot_count=1,
                            pedestrians=lambda t: FLOW(t) if FLOW is not None else (),
                            traffic_mode=traffic_mode,
                            safety_margin=0.18)
    FLOW = CustomerFlow(ENGINE)
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


def marker(name, radius, color):
    obj=bpy.data.objects.get(name)
    if obj:return obj
    bpy.ops.mesh.primitive_cylinder_add(vertices=16,radius=radius,depth=.24,location=(0,0,.26))
    obj=bpy.context.object
    obj.name=name
    for c in tuple(obj.users_collection):
        c.objects.unlink(obj)
    entity_collection().objects.link(obj)
    obj.data.materials.append(material('AIFR_MAT_'+name,color))
    return obj


def render():
    if ENGINE is None:return
    desired=set()
    for r in ENGINE.robots.values():
        name=f'AIFR_SIM_ROBOT_{r.robot_id}'
        desired.add(name)
        obj=marker(name,.22,ROBOT_COLORS[(r.robot_id-1)%len(ROBOT_COLORS)])
        obj.location=(r.pose.x,r.pose.y,.26)
        obj.rotation_euler.z=r.pose.theta
        obj['phase']=r.phase
        obj['order_id']=r.order_id or 0
    for c in FLOW.customers.values():
        if c.phase in ('QUEUED','LEFT'):continue
        name=f'AIFR_SIM_CUSTOMER_{c.customer_id}'
        desired.add(name)
        obj=marker(name,.15,(.95,.38,.25,1))
        obj.location=(c.x,c.y,.32)
        obj['phase']=c.phase
        obj['table']=c.table
        obj['order_id']=c.order_id or 0
        obj['customer_id']=c.customer_id
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
    # Update pedestrian motion first: the robot safety guard then sees the
    # latest known position. Movement is kinematic (no full crowd physics).
    FLOW.step(.05)
    ENGINE.step()
    if draw:render()


def timer_update():
    global ERR
    try:
        if ENGINE is not None:
            if not ENGINE.paused and not ENGINE.estopped:
                # Fixed 0.05s physics step: 2 steps per 0.1s timer at 1x.
                # Accelerate simulation time, never scale dt or jump the clock.
                # Hard wall-time budget preserves Blender UI responsiveness.
                # Requested speed is a target, not a guaranteed achieved rate.
                wall_start=time.perf_counter()
                for _ in range(requested_steps(bpy.context.scene.aifr_live_speed)):
                    one_step(draw=False)
                    if time.perf_counter()-wall_start >= .075:
                        break
                render()
            else:
                render()
    except Exception as exc:
        ERR=f'{type(exc).__name__}: {exc}'
        if ENGINE is not None: ENGINE.paused=True
        print('AIFR Blender runtime error:',ERR)
    return .10  # 1x is real-time target; actual speed limited by CPU workload.


def entrances(self,context):
    if FLOW is None:return [('AUTO','Automatic','Cycle through verified doors')]
    return [('AUTO','Automatic','Choose a verified entrance')] + [
        (key,key.replace('_',' ').title(),f'Walk in through {key}') for key in FLOW.entrance_points]


def stations(self,context):
    if ENGINE is None:return [('SANDWICH','SANDWICH','')]
    return [(x,x,'') for x in sorted(ENGINE.pickups)]


def tables(self,context):
    if ENGINE is None:return [('T11','T11','')]
    return [(x,x,'') for x in sorted(ENGINE.drops)]


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
            elif self.action=='add_order':
                oid=ENGINE.create_order(context.scene.aifr_live_table,
                                         context.scene.aifr_live_station)
                self.report({'INFO'},f'Manual order {oid} submitted')
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
        return {'FINISHED'}


class AIFR_OT_add_customer(bpy.types.Operator):
    bl_idname='aifr.native_add_customer'
    bl_label='Add customer through entrance'

    def execute(self,context):
        global ERR
        s=context.scene
        try:
            if ENGINE.estopped:raise ValueError('Reset after emergency stop')
            entrance=s.aifr_live_entrance
            if entrance=='AUTO':
                options=list(FLOW.entrance_points)
                entrance=min(options,key=lambda key:sum(c.phase=='QUEUED' and c.entrance==key for c in FLOW.customers.values()))
            pid=FLOW.add(entrance=entrance,table=s.aifr_live_table,
                         station=s.aifr_live_station)
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
        box.label(text='Customer journey: Entrance > CASH > Table')
        box.prop(s,'aifr_live_entrance')
        box.prop(s,'aifr_live_table')
        box.prop(s,'aifr_live_station')
        box.operator('aifr.native_add_customer',text='Add Customer',icon='OUTLINER_OB_ARMATURE')
        box.operator('aifr.native_control',text='Add Manual Order',icon='ADD').action='add_order'
        box.separator()
        box.label(text='Dynamic Obstacles (Moved Chair / Obstruction):')
        box.prop(s, 'aifr_live_obstacle_location', text='Location')
        obs_row = box.row(align=True)
        obs_row.operator('aifr.native_obstacle', text='Drop Moved Chair', icon='PHYSICS').action = 'add'
        obs_row.operator('aifr.native_obstacle', text='Clear Obstacles', icon='TRASH').action = 'clear'
        box.separator()
        box.label(text='Up to 4 customers per table; entrances queue.')
        box.label(text='Payment creates order automatically.')
        box.label(text='Order READY when customer is seated.')
        box.label(text='Customers / orders:')
        if FLOW is not None:
            for c in list(FLOW.customers.values())[-5:]:
                box.label(text=f'C{c.customer_id} {c.phase}  {c.table}  order {c.order_id or "—"}')
        if ENGINE is not None:
            for r in ENGINE.robots.values():
                box.label(text=f'R{r.robot_id}: {r.phase}  order {r.order_id or chr(8212)}')
                if r.blocked_reason:
                    box.label(text=r.blocked_reason[:55],icon='ERROR')
            box.label(text='Order lifecycle:')
            for order in ENGINE.snapshot(compact=True)['orders'][-5:]:
                box.label(text=f"O{order['id']} C{order['customer_id'] or '-'} "
                               f"R{order['robot_id'] or '-'}: {order['status']}")
        box.label(text=f'Traffic: {ENGINE.traffic_mode.title()} (margin: {int(ENGINE.safety_margin*100)}cm)' if ENGINE else '')
        box.label(text=f'Obstacles: {len(ENGINE.dynamic_obstacles)} | Replans: {ENGINE.dynamic_replans}' if ENGINE else '')
        box.label(text=f'Pedestrian yields: {ENGINE.deadlock_manager.customer_yields}' if ENGINE else '')
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
    scene.aifr_live_table=bpy.props.EnumProperty(name='Customer table',items=tables)
    def _on_traffic_mode_change(self, context):
        global ENGINE
        if ENGINE is not None:
            ENGINE.traffic_mode = self.aifr_live_traffic_mode

    scene.aifr_live_traffic_mode=bpy.props.EnumProperty(name='Traffic Mode',items=[('coordinated','Coordinated','Concurrent shared aisle'),('exclusive','Exclusive','Serialized')],default='coordinated',update=_on_traffic_mode_change)
    scene.aifr_live_obstacle_location=bpy.props.EnumProperty(name='Obstacle Location',items=[('T11_AISLE','Table 11 Aisle','Drop in front of T11'),('MAIN_CORRIDOR','Main Corridor','Drop in center'),('T12_APPROACH','Table 12 Approach','Drop near T12'),('CASH_CORRIDOR','Cashier Corridor','Drop near CASH')],default='T11_AISLE')
    reset_runtime()
    bpy.app.timers.register(timer_update,first_interval=.5,persistent=False)
    print('AIFR NATIVE LIVE: N sidebar > AIFR; no HTTP server used')


register()
