"""Live Blender viewport client for the AIFR Python simulation.

Launch the localhost Python dashboard in a separate terminal, then run Blender
with `--python tools/blender_live_integration.py`. Blender renders snapshots;
the Python service owns all simulation state, navigation and collision checks.
The recovered .blend file is never saved or overwritten by this script.
"""
from __future__ import annotations

import json
import math
from urllib.error import URLError
from urllib.request import Request, urlopen

import bpy

API = 'http://127.0.0.1:8765'
COLLECTION = 'AIFR_LIVE_ENTITIES'
ROBOT_COLORS = ((.14,.72,.47,1),(.16,.45,.90,1),(.96,.66,.22,1),(.65,.30,.87,1))


def request(path, data=None):
    payload = None if data is None else json.dumps(data).encode()
    req = Request(API + path, data=payload,
                  headers={'Content-Type':'application/json'} if payload is not None else {})
    with urlopen(req, timeout=0.35) as response:
        return json.loads(response.read().decode())


def layer():
    result = bpy.data.collections.get(COLLECTION)
    if result is None:
        result = bpy.data.collections.new(COLLECTION)
        bpy.context.scene.collection.children.link(result)
    return result


def make_material(name, rgba):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
        mat.diffuse_color = rgba
    return mat


def get_agent(name, radius, rgba):
    existing = bpy.data.objects.get(name)
    if existing is not None:
        return existing
    # Only creates visualization objects. Does not touch recovered V14 geometry.
    bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=radius, depth=0.24,
                                        location=(0,0,0.22))
    obj = bpy.context.object
    obj.name = name
    for col in tuple(obj.users_collection):
        col.objects.unlink(obj)
    layer().objects.link(obj)
    obj.data.materials.append(make_material('AIFR_LIVE_MAT_' + name, rgba))
    return obj


def apply_state(state):
    scene = bpy.context.scene
    scene.aifr_live_status = ('STOPPED' if state.get('estopped') else
                              'PAUSED' if state['paused'] else 'RUNNING') + f"  {state['time_s']:.1f}s"
    stats = state.get('deadlock_stats', {})
    scene.aifr_deadlock_stats = (f"Resolved: {stats.get('resolved', 0)} | "
                                 f"Retreats: {stats.get('retreats', 0)} | "
                                 f"Blocked: {sum(r.get('navigation_state') == 'BLOCKED' for r in state['robots'])}")
    current = set()
    for info in state['robots']:
        name = f"AIFR_LIVE_ROBOT_{info['id']}"
        current.add(name)
        rgba = ROBOT_COLORS[(info['id']-1) % len(ROBOT_COLORS)]
        obj = get_agent(name, .22, rgba)
        obj.location = (info['x'], info['y'], .23)
        obj.rotation_euler = (0,0,info['theta'])
        obj['phase'] = info['phase']
        obj['order_id'] = str(info['order_id'])
        for key in ('navigation_state', 'blocked_reason', 'retreating', 'hold', 'yields'):
            obj[key] = info.get(key, '')
    for info in state.get('pedestrians', []):
        name = f"AIFR_LIVE_PEDESTRIAN_{info['id']}"
        current.add(name)
        obj = get_agent(name, info['radius'], (.95,.35,.28,1))
        obj.location = (info['x'],info['y'],.23)
    for obj in list(layer().objects):
        if obj.name.startswith(('AIFR_LIVE_ROBOT_', 'AIFR_LIVE_PEDESTRIAN_')) and obj.name not in current:
            bpy.data.objects.remove(obj, do_unlink=True)


def poll():
    try:
        apply_state(request('/api/state'))
        bpy.context.scene.aifr_live_error = ''
    except (OSError, ValueError, KeyError, TypeError) as exc:
        bpy.context.scene.aifr_live_error = 'Python service unavailable: ' + str(exc)[:90]
    return 0.2


class AIFR_OT_action(bpy.types.Operator):
    bl_idname = 'aifr.live_action'
    bl_label = 'AIFR live command'
    action: bpy.props.StringProperty()

    def execute(self, context):
        try:
            result = request('/api/control', {'action':self.action})
            self.report({'INFO'}, result.get('message','Command sent'))
            return {'FINISHED'}
        except (OSError, ValueError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}


class AIFR_OT_order(bpy.types.Operator):
    bl_idname = 'aifr.live_order'
    bl_label = 'Submit order'

    def execute(self, context):
        scene=context.scene
        try:
            result=request('/api/order', {'table':scene.aifr_table.strip(),
                                          'station':scene.aifr_station.strip(), 'ready': True})
            self.report({'INFO'}, f"Order #{result['order_id']} accepted")
            return {'FINISHED'}
        except (OSError, ValueError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}


class AIFR_OT_pedestrian(bpy.types.Operator):
    bl_idname = 'aifr.live_pedestrian'
    bl_label = 'Spawn pedestrian'

    def execute(self, context):
        s=context.scene
        try:
            result=request('/api/pedestrian', {
                'x':s.aifr_spawn_x, 'y':s.aifr_spawn_y,
                'end_x':s.aifr_patrol_x, 'end_y':s.aifr_patrol_y})
            self.report({'INFO'}, f"Pedestrian #{result['pedestrian_id']} added")
            return {'FINISHED'}
        except (OSError, ValueError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}


class AIFR_OT_cursor(bpy.types.Operator):
    bl_idname = 'aifr.use_cursor'
    bl_label = 'Use 3D cursor position'

    def execute(self, context):
        cursor=context.scene.cursor.location
        context.scene.aifr_spawn_x=cursor.x
        context.scene.aifr_spawn_y=cursor.y
        context.scene.aifr_patrol_x=cursor.x
        context.scene.aifr_patrol_y=cursor.y
        return {'FINISHED'}


class AIFR_PT_live(bpy.types.Panel):
    bl_label='AIFR Live Simulation'
    bl_idname='AIFR_PT_LIVE'
    bl_space_type='VIEW_3D'
    bl_region_type='UI'
    bl_category='AIFR'

    def draw(self, context):
        layout=self.layout
        s=context.scene
        layout.label(text=s.aifr_live_status)
        if s.aifr_live_error:
            layout.label(text=s.aifr_live_error[:55],icon='ERROR')
        row=layout.row(align=True)
        for action in ('start','pause','step'):
            row.operator('aifr.live_action',text=action.title()).action=action
        row=layout.row(align=True)
        row.operator('aifr.live_action',text='Add Robot').action='add_robot'
        row.operator('aifr.live_action',text='Emergency Stop').action='estop'
        layout.separator()
        layout.label(text='New READY order')
        layout.prop(s,'aifr_table')
        layout.prop(s,'aifr_station')
        layout.operator('aifr.live_order')
        layout.separator()
        layout.label(text=s.aifr_deadlock_stats)
        layout.label(text='New pedestrian patrol')
        layout.operator('aifr.use_cursor')
        for name in ('aifr_spawn_x','aifr_spawn_y','aifr_patrol_x','aifr_patrol_y'):
            layout.prop(s,name)
        layout.operator('aifr.live_pedestrian')
        layout.separator()
        layout.label(text='Python backend: localhost:8765')
        layout.label(text='Safety checks run in Python')


CLASSES=(AIFR_OT_action,AIFR_OT_order,AIFR_OT_pedestrian,AIFR_OT_cursor,AIFR_PT_live)


def register():
    for cls in CLASSES:
        if not hasattr(bpy.types, cls.__name__):
            bpy.utils.register_class(cls)
    s=bpy.types.Scene
    s.aifr_live_status=bpy.props.StringProperty(default='DISCONNECTED')
    s.aifr_live_error=bpy.props.StringProperty(default='')
    s.aifr_deadlock_stats=bpy.props.StringProperty(default='')
    s.aifr_table=bpy.props.StringProperty(name='Table',default='T11')
    s.aifr_station=bpy.props.StringProperty(name='Station',default='SANDWICH')
    for name in ('aifr_spawn_x','aifr_spawn_y','aifr_patrol_x','aifr_patrol_y'):
        setattr(s,name,bpy.props.FloatProperty(name=name.removeprefix('aifr_').replace('_',' ').title(),default=0.0))
    if not bpy.app.timers.is_registered(poll):
        bpy.app.timers.register(poll,first_interval=0.5,persistent=True)


register()
print('AIFR LIVE: Open Viewport > Sidebar (N) > AIFR; Python service localhost:8765')
