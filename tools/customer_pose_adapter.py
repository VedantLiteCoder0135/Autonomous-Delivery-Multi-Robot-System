"""Member 3: Blender-only, reusable chair-oriented customer pose adapter.

A simple articulated avatar is created if no compatible character rig exists.
This adapter never changes the saved .blend or requires bpy in unit tests.
The precise pose can only be visually verified by opening the Blender scene.
"""
from __future__ import annotations

import math


def seating_fraction(phase: str, progress: float = 0.0) -> float:
    if phase in ("SEATED", "EATING"):
        return 1.0
    if phase == "SITTING":
        return max(0.0, min(1.0, progress / .6))
    if phase == "STANDING":
        return 1.0 - max(0.0, min(1.0, progress / .6))
    return 0.0


def _segment(obj, start, end, width):
    from mathutils import Vector
    start, end = Vector(start), Vector(end)
    direction = end-start
    obj.location = (start+end)*.5
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector((0,0,1)).rotation_difference(direction)
    obj.scale = (width, width, direction.length*.5)


def _new_part(bpy, name, root, collection, *, shape='sphere'):
    if shape == 'rod':
        bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=1, depth=2)
    else:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=8, radius=1)
    obj = bpy.context.object
    obj.name = name
    for old in tuple(obj.users_collection):
        old.objects.unlink(obj)
    collection.objects.link(obj)
    obj.parent = root
    return obj


def update_customer_pose(customer, collection):
    """Create/update a seated or walking avatar in the active Blender scene.

    A heading towards the table is used when sitting. The root is returned for
    property tagging by the host controller. Uses one root and seven pooled
    body parts instead of rebuilding meshes on every frame.
    """
    import bpy
    base = f'AIFR_SIM_CUSTOMER_{customer.customer_id}'
    root = bpy.data.objects.get(base)
    if root is not None and root.type != 'EMPTY':
        bpy.data.objects.remove(root, do_unlink=True)
        root = None
    if root is None:
        root = bpy.data.objects.new(base, None)
        collection.objects.link(root)
        root.empty_display_type = 'PLAIN_AXES'
        root.empty_display_size = .06
    names = ("torso", "head", "left_thigh", "right_thigh",
             "left_shin", "right_shin", "left_arm", "right_arm")
    parts = {}
    for part in names:
        name = f'AIFR_POSE_C{customer.customer_id}_{part}'
        obj = bpy.data.objects.get(name)
        if obj is None:
            obj = _new_part(bpy, name, root, collection,
                            shape='sphere' if part in ("torso", "head") else 'rod')
            mat_name = 'AIFR_AVATAR_SKIN' if part == 'head' else 'AIFR_AVATAR_BLUE'
            mat = bpy.data.materials.get(mat_name)
            if mat is None:
                mat = bpy.data.materials.new(mat_name)
                mat.diffuse_color = ((.93,.63,.44,1) if part == 'head'
                                     else (.22,.45,.83,1))
            obj.data.materials.clear()
            obj.data.materials.append(mat)
        parts[part] = obj

    f = seating_fraction(customer.phase, customer.transition_elapsed)
    root.location = (customer.x, customer.y, 0)
    heading = customer.seat_heading
    if customer.phase in ('TO_CASH', 'TO_CASH_QUEUE', 'TO_TABLE', 'TO_EXIT'):
        if customer.route_index < len(customer.route):
            tx, ty = customer.route[customer.route_index]
            dx, dy = tx - customer.x, ty - customer.y
            if math.hypot(dx, dy) > .05:
                heading = math.atan2(-dx, dy)
    root.rotation_euler.z = heading
    # Chair seat top in the V14 source is about 0.458m.
    hip_z = 1.00 - .46*f
    shoulder_z = hip_z + .43
    head_z = shoulder_z + .19
    parts['torso'].location = (0, 0, hip_z + .24)
    parts['torso'].scale = (.16, .105, .29)
    parts['head'].location = (0, .035, head_z)
    parts['head'].scale = (.12,.12,.12)
    for side, x in (('left',-.095),('right',.095)):
        hip = (x, 0, hip_z)
        knee = (x, .27*f, .53*f + .55*(1-f))
        ankle = (x, .29*f, .09)
        _segment(parts[f'{side}_thigh'], hip, knee, .052)
        _segment(parts[f'{side}_shin'], knee, ankle, .050)
        shoulder = (x*1.4, .005, shoulder_z)
        hand = (x*1.5, .12*f + .02, hip_z+.09)
        _segment(parts[f'{side}_arm'], shoulder, hand, .040)
    return root


def clear_stale_poses(active_ids: set[int], collection):
    import bpy
    prefix = 'AIFR_POSE_C'
    for obj in tuple(collection.objects):
        if not obj.name.startswith(prefix):
            continue
        token = obj.name[len(prefix):].split('_', 1)[0]
        if token.isdigit() and int(token) not in active_ids:
            bpy.data.objects.remove(obj, do_unlink=True)
