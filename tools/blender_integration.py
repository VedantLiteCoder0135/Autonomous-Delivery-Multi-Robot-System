"""Blender offline animation of *verified* Stages 7–9 navigation.

Use the existing Git checkout, not an extracted temporary copy:
  python tools/run_stage79.py
  /Applications/Blender.app/Contents/MacOS/Blender \
    "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" --background \
    --python "$(pwd)/tools/blender_stage79_bridge.py"

Creates outputs/AIFR_Stages_7_9_Demo.blend. Original scene is never saved.
This is playback of Python motion, not real-time Blender steering/physics.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

import bpy

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
from tools.blender_data_export import (SOURCE_SCENE, animation_frame,
                                       load_and_validate, sample_robot)

OUTPUT = PROJECT / 'outputs'
COLLECTION_NAME = 'AIFR_STAGE79_VISUALIZATION'
FPS = 30
STRIDE = 5
COLORS = [(0.08, 0.72, 0.24, 1), (0.05, 0.48, 0.96, 1),
          (0.98, 0.6, 0.08, 1), (0.64, 0.28, 0.95, 1)]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda: f.read(1 << 20), b''):
            h.update(data)
    return h.hexdigest()


def material(name, color):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    node = mat.node_tree.nodes.get('Principled BSDF')
    if node:
        node.inputs['Base Color'].default_value = color
    return mat


def relink(obj, collection):
    for coll in tuple(obj.users_collection):
        coll.objects.unlink(obj)
    collection.objects.link(obj)


def polyline(name, points, color, collection, z=0.13, thickness=0.015):
    if len(points) < 2:
        return
    curve = bpy.data.curves.new(name, 'CURVE')
    curve.dimensions = '3D'
    curve.bevel_depth = thickness
    curve.bevel_resolution = 1
    spline = curve.splines.new('POLY')
    spline.points.add(len(points) - 1)
    for point, (x, y) in zip(spline.points, points):
        point.co = (float(x), float(y), z, 1)
    obj = bpy.data.objects.new(name, curve)
    collection.objects.link(obj)
    curve.materials.append(material(name + '_MAT', color))


def label(name, caption, location, collection, size=0.22):
    font = bpy.data.curves.new(name + '_FONT', 'FONT')
    font.body = caption
    font.size = size
    font.extrude = 0.002
    obj = bpy.data.objects.new(name, font)
    collection.objects.link(obj)
    obj.location = location
    return obj


def robot_mesh(rid, collection, color):
    bpy.ops.mesh.primitive_cube_add(size=1)
    obj = bpy.context.object
    obj.name = f'AIFR_S79_ROBOT_{rid}'
    obj.dimensions = (0.40, 0.36, 0.28)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(material(f'AIFR_S79_ROBOT_{rid}_MAT', color))
    relink(obj, collection)
    return obj


def linear_keys(obj):
    action = getattr(getattr(obj, 'animation_data', None), 'action', None)
    if action is None:
        return False
    curves = list(getattr(action, 'fcurves', ()))
    if not curves and hasattr(action, 'layers'):
        try:
            for layer in action.layers:
                for strip in layer.strips:
                    for slot in action.slots:
                        bag = strip.channelbag(slot, ensure=False)
                        if bag:
                            curves.extend(bag.fcurves)
        except (AttributeError, TypeError):
            return False
    for fc in curves:
        for key in fc.keyframe_points:
            key.interpolation = 'LINEAR'
    return bool(curves)


def main():
    original = (PROJECT / SOURCE_SCENE).resolve()
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != original:
        raise RuntimeError('Open the recovered V14 cafeteria .blend supplied with this project')
    report, trace = load_and_validate(PROJECT)
    original_digest = sha256(original)
    scene = bpy.context.scene
    collection = bpy.data.collections.get(COLLECTION_NAME)
    if collection is None:
        collection = bpy.data.collections.new(COLLECTION_NAME)
        scene.collection.children.link(collection)
    # Remove only *our own* visuals. Never touch cafeteria geometry or prior demos.
    for obj in tuple(collection.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    created = {}
    dt = float(trace['dt'])
    for idx, (rid, samples) in enumerate(sorted(trace['robots'].items(), key=lambda kv: int(kv[0]))):
        color = COLORS[idx % len(COLORS)]
        sampled = sample_robot(samples, STRIDE)
        robot = robot_mesh(rid, collection, color)
        phase_label = label(f'AIFR_S79_LABEL_{rid}', f'ROBOT {rid}', (0, 0, 0), collection)
        phase_label.parent = robot
        phase_label.location = (-0.22, -0.1, 0.32)
        polyline(f'AIFR_S79_TRACE_{rid}', [(s['x'], s['y']) for s in samples[::STRIDE]]
                 + [(samples[-1]['x'], samples[-1]['y'])], color, collection)
        for s in sampled:
            frame = animation_frame(s, dt, FPS)
            robot.location = (s['x'], s['y'], 0.25)
            robot.rotation_euler[2] = s['theta']
            robot.keyframe_insert(data_path='location', frame=frame)
            robot.keyframe_insert(data_path='rotation_euler', index=2, frame=frame)
        created[str(rid)] = {'sample_count': len(samples), 'keyframes': len(sampled),
                             'linear_keyframes': linear_keys(robot)}
    for idx, route in enumerate(trace.get('routes', [])):
        pts = route['path']
        if len(pts) > 1:
            rid = int(route['robot_id'])
            color = COLORS[(rid - 1) % len(COLORS)]
            polyline(f'AIFR_S79_ROUTE_{idx:03d}_R{rid}_{route.get("purpose", "PATH")}',
                     pts, (*color[:3], 1), collection, z=0.065, thickness=0.007)
    for entrance in report['entrances']:
        x, y = float(entrance['center_x']), float(entrance['center_y'])
        # A marker on the floor: an entrance is wall-verified, not necessarily
        # a robot-traversable *grid centre* after footprint inflation.
        bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=6, radius=0.10,
                                             location=(x, y, 0.14))
        mark = bpy.context.object
        mark.name = 'AIFR_S79_ENTRANCE_' + entrance['name']
        mark.data.materials.append(material(mark.name + '_MAT', (1.0, 0.65, 0.05, 1)))
        relink(mark, collection)
        label('AIFR_S79_ENTRANCE_TEXT_' + entrance['name'], entrance['name'],
              (x + 0.14, y, 0.16), collection, size=0.18)
    # Event timestamps appear on the timeline; Blender string properties cannot
    # be keyframed as ordinary numeric animation curves.
    for event_index, event in enumerate(report.get('events', [])):
        if event.get('event') not in ('ASSIGNED', 'LOADING', 'UNLOADING',
                                      'DELIVERED_RETURN_PENDING', 'RETURNED_TO_BASE'):
            continue
        frame = 1 + round(int(event['tick']) * dt * FPS)
        title = f"{event.get('event', 'EVENT')} R{event.get('robot_id', '?')}"
        scene.timeline_markers.new(f'AIFR_S79_{event_index}_{title}', frame=frame)
    scene.render.fps = FPS
    scene.frame_start = 1
    scene.frame_end = max(animation_frame(s[-1], dt, FPS) for s in trace['robots'].values())
    scene.frame_set(1)
    scene['AIFR_stage79_note'] = 'Offline visualization from verified Python simulation'
    scene['AIFR_stage79_orders_delivered'] = len(report['orders'])
    scene['AIFR_stage79_entrances_verified'] = len(report['entrances'])
    destination = (OUTPUT / 'AIFR_Stages_7_9_Demo.blend').resolve()
    if destination == original:
        raise RuntimeError('Refusing to overwrite recovered V14 cafeteria')
    bpy.ops.wm.save_as_mainfile(filepath=str(destination))
    unchanged = sha256(original) == original_digest
    result = {'success': unchanged, 'original_scene_unchanged': unchanged,
              'source_scene': str(original), 'demo_scene': str(destination),
              'robots': created, 'entrances': len(report['entrances']),
              'orders_delivered': len(report['orders']), 'frames': scene.frame_end,
              'fps': FPS, 'mode': 'offline Python trace playback; no Blender physics steering'}
    (OUTPUT / 'stage79_blender_report.json').write_text(json.dumps(result, indent=2))
    if not unchanged:
        raise RuntimeError('Original cafeteria scene changed unexpectedly')
    print('STAGES 7–9 BLENDER PLAYBACK SAVED:', destination)
    print('SOURCE V14 PRESERVED:', unchanged)


if __name__ == '__main__':
    OUTPUT.mkdir(parents=True, exist_ok=True)
    try:
        main()
    except Exception as exc:
        (OUTPUT / 'stage79_blender_report.json').write_text(
            json.dumps({'success': False, 'error': str(exc),
                        'traceback': traceback.format_exc()}, indent=2))
        raise
