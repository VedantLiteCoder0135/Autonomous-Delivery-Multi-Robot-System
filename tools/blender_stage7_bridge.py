"""Stage 7 Part 4: visualize verified Python DWA trajectory in the approved scene.

Run from the project root on macOS:
  /Applications/Blender.app/Contents/MacOS/Blender \
    "$(pwd)/AIFR_Simulation_Stage3.blend" --background \
    --python "$(pwd)/tools/blender_stage7_bridge.py"

Creates outputs/AIFR_Stage7_Demo.blend; NEVER saves over the approved scene.
The robot animation is offline playback of 20 Hz closed-loop Python navigation,
not a Blender physics controller or a claim of multi-robot coordination.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import traceback

import bpy

PROJECT = Path(__file__).resolve().parent.parent
OUT = PROJECT / "outputs"
OUT.mkdir(parents=True, exist_ok=True)
DEMO_COLLECTION = "AIFR_STAGE7_DEMO"


def read(path):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}; run tools/run_stage7_cafeteria.py first")
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def material(name, rgba):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = rgba
    mat.use_nodes = True
    node = mat.node_tree.nodes.get("Principled BSDF")
    if node:
        node.inputs["Base Color"].default_value = rgba
    return mat


def move_to_collection(obj, collection):
    for old in tuple(obj.users_collection):
        old.objects.unlink(obj)
    collection.objects.link(obj)


def path_curve(name, points, z, rgba, collection):
    if len(points) < 2:
        return None
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = 0.018
    curve.bevel_resolution = 2
    spline = curve.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for dst, (x, y) in zip(spline.points, points):
        dst.co = (float(x), float(y), z, 1.0)
    obj = bpy.data.objects.new(name, curve)
    collection.objects.link(obj)
    curve.materials.append(material(name + "_mat", rgba))
    return obj


def linear_interpolation(obj):
    """Set keyframes to linear for supported Blender Action API versions."""
    anim = obj.animation_data
    if anim is None or anim.action is None:
        return False
    action = anim.action
    curves = list(getattr(action, "fcurves", ()))
    if not curves and hasattr(action, "layers"):
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
        for point in fc.keyframe_points:
            point.interpolation = "LINEAR"
    return bool(curves)


def main():
    approved = PROJECT / "AIFR_Simulation_Stage3.blend"
    if not approved.is_file():
        raise FileNotFoundError(f"Approved scene not found: {approved}")
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != approved.resolve():
        raise RuntimeError("Open AIFR_Simulation_Stage3.blend, not a different Blender file")
    report = read(OUT / "stage7_cafeteria_report.json")
    trace = read(OUT / "stage7_cafeteria_trace.json")
    if report.get("success") is not True:
        raise RuntimeError("Stage 7 Python run did not pass; refusing to visualize a failed mission")
    if Path(str(trace.get("source_scene", ""))).resolve() != approved.resolve():
        raise RuntimeError("Trace was generated for a different Blender scene")
    samples = trace.get("samples", [])
    if len(samples) < 2 or len(samples) != report.get("trace_points"):
        raise RuntimeError("Stage 7 trace is incomplete")
    if not all(math.isfinite(float(s["time_s"])) and
               all(math.isfinite(float(s["robot"][k])) for k in ("x", "y", "theta"))
               for s in samples):
        raise RuntimeError("Stage 7 trace contains non-finite pose values")
    original_hash = digest(approved)

    collection = bpy.data.collections.get(DEMO_COLLECTION)
    if collection is None:
        collection = bpy.data.collections.new(DEMO_COLLECTION)
        bpy.context.scene.collection.children.link(collection)
    # Remove only objects created by this bridge. Never touch cafeteria objects.
    for obj in tuple(collection.objects):
        if obj.name.startswith("AIFR_STAGE7_"):
            bpy.data.objects.remove(obj, do_unlink=True)
    path_curve("AIFR_STAGE7_ACTUAL_TRACE",
               [(s["robot"]["x"], s["robot"]["y"]) for s in samples],
               0.11, (0.05, 0.75, 0.18, 1.0), collection)
    path_curve("AIFR_STAGE7_ASTAR_PICKUP", trace["global_paths"]["to_pickup"],
               0.07, (0.2, 0.5, 1.0, 1.0), collection)
    path_curve("AIFR_STAGE7_ASTAR_DELIVERY", trace["global_paths"]["to_delivery"],
               0.07, (1.0, 0.65, 0.1, 1.0), collection)

    bpy.ops.mesh.primitive_cube_add(size=1)
    robot = bpy.context.object
    robot.name = "AIFR_STAGE7_ROBOT"
    robot.scale = (0.4, 0.36, 0.28)
    robot.data.materials.append(material("AIFR_STAGE7_ROBOT_MAT", (0.05, 0.8, 0.2, 1)))
    move_to_collection(robot, collection)

    pedestrian = None
    if report.get("pedestrian_enabled"):
        if not all(s.get("pedestrian") for s in samples):
            raise RuntimeError("Pedestrian was enabled but is absent from trace")
        bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=0.16, depth=1.4)
        pedestrian = bpy.context.object
        pedestrian.name = "AIFR_STAGE7_PEDESTRIAN"
        pedestrian.data.materials.append(material("AIFR_STAGE7_PERSON_MAT", (1.0, 0.27, 0.1, 1)))
        move_to_collection(pedestrian, collection)

    scene = bpy.context.scene
    scene.render.fps = 30
    for s in samples:
        frame = 1 + round(float(s["time_s"]) * scene.render.fps)
        p = s["robot"]
        robot.location = (float(p["x"]), float(p["y"]), 0.24)
        robot.rotation_euler[2] = float(p["theta"])
        robot.keyframe_insert(data_path="location", frame=frame)
        robot.keyframe_insert(data_path="rotation_euler", index=2, frame=frame)
        if pedestrian is not None:
            person = s["pedestrian"]
            pedestrian.location = (float(person["x"]), float(person["y"]), 0.7)
            pedestrian.keyframe_insert(data_path="location", frame=frame)
    scene.frame_start = 1
    scene.frame_end = 1 + round(float(samples[-1]["time_s"]) * scene.render.fps)
    linear_robot = linear_interpolation(robot)
    linear_person = linear_interpolation(pedestrian) if pedestrian else True
    scene.frame_set(1)
    dest = OUT / "AIFR_Stage7_Demo.blend"
    if dest.resolve() == approved.resolve():
        raise RuntimeError("Refusing to overwrite approved cafeteria scene")
    bpy.ops.wm.save_as_mainfile(filepath=str(dest))
    preserved = digest(approved) == original_hash
    viz_report = {
        "success": bool(preserved),
        "original_scene_unchanged": preserved,
        "approved_scene": str(approved),
        "demo_scene": str(dest),
        "animation_frames": scene.frame_end,
        "fps": scene.render.fps,
        "robot_keyframes": len(samples),
        "pedestrian_animated": pedestrian is not None,
        "linear_keyframes": linear_robot and linear_person,
        "note": "Blender playback of closed-loop Python simulation, not Blender real-time control",
    }
    (OUT / "stage7_blender_report.json").write_text(json.dumps(viz_report, indent=2), encoding="utf-8")
    if not preserved:
        raise RuntimeError("Approved Blender file hash changed unexpectedly")
    print("STAGE 7 BLENDER VISUALIZATION SAVED:", dest)
    print("Original cafeteria scene preserved:", preserved)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        (OUT / "stage7_blender_report.json").write_text(
            json.dumps({"success": False, "error": str(exc),
                        "traceback": traceback.format_exc()}, indent=2), encoding="utf-8")
        print("STAGE 7 BLENDER BRIDGE ERROR:", exc)
        raise
