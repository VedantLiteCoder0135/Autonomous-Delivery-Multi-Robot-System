"""scene_extractor.py — Extract raw geometry from a live Blender scene.

This module is the ONLY part of Stage 3 that imports ``bpy``.
Everything it produces is plain Python dicts/lists so the rest of
the system remains independent of Blender.

Usage (inside Blender scripting console or via subprocess):
    from aifr.environment.scene_extractor import SceneExtractor
    raw = SceneExtractor().extract()
    # raw is a plain dict — serialise with json.dumps if needed.

The extractor never modifies the scene.
"""

from __future__ import annotations

from typing import Any

# ---------------------------------------------------------------------------
# Internal helpers — do NOT call bpy at import time so the module can be
# imported outside Blender (for static analysis, tests that mock bpy, etc.)
# ---------------------------------------------------------------------------

def _bpy() -> Any:
    """Lazy import of bpy — raises ImportError outside Blender."""
    import importlib
    return importlib.import_module("bpy")


def _world_bbox(obj: Any) -> dict[str, float]:
    """Return the axis-aligned world-space bounding box of *obj*.

    Returns
    -------
    dict with keys: min_x, max_x, min_y, max_y, min_z, max_z
    """
    bpy = _bpy()
    matrix = obj.matrix_world
    if obj.type == "MESH" and obj.data.vertices:
        corners = [matrix @ v.co for v in obj.data.vertices]
    else:
        # Empty / text / camera: use the 8 corners of the local bounding box
        corners = [matrix @ bpy.context.scene.cursor.location.__class__(co)
                   for co in obj.bound_box]
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    zs = [c[2] for c in corners]
    return {
        "min_x": min(xs), "max_x": max(xs),
        "min_y": min(ys), "max_y": max(ys),
        "min_z": min(zs), "max_z": max(zs),
    }


def _obj_record(obj: Any) -> dict[str, Any]:
    """Convert a Blender object to a plain-dict record."""
    loc = obj.matrix_world.to_translation()
    dim = obj.dimensions
    rot = obj.matrix_world.to_euler()
    record: dict[str, Any] = {
        "name": obj.name,
        "type": obj.type,
        "location": [round(loc.x, 6), round(loc.y, 6), round(loc.z, 6)],
        "dimensions": [round(dim.x, 6), round(dim.y, 6), round(dim.z, 6)],
        "rotation_euler_rad": [round(rot.x, 6), round(rot.y, 6), round(rot.z, 6)],
        "parent": obj.parent.name if obj.parent else None,
    }
    try:
        bbox = _world_bbox(obj)
        record["bbox"] = {k: round(v, 6) for k, v in bbox.items()}
    except Exception as exc:  # noqa: BLE001
        record["bbox_error"] = str(exc)
    return record


class SceneExtractor:
    """Extract every object from the active Blender scene.

    Parameters
    ----------
    include_types:
        Blender object types to include.  Defaults to MESH and EMPTY so
        labels (FONT), cameras and lights are skipped.
    """

    _DEFAULT_TYPES = frozenset({"MESH", "EMPTY"})

    def __init__(self, include_types: frozenset[str] | None = None) -> None:
        self._include = include_types if include_types is not None else self._DEFAULT_TYPES

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def extract(self) -> dict[str, Any]:
        """Return a plain-dict snapshot of the scene.

        The dict has the structure::

            {
                "scene_name": str,
                "object_count": int,
                "objects": [<record>, ...],
                "custom_props": {str: str, ...},
            }

        Raises
        ------
        ImportError
            When called outside a Blender Python runtime.
        """
        bpy = _bpy()
        scene = bpy.context.scene
        objects = [
            _obj_record(obj)
            for obj in scene.objects
            if obj.type in self._include
        ]
        custom_props: dict[str, str] = {}
        for key in scene.keys():
            if not key.startswith("_"):
                val = scene[key]
                custom_props[key] = val if isinstance(val, str) else str(val)

        return {
            "scene_name": scene.name,
            "object_count": len(objects),
            "objects": objects,
            "custom_props": custom_props,
        }
