"""Stage 3 — environment extraction, mapping and waypoint management.

Submodules
----------
scene_extractor    Extract raw geometry from a Blender scene (bpy-dependent).
environment_mapper Classify geometry into structured, serialisable env data (pure Python).
waypoint_manager   Generate and validate spawn / pickup / delivery waypoints (pure Python).
"""

from aifr.environment.environment_mapper import (
    CafeEnvironment,
    EnvironmentMapper,
    ObstacleRect,
)
from aifr.environment.waypoint_manager import (
    WaypointCandidate,
    WaypointManager,
    WaypointResult,
)

__all__ = [
    "CafeEnvironment",
    "EnvironmentMapper",
    "ObstacleRect",
    "WaypointCandidate",
    "WaypointManager",
    "WaypointResult",
]
