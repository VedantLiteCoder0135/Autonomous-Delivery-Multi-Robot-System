"""Stage 4: pure-Python geometry-based occupancy and collision checking."""
from .occupancy_grid import OccupancyGrid
from .collision_checker import CollisionChecker
__all__ = ["OccupancyGrid", "CollisionChecker"]
