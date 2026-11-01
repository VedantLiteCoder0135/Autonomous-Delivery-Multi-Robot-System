import math
from aifr.environment.environment_mapper import CafeEnvironment,FloorRegion,WallSegment,Rect2D
from aifr.navigation import CollisionChecker,OccupancyGrid

def scene():
    e=CafeEnvironment()
    e.floor_regions=[FloorRegion('arm',Rect2D(0,4,0,2)),FloorRegion('wing',Rect2D(2,4,2,4))]
    e.walls=[WallSegment('wall',Rect2D(1,1.2,0,0.7))]
    return e

def test_grid_and_coordinates():
    g=OccupancyGrid.from_environment(scene(),0.2)
    assert g.width==20 and g.height==20
    assert g.is_free(*g.world_to_grid(3,3))
    assert not g.is_free(*g.world_to_grid(0.5,3))
    assert not g.is_free(*g.world_to_grid(1.1,0.4))
    assert abs(g.grid_to_world(0,0)[0]-.1)<1e-10

def test_union_and_obstacles():
    c=CollisionChecker(scene(),clearance=0.0)
    assert c.is_pose_free(3,2,0) # straddles floor seam
    assert not c.is_pose_free(1.1,0.4,0)
    assert not c.is_pose_free(1.9,2.1,math.pi/4) # L notch
    assert not c.is_motion_free((.5,.5,0),(1.5,.5,0))

def test_validation():
    import pytest
    with pytest.raises(ValueError): OccupancyGrid.from_environment(scene(),0)
    with pytest.raises(ValueError):CollisionChecker(CafeEnvironment())
