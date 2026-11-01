"""Stage 4 occupancy grid, derived solely from extracted Stage 3 geometry.

Grid cells are FREE only if an orientation-independent conservative robot
circumscribed disk fits within the floor and avoids all static obstacles.
This is conservative; the orientation-aware CollisionChecker can validate finer
poses in narrow corridors. No fabricated cafeteria coordinates.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from aifr.environment.environment_mapper import CafeEnvironment
from .collision_checker import CollisionChecker

@dataclass(frozen=True)
class OccupancyGrid:
    origin_x:float
    origin_y:float
    resolution:float
    width:int
    height:int
    cells:tuple[tuple[int,...],...] # 0 free; 1 occupied

    @classmethod
    def from_environment(cls, env:CafeEnvironment, resolution:float=0.10, robot_length:float=0.40, robot_width:float=0.36, clearance:float=0.05)->'OccupancyGrid':
        if resolution<=0 or not math.isfinite(resolution):raise ValueError('Invalid resolution')
        if not env.floor_regions:raise ValueError('Missing floor geometry')
        fs=[f.rect for f in env.floor_regions];xmin=min(f.min_x for f in fs); ymin=min(f.min_y for f in fs)
        xmax=max(f.max_x for f in fs);ymax=max(f.max_y for f in fs)
        w=math.ceil((xmax-xmin)/resolution);h=math.ceil((ymax-ymin)/resolution)
        # Exact circular envelope: orientation-independent collision proxy.
        radius=math.hypot(robot_length/2,robot_width/2)
        checker=CollisionChecker(env,length=2*radius,width=2*radius,clearance=clearance)
        rows=[]
        for row in range(h):
            yy=ymin+(row+0.5)*resolution
            rows.append(tuple(0 if checker.is_pose_free(xmin+(col+0.5)*resolution,yy,0) else 1 for col in range(w)))
        return cls(xmin,ymin,resolution,w,h,tuple(rows))

    def world_to_grid(self,x:float,y:float)->tuple[int,int]:
        return (math.floor((x-self.origin_x)/self.resolution),math.floor((y-self.origin_y)/self.resolution))

    def grid_to_world(self,col:int,row:int)->tuple[float,float]:
        if not 0<=col<self.width or not 0<=row<self.height:raise IndexError('Cell outside grid')
        return self.origin_x+(col+0.5)*self.resolution,self.origin_y+(row+0.5)*self.resolution

    def is_free(self,col:int,row:int)->bool:
        return 0<=col<self.width and 0<=row<self.height and self.cells[row][col]==0

    def to_dict(self)->dict:
        return {'origin':[self.origin_x,self.origin_y],'resolution':self.resolution,'width':self.width,'height':self.height,'cells':[list(row) for row in self.cells], 'legend':{'0':'free','1':'blocked'}}
