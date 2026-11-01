"""Stage 5 single-robot mission runner built on the Stage 2 order lifecycle.

Offline, pure Python, fixed-step. The robot uses a real obstacle grid; no
hardcoded cafeteria locations. This does not modify Blender geometry.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import math
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.navigation.astar import find_path
from aifr.navigation.kinematics import Pose,advance
from aifr.navigation.controller import command
from aifr.perception.sensors import SensorSuite
from aifr.perception.ekf import PoseEKF
@dataclass
class SingleRobot:
    grid:OccupancyGrid
    pose:Pose
    dt:float=0.05
    sensors:SensorSuite=field(default_factory=SensorSuite)
    trace:list[Pose]=field(default_factory=list)
    def __post_init__(self):
        self.filter=PoseEKF(self.pose)
        self.trace.append(self.pose)
    def travel(self,destination:tuple[float,float],max_steps:int=25000)->bool:
        path=find_path(self.grid,(self.pose.x,self.pose.y),destination)
        if not path:return False
        for waypoint in path[1:]:
            for _ in range(max_steps):
                if math.hypot(waypoint[0]-self.pose.x,waypoint[1]-self.pose.y)<0.055:break
                v,w=command(self.filter.pose,waypoint)
                nxt=advance(self.pose,v,w,self.dt)
                col,row=self.grid.world_to_grid(nxt.x,nxt.y)
                if not self.grid.is_free(col,row):return False
                self.pose=nxt;self.trace.append(nxt)
                od=self.sensors.odometry(v,w)
                self.filter.predict(od.linear,od.angular,self.dt)
                if len(self.trace)%10==0:
                    lm=self.sensors.landmark(self.pose)
                    self.filter.correct_xy(lm.x,lm.y)
            else:return False
        return math.hypot(path[-1][0]-self.pose.x,path[-1][1]-self.pose.y)<0.10
    def deliver(self,pickup:tuple[float,float],table:tuple[float,float])->bool:
        return self.travel(pickup) and self.travel(table)
