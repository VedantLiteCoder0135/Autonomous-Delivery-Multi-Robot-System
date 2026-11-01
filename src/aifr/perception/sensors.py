"""Deterministic-by-seed noisy sensor measurements; no Blender dependency."""
from __future__ import annotations
from dataclasses import dataclass
import math,random
from aifr.navigation.kinematics import Pose,normalize
@dataclass(frozen=True)
class MotionReading:
    linear:float
    angular:float
@dataclass(frozen=True)
class LandmarkReading:
    x:float
    y:float
class SensorSuite:
    def __init__(self,seed:int=1,velocity_sigma:float=0.015,yaw_sigma:float=0.025,position_sigma:float=0.05):
        self.rng=random.Random(seed);self.vs=velocity_sigma;self.ws=yaw_sigma;self.ps=position_sigma
    def odometry(self,linear:float,angular:float)->MotionReading:
        return MotionReading(linear+self.rng.gauss(0,self.vs), angular+self.rng.gauss(0,self.ws))
    def landmark(self,true_pose:Pose)->LandmarkReading:
        return LandmarkReading(true_pose.x+self.rng.gauss(0,self.ps),true_pose.y+self.rng.gauss(0,self.ps))
def lidar_scan(pose:Pose, obstacles:list[tuple[float,float,float,float]], rays:int=36, max_range:float=4.0, step:float=0.04)->list[float]:
    """Raycast against axis-aligned obstacle bounding boxes; simplified 2D LiDAR."""
    if rays<1 or max_range<=0 or step<=0:raise ValueError('invalid lidar parameters')
    result=[]
    for i in range(rays):
        angle=pose.theta+2*math.pi*i/rays;c=math.cos(angle);s=math.sin(angle);d=max_range
        ticks=math.ceil(max_range/step)
        for k in range(1,ticks+1):
            r=min(k*step,max_range);x=pose.x+r*c;y=pose.y+r*s
            if any(x0<=x<=x1 and y0<=y<=y1 for x0,y0,x1,y1 in obstacles):d=r;break
        result.append(d)
    return result
