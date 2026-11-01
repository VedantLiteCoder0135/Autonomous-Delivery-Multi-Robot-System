"""Differential-drive unicycle robot model (SI units)."""
from __future__ import annotations
from dataclasses import dataclass
import math
@dataclass(frozen=True)
class Pose:
    x:float
    y:float
    theta:float=0.0
def normalize(angle:float)->float:return (angle+math.pi)%(2*math.pi)-math.pi
def advance(pose:Pose,linear:float,angular:float,dt:float)->Pose:
    if dt<=0:raise ValueError('dt must be > 0')
    if abs(angular)<1e-9:
        return Pose(pose.x+linear*dt*math.cos(pose.theta),pose.y+linear*dt*math.sin(pose.theta),pose.theta)
    th=pose.theta+angular*dt
    return Pose(pose.x+linear/angular*(math.sin(th)-math.sin(pose.theta)),pose.y-linear/angular*(math.cos(th)-math.cos(pose.theta)),normalize(th))
def wheel_speeds(linear:float,angular:float,wheel_radius:float=0.06,track_width:float=0.30)->tuple[float,float]:
    if wheel_radius<=0 or track_width<=0:raise ValueError('invalid wheel geometry')
    return ((linear-angular*track_width/2)/wheel_radius,(linear+angular*track_width/2)/wheel_radius)
