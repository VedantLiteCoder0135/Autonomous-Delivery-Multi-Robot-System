"""Simple waypoint-following controller; not a replacement for future DWA."""
from __future__ import annotations
import math
from .kinematics import Pose, normalize
def command(pose:Pose,goal:tuple[float,float],max_linear:float=0.35,max_angular:float=1.5)->tuple[float,float]:
    dx=goal[0]-pose.x;dy=goal[1]-pose.y
    distance=math.hypot(dx,dy)
    if distance<0.035:return 0.,0.
    err=normalize(math.atan2(dy,dx)-pose.theta)
    angular=max(-max_angular,min(max_angular,2.5*err))
    linear=min(max_linear,1.0*distance)*max(0.,math.cos(err)) if abs(err)<1.3 else 0.
    return linear,angular
