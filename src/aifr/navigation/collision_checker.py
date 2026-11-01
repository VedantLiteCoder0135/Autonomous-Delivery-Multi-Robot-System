"""Exact 2D oriented-rectangle robot checks against Stage 3 obstacle rectangles.

No Blender dependency. XY positions are metres, theta is radians. A safety margin
is treated conservatively by expanding each static obstacle rectangle.
"""
from __future__ import annotations
import math
from aifr.environment.environment_mapper import CafeEnvironment, Rect2D

def _corners(x: float, y: float, theta: float, length: float, width: float):
    c,s=math.cos(theta), math.sin(theta)
    return [(x+a*c-b*s,y+a*s+b*c) for a,b in ((-length/2,-width/2),(-length/2,width/2),(length/2,width/2),(length/2,-width/2))]

def _sat(a,b):
    """Separating axis test: touching is treated as collision."""
    for poly in (a,b):
        for i in range(len(poly)):
            u,v=poly[i],poly[(i+1)%len(poly)]
            ax,ay=-(v[1]-u[1]),v[0]-u[0]
            aa=[x*ax+y*ay for x,y in a]; bb=[x*ax+y*ay for x,y in b]
            if max(aa)<min(bb)-1e-10 or max(bb)<min(aa)-1e-10:return False
    return True

def _rect_corners(r:Rect2D):
    return [(r.min_x,r.min_y),(r.min_x,r.max_y),(r.max_x,r.max_y),(r.max_x,r.min_y)]

class CollisionChecker:
    def __init__(self, environment:CafeEnvironment, length:float=0.40, width:float=0.36, clearance:float=0.05):
        if min(length,width)<=0 or clearance<0: raise ValueError('Invalid footprint or clearance')
        if not environment.floor_regions: raise ValueError('No floor regions: Stage 3 export required')
        self.environment=environment; self.length=length; self.width=width; self.clearance=clearance
        self.floors=[f.rect for f in environment.floor_regions]
        self.obstacles=[r.expanded(clearance) for r in environment.all_obstacles()]

    def is_pose_free(self,x:float,y:float,theta:float=0.0)->bool:
        if not all(math.isfinite(v) for v in (x,y,theta)): return False
        poly=_corners(x,y,theta,self.length,self.width)
        # Check entire footprint against UNION of floor rectangles, not a single arm.
        # A rectangle's vertices alone are insufficient near an L-shaped notch.
        xs=sorted(set([pt[0] for pt in poly]+[f.min_x for f in self.floors]+[f.max_x for f in self.floors]))
        ys=sorted(set([pt[1] for pt in poly]+[f.min_y for f in self.floors]+[f.max_y for f in self.floors]))
        # Clip to footprint polygon by evaluating boundary vertices, polygon/rectangle
        # intersections and intersections of vertical/horizontal floor partition lines.
        samples=list(poly)
        for f in self.floors:
            for a in _rect_corners(f):
                if _point_in_convex(a,poly):samples.append(a)
        for i in range(4):
            a,b=poly[i],poly[(i+1)%4]
            for f in self.floors:
                for edge_axis, value in ((0,f.min_x),(0,f.max_x),(1,f.min_y),(1,f.max_y)):
                    dv=b[edge_axis]-a[edge_axis]
                    if abs(dv)>1e-12:
                        t=(value-a[edge_axis])/dv
                        if 0<=t<=1:samples.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
        # Additional cell-centre probes between critical X/Y partition coordinates
        # catch holes within footprint when floor geometry is a rectangle union.
        xcuts=sorted(set([q[0] for q in samples]+[f.min_x for f in self.floors]+[f.max_x for f in self.floors]))
        ycuts=sorted(set([q[1] for q in samples]+[f.min_y for f in self.floors]+[f.max_y for f in self.floors]))
        for xx,yy in (( (xcuts[i]+xcuts[i+1])/2,(ycuts[j]+ycuts[j+1])/2) for i in range(len(xcuts)-1) for j in range(len(ycuts)-1)):
            if _point_in_convex((xx,yy),poly):samples.append((xx,yy))
        if not all(any(f.contains_point(px,py) for f in self.floors) for px,py in samples):return False
        return not any(_sat(poly,_rect_corners(o)) for o in self.obstacles)

    def is_motion_free(self,start:tuple[float,float,float],end:tuple[float,float,float],linear_step:float=0.025,angular_step:float=0.10)->bool:
        if linear_step<=0 or angular_step<=0:raise ValueError('Step sizes must be positive')
        x0,y0,t0=start;x1,y1,t1=end
        dt=(t1-t0+math.pi)%(2*math.pi)-math.pi
        steps=max(1,math.ceil(math.hypot(x1-x0,y1-y0)/linear_step),math.ceil(abs(dt)/angular_step))
        return all(self.is_pose_free(x0+(x1-x0)*i/steps,y0+(y1-y0)*i/steps,t0+dt*i/steps) for i in range(steps+1))

def _point_in_convex(p,poly):
    cross=[]
    for i in range(len(poly)):
        a,b=poly[i],poly[(i+1)%len(poly)]
        cross.append((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]))
    return min(cross)>=-1e-9 or max(cross)<=1e-9
