"""Stage 8 adaptive large-neighborhood search for collision-free route geometry.

The FCFS scheduler owns order assignment. ALNS only optimizes the geometry
of a *fixed* start-to-goal route. This is not a vehicle-routing solver.
Destroy operators remove route checkpoints; repair operators connect remaining
points via a verified straight segment or A* subpath. The best route is always
checked on the inflated Stage 4 occupancy grid.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Sequence

from .astar import find_path
from .dwa_route import line_clear
from .occupancy_grid import OccupancyGrid

Point = tuple[float, float]


def route_cost(path: Sequence[Point]) -> float:
    if len(path) < 2:
        return 0.
    length = sum(math.dist(a,b) for a,b in zip(path,path[1:]))
    turns = 0.
    for a,b,c in zip(path,path[1:],path[2:]):
        u = math.atan2(b[1]-a[1],b[0]-a[0])
        v = math.atan2(c[1]-b[1],c[0]-b[0])
        turns += abs((v-u+math.pi)%(2*math.pi)-math.pi)
    return length + .07*turns


@dataclass(frozen=True)
class ALNSResult:
    path: tuple[Point, ...]
    initial_cost: float
    optimized_cost: float
    iterations: int
    accepted: int
    operator_weights: tuple[float, float]


class ALNSRouteOptimizer:
    def __init__(self, grid: OccupancyGrid, seed: int = 11):
        self.grid = grid
        self.seed = seed

    def optimize(self, start: Point, goal: Point, iterations: int = 60) -> ALNSResult:
        if iterations < 0:
            raise ValueError('iterations must be nonnegative')
        original = find_path(self.grid,start,goal)
        if not original:
            raise ValueError('No traversable route between endpoints')
        # Preserve precise physical endpoints when their connecting cells are clear.
        if line_clear(self.grid,start,original[0]): original[0]=start
        if line_clear(self.grid,original[-1],goal): original[-1]=goal
        current=list(original);best=list(original)
        initial=route_cost(original);best_cost=initial
        rng=random.Random(self.seed)
        weights=[1.,1.]
        accepted=0
        for k in range(iterations):
            if len(current) < 3: break
            op = rng.choices([0,1],weights=weights)[0]
            if op == 0: # random destroy of contiguous checkpoints
                i=rng.randrange(0,len(current)-2)
                j=rng.randrange(i+2,min(len(current),i+13))
            else: # destroy around the sharpest turn, with a little exploration
                angles=[]
                for n in range(1,len(current)-1):
                    a,b,c=current[n-1:n+2]
                    t1=math.atan2(b[1]-a[1],b[0]-a[0]);t2=math.atan2(c[1]-b[1],c[0]-b[0])
                    angles.append(abs((t2-t1+math.pi)%(2*math.pi)-math.pi))
                worst=max(range(len(angles)),key=lambda n:angles[n]+rng.random()*.08)+1
                i=max(0,worst-rng.randrange(1,4))
                j=min(len(current)-1,worst+rng.randrange(1,5))
                if j <= i+1:continue
            a,b=current[i],current[j]
            if line_clear(self.grid,a,b,extra_clearance=.12):
                bridge=[a,b]
            else:
                bridge=find_path(self.grid,a,b)
                if not bridge: continue
                if line_clear(self.grid,a,bridge[0]): bridge[0]=a
                if line_clear(self.grid,bridge[-1],b): bridge[-1]=b
                if any(not line_clear(self.grid,x,y) for x,y in zip(bridge,bridge[1:])):
                    continue
            candidate=current[:i]+bridge+current[j+1:]
            if any(not line_clear(self.grid,x,y) for x,y in zip(candidate,candidate[1:])):
                continue
            cost=route_cost(candidate)
            temperature=max(.001,.2*(1-k/max(1,iterations)))
            if cost <= route_cost(current)+1e-10 or rng.random() < math.exp(min(0.,(route_cost(current)-cost)/temperature)):
                current=candidate
                accepted+=1
                weights[op]=min(8.,weights[op]*.92+(2. if cost<best_cost-1e-9 else .4))
                if cost < best_cost-1e-9:
                    best=list(candidate);best_cost=cost
            else:
                weights[op]=max(.1,weights[op]*.97)
        return ALNSResult(tuple(best),initial,best_cost,iterations,accepted,tuple(weights))
