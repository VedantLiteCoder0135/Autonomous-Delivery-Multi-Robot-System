"""8-connected A* with no diagonal corner cutting."""
from __future__ import annotations
import heapq, math
from .occupancy_grid import OccupancyGrid
def find_path(grid: OccupancyGrid, start:tuple[float,float], goal:tuple[float,float])->list[tuple[float,float]]:
    s=grid.world_to_grid(*start); g=grid.world_to_grid(*goal)
    if not grid.is_free(*s) or not grid.is_free(*g): return []
    def heur(n):return math.hypot(n[0]-g[0],n[1]-g[1])
    q=[(heur(s),0.0,s)]; costs={s:0.}; parents={}; done=set()
    while q:
        _,cost,node=heapq.heappop(q)
        if node in done:continue
        if node==g:
            path=[node]
            while node in parents:node=parents[node];path.append(node)
            return [grid.grid_to_world(*v) for v in reversed(path)]
        done.add(node)
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if dx==dy==0:continue
                n=(node[0]+dx,node[1]+dy)
                if not grid.is_free(*n):continue
                if dx and dy and (not grid.is_free(node[0]+dx,node[1]) or not grid.is_free(node[0],node[1]+dy)):continue
                nc=cost+math.hypot(dx,dy)
                if nc+1e-10<costs.get(n,float('inf')):
                    costs[n]=nc;parents[n]=node;heapq.heappush(q,(nc+heur(n),nc,n))
    return []
