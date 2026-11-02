"""Static egress / waiting-zone feasibility for the recovered V14 cafeteria.

This is a geometric *preflight*, not a promise that DWA will complete each
maneuver when robots or customers are moving. Never modifies the .blend/map.
"""
from __future__ import annotations
import math
from aifr.navigation.astar import find_path
from aifr.navigation.collision_checker import CollisionChecker
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.environment.waypoint_manager import WaypointManager,WaypointKind
from tools.geometry_audit import audit_environment


def _reserve(grid: OccupancyGrid, center, radius=.40):
    rows=[list(row) for row in grid.cells]
    c0,r0=grid.world_to_grid(center[0]-radius,center[1]-radius)
    c1,r1=grid.world_to_grid(center[0]+radius,center[1]+radius)
    for r in range(max(0,r0),min(grid.height,r1+1)):
        for c in range(max(0,c0),min(grid.width,c1+1)):
            x,y=grid.grid_to_world(c,r)
            if math.hypot(x-center[0],y-center[1])<=radius:
                rows[r][c]=1
    return OccupancyGrid(grid.origin_x,grid.origin_y,grid.resolution,
                         grid.width,grid.height,tuple(map(tuple,rows)))


def audit_egress_and_waiting(env,grid):
    basic=audit_environment(env,grid)
    waypoints=WaypointManager(env).generate().valid
    spawn=tuple(basic['spawn'])
    checker=CollisionChecker(env)
    picks=[w for w in waypoints if w.kind is WaypointKind.PICKUP]
    destinations=[(w.label,(w.x,w.y)) for w in picks]
    reachable={label for label,goal in destinations if find_path(grid,spawn,goal)}
    pickup_tests=[]
    for w in picks:
        outbound=find_path(grid,spawn,(w.x,w.y))
        egress=find_path(grid,(w.x,w.y),spawn)
        turn=all(checker.is_pose_free(w.x,w.y,w.heading+2*math.pi*k/36)
                 for k in range(36))
        pickup_tests.append({'label':w.label,'station':w.label.removeprefix('pickup_').split('_0')[0],
                             'approach_reachable':bool(outbound),'egress_reachable':bool(egress),
                             'full_rotation_clear':turn,
                             'egress_segments':max(0,len(egress)-1),
                             'motion_controller_verified':False})
    waiting=[]
    for w in waypoints:
        if w.kind is not WaypointKind.WAITING:continue
        overlay=_reserve(grid,(w.x,w.y))
        if not overlay.is_free(*overlay.world_to_grid(*spawn)):
            blocked=sorted(reachable)
        else:
            blocked=[label for label,goal in destinations if label in reachable
                     and not find_path(overlay,spawn,goal)]
        waiting.append({'label':w.label,'xy':[w.x,w.y],
                        'blocks_pickup_access':blocked,
                        'nonblocking_for_pickup_access':not blocked,
                        'candidate_only':True})
    return {'entrances':len(env.entrances), 'pickups':pickup_tests,
            'waiting':waiting,'map_unchanged':True,
            'limitations':[
               'Egress A* path and stationary rotation do not prove DWA maneuver feasibility.',
               'Nonblocking assessment checks static reachability from primary base to pickups, not dynamic traffic capacity.',
               'Reservation checks use conservative occupied discs over the inflated occupancy grid.'
            ]}
