"""Phase 1: conservative, read-only V14 pickup/aisle geometry audit.

No scene edits, no scheduler changes. All positions are world XY metres.
The occupancy grid is *already robot-inflated*: grid clearances are
remaining margin to inflated cells, not a measurement of physical aisle width.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any

from aifr.environment.waypoint_manager import WaypointKind, WaypointManager
from aifr.navigation.astar import find_path
from aifr.navigation.collision_checker import CollisionChecker


@dataclass(frozen=True)
class AuditSettings:
    approach_distances_m: tuple[float, ...] = (0.60, 0.40, 0.20)
    min_approach_m: float = 0.20
    low_clearance_margin_m: float = 0.20
    robot_length_m: float = 0.40
    robot_width_m: float = 0.36
    safety_margin_m: float = 0.05


def grid_clearance(grid):
    """8-connected distance-to-occupied map, in metres, incl. map boundary.

    Note: grid.cells already include obstacle inflation. This is a conservative
    *residual* clearance measure, not raw counter/table gap width.
    """
    h, w, res = grid.height, grid.width, grid.resolution
    dist = [[float('inf')] * w for _ in range(h)]
    heap = []
    for y in range(h):
        for x in range(w):
            if not grid.is_free(x, y) or x in (0,w-1) or y in (0,h-1):
                dist[y][x] = 0.0
                heap.append((0., x, y))
    heapq.heapify(heap)
    dirs = [(dx,dy,res*math.hypot(dx,dy))
            for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
    while heap:
        d,x,y=heapq.heappop(heap)
        if d > dist[y][x] + 1e-10:
            continue
        for dx,dy,cost in dirs:
            xx,yy=x+dx,y+dy
            if not (0<=xx<w and 0<=yy<h):
                continue
            nd=d+cost
            if nd+1e-10 < dist[yy][xx]:
                dist[yy][xx]=nd
                heapq.heappush(heap,(nd,xx,yy))
    return dist


def _path_min_clearance(grid, clearance, path):
    if not path:
        return None
    return round(min(clearance[grid.world_to_grid(x,y)[1]][grid.world_to_grid(x,y)[0]]
                     for x,y in path), 3)


def _straight_pose_sweep(checker, x0, y0, theta, x1, y1):
    return checker.is_motion_free((x0,y0,theta), (x1,y1,theta))


def _approach_candidates(x, y, heading, checker, grid, settings):
    """Try a collinear final approach, ending facing the counter.

    Heading points *toward* the counter. Start is behind final service pose.
    Does NOT assert a differential-drive robot can reach the start heading.
    """
    results=[]
    for d in settings.approach_distances_m:
        sx=x-d*math.cos(heading)
        sy=y-d*math.sin(heading)
        if not grid.is_free(*grid.world_to_grid(sx,sy)):
            continue
        if checker.is_pose_free(sx,sy,heading) and _straight_pose_sweep(checker,sx,sy,heading,x,y):
            results.append({'distance_m':d,'entry_xy':[round(sx,3),round(sy,3)],
                            'heading_rad':round(heading,6)})
    return results


def _rotation_test(checker, x, y, heading):
    """Pure rotation full turn at this position; strict but not a requirement."""
    for i in range(33):
        if not checker.is_pose_free(x,y,heading + 2*math.pi*i/32):
            return False
    return True


def _rect_gap(a,b):
    # Only report nearest opposing faces when their projections overlap.
    yover=min(a.max_y,b.max_y)-max(a.min_y,b.min_y)
    xover=min(a.max_x,b.max_x)-max(a.min_x,b.min_x)
    if yover > 0:
        if a.max_x <= b.min_x: return b.min_x-a.max_x, 'horizontal'
        if b.max_x <= a.min_x: return a.min_x-b.max_x, 'horizontal'
    if xover > 0:
        if a.max_y <= b.min_y: return b.min_y-a.max_y, 'vertical'
        if b.max_y <= a.min_y: return a.min_y-b.max_y, 'vertical'
    return None


def _nearby_gaps(env, counter, count=5):
    obstacles=[]
    for collection in (env.tables, env.chairs, env.walls, env.pillars):
        for item in collection:
            rect=item.rect
            label=(getattr(item,'table_id',None) or getattr(item,'name',None)
                   or getattr(item,'chair_id',None) or type(item).__name__)
            match=_rect_gap(counter.rect,rect)
            if match:
                gap,direction=match
                obstacles.append((gap,str(label),direction))
    return [{'opposing_object':label,'axis':axis,'raw_face_gap_m':round(gap,3)}
            for gap,label,axis in sorted(obstacles)[:count]]


def audit_environment(env, grid, *, settings=AuditSettings()) -> dict[str,Any]:
    """Read-only report: explicit positions, route feasibility and caution flags.

    It is intentionally NOT a guarantee of DWA reachability, dynamic traffic
    clearance, or valid steering into the pickup pose.
    """
    checker=CollisionChecker(env,settings.robot_length_m,settings.robot_width_m,
                             settings.safety_margin_m)
    waypoints=WaypointManager(env).generate()
    clearances=grid_clearance(grid)
    spawn=next((w for w in waypoints.valid if w.kind==WaypointKind.SPAWN
                and grid.is_free(*grid.world_to_grid(w.x,w.y))),None)
    if spawn is None:
        raise ValueError('No valid robot spawn in this cafeteria')
    pickups=[]
    all_stations=sorted(env.food_stations(),key=lambda s:s.name)
    for station in all_stations:
        choices=[w for w in waypoints.valid if w.kind==WaypointKind.PICKUP
                 and w.label.removeprefix('pickup_').split('_0')[0]==station.name]
        poses=[]
        for w in choices:
            valid_pose=checker.is_pose_free(w.x,w.y,w.heading)
            path=find_path(grid,(spawn.x,spawn.y),(w.x,w.y))
            approaches=_approach_candidates(w.x,w.y,w.heading,checker,grid,settings) if valid_pose else []
            # Reach the straight final approach start on grid; endpoint alone is insufficient.
            approach_reachable=next((a for a in approaches
                                   if find_path(grid,(spawn.x,spawn.y),tuple(a['entry_xy']))),None)
            c,r=grid.world_to_grid(w.x,w.y)
            residual=clearances[r][c] if grid.is_free(c,r) else 0.
            rotation=_rotation_test(checker,w.x,w.y,w.heading) if valid_pose else False
            status=('CANDIDATE_FEASIBLE' if path and approach_reachable else
                    'UNVERIFIED_APPROACH' if valid_pose else 'UNSAFE_POSE')
            poses.append({'label':w.label,'xy':[round(w.x,3),round(w.y,3)],
                          'heading_rad':round(w.heading,6),'footprint_clear':valid_pose,
                          'astar_reachable':bool(path),'grid_path_cells':len(path),
                          'residual_grid_clearance_m':round(residual,3),
                          'min_grid_clearance_along_path_m':_path_min_clearance(grid,clearances,path),
                          'full_in_place_rotation_clear':rotation,
                          'verified_straight_final_approach':approach_reachable,
                          'status':status,
                          'warnings':(["LOW_GRID_CLEARANCE"] if residual<settings.low_clearance_margin_m else []) +
                                     (["ROTATION_RESTRICTED"] if not rotation else []) +
                                     (["STEERING_TO_ENTRY_UNVERIFIED"] if approach_reachable else [])})
        pickups.append({'station':station.name,'station_rect':station.rect.to_dict(),
                        'nearby_raw_object_gaps':_nearby_gaps(env,station),
                        'pickup_poses':poses,
                        'candidate_feasible':sum(p['status']=='CANDIDATE_FEASIBLE' for p in poses)})
    waiting=[]
    for w in waypoints.valid:
        if w.kind != WaypointKind.WAITING: continue
        c,r=grid.world_to_grid(w.x,w.y)
        path=find_path(grid,(spawn.x,spawn.y),(w.x,w.y))
        waiting.append({'label':w.label,'xy':[round(w.x,3),round(w.y,3)],
                        'footprint_clear':checker.is_pose_free(w.x,w.y,w.heading),
                        'astar_reachable':bool(path),
                        'residual_grid_clearance_m':round(clearances[r][c],3),
                        'full_in_place_rotation_clear':_rotation_test(checker,w.x,w.y,w.heading),
                        'non_blocking_for_traffic':'NOT_YET_VERIFIED'})
    # Grid bottlenecks near a food counter: free cells with small residual
    # clearance; they are *candidates*, not measured widths.
    risks=[]
    for station in all_stations:
        cx,cy=station.rect.center_x,station.rect.center_y
        cells=[]
        for r in range(grid.height):
            for c in range(grid.width):
                if not grid.is_free(c,r):continue
                x,y=grid.grid_to_world(c,r)
                if math.hypot(x-cx,y-cy)<=2.0 and clearances[r][c]<settings.low_clearance_margin_m:
                    cells.append((x,y,clearances[r][c]))
        risks.append({'station':station.name,'low_residual_clearance_cells_within_2m':len(cells),
                      'example_cells_xy':[[round(x,2),round(y,2)] for x,y,_ in cells[:12]]})
    return {'schema_version':1,'source':'Stage 1-6 exported V14 map',
            'robot_footprint_m':[settings.robot_length_m,settings.robot_width_m],
            'static_clearance_margin_m':settings.safety_margin_m,
            'grid_resolution_m':grid.resolution,
            'grid_already_footprint_inflated':True,
            'spawn':[spawn.x,spawn.y],
            'entrances_count':len(env.entrances),
            'pickup_stations':pickups,'waiting_candidates':waiting,
            'low_clearance_regions':risks,
            'scope_limitations':[
                'Candidate feasible means collision-free pickup pose, grid reachability and a straight final approach segment; differential-drive steering to entry pose is NOT verified.',
                'Raw counter-to-object gaps omit turns and other obstacles around the route.',
                'Grid clearance is residual clearance to inflated occupancy, NOT physical aisle width.',
                'Waiting points have not been proved not to block other agents.',
                'Moving pedestrians, robot reservations and dynamic obstacles are not included.',
                'No modifications are made to Blender geometry, engine routing or scheduler.'
            ]}
