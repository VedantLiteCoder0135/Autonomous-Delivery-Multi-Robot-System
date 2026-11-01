"""Run inside Blender with the approved Stage-3 .blend already open.

Example Mac terminal:
  /Applications/Blender.app/Contents/MacOS/Blender /path/to/AIFR_Simulation_Stage3.blend --background --python tools/blender_stage36_bridge.py
No original geometry is modified; only AIFR_DEMO_* objects are created.
"""
import bpy, sys, json, math, traceback
from pathlib import Path

# Project directory supplied via AIFR_PROJECT_DIR environment variable or script path.
import os
PROJECT = Path(os.environ.get('AIFR_PROJECT_DIR', str(Path(__file__).resolve().parent.parent))).expanduser().resolve()
sys.path.insert(0, str(PROJECT/'src'))
from aifr.environment.scene_extractor import SceneExtractor
from aifr.environment.environment_mapper import EnvironmentMapper
from aifr.environment.waypoint_manager import WaypointManager, WaypointKind
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.navigation.astar import find_path
from aifr.navigation.kinematics import Pose
from aifr.simulation.delivery import SingleRobot
from aifr.navigation.collision_checker import CollisionChecker
from aifr.common.clock import SimClock
from aifr.common.events import EventBus
from aifr.orders.manager import OrderManager
from aifr.orders.model import OrderStatus
from aifr.fleet.registry import RobotRegistry
from aifr.scheduling.scheduler import GlobalScheduler

RESULTS = PROJECT/'outputs'; RESULTS.mkdir(parents=True,exist_ok=True)
REPORT={'scene':bpy.data.filepath,'project':str(PROJECT),'success':False,'warnings':[]}
def save():
    (RESULTS/'stage36_report.json').write_text(json.dumps(REPORT,indent=2,default=str))

def viz_polyline(name, pts, z, rgba):
    curve=bpy.data.curves.new(name,'CURVE');curve.dimensions='3D';curve.bevel_depth=0.025;curve.bevel_resolution=2
    line=curve.splines.new('POLY');line.points.add(len(pts)-1)
    for p,(x,y) in zip(line.points,pts):p.co=(x,y,z,1)
    obj=bpy.data.objects.new(name,curve);bpy.context.collection.objects.link(obj)
    mat=bpy.data.materials.new(name+'_material');mat.diffuse_color=rgba;curve.materials.append(mat)
    return obj

try:
    if not bpy.data.filepath:raise RuntimeError('Save/open AIFR_Simulation_Stage3.blend before running the bridge')
    # Remove ONLY previous demo artifacts, not the user's cafeteria geometry.
    for obj in list(bpy.data.objects):
        if obj.name.startswith('AIFR_DEMO_'):bpy.data.objects.remove(obj,do_unlink=True)
    raw=SceneExtractor().extract();env=EnvironmentMapper(raw).map();REPORT['counts']={
        'objects':raw['object_count'],'floors':len(env.floor_regions),'walls':len(env.walls),
        'pillars':len(env.pillars),'tables':len(env.tables),'chairs':len(env.chairs),
        'counters':len(env.counters),'entrances_classified':len(env.entrances),
        'unknown':len(env.unknown_objects)}
    (RESULTS/'stage3_raw_scene.json').write_text(json.dumps(raw,indent=2))
    (RESULTS/'stage3_environment.json').write_text(json.dumps(env.to_dict(),indent=2))
    REPORT['unknown_objects']=env.unknown_objects
    # V14 entrances are classified from metadata only after wall-gap validation.
    try: REPORT['v14_door_metadata']=json.loads(raw['custom_props'].get('AIFR_doors','{}'))
    except (TypeError,ValueError):REPORT['warnings'].append('Door metadata could not be parsed')
    if len(env.tables)!=18:REPORT['warnings'].append('Expected 18 tables; verify scene and classification')
    if len(env.chairs)!=72:REPORT['warnings'].append('Expected 72 chair seats; verify scene and classification')
    if len(REPORT.get('v14_door_metadata',{}))!=4 or len(env.entrances)!=4:
        raise RuntimeError('V14 entrance classification failed: expected four validated wall openings')
    if not env.floor_regions:raise RuntimeError('No floor objects found; cannot build map')
    if len(env.tables)!=18 or len(env.chairs)!=72 or len(env.pillars)!=6:
        raise RuntimeError('Incomplete cafeteria geometry: expected 18 tables, 72 chairs, 6 pillars')
    grid=OccupancyGrid.from_environment(env,resolution=0.10)
    (RESULTS/'stage4_occupancy_grid.json').write_text(json.dumps(grid.to_dict()))
    REPORT['grid']={'width':grid.width,'height':grid.height,'resolution':grid.resolution,
                   'free_cells':sum(c==0 for row in grid.cells for c in row)}
    waypoints=WaypointManager(env).generate()
    (RESULTS/'stage3_waypoints.json').write_text(json.dumps(waypoints.to_dict(),indent=2))
    def matching(kind,part):
        return [p for p in waypoints.valid if p.kind==kind and part in p.label.upper() and grid.is_free(*grid.world_to_grid(p.x,p.y))]
    spawns=matching(WaypointKind.SPAWN,'SPAWN')
    pickups=matching(WaypointKind.PICKUP,'SANDWICH')
    drops=matching(WaypointKind.DELIVERY,'T11')
    REPORT['candidate_counts']={'spawn':len(spawns),'sandwich':len(pickups),'T11':len(drops)}
    if not (spawns and pickups and drops):
        raise RuntimeError('No valid spawn/Sandwich/T11 candidates; inspect outputs/stage3_waypoints.json')
    # Select actual reachable candidate combination, not a fabricated coordinate.
    choice=None
    for start in spawns:
        for pu in pickups:
            first=find_path(grid,(start.x,start.y),(pu.x,pu.y))
            if not first:continue
            for drop in drops:
                second=find_path(grid,(pu.x,pu.y),(drop.x,drop.y))
                if second:choice=(start,pu,drop,first,second);break
            if choice:break
        if choice:break
    if choice is None:raise RuntimeError('No reachable Sandwich-to-T11 mission on the extracted grid; check geometry and doors')
    spawn,pickup,drop,p1,p2=choice
    REPORT['mission']={'spawn':spawn.to_dict(),'pickup':pickup.to_dict(),'drop':drop.to_dict(),
                       'planned_path_points':len(p1)+len(p2)}
    # Stage 2: authentic order and robot state machine. This does NOT bypass the scheduler.
    clock=SimClock(50_000)
    bus=EventBus(record=True)
    orders=OrderManager(bus,clock)
    fleet=RobotRegistry(bus,clock,robot_count=3)
    scheduler=GlobalScheduler(bus,clock,orders,fleet)
    order=orders.create_order('T11','SANDWICH')
    orders.start_preparing(order.order_id)
    # The staff marks food READY; Stage 2 ready-time FCFS determines priority.
    clock.advance()
    orders.mark_ready(order.order_id)
    assignments=scheduler.dispatch()
    if len(assignments)!=1 or assignments[0].order_id!=order.order_id:
        raise RuntimeError('Stage 2 scheduler did not dispatch the READY test order')
    assignment=assignments[0]
    REPORT['stage2']={'order_id':int(order.order_id), 'assigned_robot':int(assignment.robot_id),
                      'ready_sequence':order.ready_seq, 'scheduler_policy':'READY-time FCFS, robot ID order',
                      'fleet_size':len(fleet)}
    robot=SingleRobot(grid,Pose(spawn.x,spawn.y,spawn.heading))
    checker=CollisionChecker(env)
    if not checker.is_pose_free(robot.pose.x,robot.pose.y,robot.pose.theta):
        raise RuntimeError('Spawn fails exact robot footprint collision validation')
    orders.advance(order.order_id, OrderStatus.TO_PICKUP)
    reached_pickup=robot.travel((pickup.x,pickup.y))
    if not reached_pickup:raise RuntimeError('Robot could not reach the food pickup point')
    if not all(checker.is_pose_free(p.x,p.y,p.theta) for p in robot.trace):
        raise RuntimeError('Robot trace collides before pickup')
    orders.advance(order.order_id, OrderStatus.LOADING)
    # Simulated counter-staff loading confirmation: not a live Blender UI yet.
    clock.advance()
    orders.advance(order.order_id, OrderStatus.TO_CUSTOMER)
    reached_table=robot.travel((drop.x,drop.y))
    if not reached_table:raise RuntimeError('Robot could not reach the customer table')
    valid=all(checker.is_pose_free(p.x,p.y,p.theta) for p in robot.trace)
    if not valid:raise RuntimeError('Robot trace fails oriented footprint validation')
    orders.advance(order.order_id, OrderStatus.UNLOADING)
    clock.advance()
    orders.advance(order.order_id, OrderStatus.DELIVERED)
    scheduler.ledger.check_consistency()
    if not fleet.is_available(assignment.robot_id):
        raise RuntimeError('Stage 2 robot not released after completed order')
    REPORT['stage2'].update({'final_order_state':order.status.value,
                             'robot_available_after_delivery':True,
                             'order_history':[s.value for _,s in order.history]})
    REPORT['mission']['completed']=True
    REPORT['mission']['trace_points']=len(robot.trace)
    REPORT['mission']['trace_footprint_clear']=valid
    REPORT['stage6']={'ekf_final_pose':{'x':robot.filter.pose.x,
                                      'y':robot.filter.pose.y,
                                      'theta':robot.filter.pose.theta},
                      'ground_truth_final_pose':{'x':robot.pose.x,'y':robot.pose.y,'theta':robot.pose.theta},
                      'note':'EKF uses simulated noisy odometry and XY observations; LiDAR is not driving control yet.'}
    REPORT['stage1']={'approved_scene_preserved':True,
                      'blender_only_for_extraction_and_visualization':True}
    # Visualize a verified simulated robot trace. This is trajectory playback, not live Blender physics.
    coords=[(p.x,p.y) for p in robot.trace]
    stride=max(1,len(coords)//1600)
    viz_polyline('AIFR_DEMO_ROBOT_TRACE',coords[::stride]+[coords[-1]],0.15,(0.05,0.8,0.1,1))
    bpy.ops.mesh.primitive_cube_add(size=1,location=(spawn.x,spawn.y,0.25))
    cube=bpy.context.object;cube.name='AIFR_DEMO_ROBOT';cube.dimensions=(0.4,0.36,0.28)
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    frames_per_step=1
    for index,p in enumerate(robot.trace[::stride]):
        frame=1+index*frames_per_step
        cube.location=(p.x,p.y,0.25);cube.rotation_euler[2]=p.theta
        cube.keyframe_insert(data_path='location',frame=frame)
        cube.keyframe_insert(data_path='rotation_euler',index=2,frame=frame)
    bpy.context.scene.frame_end=max(1,len(robot.trace[::stride]))
    bpy.context.scene.frame_set(1)
    if REPORT['stage2']['final_order_state'] != 'DELIVERED' or not REPORT['mission']['trace_footprint_clear']:
        raise RuntimeError('Mission or collision validation failed')
    REPORT['success']=True
    REPORT['integration_level']='one real Stage 2 order, Stage 3 extraction, Stage 4 safety map, Stage 5 trajectory, Stage 6 EKF; offline Blender playback'
    REPORT['warnings'].append('Trajectory is offline playback; real-time DWA and multi-robot traffic control are not integrated. Stage 2 scheduler is integrated, but base-return and order UI are not implemented.')
    print('AIFR STAGES 1–6 SUCCESS: outputs/stage36_report.json')
except Exception as exc:
    REPORT['error']=str(exc);REPORT['traceback']=traceback.format_exc()
    print('AIFR BRIDGE ERROR:',exc)
finally:
    save()
