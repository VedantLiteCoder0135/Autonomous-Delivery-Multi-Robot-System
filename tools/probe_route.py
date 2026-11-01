from pathlib import Path
import json,sys,math,time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from aifr.navigation.dwa_route import RouteDWA
from aifr.navigation.astar import find_path
from aifr.navigation.kinematics import Pose,advance
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.navigation.dwa import DWAConfig
from aifr.environment.environment_mapper import CafeEnvironment
p=Path(__file__).resolve().parents[1]
g=json.loads((p/'data/reference_cafeteria/stage4_occupancy_grid.json').read_text())
grid=OccupancyGrid(*g['origin'],g['resolution'],g['width'],g['height'],tuple(tuple(row) for row in g['cells']))
r=json.loads((p/'data/reference_cafeteria/stage36_report.json').read_text())['mission']
pose=Pose(r['spawn']['x'],r['spawn']['y'],r['spawn']['heading'])
config=DWAConfig()
planner=RouteDWA(grid,config)
v=w=0.;t0=time.time()
for leg in ['pickup','drop','spawn']:
 dest=r[leg] if leg!='spawn' else r['spawn']
 path=find_path(grid,(pose.x,pose.y),(dest['x'],dest['y']))
 print('leg',leg,'pathlen',len(path),flush=True)
 planner.set_path(path)
 for i in range(5000):
  cmd=planner.plan(pose,v,w)
  if not cmd.safe:
   print('FAILED',leg,i,pose,v,w,cmd,planner.last_target,flush=True);break
  if cmd.reached:
   print('REACHED',leg,i,pose,'elapsed',time.time()-t0,flush=True);break
  v,w=cmd.linear,cmd.angular
  pose=advance(pose,v,w,config.dt)
 else:print('TIMEOUT',leg,pose,flush=True)
