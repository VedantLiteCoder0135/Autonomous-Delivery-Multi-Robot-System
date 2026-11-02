"""Phase 1 coverage for narrow-space and pickup-zone audit."""
from pathlib import Path
import json
import math

from tools.geometry_audit import _rect_gap, _approach_candidates, audit_environment,grid_clearance
from tools.stage79_engine import load_reference
from aifr.environment.environment_mapper import Rect2D
from aifr.navigation.collision_checker import CollisionChecker
from aifr.navigation.occupancy_grid import OccupancyGrid
from tools.geometry_audit import AuditSettings


def test_opposing_faces_only_when_projection_overlaps():
    a=Rect2D(0,1,0,1)
    assert _rect_gap(a,Rect2D(1.25,2,0.2,0.8))==(0.25,'horizontal')
    assert _rect_gap(a,Rect2D(1.25,2,1.2,2)) is None


def test_distance_field_reaches_obstacles_and_edges():
    g=OccupancyGrid(0,0,.1,7,7,tuple(tuple(1 if x==3 and y==3 else 0 for x in range(7)) for y in range(7)))
    d=grid_clearance(g)
    assert d[3][3]==0
    assert math.isclose(d[3][4],.1)
    assert d[0][0]==0
    assert d[2][3] == .1


def test_actual_v14_stations_and_heading_check():
    p=Path(__file__).resolve().parents[1]
    env,g,_=load_reference(p)
    report=audit_environment(env,g)
    assert report['entrances_count']==4
    assert {p['station'] for p in report['pickup_stations']}=={'MAIN_COURSE','JUICE','TEA','SANDWICH'}
    assert len(report['waiting_candidates'])>0
    assert report['grid_already_footprint_inflated'] is True
    for station in report['pickup_stations']:
        for pose in station['pickup_poses']:
            assert pose['status'] in ('CANDIDATE_FEASIBLE','UNVERIFIED_APPROACH','UNSAFE_POSE')
            assert isinstance(pose['full_in_place_rotation_clear'],bool)
            if pose['status']=='CANDIDATE_FEASIBLE':
                assert pose['verified_straight_final_approach'] is not None
                assert pose['astar_reachable']
    assert 'steering' in ' '.join(report['scope_limitations']).lower()


def test_reject_near_wall_final_approach():
    # A full synthetic environment isn't needed to verify the checker guard:
    # impossible final pose is rejected before approach searching.
    class Reject:
        def is_pose_free(self,*args):return False
        def is_motion_free(self,*args):return False
    g=OccupancyGrid(0,0,.1,20,20,tuple(tuple(0 for _ in range(20)) for _ in range(20)))
    assert _approach_candidates(.4,.4,0,Reject(),g,AuditSettings())==[]


def test_audit_is_deterministic_and_read_only():
    p=Path(__file__).resolve().parents[1]
    env,g,_=load_reference(p)
    before=g.cells
    a=audit_environment(env,g)
    b=audit_environment(env,g)
    assert json.dumps(a,sort_keys=True)==json.dumps(b,sort_keys=True)
    assert g.cells==before
