import math
from aifr.navigation.occupancy_grid import OccupancyGrid
from aifr.navigation.astar import find_path
from aifr.navigation.kinematics import Pose,advance,wheel_speeds
from aifr.perception.ekf import PoseEKF
from aifr.perception.sensors import SensorSuite,lidar_scan
from aifr.simulation.delivery import SingleRobot
def grid():return OccupancyGrid(0.,0.,0.2,15,15,tuple(tuple(0 for _ in range(15)) for _ in range(15)))
def test_astar():
    p=find_path(grid(),(0.5,0.5),(2.5,2.5)); assert len(p)>2
def test_astar_no_diagonal_corner_cut():
    cells=[[0]*3 for _ in range(3)];cells[0][1]=1;cells[1][0]=1
    g=OccupancyGrid(0,0,1,3,3,tuple(map(tuple,cells)))
    assert not find_path(g,(0.5,0.5),(2.5,2.5))
def test_motion():
    p=advance(Pose(0.,0.,0.),0.4,0.,1.);assert math.isclose(p.x,.4)
    assert wheel_speeds(0.,0.)==(0.,0.)
def test_ekf_reduces_error():
    ekf=PoseEKF(Pose(0.,0.,0.));ekf.predict(0.2,0.,1.)
    before=abs(ekf.pose.x-0.5);ekf.correct_xy(.5,0.,0.0001)
    assert abs(ekf.pose.x-0.5)<before
def test_sensors_repeatable():
    a=SensorSuite(seed=5);b=SensorSuite(seed=5)
    assert a.odometry(.2,.1)==b.odometry(.2,.1)
    assert lidar_scan(Pose(0,0,0),[(1.,-0.2,1.2,.2)],rays=4)[0]<2
def test_single_robot_delivery():
    r=SingleRobot(grid(),Pose(.5,.5,0.))
    assert r.deliver((1.3,.5),(1.3,1.3))
    assert len(r.trace)>10
