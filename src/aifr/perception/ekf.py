"""3-state EKF with velocity odometry prediction and XY landmark correction.

Matrices are kept as small lists to avoid dependencies. Uses Joseph covariance
update for stability and leaves heading unobserved by the XY sensor.
"""
from __future__ import annotations
import math
from aifr.navigation.kinematics import Pose,advance,normalize
class PoseEKF:
    def __init__(self,initial:Pose,variance:float=0.01):
        if variance<=0:raise ValueError('variance must be positive')
        self.pose=initial
        self.P=[[variance if i==j else 0. for j in range(3)] for i in range(3)]
    def predict(self,v:float,w:float,dt:float,motion_variance:float=0.001)->Pose:
        if dt<=0 or motion_variance<0:raise ValueError('invalid prediction parameters')
        t=self.pose.theta;self.pose=advance(self.pose,v,w,dt)
        # First-order midpoint linearization of motion model.
        m=t+w*dt*0.5
        F=[[1.,0.,-v*dt*math.sin(m)],[0.,1.,v*dt*math.cos(m)],[0.,0.,1.]]
        A=[[sum(F[i][k]*self.P[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        self.P=[[sum(A[i][k]*F[j][k] for k in range(3))+(motion_variance*dt if i==j else 0.) for j in range(3)] for i in range(3)]
        return self.pose
    def correct_xy(self,x:float,y:float,measurement_variance:float=0.0025)->Pose:
        if measurement_variance<=0:raise ValueError('measurement variance must be positive')
        p=self.P;s00=p[0][0]+measurement_variance;s01=p[0][1];s11=p[1][1]+measurement_variance
        det=s00*s11-s01*s01
        if det<=0:raise ValueError('singular innovation covariance')
        inv=[[s11/det,-s01/det],[-s01/det,s00/det]]
        K=[[p[i][0]*inv[0][j]+p[i][1]*inv[1][j] for j in range(2)] for i in range(3)]
        z=[x-self.pose.x,y-self.pose.y]
        self.pose=Pose(self.pose.x+sum(K[0][j]*z[j] for j in range(2)),self.pose.y+sum(K[1][j]*z[j] for j in range(2)),normalize(self.pose.theta+sum(K[2][j]*z[j] for j in range(2))))
        IK=[[float(i==j)-(K[i][j] if j<2 else 0.) for j in range(3)] for i in range(3)]
        A=[[sum(IK[i][k]*p[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        self.P=[[sum(A[i][k]*IK[j][k] for k in range(3))+measurement_variance*sum(K[i][k]*K[j][k] for k in range(2)) for j in range(3)] for i in range(3)]
        return self.pose
