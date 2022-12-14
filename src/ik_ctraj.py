#!/usr/bin/env python3

import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from math import pi

# local import
from iiwa_ros import IiwaROS

# create communication via ROS for Kuka Iiwa 
comm = IiwaROS()

# start kuka model
# iiwa = rtb.models.URDF.LBR()
iiwa =rtb.DHRobot([ 
rtb.RevoluteDH( a=0, alpha=-pi/2, d=0.36, offset=-pi/2),
rtb.RevoluteDH( a=0, alpha=pi/2, d=0, offset=0),
rtb.RevoluteDH( a=0, alpha=pi/2, d=0.42, offset=0),
rtb.RevoluteDH( a=0, alpha=-pi/2, d=0, offset=0),
rtb.RevoluteDH( a=0, alpha=-pi/2, d=0.4, offset=0),
rtb.RevoluteDH( a=0, alpha=pi/2, d=0, offset=0),
rtb.RevoluteDH( a=0, alpha=0, d=0.126, offset=pi/2)], name='KUKA LBR iiwa 14 R820')

# cartesian pose
T0 = SE3(0.459, -0.389, 0.329) * SE3.RPY([2.996, 0.183, 1.697], order='zyx')

T1 = SE3(0.557, -0.1, 0.152) *  SE3.RPY([2.996, 0.183, 1.697], order='zyx')

# joint trajectory
T_traj = rtb.ctraj(T0, T1, 10)

# inverse kinematics
sol_traj = []
q0 = [0, 0, 0, 0, 0, 0, 0]
for T in T_traj:
    sol = iiwa.ikine_LMS(T, q0=q0)
    sol_traj.append(sol)
    q0 = sol.q

print(sol_traj)
# wait for kuka to be ready
comm.wait_for_kuka()

# main loop
for sol in sol_traj:
    comm.move_and_wait(sol.q, unit='rad')