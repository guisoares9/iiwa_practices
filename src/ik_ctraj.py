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
# t = np.arange(0, 2, 0.10)
# sol_q0 = iiwa.ikine_LMS(T0)
# sol_q1 = iiwa.ikine_LMS(T1, q0=sol_q0.q)

# print('\nInitial point:')
# print(f'Desired cartesian pose: {T0.t} {T0.rpy(order="zyx")}')
# T0_check = iiwa.fkine(sol_q0[0])
# print(f'Forward kinematics check: {T0_check.t} {T0_check.rpy(order="zyx")}')
# print(f'Inverse kinematics solution: \nSuccess={sol_q0[1]==1} \nQ0: {sol_q0[0]} \nResidual (mm): {sol_q0[4]*1000}')

# print('\nFinal point:')
# print(f'Desired cartesian pose: {T1.t} {T1.rpy(order="zyx")}')
# T1_check = iiwa.fkine(sol_q1[0])
# print(f'Forward kinematics check: {T1_check.t} {T1_check.rpy(order="zyx")}')
# print(f'Inverse kinematics solution: \nSuccess={sol_q1[1]==1} \n Q1: {sol_q1[0]} \nResidual (mm): {sol_q1[4]*1000}')

# joint trajectory
T_traj = rtb.ctraj(T0, T1, 10)
# sol_traj = [iiwa.ikine_LMS(T) for T in T_traj]
sol_traj = iiwa.ikine_LM(T_traj, ilimit=2000)

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