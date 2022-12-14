#!/usr/bin/env python3

import roboticstoolbox as rtb
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

# joint trajectory
q1 = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
q2 = np.deg2rad([50, 100, 50, 0, 0, 0, 100])
traj = rtb.jtraj(q1, q2, 20)

# wait for kuka to be ready
comm.wait_for_kuka()

# main loop
for q in traj.q:
    comm.move_and_wait(q, unit='rad')