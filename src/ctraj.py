#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

# Callback for kuka isReadyToMove flag that tells if kuka is ready to perform a new movement
is_ready = False
def ready_callback(data):
    global is_ready
    is_ready = bool(data.data)

# receives a list of joint angles and returns a message prepared for sending to kuka through /kuka_command topic
# example: [0, 0, 0, 0, 0, 0, 0] -> "SetPosition 0 0 0 0 0 0 0"
def qpos_to_msg(qpos):
    return 'SetPosition ' + ' '.join(map(str, qpos))

def move_and_wait(msg):
    global is_ready
    iiwa_pub.publish(msg)
    rospy.sleep(0.1)
    while not is_ready:
        rospy.sleep(0.01)
    return

iiwa = rtb.models.URDF.LBR()

# T0 = SE3(-0.26021681811932573, 0.06252253712579554, 1.073962547966687)
# sol = iiwa.ik_lm_chan(T0, tol=1e-3, ilimit=1000)
# print(sol)
# input()

# cartesian trajectory
t = np.arange(0, 2, 0.10)
T0 = SE3(-0.26021681811932573, 0.06252253712579554, 1.073962547966687)
# T0 = SE3(0.72, 0.2714988360920022, 0.36681749584930307)
T1 = SE3(0.7121351744423083, 0.2714988360920022, 0.36681749584930307)
Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))
# sol = iiwa.ikine_min(Ts, q0=[0, 0, 1.57, 0, 0, 0, 0], tol=1e-3, ilimit=1000, mask=[1, 1, 1, 0, 0, 0])
sol_all = [iiwa.ik_lm_chan(T, tol=1e-2, ilimit=1000) for T in Ts]
print(sol_all)
# print(sol[0])

# Publisher for kuka
iiwa_pub = rospy.Publisher('kuka_command', String, queue_size=10)

# subscriber for kuka isReadyToMove flag
rospy.Subscriber("isReadyToMove", String, ready_callback)

# init ros node
rospy.init_node('cartesian_planner', disable_signals=True)

# warm up for safety
print("Waiting for kuka to be ready...")
while not is_ready:
    rospy.sleep(0.1)
print("Kuka is ready!")
print("Warm up: take SmartPad and hold safety button")
input('Press enter to start the movement...')

# main loop
for sol in sol_all:
    if rospy.is_shutdown():
        break
    
    q = np.rad2deg(sol[0])
    q_msg = qpos_to_msg(q)
    print("Sending qpos: " + q_msg)
    move_and_wait(q_msg)
    
    rospy.sleep(0.01)