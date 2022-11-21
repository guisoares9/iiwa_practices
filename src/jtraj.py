#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import roboticstoolbox as rtb
import numpy as np

# Callback for kuka isReadyToMove flag that tells if kuka is ready to perform a new movement
is_ready = False
def ready_callback(data):
    global is_ready
    is_ready = data.data == 'True'
    return

# receives a list of joint angles and returns a message prepared for sending to kuka through /kuka_command topic
# example: [0, 0, 0, 0, 0, 0, 0] -> "SetPosition 0 0 0 0 0 0 0"
def qpos_to_msg(qpos):
    return 'SetPosition ' + ' '.join(map(str, qpos))

def distance(q1, q2):
    return np.sqrt(np.sum(np.square(q1 - q2)))

def move_and_wait(msg, qdes):
    global is_ready
    iiwa_pub.publish(msg)
    print('waiting')
    # rospy.sleep(1)
    # wait for the robot start movement
    while is_ready:
        pass
    print('started')
    # wait for the robot to reach the desired position
    while not is_ready:
        rospy.sleep(0.01)
    print('finished')
    return

# Publisher for kuka
iiwa_pub = rospy.Publisher('kuka_command', String, queue_size=10)

# subscriber for kuka isReadyToMove flag
rospy.Subscriber("isReadyToMove", String, ready_callback)

# init ros node
rospy.init_node('cartesian_planner', disable_signals=True)

# init iiwa robot model
iiwa = rtb.models.URDF.LBR()

# joint trajectory
qdes = np.deg2rad([50, 100, 50, 100, 100, 100, 100])
traj = rtb.jtraj(iiwa.qz, qdes, 10)

# warm up for safety
print("Waiting for kuka to be ready...")
while not is_ready:
    rospy.sleep(0.1)
print("Kuka is ready!")
print("Warm up: take SmartPad and hold safety button")
# input('Press enter to start the movement...')

# main loop
for q in traj.q:
    if rospy.is_shutdown():
        break
    
    q = np.rad2deg(q)
    print(f'Moving to {q}')
    q_msg = qpos_to_msg(q)
    move_and_wait(q_msg, q)
    
    rospy.sleep(0.01)