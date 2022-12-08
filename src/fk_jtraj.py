#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import roboticstoolbox as rtb
import numpy as np

# Callback for kuka isReadyToMove flag that tells if kuka is ready to perform a new movement
is_finished = False
def ready_callback(data):
    global is_finished
    is_finished = data.data.split(' ')[0] == 'True'

# receives a list of joint angles and returns a message prepared for sending to kuka through /kuka_command topic
# example: [0, 0, 0, 0, 0, 0, 0] -> "SetPosition 0 0 0 0 0 0 0"
def qpos_to_msg(qpos):
    return 'setPosition ' + ' '.join(map(str, qpos))

def move_and_wait(msg):
    global is_finished
    iiwa_pub.publish(msg)
    print('waiting')
    # rospy.sleep(1)
    # wait for the robot start movement
    while is_finished:
        pass
    print('started')
    # wait for the robot to reach the desired position
    while not is_finished:
        rospy.sleep(0.01)
    print('finished')
    return

# Set kuka communication
iiwa_pub = rospy.Publisher('kuka_command', String, queue_size=10)
rospy.Subscriber("isFinished", String, ready_callback)
rospy.init_node('cartesian_planner', disable_signals=True)

# init iiwa robot model
iiwa = rtb.models.URDF.LBR()

# joint trajectory
qdes = np.deg2rad([50, 100, 50, 0, 0, 0, 100])
traj = rtb.jtraj(iiwa.qz, qdes, 10)

# warm up for safety
print("Waiting for kuka to be ready...")
while not is_finished:
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
    move_and_wait(q_msg)
    
    rospy.sleep(0.01)