import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from math import pi
import rospy
from std_msgs.msg import String

class IiwaROS:
    def __init__(self):
        # Publisher for kuka
        self.iiwa_pub = rospy.Publisher('kuka_command', String, queue_size=10)

        # subscriber for kuka isReadyToMove flag
        self.is_finished = False
        rospy.Subscriber("isFinished", String, self.ready_callback)

        # init ros node
        rospy.init_node('cartesian_planner', disable_signals=True)
    
    # Callback for kuka isFinished flag that tells if kuka is ready to perform a new movement
    def ready_callback(self, data):
        self.is_finished = data.data.split(' ')[0] == 'True'

    # Wait for kuka to be ready
    def wait_for_kuka(self):
        # warm up for safety
        print("Waiting for kuka to be ready...")
        while not self.is_finished:
            rospy.sleep(0.1)
        print("Kuka is ready!")
        print("Warm up: take SmartPad and hold safety button")
        input('Press enter to start the movement...')
    
    # receives a list of joint angles and returns a message prepared for sending to kuka through /kuka_command topic
    # example: [0, 0, 0, 0, 0, 0, 0] -> "SetPosition 0 0 0 0 0 0 0"
    def qpos_to_msg(self, qpos):
        return 'setPosition ' + ' '.join(map(str, qpos))

    # Joint space movement and wait for the robot to finish the movement
    def move_and_wait(self, qpos, unit='deg'):
        
        if unit == 'rad':
            qpos = np.rad2deg(qpos)
        
        # convert qpos to message
        q_msg = self.qpos_to_msg(qpos)

        # send the message to kuka
        print("Sending qpos: " + q_msg)
        self.iiwa_pub.publish(q_msg)

        # wait for the robot start movement
        # print('waiting')
        while self.is_finished:
            continue

        # wait for the robot to reach the desired position
        # print('started')
        while not self.is_finished:
            rospy.sleep(0.01)
        # print('finished')
        
        return


