#!/usr/bin/env python3

import rospy
import roboticstoolbox as rtb
from math import pi
import swift
import time
import numpy as np
from std_msgs.msg import String


class KukaSimulatedROS:
    def __init__(self) -> None:
        
        
        # Kuka LBR iiwa 14 R820 from robotic toolbox library
        self.kuka_robot_urdf = rtb.models.URDF.LBR()

        #    Make a listener for kuka_iiwa commands
        rospy.Subscriber("kuka_command", String, self.command_callback)

        #   Make Publishers for all kuka_iiwa data
        self.pub_JointPosition = rospy.Publisher('JointPosition', String, queue_size=10)
        self.pub_ToolPosition = rospy.Publisher('ToolPosition', String, queue_size=10)
        self.pub_ToolForce = rospy.Publisher('ToolForce', String, queue_size=10)
        self.pub_ToolTorque = rospy.Publisher('ToolTorque', String, queue_size=10)
        self.pub_isCompliance = rospy.Publisher('isCompliance', String, queue_size=10)
        self.pub_isCollision = rospy.Publisher('isCollision', String, queue_size=10)
        self.pub_isReadyToMove = rospy.Publisher('isReadyToMove', String, queue_size=10)
        self.pub_isMastered = rospy.Publisher('isMastered', String, queue_size=10)
        self.pub_OperationMode = rospy.Publisher('OperationMode', String, queue_size=10)
        self.pub_JointAcceleration = rospy.Publisher('JointAcceleration', String, queue_size=10)
        self.pub_JointVelocity = rospy.Publisher('JointVelocity', String, queue_size=10)
        self.pub_JointJerk = rospy.Publisher('JointJerk', String, queue_size=10)
        self.pub_isFinished = rospy.Publisher('isFinished', String, queue_size=10)
        self.pub_hasError = rospy.Publisher('hasError', String, queue_size=10)
        self.pub_OperatorAck = rospy.Publisher('OperatorAck', String, queue_size=10)

        #   Make kuka_iiwa node
        rospy.init_node('kuka_iiwa', disable_signals=True)
        rate = rospy.Rate(100) #    100hz update rate.

        # Kuka state
        self.JointPosition = [0,0,0,0,0,0,0]
        self.ToolPosition = [0,0,0,0,0,0]
        self.ToolForce = [0,0,0]
        self.ToolTorque = [0,0,0]
        self.isCompliance = False
        self.isCollision = False
        self.isReadyToMove = True
        self.isMastered = True
        self.OperationMode = 0
        self.JointAcceleration = [0,0,0,0,0,0,0]
        self.JointVelocity = [0,0,0,0,0,0,0]
        self.JointJerk = [0,0,0,0,0,0,0]
        self.isFinished = True
        self.hasError = False
        self.OperatorAck = False

        # Desired state
        self.DesPosition = [0,0,0,0,0,0,0]
        self.move_flag = False

        # start a rostimer to publish states
        publish_fps = 100
        rospy.Timer(rospy.Duration(1/publish_fps), self.publish_states)

    def command_callback(self, msg):
        rospy.loginfo("Command received: %s", msg.data)
        
        # handle if command is SetPosition
        data = msg.data
        command = data.split(' ')[0]
        args = data.split(' ')[1:]
        if command == "SetPosition":
            if len(args) != 7:
                rospy.logerr("SetPosition command requires 7 arguments")
                return
            
            # get desired joint position
            for i in range(7):
                self.DesPosition[i] = args[i]

            # set move flag
            self.move_flag = True
        
        # TODO: implement another commands
        
        return
    def list_to_string(self, list):
        return ' '.join(map(str, list))

    def publish_states(self, event):

        # update joint position in degrees
        self.JointPosition = [round(qi * 180 / pi, 3) for qi in self.kuka_robot_urdf.q]

        # update tool position
        self.ToolPosition = self.kuka_robot_urdf.fkine(self.JointPosition).t

        # update joint velocity
        # TODO

        # update joint acceleration
        # TODO

        #  publish robot joint states
        self.pub_JointPosition.publish(self.list_to_string(self.JointPosition))
        self.pub_ToolPosition.publish(self.list_to_string(self.ToolPosition))
        # self.pub_ToolForce.publish(self.list_to_string(self.ToolForce))
        # self.pub_ToolTorque.publish(self.list_to_string(self.ToolTorque))
        # self.pub_JointAcceleration.publish(self.list_to_string(self.JointAcceleration))
        # self.pub_JointVelocity.publish(self.list_to_string(self.JointVelocity))
        # self.pub_JointJerk.publish(self.list_to_string(self.JointJerk))

        # publish robot boolean states
        self.pub_isCompliance.publish(str(self.isCompliance))
        self.pub_isCollision.publish(str(self.isCollision))
        self.pub_isReadyToMove.publish(str(self.isReadyToMove))
        self.pub_isMastered.publish(str(self.isMastered))
        self.pub_OperationMode.publish(str(self.OperationMode))
        self.pub_isFinished.publish(str(self.isFinished))
        self.pub_hasError.publish(str(self.hasError))
        self.pub_OperatorAck.publish(str(self.OperatorAck))
        return

# Simulate ros topics
kuka_ros = KukaSimulatedROS()

# Launch the simulator Swift
env = swift.Swift()
env.launch()    

# Add the robot to the simulator
env.add(kuka_ros.kuka_robot_urdf)

# This is our callback funciton from the sliders in Swift which set
# the joint angles of our robot to the value of the sliders
def set_joint(j, value):
    # print(f"aaaa{j}")
    kuka_ros.kuka_robot_urdf.q[j] = np.deg2rad(float(value))

def add_slider():
    # Loop through each link in the Kuka and if it is a variable joint,
    # add a slider to Swift to control it
    j = 0
    for link in kuka_ros.kuka_robot_urdf.links:
        if link.isjoint:

            # We use a lambda as the callback function from Swift
            # j=j is used to set the value of j rather than the variable j
            # We use the HTML unicode format for the degree sign in the unit arg
            env.add(
                swift.Slider(
                    lambda x, j=j: set_joint(j, x),
                    min=np.round(np.rad2deg(link.qlim[0]), 2),
                    max=np.round(np.rad2deg(link.qlim[1]), 2),
                    step=1,
                    value=np.round(np.rad2deg(kuka_ros.kuka_robot_urdf.q[j]), 2),
                    desc="Joint " + str(j),
                    unit="&#176;",
                )
            )
            j += 1

# move robot to the desird joint position with a joint motion planning
def move(des_position, t_move = 5):
    # trajectory steps and time to move
    steps = 100
    dt = t_move/steps

    # joint motion planning
    q_traj = rtb.jtraj(kuka_ros.kuka_robot_urdf.q, np.deg2rad(np.array(des_position, dtype=np.float32)), steps)

    # move the simulated robot
    for q in q_traj.q:
        kuka_ros.kuka_robot_urdf.q = q
        env.step(0)
        time.sleep(dt) 

# Add the sliders to Swift
add_slider()

q = np.array([0,0,0,0,0,0,0])
while True:
    # Process the event queue from Swift, this invokes the callback functions
    # from the sliders if the slider value was changed
    # env.process_events()
    
    if kuka_ros.move_flag:
        # update flags for busy state
        kuka_ros.isReadyToMove = False
        kuka_ros.isFinished = False

        # move robot on simulation
        move(kuka_ros.DesPosition)
        # move([100,50,50,20,4,56,0])

        # update flags for ready state
        kuka_ros.move_flag = False
        kuka_ros.isReadyToMove = True
        kuka_ros.isFinished = True

    # Update the environment with the new robot pose
    env.step(0)

    time.sleep(0.01)
