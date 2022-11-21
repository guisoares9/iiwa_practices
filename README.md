# iiwa_practices: KUKA LBR Iiwa 14 simulation and real manipulation exercises

Simulation and real manipulation exercises with LBR Iiwa 14 using robotic toobox and ROS. It is rendered with Swift. All the rostopics try to simulate the real robot, that works with our version of KUKA-IIWA-API.

To run the simulation:

>Terminal 1:

```bash
roscore
```

>Terminal 2:

```bash
cd src
python3 iiwa_simulation_node.py
```

This script will start a node that is subscribing the /kuka_command topic. To move the robot to a desired joint position we need to publish on /kuka_command topic a std_msgs/String message with a correct formatation: "SetPosition q1 q2 q3 q4 q5 q6 q7".

>Terminal 3:

```bash
rostopic pub /kuka_command std_msgs/String "data: 'SetPosition 100 100 0 50 50 10 0'"
```

The robot will simulate the joint tracjetory from the current joint position to the desired joint position.