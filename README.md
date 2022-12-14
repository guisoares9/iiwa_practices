# iiwa_practices: KUKA LBR Iiwa 14 simulation and real manipulation exercises

Simulation and real manipulation exercises with LBR Iiwa 14 using robotic toobox and ROS. It is rendered with Swift. All the rostopics try to simulate the real robot, that works with our version of KUKA-IIWA-API.

## Installation

This is a ROS package, so it is necessary to clone to your ros workspace and build.

```bash
cd ~/catkin_ws/src
git clone https://github.com/guisoares9/iiwa_practices.git
cd ~/catkin_ws
catkin_make
```

## Exercise 1: send commands to iiwa simulation

To run the simulation:

>Terminal 1:

```bash
roslaunch iiwa_practices simulation.launch
```

This script will start a node that is subscribing the /kuka_command topic. To move the robot to a desired joint position we need to publish on /kuka_command topic a std_msgs/String message with a correct formatation: "SetPosition q1 q2 q3 q4 q5 q6 q7".

>Terminal 2:

```bash
rostopic pub /kuka_command std_msgs/String "data: 'setPosition 0 -40 0 105 0 -30 0'"
```

The robot will simulate the joint tracjetory from the current joint position to the desired joint position.

## Exercise 2: send command to real iiwa

Run the node that connects to iiwa via socket and then we can control iiwa with ROS

```bash
roslaunch iiwa_practices iiwa.launch
```

A simple movimentation is made by publishing on `setPosition` topic.

> CAUTION: DON'T stay near to the robot. Take the recommended security space to the operation. Ensure that the operation mode is T1 and press the dead man button to release the robot.
>

```bash
rostopic pub /kuka_command std_msgs/String "data: 'setPosition 0 -40 0 105 0 -30 0'"
```

## Exercise 3: planning point to point motions with joint configurations

Here we will define some spacial points (you can configure it) and send the robot for the point with a specific order

For now, we define 4 joint configurations for a simple pick and place task:

```python
[[0, 50, 0, -120, 0, 25, 0],
[0, 30, -10, -120, 0, 34, 0],
[70, 30, -10, -120, 0, 34, 0],
[70, 50, 0, -120, 0, 25, 0]]
```

First let's test with simulation. On terminal 1:

```bash
roslaunch iiwa_practices simulation.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ptp_joint.py
```

If it's all ok, run the same script now with the real robot. On terminal 1:

```bash
roslaunch iiwa_practices iiwa.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ptp_joint.py
```

## Exercise 4: point to point with joint trajectory

This exercise will show limitations about this external controllers. We will use the joint trajectory procedure and try to apply it to iiwa. First, we will test the code on the simulation.

Terminal 1:

```bash
roslaunch iiwa_practices simulation.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ptp_jtraj.py
```

We made the simulation to behave exactly as the real robot. The stops on the trajectory occurs because the robot has it own joint trajectory planner. When we send the desired joint configuration to the robot, it takes the actual position and build a joint trajectory to the desired position with the constrains that the initial and final velocity is zero, even for close gaps.

To see it on the real robot, on terminal 1

```bash
roslaunch iiwa_practices iiwa.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ptp_jtraj.py
```

## Exercise 5: inverse kinematicsssssss!

Lets test the inverse kinematics on the simulation by choosing an arbitrary achievable points on space, calculate the joint configuration by numerical inverse kinematics and send it to the robot

Terminal 1:

```bash
roslaunch iiwa_practices simulation.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ptp_ik.py
```

If its all ok, let's test on the real robot.

```bash
roslaunch iiwa_practices iiwa.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ptp_ik.py
```

## Exercise 6: cartesian pose with joint trajectory

Lets test the difference between joint trajectory and cartesian trajectory between two points on space. On the simulation first:

Terminal 1:

```bash
roslaunch iiwa_practices simulation.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ik_jtraj.py
```

If all ok:

```bash
roslaunch iiwa_practices iiwa.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ik_jtraj.py
```

## Exercise 7: cartesian pose with cartesian trajectory


Lets test the difference between joint trajectory and cartesian trajectory between two points on space. On the simulation first for the cartesian trajectory now:

Terminal 1:

```bash
roslaunch iiwa_practices simulation.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ik_ctraj.py
```

If all ok:

```bash
roslaunch iiwa_practices iiwa.launch
```

Terminal 2:

```bash
roscd iiwa_practices/src
python3 ik_ctraj.py
```
