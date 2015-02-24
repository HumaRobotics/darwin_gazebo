## darwin_gazebo

ROS package providing Gazebo simulation of the Darwin OP robot.
Also provides a Python interface to the joints and some walk capabilities.

These have been tested in simulation and need some work to be used on the real robot, do not use as-is.

![Darwin model in Gazebo](/darwin.png?raw=true "Darwin model in Gazebo")

## Tutorial

A tutorial describing how to use this package can be found at:

http://www.generationrobots.com/en/content/83-carry-out-simulations-and-make-your-darwin-op-walk-with-gazebo-and-ros

## Install

Clone in your catkin workspace and catkin_make it.
Make sure you also have the following packages in your workspace
* darwin_description: https://github.com/HumaRobotics/darwin_description
* darwin_control: https://github.com/HumaRobotics/darwin_control
    
## Usage

You can launch the simulation with:

    roslaunch darwin_gazebo darwin_gazebo.launch
    
PRESS PLAY IN GAZEBO ONLY WHEN EVERYTHING IS LOADED (wait for controllers)

You can run a walk demo with:

    rosrun darwin_gazebo walker_demo.py

## ROS API

All topics are provided in the /darwin namespace.

Sensors:

    /darwin/camera/image_raw
    /darwin/imu
    /darwin/joint_states

Actuators (radians for position control, arbitrary normalized speed for cmd_vel):

    /darwin/cmd_vel
    /darwin/j_ankle1_l_position_controller/command
    /darwin/j_ankle1_r_position_controller/command
    /darwin/j_ankle2_l_position_controller/command
    /darwin/j_ankle2_r_position_controller/command
    /darwin/j_gripper_l_position_controller/command
    /darwin/j_gripper_r_position_controller/command
    /darwin/j_high_arm_l_position_controller/command
    /darwin/j_high_arm_r_position_controller/command
    /darwin/j_low_arm_l_position_controller/command
    /darwin/j_low_arm_r_position_controller/command
    /darwin/j_pan_position_controller/command
    /darwin/j_pelvis_l_position_controller/command
    /darwin/j_pelvis_r_position_controller/command
    /darwin/j_shoulder_l_position_controller/command
    /darwin/j_shoulder_r_position_controller/command
    /darwin/j_thigh1_l_position_controller/command
    /darwin/j_thigh1_r_position_controller/command
    /darwin/j_thigh2_l_position_controller/command
    /darwin/j_thigh2_r_position_controller/command
    /darwin/j_tibia_l_position_controller/command
    /darwin/j_tibia_r_position_controller/command
    /darwin/j_tilt_position_controller/command
    /darwin/j_wrist_l_position_controller/command
    /darwin/j_wrist_r_position_controller/command

## Python API

Basic usage:
```python
import rospy
from darwin_gazebo.darwin import Darwin

rospy.init_node("walker_demo")

darwin=Darwin()
rospy.sleep(1)

darwin.set_walk_velocity(1,0,0) # Set full speed ahead for 5 secs
rospy.sleep(5)
darwin.set_walk_velocity(0,0,0) # Stop
```
## Dependencies

The following ROS packages have to be installed:
* gazebo_ros_control
* hector_gazebo

## License

This software is provided by Génération Robots http://www.generationrobots.com and HumaRobotics http://www.humarobotics.com under the Simplified BSD license