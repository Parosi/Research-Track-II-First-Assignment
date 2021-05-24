# Research Track II - First Assignment
> The goal of this assignment is the development of a simulation in a 3D enviroment via Coppelia Simulator of a mobile robot.

In particular we have used the Pioneer P3DX as the model of the robot.

## Implementation

The package **rt2_assignment1** was given. It contains four nodes and three custom services. It is used to get a random point in the space and control the robot to get it to that point. This package also contains two launch files. The first one **sim.launch** was given and it implements robot simulation in RViz and Gazibo. The second one is **sim_coppelia.launch** and it starts the four Ros nodes.

The second package is **simExtROS**. It is used to customize the Coppelia-Ros integration. However, all the messages needed from Coppelia are not custom ones and so they were already implemented in the package.

Regarding the simulation we have create a Coppelia's scene **rt2_assignment1.ttt**. Inside it we have deleted the default sensors of the P3DX, since there are no obstacles in the enviroment. We have also resized the floor because it was too small and the robot fell sometimes. In the script of the robot we have created a publisher to the topic **/odom**, which is used to publish the current position of the robot, and a subscriber to the topic **/cmd_vel**, which is used to receive the linear and angular velocities. The callback of the subscriber transforms the angular and linear velocities (from **geometry_msgs/Twist** message) to the differential velocities related to the two wheels of the P3DX and set them for the two motors (left and right). In the **sysCall_actuation** we get the **Pose** (**Position** plus **Orientation**) of the robot and we insert it in a **nav_msgs/Odometry** message. After that, we publish the message and wait (via the sleep function) 100 ms before restart the actuation function. 


## Installation

This project is developed and tested with ROS Noetic, it may not work properly with other ROS versions.

Linux:

Copy the two ROS packages (rt2_assignment1 and simExtROS) into your ROS workspace and build your workspace with the command catkin_make in the shell.


## Usage example

Linux:

First run the ros simulation:
```sh
roslaunch rt2_assignment1 sim_coppelia.launch
```

Run the Coppelia Sim (in your Coppelia's folder):
```sh
./coppeliaSim.sh
```

Open the Coppelia's scene **rt2_assignment1.ttt**

## Meta

Riccardo Parosi – paros97@gmail.com

[https://github.com/Parosi/github-link](https://github.com/Parosi/)


