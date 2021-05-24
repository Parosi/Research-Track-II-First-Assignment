# Research Track II - First Assignment
> The goal of this assignment is the development of a simulation in a 3D enviroment via Gazebo and RViz of a mobile robot.

In particular we have modified the given **go_to_point** node to implement an action instead of a service. This allows the user to stop the robot anytime in its current position.

## Implementation

In order to do that we have created a custom action **GoToPoint.action**. It has a goal with three variables (x, y, theta) which represent the position and orientation that the robot has to reach. The result and the feedback have the same structure, they are composed by four variables (x, y, theta, state) which represent the current position of the robot and its current state.

The **go_to_point** node has been modified in order to be an action server.

We have modified also the **state_machine** node. In particular we have implemented an action client of the **GoToPoint** action. After that, to use the action in the right way (the user can stop it anytime) we have changed the logic in the main function. It is composed by three if-else statements based on the two boolean variables **start** and **previous_start**. These statements implement the three possible behaviours of the system: 

* the robot needs a new goal, this happens if the user starts the robot or if the goal has been reached. So here we get a new random position from the **position_service** and we send it to the **go_to_point** action server.
* the robot has reached the target
* the user has stopped the robot


## Installation

This project is developed and tested with ROS Noetic, it may not work properly with other ROS versions.

Linux:

Copy the ROS package (**rt2_assignment1**) into your ROS workspace and build it with the command catkin_make in the shell.


## Usage example

Linux:

Run the simulation:
```sh
roslaunch rt2_assignment1 sim.launch
```

## Meta

Riccardo Parosi – paros97@gmail.com

[https://github.com/Parosi/github-link](https://github.com/Parosi/)


