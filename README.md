# Research Track II - First Assignment
> The goal of this assignment is the development of a simulation in a 3D enviroment via Gazebo and RViz of a mobile robot.

In particular we have written two of the four given nodes in ROS2, and we have implemented a bridge to let communicate ROS and ROS2.

## Implementation

In order to do that we have created a ROS2 package **rt2_assignment1**, it contains three custom services (**Command**, **Position**, **RandomPosition**) and the mapping rules file (**rt2_assignment1_mapping_rules.yaml**) which describes the map between ROS and ROS2 services.

Regarding the nodes of the **rt2_assignment1** package, they are implemented as components so they are subclasses of the class **Node**. In particular they are two:

**position_service** is a ROS2 service that generates a random point inside a given interval and also a random orientation.

**state_machine** is another ROS2 service which permits to control the various states of the robot. It is also a client of the above described service and of the ROS **go_to_point** service. Since it is a component, it doesn't have a main function. In fact, to spin it we have used a timer (500ms) with its associated callback. As the rclcpp:spin() function does, the system has to send a request and receive the reply in different iteration of the timer callback. To do that we have structured the callback in three if-else statements based on three boolean variables (**start_**, **rp_waiting_**, **p_waiting_**). These stamements implement the three different states of the system:

* the robot needs a new goal, this happens if the user starts the robot or if the goal has been reached. So here we send a request to the **position_service**.
* we are waiting a response from the above service. We wait for it 50 milliseconds. If it arrives within this time interval, we use it to compose and send the request to the **go_to_point** service.
* we are waiting a reply from the just cited service. As before, we wait for it 50 milliseconds. If it arrives, we print on the screen "Position reached" and we set the the three booleans in order to start from the first state, in the new iteration of the callback.

Finally in the **rt2_assignment1** package there is also a launch script **rt2_assignment1_launch.py** which runs the two, above described, components in a single process container.

The **ros1_bridge** is a ROS2 component which implements the bridge between ROS and ROS2.

Regarding the ROS part, it is the given one, except an additional launch file (**sim_with_ros2.launch**) which launches only two nodes (**go_to_point** and **userInterface**) and the simulation (Gazebo and RViz).

Instead the **.sh** files are used to source the correct operating system (ROS or ROS2) and to run the simulation (**rt2_assignment1.sh**).


## Installation

This project is developed and tested with ROS Noetic and ROS2 Foxy, it may not work properly with other ROS and ROS2 versions.

Linux:

* Copy the four **.sh** files where you prefer and modify them with your ROS and ROS2 workspaces paths.
* Copy the ROS launch file in your ROS workspace and build it, in a new terminal, writing the following two commands:
```sh
source ros.sh
catkin_make
```
* Copy the ROS2 packages into your ROS2 workspace and build it, in a new shell, writing the following two commands:
```sh
source ros2.sh
colcon build --packages-skip ros1_bridge
```
* Build also the bridge, in a new terminal, with the following commands:
```sh
source ros12.sh
colcon build --packages-select ros1_bridge --cmake-force-configure
```


## Usage example

Linux:

Run the simulation:
```sh
source rt2_assignment1.sh
```

## Meta

Riccardo Parosi – paros97@gmail.com

[https://github.com/Parosi/github-link](https://github.com/Parosi/)


