#!/bin/bash

gnome-terminal --tab --title="ros_rt2_assignment1" -- bash -c "source ros.sh; roslaunch rt2_assignment1 sim_with_ros2.launch"

gnome-terminal --tab --title="bridge" -- bash -c "source ros12.sh; ros2 run ros1_bridge dynamic_bridge"

gnome-terminal --tab --title="ros2_rt2_assignment1" -- bash -c "source ros2.sh; ros2 launch rt2_assignment1 rt2_assignment1_launch.py"

