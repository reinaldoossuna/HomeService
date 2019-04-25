#!/bin/bash

x-terminal-emulator -x roslaunch turtlebot_gazebo turtlebot_world.launch &

sleep 5 &&

x-terminal-emulator -x roslaunch turtlebot_gazebo amcl_demo.launch &

sleep 5 &&

x-terminal-emulator -x roslaunch turtlebot_rviz_launchers homeservice.launch &

sleep 5 &&

x-terminal-emulator -x rosrun pick_objects pick_objects &

sleep 5 &&

x-terminal-emulator -x rosrun add_markers add_markers

